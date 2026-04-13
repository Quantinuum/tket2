//! In-place materialization for structured CFG rewrite templates.
//!
//! The template phase emits valid placeholder `DFG` nodes for original CFG
//! basic blocks. After inserting the template into the destination HUGR, this
//! module swaps the original block subtrees into those placeholder locations so
//! function calls, constants, and other static edges remain attached to the
//! original nodes.

use hugr::hugr::hugrmut::HugrMut;
use hugr::ops::{DFG, OpParent, OpType};
use hugr::{HugrView, IncomingPort, Node, OutgoingPort};
use itertools::Itertools;

use super::template::{BlockMaterialization, LoweredCfgTemplate};
use crate::control::structuralize::StructuralizationError;

/// Materializes one detached template into the destination CFG root.
///
/// # Errors
///
/// Returns an error when the template cannot be inserted, when a placeholder
/// cannot be matched back to its original block, or when the final CFG-to-DFG
/// swap cannot be completed consistently.
pub(super) fn materialize_cfg_rewrite<H: HugrMut<Node = Node>>(
    hugr: &mut H,
    cfg: Node,
    replacement: LoweredCfgTemplate,
) -> Result<(), StructuralizationError> {
    let parent = hugr
        .get_parent(cfg)
        .ok_or_else(|| StructuralizationError::Materialization {
            reason: format!("cfg {cfg} has no parent"),
        })?;
    let inserted = hugr.insert_hugr(parent, replacement.hugr);

    for placeholder in replacement.placeholders {
        let inserted_placeholder = inserted
            .node_map
            .get(&placeholder.placeholder)
            .copied()
            .ok_or_else(|| StructuralizationError::Materialization {
                reason: format!(
                    "inserted template did not contain placeholder {} for block {}",
                    placeholder.placeholder, placeholder.original
                ),
            })?;
        match placeholder.materialization {
            BlockMaterialization::Move => {
                materialize_block_placeholder(hugr, placeholder.original, inserted_placeholder)?
            }
            BlockMaterialization::Clone => {
                clone_block_placeholder(hugr, placeholder.original, inserted_placeholder)?
            }
        }
    }

    replace_cfg_root(hugr, cfg, inserted.inserted_entrypoint)
}

/// Swaps one placeholder `DFG` for the original CFG block subtree.
///
/// The original block is detached from the CFG, converted into a nested `DFG`,
/// rewired to the placeholder's external edges, and then left in the
/// structured container while the placeholder subtree is removed.
fn materialize_block_placeholder<H: HugrMut<Node = Node>>(
    hugr: &mut H,
    original: Node,
    placeholder: Node,
) -> Result<(), StructuralizationError> {
    let parent =
        hugr.get_parent(placeholder)
            .ok_or_else(|| StructuralizationError::Materialization {
                reason: format!("placeholder {placeholder} has no parent"),
            })?;
    let signature = hugr
        .get_optype(original)
        .inner_function_type()
        .ok_or_else(|| StructuralizationError::Materialization {
            reason: format!("block {original} is not a dataflow block"),
        })?
        .into_owned();
    let placeholder_input_count = hugr.get_optype(placeholder).input_count();
    let placeholder_output_count = hugr.get_optype(placeholder).output_count();

    disconnect_node_ports(hugr, original);
    hugr.set_parent(original, parent);
    hugr.replace_op(original, DFG { signature });
    let (input_count, output_count) = {
        let op = hugr.get_optype(original);
        (op.input_count(), op.output_count())
    };
    hugr.set_num_ports(original, input_count, output_count);
    if input_count != placeholder_input_count || output_count != placeholder_output_count {
        return Err(StructuralizationError::Materialization {
            reason: format!(
                "placeholder {placeholder} port counts ({placeholder_input_count}, {placeholder_output_count}) \
                 did not match block {original} port counts ({input_count}, {output_count})"
            ),
        });
    }

    rewire_placeholder_inputs(hugr, placeholder, original, input_count);
    rewire_placeholder_outputs(hugr, placeholder, original, output_count);
    hugr.remove_subtree(placeholder);
    Ok(())
}

/// Replaces one placeholder with a cloned copy of an already-analyzed block subtree.
///
/// Duplicated preprocessed CFG nodes map back to the same original HUGR block.
/// The first occurrence moves that block subtree into the structured rewrite;
/// later occurrences clone the moved subtree so each duplicated CFG node keeps
/// its own materialized block body.
fn clone_block_placeholder<H: HugrMut<Node = Node>>(
    hugr: &mut H,
    original: Node,
    placeholder: Node,
) -> Result<(), StructuralizationError> {
    let parent =
        hugr.get_parent(placeholder)
            .ok_or_else(|| StructuralizationError::Materialization {
                reason: format!("placeholder {placeholder} has no parent"),
            })?;
    let signature = hugr
        .get_optype(original)
        .inner_function_type()
        .ok_or_else(|| StructuralizationError::Materialization {
            reason: format!("block {original} is not a dataflow block"),
        })?
        .into_owned();
    let clone = hugr.add_node_with_parent(parent, DFG { signature });
    hugr.copy_descendants(original, clone, None);

    let placeholder_input_count = hugr.get_optype(placeholder).input_count();
    let placeholder_output_count = hugr.get_optype(placeholder).output_count();
    let clone_input_count = hugr.get_optype(clone).input_count();
    let clone_output_count = hugr.get_optype(clone).output_count();
    if clone_input_count != placeholder_input_count
        || clone_output_count != placeholder_output_count
    {
        return Err(StructuralizationError::Materialization {
            reason: format!(
                "placeholder {placeholder} port counts ({placeholder_input_count}, {placeholder_output_count}) \
                 did not match cloned block {clone} port counts ({clone_input_count}, {clone_output_count})"
            ),
        });
    }

    rewire_placeholder_inputs(hugr, placeholder, clone, clone_input_count);
    rewire_placeholder_outputs(hugr, placeholder, clone, clone_output_count);
    hugr.remove_subtree(placeholder);
    Ok(())
}

/// Disconnects every external port of a node from its current surroundings.
///
/// The node's internal subtree remains intact; only root-level CFG wiring is
/// cleared so the node can be reattached inside the structured replacement.
fn disconnect_node_ports<H: HugrMut<Node = Node>>(hugr: &mut H, node: Node) {
    let (input_count, output_count) = {
        let op = hugr.get_optype(node);
        (op.input_count(), op.output_count())
    };
    for port in 0..input_count {
        hugr.disconnect(node, IncomingPort::from(port));
    }
    for port in 0..output_count {
        hugr.disconnect(node, OutgoingPort::from(port));
    }
}

/// Reconnects all placeholder incoming edges onto the moved original block.
fn rewire_placeholder_inputs<H: HugrMut<Node = Node>>(
    hugr: &mut H,
    placeholder: Node,
    original: Node,
    input_count: usize,
) {
    let incoming_edges = (0..input_count)
        .flat_map(|port| {
            hugr.linked_outputs(placeholder, IncomingPort::from(port))
                .map(move |(src, src_port)| (src, src_port, IncomingPort::from(port)))
                .collect_vec()
        })
        .collect_vec();
    for (src, src_port, dst_port) in incoming_edges {
        hugr.disconnect_edge(src, src_port, placeholder, dst_port);
        hugr.connect(src, src_port, original, dst_port);
    }
}

/// Reconnects all placeholder outgoing edges onto the moved original block.
fn rewire_placeholder_outputs<H: HugrMut<Node = Node>>(
    hugr: &mut H,
    placeholder: Node,
    original: Node,
    output_count: usize,
) {
    let outgoing_edges = (0..output_count)
        .flat_map(|port| {
            hugr.linked_inputs(placeholder, OutgoingPort::from(port))
                .map(move |(dst, dst_port)| (OutgoingPort::from(port), dst, dst_port))
                .collect_vec()
        })
        .collect_vec();
    for (src_port, dst, dst_port) in outgoing_edges {
        hugr.disconnect_edge(placeholder, src_port, dst, dst_port);
        hugr.connect(original, src_port, dst, dst_port);
    }
}

/// Replaces the original CFG root with the inserted structured replacement.
///
/// By the time this runs, all dataflow blocks referenced by the template have
/// already been moved out of the CFG and into the structured replacement, so
/// removing the remaining CFG children only drops the old CFG boundary nodes
/// and any blocks that were never materialized.
fn replace_cfg_root<H: HugrMut<Node = Node>>(
    hugr: &mut H,
    cfg: Node,
    inserted_root: Node,
) -> Result<(), StructuralizationError> {
    let signature = hugr
        .get_optype(inserted_root)
        .inner_function_type()
        .ok_or_else(|| StructuralizationError::Materialization {
            reason: format!("inserted replacement root {inserted_root} is not a DFG"),
        })?
        .into_owned();

    let children = hugr.children(cfg).collect_vec();
    for child in &children {
        if matches!(
            hugr.get_optype(*child),
            OpType::DataflowBlock(_) | OpType::ExitBlock(_)
        ) {
            continue;
        }
        hugr.set_parent(*child, inserted_root);
    }
    for child in children {
        if matches!(
            hugr.get_optype(child),
            OpType::DataflowBlock(_) | OpType::ExitBlock(_)
        ) {
            hugr.remove_subtree(child);
        }
    }
    hugr.replace_op(cfg, DFG { signature });
    while let Some(child) = hugr.first_child(inserted_root) {
        hugr.set_parent(child, cfg);
    }
    hugr.remove_node(inserted_root);
    Ok(())
}

#[cfg(test)]
mod test {
    use hugr::builder::{CFGBuilder, Container, DataflowSubContainer, HugrBuilder, endo_sig};
    use hugr::extension::prelude::usize_t;
    use hugr::ops::Value;
    use hugr::ops::handle::ConstID;
    use hugr::types::{Signature, TypeRow};
    use hugr::{Hugr, HugrView};
    use rstest::{fixture, rstest};

    use super::super::template::{BlockMaterialization, prepare_cfg_replacement};
    use crate::control::structuralize::shared::analyze_block;
    use crate::control::structuralize::{
        RegionIo, StructuredNode, StructuredRegion, StructuredRegionBody,
    };
    use crate::control::{CfgNodeMap, IdentityCfgMap};

    /// Builds a tiny CFG with one identity block between entry and exit.
    #[fixture]
    fn single_block_cfg() -> Hugr {
        let mut cfg_builder = CFGBuilder::new(Signature::new_endo([usize_t()])).unwrap();
        let const_unit = cfg_builder.add_constant(Value::unary_unit_sum());

        let entry = n_identity(
            cfg_builder
                .simple_entry_builder(vec![usize_t()].into(), 1)
                .unwrap(),
            &const_unit,
        )
        .unwrap();
        let block = n_identity(
            cfg_builder
                .simple_block_builder(endo_sig([usize_t()]), 1)
                .unwrap(),
            &const_unit,
        )
        .unwrap();
        let exit = cfg_builder.exit_block();

        cfg_builder.branch(&entry, 0, &block).unwrap();
        cfg_builder.branch(&block, 0, &exit).unwrap();
        cfg_builder.finish_hugr().unwrap()
    }

    /// Duplicated placeholders move the first block occurrence and clone the rest.
    #[rstest]
    fn materializes_duplicated_blocks(single_block_cfg: Hugr) {
        let cfg = single_block_cfg
            .nodes()
            .find(|node| single_block_cfg.get_optype(*node).is_cfg())
            .unwrap();
        let cfg_view = single_block_cfg.with_entrypoint(cfg);
        let cfg_map = IdentityCfgMap::new(cfg_view.clone());
        let entry = cfg_map.entry_node();
        let block = cfg_map.successors(entry).next().unwrap();
        let exit = cfg_map.exit_node();

        let block_summary = analyze_block(&cfg_view, block).unwrap();
        let exit_summary = analyze_block(&cfg_view, exit).unwrap();
        let region = StructuredRegion {
            io: RegionIo {
                inputs: TypeRow::from([usize_t()]),
                outputs: TypeRow::from([usize_t()]),
            },
            body: StructuredRegionBody::Sequence(vec![
                StructuredNode::Block(block_summary.clone()),
                StructuredNode::Block(block_summary),
                StructuredNode::Block(exit_summary),
            ]),
        };

        let replacement = prepare_cfg_replacement(&cfg_view, &region).unwrap();
        assert_eq!(replacement.placeholders.len(), 2);
        assert_eq!(
            replacement.placeholders[0].materialization,
            BlockMaterialization::Move
        );
        assert_eq!(
            replacement.placeholders[1].materialization,
            BlockMaterialization::Clone
        );

        let mut hugr = single_block_cfg;
        super::materialize_cfg_rewrite(&mut hugr, cfg, replacement).unwrap();
        hugr.validate().unwrap();
        assert_eq!(
            hugr.nodes()
                .filter(|n| hugr.get_optype(*n).is_cfg())
                .count(),
            0
        );
    }

    /// Finishes one identity-like CFG block builder.
    fn n_identity<T: DataflowSubContainer>(
        mut dataflow_builder: T,
        pred_const: &ConstID,
    ) -> Result<T::ContainerHandle, hugr::builder::BuildError> {
        let wires = dataflow_builder.input_wires();
        let unit = dataflow_builder.load_const(pred_const);
        dataflow_builder.finish_with_outputs([unit].into_iter().chain(wires))
    }
}
