//! CFG normalizations.
//!
//! * Merge BBs along control-flow edges where the source BB has no other successors
//!   and the target BB has no other predecessors.
//! * Move entry/last-before-exit blocks outside of CFG when possible.
//! * Convert whole CFG to DFG if straight-line control-flow
use std::collections::HashMap;

use hugr_core::extension::prelude::UnpackTuple;
use hugr_core::hugr::hugrmut::HugrMut;
use hugr_core::types::{EdgeKind, Signature, TypeRow};
use itertools::{Either, Itertools};

use hugr_core::hugr::patch::inline_dfg::InlineDFG;
use hugr_core::hugr::patch::replace::{NewEdgeKind, NewEdgeSpec, Replacement};
use hugr_core::ops::{
    CFG, DFG, DataflowBlock, DataflowParent, ExitBlock, Input, OpTag, OpTrait, OpType, Output,
};
use hugr_core::{Direction, Hugr, HugrView, Node, OutgoingPort, PortIndex};

use crate::passes::composable::WithScope;
use crate::passes::{ComposablePass, PassScope};

/// Merge any basic blocks that are direct children of the specified [`CFG`]-entrypoint
/// Hugr.
///
/// That is, where a basic block B has a single successor B' whose only predecessor
/// is B, B and B' can be combined.
///
/// Returns the number of merged blocks.
///
/// # Errors
///
/// [NormalizeCFGError::NotCFG] If the entrypoint is not a CFG
pub fn merge_basic_blocks<H: HugrMut>(cfg: &mut H) -> Result<usize, NormalizeCFGError> {
    if !cfg.entrypoint_optype().is_cfg() {
        return Err(NormalizeCFGError::NotCFG(cfg.entrypoint_optype().tag()));
    }
    let mut num_merged = 0;
    let mut worklist = cfg.children(cfg.entrypoint()).collect::<Vec<_>>();
    while let Some(n) = worklist.pop() {
        // Consider merging n with its successor
        let Ok(succ) = cfg.output_neighbours(n).exactly_one() else {
            continue;
        };
        if cfg.input_neighbours(succ).count() != 1 {
            continue;
        }
        if cfg.children(cfg.entrypoint()).take(2).contains(&succ) {
            // If succ is...
            //   - the entry block, that has an implicit extra in-edge, so cannot merge with n.
            //   - the exit block, nodes in n should move *outside* the CFG - a separate pass.
            continue;
        }
        num_merged += 1;
        let (rep, merge_bb, dfgs) = mk_rep(cfg, n, succ);
        let node_map = cfg.apply_patch(rep).unwrap();
        let merged_bb = *node_map.get(&merge_bb).unwrap();
        for dfg_id in dfgs {
            let n_id = *node_map.get(&dfg_id).unwrap();
            cfg.apply_patch(InlineDFG(n_id.into())).unwrap();
        }
        worklist.push(merged_bb);
    }
    Ok(num_merged)
}

/// Errors from [normalize_cfg]
#[derive(Clone, Debug, thiserror::Error, PartialEq, Eq)]
pub enum NormalizeCFGError {
    /// The requested node was not a CFG
    #[error("Requested node was not a CFG but {_0}")]
    NotCFG(OpTag),
}

/// Result from [normalize_cfg], i.e. a report of what changes were made to the Hugr.
///
/// `N` is the type of the nodes in the Hugr.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum NormalizeCFGResult<N = Node> {
    /// The entire [`CFG`] was converted into a [`DFG`].
    ///
    /// The entrypoint node id is preserved after the conversion, but now identifies
    /// the new [`DFG`].
    CFGToDFG,
    /// The CFG was preserved, but the entry or exit blocks may have changed.
    CFGPreserved {
        /// Nodes that were in the entry block but have been moved to be siblings of the CFG.
        /// (Either empty, or all the nodes in the entry block except [Input]/[Output].)
        entry_nodes_moved: Vec<N>,
        /// If `Some`, the new [DFG] of what was previously in the last block before the exit
        exit_dfg: Option<N>,
        /// The number of basic blocks merged together.
        /// Does not include any lifted to become DFGs outside.
        num_merged: usize,
    },
}

/// A [ComposablePass] that normalizes CFGs (i.e. [normalize_cfg]) in a Hugr.
#[derive(Clone, Debug, Default)]
pub struct NormalizeCFGPass {
    scope: PassScope,
}

impl<H: HugrMut> ComposablePass<H> for NormalizeCFGPass {
    type Error = NormalizeCFGError;

    /// For each CFG node that was normalized, the [NormalizeCFGResult] for that CFG
    type Result = HashMap<H::Node, NormalizeCFGResult<H::Node>>;

    fn run(&self, hugr: &mut H) -> Result<Self::Result, Self::Error> {
        let ctrs = {
            let r = self.scope.root(hugr);
            if let Some(r) = r.filter(|_| self.scope.recursive()) {
                Either::Right(hugr.descendants(r))
            } else {
                Either::Left(r.into_iter())
            }
        };

        let cfgs = {
            let mut cfgs: Vec<H::Node> = ctrs.filter(|n| hugr.get_optype(*n).is_cfg()).collect();
            // Process inner CFGs first, in case they are removed (if they are in a completely
            // disconnected block when the Entry node has only the Exit as successor).
            cfgs.reverse();
            cfgs
        };
        let mut results = HashMap::new();
        for cfg in cfgs {
            let res = normalize_cfg(&mut hugr.with_entrypoint_mut(cfg))?;
            results.insert(cfg, res);
        }
        Ok(results)
    }
}

impl WithScope for NormalizeCFGPass {
    fn with_scope(mut self, scope: impl Into<PassScope>) -> Self {
        self.scope = scope.into();
        self
    }
}

/// Normalize a CFG in a Hugr:
/// * Merge consecutive basic blocks i.e. where a BB has only a single successor which
///   has no predecessors
/// * If the entry block has only one successor, and no predecessors, then move its contents
///   into a DFG outside/before CFG.
///    * If that successor is the exit block, then convert the entire CFG to a DFG.
///      This will be reported via [NormalizeCFGResult::CFGToDFG]
/// * (Similarly) if the exit block has only one predecessor, then move contents into a DFG
///   outside/after CFG.
///
/// # Errors
///
/// [NormalizeCFGError::NotCFG] If the entrypoint is not a CFG
pub fn normalize_cfg<H: HugrMut>(
    h: &mut H,
) -> Result<NormalizeCFGResult<H::Node>, NormalizeCFGError> {
    let num_merged = merge_basic_blocks(h)?;
    let cfg_node = h.entrypoint();
    fn cfg_ty_mut<H: HugrMut>(h: &mut H, n: H::Node) -> &mut CFG {
        match h.optype_mut(n) {
            OpType::CFG(cfg) => cfg,
            _ => unreachable!(), // Checked at entry to normalize_cfg
        }
    }
    let ancestor_block = |h: &H, mut n: H::Node| {
        while let Some(p) = h.get_parent(n) {
            if p == cfg_node {
                return Some(n);
            }
            n = p;
        }
        None
    };

    // Further normalizations with effects outside the CFG
    let [entry, exit] = h.children(cfg_node).take(2).collect_array().unwrap();
    let entry_blk = h.get_optype(entry).as_dataflow_block().unwrap();
    let cfg_parent = h.get_parent(cfg_node).unwrap();
    // 1. If the entry block has no predecessors, then we can move contents outside/before the CFG.
    // However, we only do this if the Entry block has just one successor (i.e. we can remove
    // the entry block altogether) - an extension would be to do this in other cases, preserving
    // the Entry block as an empty branch.
    let mut entry_nodes_moved = Vec::new();
    if let Some(succ) = h
        .output_neighbours(entry)
        .exactly_one()
        .ok()
        .filter(|_| h.input_neighbours(entry).next().is_none())
    {
        if succ == exit {
            // 1a. Turn the CFG into a DFG containing only what was in the entry block
            assert_eq!(
                &Signature::new(
                    entry_blk.inputs.clone(),
                    entry_blk.successor_input(0).unwrap()
                ),
                h.signature(cfg_node).unwrap().as_ref()
            );
            // Annoying here - "while let Some(blk) = cfg.children(...).skip(1).next()" keeps iterator alive
            let children_to_remove: Vec<_> = h.children(cfg_node).skip(1).collect();
            for blk in children_to_remove {
                h.remove_subtree(blk);
            }
            while let Some(ch) = h.first_child(entry) {
                h.set_parent(ch, cfg_node);
            }
            h.remove_node(entry);
            let signature = std::mem::take(&mut cfg_ty_mut(h, cfg_node).signature);
            let result_tys = signature.output.clone();
            h.replace_op(cfg_node, OpType::DFG(DFG { signature }));
            unpack_before_output(h, h.get_io(cfg_node).unwrap()[1], result_tys);
            return Ok(NormalizeCFGResult::CFGToDFG);
        }
        // 1b. Move entry block outside/before the CFG; its successor becomes the entry block.
        let new_cfg_inputs = entry_blk.successor_input(0).unwrap();
        // Look for nonlocal `Dom` edges from the entry block. (Ignore `Ext` edges.)
        let nonlocal_srcs = h
            .children(entry)
            .filter(|n| {
                h.output_neighbours(*n).any(|succ| {
                    ancestor_block(h, succ).expect("Dom edges within entry, Ext within CFG")
                        != entry
                })
            })
            .collect::<Vec<_>>();
        // Move entry block contents into DFG.
        let dfg = h.add_node_with_parent(
            cfg_parent,
            DFG {
                signature: Signature::new(entry_blk.inputs.clone(), new_cfg_inputs.clone()),
            },
        );
        let [_, entry_output] = h.get_io(entry).unwrap();
        while let Some(n) = h.first_child(entry) {
            h.set_parent(n, dfg);
        }
        h.move_before_sibling(succ, entry);
        h.remove_node(entry);

        unpack_before_output(h, entry_output, new_cfg_inputs.clone());

        // Inputs to CFG go directly to DFG
        for inp in h.node_inputs(cfg_node).collect::<Vec<_>>() {
            for src in h.linked_outputs(cfg_node, inp).collect::<Vec<_>>() {
                h.connect(src.0, src.1, dfg, inp.index());
            }
            h.disconnect(cfg_node, inp);
        }

        // Update input ports
        let cfg_ty = cfg_ty_mut(h, cfg_node);
        let inputs_to_add = new_cfg_inputs.len() as isize - cfg_ty.signature.input.len() as isize;
        cfg_ty.signature.input = new_cfg_inputs;
        h.add_ports(cfg_node, Direction::Incoming, inputs_to_add);

        // Wire outputs of DFG directly to CFG
        for src in h.node_outputs(dfg).collect::<Vec<_>>() {
            h.connect(dfg, src, cfg_node, src.index());
        }
        // Inline DFG to ensure that any nonlocal (`Dom`) edges from it, become valid `Ext` edges
        for n in nonlocal_srcs {
            // With required Order edge. (Do this before inlining, in case n is Input.)
            h.add_other_edge(n, cfg_node);
        }
        entry_nodes_moved.extend(h.children(dfg).skip(2)); // Skip Input/Output nodes
        h.apply_patch(InlineDFG(dfg.into())).unwrap();
    }
    // 2. If the exit node has a single predecessor and that predecessor has no other successors...
    let mut exit_dfg = None;
    if let Some(pred) = h.input_neighbours(exit).exactly_one().ok().filter(|pred| {
        // Allow only if there are no `Dom` edges into `pred`. (Ignore `Ext` edges.)
        let src_nodes = h.children(*pred).flat_map(|ch| h.input_neighbours(ch));
        h.output_neighbours(*pred).count() == 1
            && src_nodes.dedup().all(|src| {
                ancestor_block(h, src).is_none_or(|src| src == *pred) // Nones are `Ext` edges.
            })
    }) {
        // Code in that predecessor can be moved outside (into a new DFG after the CFG),
        // and the predecessor deleted
        let [_, output] = h.get_io(pred).unwrap();
        let pred_blk = h.get_optype(pred).as_dataflow_block().unwrap();
        let new_cfg_outs = pred_blk.inner_signature().into_owned().input;

        // new CFG result type and exit block
        let cfg_ty = cfg_ty_mut(h, cfg_node);
        let result_tys = std::mem::replace(&mut cfg_ty.signature.output, new_cfg_outs.clone());
        h.add_ports(
            cfg_node,
            Direction::Outgoing,
            new_cfg_outs.len() as isize - result_tys.len() as isize,
        );

        *h.optype_mut(pred) = ExitBlock {
            cfg_outputs: new_cfg_outs.clone(),
        }
        .into();
        debug_assert_eq!(h.num_ports(pred, Direction::Outgoing), 1);
        h.set_num_ports(pred, 1, 0);

        h.move_before_sibling(pred, exit);
        h.remove_node(exit);
        // Move contents into new DFG
        let dfg = h.add_node_with_parent(
            cfg_parent,
            DFG {
                signature: Signature::new(new_cfg_outs, result_tys.clone()),
            },
        );
        while let Some(n) = h.first_child(pred) {
            h.set_parent(n, dfg);
        }
        unpack_before_output(h, output, result_tys);

        // Move output edges.
        for p in h.node_outputs(cfg_node).collect_vec() {
            let tgts = h.linked_inputs(cfg_node, p).collect_vec();
            h.disconnect(cfg_node, p);
            for tgt in tgts {
                h.connect(dfg, p, tgt.0, tgt.1)
            }
        }
        for p in h.node_inputs(dfg).collect_vec() {
            h.connect(cfg_node, p.index(), dfg, p);
        }
        exit_dfg = Some(dfg);
    }
    Ok(NormalizeCFGResult::CFGPreserved {
        entry_nodes_moved,
        exit_dfg,
        num_merged,
    })
}

fn mk_rep<H: HugrView>(
    cfg: &H,
    pred: H::Node,
    succ: H::Node,
) -> (Replacement<H::Node>, Node, [Node; 2]) {
    let pred_ty = cfg.get_optype(pred).as_dataflow_block().unwrap();
    let succ_ty = cfg.get_optype(succ).as_dataflow_block().unwrap();
    let succ_sig = succ_ty.inner_signature();

    // Make a Hugr with just a single CFG root node having the same signature.
    let mut replacement: Hugr = Hugr::new_with_entrypoint(cfg.entrypoint_optype().clone())
        .expect("Replacement should have a CFG entrypoint");

    let merged = replacement.add_node_with_parent(replacement.entrypoint(), {
        DataflowBlock {
            inputs: pred_ty.inputs.clone(),
            ..succ_ty.clone()
        }
    });
    let input = replacement.add_node_with_parent(
        merged,
        Input {
            types: pred_ty.inputs.clone(),
        },
    );
    let output = replacement.add_node_with_parent(
        merged,
        Output {
            types: succ_sig.output.clone(),
        },
    );

    let dfg1 = replacement.add_node_with_parent(
        merged,
        DFG {
            signature: pred_ty.inner_signature().into_owned(),
        },
    );
    for inport in replacement.node_inputs(dfg1).collect::<Vec<_>>() {
        replacement.connect(input, inport.index(), dfg1, inport);
    }

    let dfg2 = replacement.add_node_with_parent(
        merged,
        DFG {
            signature: succ_sig.as_ref().clone(),
        },
    );
    for outport in replacement.node_outputs(dfg2).collect::<Vec<_>>() {
        replacement.connect(dfg2, outport, output, outport.index());
    }

    // At the junction, must unpack the first (tuple, branch predicate) output
    let dfg1_outs = replacement
        .out_value_types(dfg1)
        .enumerate()
        .map(|(i, _)| (dfg1, i.into()))
        .collect::<Vec<_>>();

    let dfg_order_out = replacement.get_optype(dfg1).other_output_port().unwrap();
    let order_srcs = Some((dfg1, dfg_order_out));
    wire_unpack_first(&mut replacement, dfg1_outs, order_srcs, dfg2);

    // If there are edges from succ back to pred, we cannot do these via the mu_inp/out/new
    // edge-maps as both source and target of the new edge are in the replacement Hugr
    for (_, src_pos) in cfg.all_linked_outputs(pred).filter(|(src, _)| *src == succ) {
        replacement.connect(merged, src_pos, merged, 0);
    }
    let rep = Replacement {
        removal: vec![pred, succ],
        replacement,
        adoptions: HashMap::from([(dfg1, pred), (dfg2, succ)]),
        mu_inp: cfg
            .all_linked_outputs(pred)
            .filter(|(src, _)| *src != succ)
            .map(|(src, src_pos)| NewEdgeSpec {
                src,
                tgt: merged,
                kind: NewEdgeKind::ControlFlow { src_pos },
            })
            .collect(),
        mu_out: cfg
            .node_outputs(succ)
            .filter_map(|src_pos| {
                let tgt = cfg
                    .linked_inputs(succ, src_pos)
                    .exactly_one()
                    .ok()
                    .unwrap()
                    .0;
                if tgt == pred {
                    None
                } else {
                    Some(NewEdgeSpec {
                        src: merged,
                        tgt,
                        kind: NewEdgeKind::ControlFlow { src_pos },
                    })
                }
            })
            .collect(),
        mu_new: vec![],
    };
    (rep, merged, [dfg1, dfg2])
}

type NodePorts<N> = Vec<(N, OutgoingPort)>;
/// Remove all input wires to `n` and return them in two groups:
/// the [EdgeKind::Value] inputs, and the [EdgeKind::StateOrder] inputs
fn take_inputs<H: HugrMut>(h: &mut H, n: H::Node) -> (NodePorts<H::Node>, NodePorts<H::Node>) {
    let mut values = vec![];
    let mut orders = vec![];
    for p in h.node_inputs(n).collect_vec() {
        let srcs = h.linked_outputs(n, p).collect_vec();
        h.disconnect(n, p);
        match h.get_optype(n).port_kind(p) {
            Some(EdgeKind::Value(_)) => {
                assert_eq!(srcs.len(), 1);
                values.extend(srcs);
            }
            Some(EdgeKind::StateOrder) => {
                assert_eq!(orders, []);
                orders.extend(srcs);
            }
            k => panic!("Unexpected port kind: {:?}", k),
        }
    }
    (values, orders)
}

fn tuple_elems<H: HugrView>(h: &H, n: H::Node, p: OutgoingPort) -> TypeRow {
    match h.get_optype(n).port_kind(p) {
        Some(EdgeKind::Value(ty)) => ty.as_sum().unwrap().as_tuple().unwrap().clone(),
        p => panic!("Expected Value port not {:?}", p),
    }
    .try_into()
    .unwrap()
}

/// Unpack the first `value_srcs`; wire the unpacked elements and remaining `value_srcs` into
/// consecutive ports of `dst`. Finally wire `order_srcs` all to the order input of `dst`.
fn wire_unpack_first<H: HugrMut>(
    h: &mut H,
    value_srcs: impl IntoIterator<Item = (H::Node, OutgoingPort)>,
    order_srcs: impl IntoIterator<Item = (H::Node, OutgoingPort)>,
    dst: H::Node,
) {
    let parent = h.get_parent(dst).unwrap();
    let mut srcs = value_srcs.into_iter();
    let src_to_unpack = srcs.next().unwrap();
    let tuple_tys = tuple_elems(h, src_to_unpack.0, src_to_unpack.1);
    let tuple_len = tuple_tys.len();
    let unp = h.add_node_with_parent(parent, UnpackTuple::new(tuple_tys));
    h.connect(src_to_unpack.0, src_to_unpack.1, unp, 0);

    for i in 0..tuple_len {
        h.connect(unp, i, dst, i);
    }
    assert_eq!(
        h.get_optype(dst).other_port_kind(Direction::Incoming),
        Some(EdgeKind::StateOrder)
    );
    let order_tgt = h.get_optype(dst).other_input_port().unwrap();
    for (i, (src, src_p)) in srcs.enumerate() {
        assert!(i + tuple_len < order_tgt.index());
        h.connect(src, src_p, dst, i + tuple_len);
    }
    for (src, src_p) in order_srcs {
        h.connect(src, src_p, dst, order_tgt);
    }
}

/// Unpack the first input to specified [Output] node and shuffle all the rest along
fn unpack_before_output<H: HugrMut>(h: &mut H, output_node: H::Node, new_types: TypeRow) {
    let (values, orders) = take_inputs(h, output_node);
    let OpType::Output(ou) = h.optype_mut(output_node) else {
        panic!()
    };
    let ports_to_add = new_types.len() as isize - ou.types.len() as isize;
    ou.types = new_types;
    h.add_ports(output_node, Direction::Incoming, ports_to_add);
    wire_unpack_first(h, values, orders, output_node);
}

#[cfg(test)]
mod test {
    use hugr::builder::Container;
    use hugr::hugr::hugrmut::HugrMut;
    use itertools::Itertools;
    use rstest::rstest;
    use std::collections::HashSet;
    use std::sync::Arc;

    use hugr::extension::simple_op::MakeExtensionOp;
    use hugr::extension::{ExtensionId, OpDef, Version};
    use hugr_core::builder::{
        CFGBuilder, Dataflow, HugrBuilder, SubContainer, endo_sig, inout_sig,
    };
    use hugr_core::extension::prelude::{
        ConstUsize, Noop, PRELUDE_ID, UnpackTuple, bool_t, qb_t, usize_t,
    };
    use hugr_core::ops::{ExtensionOp, OpName, constant::Value, handle::NodeHandle};
    use hugr_core::ops::{LoadConstant, OpTag, OpTrait, OpType, Tag};

    use hugr_core::types::{PolyFuncTypeRV, Signature, Type, TypeBound, TypeRow};
    use hugr_core::{Extension, HugrView, const_extension_ids, type_row};

    use super::{NormalizeCFGPass, NormalizeCFGResult, merge_basic_blocks, normalize_cfg};
    use crate::passes::ComposablePass;

    const_extension_ids! {
        const EXT_ID: ExtensionId = "TestExt";
    }

    fn extension() -> Arc<Extension> {
        Extension::new_arc(
            EXT_ID,
            hugr_core::extension::Version::new(0, 1, 0),
            |ext, extension_ref| {
                ext.add_op(
                    "Test".into(),
                    String::new(),
                    Signature::new(
                        vec![qb_t(), usize_t()],
                        TypeRow::from(vec![Type::new_sum(vec![vec![qb_t()], vec![usize_t()]])]),
                    ),
                    extension_ref,
                )
                .unwrap();
            },
        )
    }

    #[rstest]
    #[case(true)]
    #[case(false)]
    fn merge_bbs_in_loop(#[case] self_loop: bool) -> Result<(), Box<dyn std::error::Error>> {
        /* self_loop==False:
           -> Noop1 -----> Test -> Exit       -> Noop1AndTest --> Exit
               |            |            =>     /            \
               \-<- Noop2 <-/                   \-<- Noop2 <-/
           (Noop2 -> Noop1 cannot be merged because Noop1 is the entry node)

           self_loop==True:
           -> Noop --> Test -> Exit           -> NoopAndTest --> Exit
               |        |                =>     /           \
               \--<--<--/                       \--<-----<--/
        */

        let loop_variants: TypeRow = vec![qb_t()].into();
        let exit_types: TypeRow = vec![usize_t()].into();
        let e = extension();
        let tst_op = e.instantiate_extension_op("Test", [])?;
        let mut h = CFGBuilder::new(inout_sig(loop_variants.clone(), exit_types.clone()))?;
        let mut no_b1 = h.simple_entry_builder(loop_variants.clone(), 1)?;
        let n = no_b1.add_dataflow_op(Noop::new(qb_t()), no_b1.input_wires())?;
        let br = no_b1.add_load_value(Value::unary_unit_sum());
        let no_b1 = no_b1.finish_with_outputs(br, n.outputs())?;
        let mut test_block = h.block_builder(
            loop_variants.clone(),
            vec![loop_variants.clone(), exit_types],
            type_row![],
        )?;
        let [test_input] = test_block.input_wires_arr();
        let usize_cst = test_block.add_load_value(ConstUsize::new(1));
        let [tst] = test_block
            .add_dataflow_op(tst_op, [test_input, usize_cst])?
            .outputs_arr();
        let test_block = test_block.finish_with_outputs(tst, [])?;
        let loop_backedge_target = if self_loop {
            no_b1
        } else {
            let mut no_b2 = h.simple_block_builder(endo_sig(loop_variants), 1)?;
            let n = no_b2.add_dataflow_op(Noop::new(qb_t()), no_b2.input_wires())?;
            let br = no_b2.add_load_value(Value::unary_unit_sum());
            let nid = no_b2.finish_with_outputs(br, n.outputs())?;
            h.branch(&nid, 0, &no_b1)?;
            nid
        };
        h.branch(&no_b1, 0, &test_block)?;
        h.branch(&test_block, 0, &loop_backedge_target)?;
        h.branch(&test_block, 1, &h.exit_block())?;

        let mut h = h.finish_hugr()?;
        let r = h.entrypoint();
        let num_merged = merge_basic_blocks(&mut h)?;
        assert_eq!(num_merged, 1);
        h.validate().unwrap();
        assert_eq!(r, h.entrypoint());
        assert!(matches!(h.get_optype(r), OpType::CFG(_)));
        let [entry, exit] = h
            .children(r)
            .take(2)
            .collect::<Vec<_>>()
            .try_into()
            .unwrap();
        // Check the Noop('s) is/are in the right block(s)
        let nops = h
            .entry_descendants()
            .filter(|n| h.get_optype(*n).cast::<Noop>().is_some());
        let (entry_nop, expected_backedge_target) = if self_loop {
            assert_eq!(h.children(r).count(), 2);
            (nops.exactly_one().ok().unwrap(), entry)
        } else {
            let [_, _, no_b2] = h.children(r).collect::<Vec<_>>().try_into().unwrap();
            let mut nops = nops.collect::<Vec<_>>();
            let entry_nop_idx = nops
                .iter()
                .position(|n| h.get_parent(*n) == Some(entry))
                .unwrap();
            let entry_nop = nops[entry_nop_idx];
            nops.remove(entry_nop_idx);
            let [n_op2] = nops.try_into().unwrap();
            assert_eq!(h.get_parent(n_op2), Some(no_b2));
            (entry_nop, no_b2)
        };
        assert_eq!(h.get_parent(entry_nop), Some(entry));
        assert_eq!(
            h.output_neighbours(entry).collect::<HashSet<_>>(),
            HashSet::from([expected_backedge_target, exit])
        );
        // And the Noop in the entry block is consumed by the custom Test op
        let tst = find_unique(
            h.entry_descendants(),
            |n| matches!(h.get_optype(*n), OpType::ExtensionOp(c) if c.def().extension_id() != &PRELUDE_ID),
        );
        assert_eq!(h.get_parent(tst), Some(entry));
        assert_eq!(
            h.output_neighbours(entry_nop).collect::<Vec<_>>(),
            vec![tst]
        );
        Ok(())
    }

    #[test]
    fn elide_triple_with_permute() -> Result<(), Box<dyn std::error::Error>> {
        // Blocks are just BB1 -> BB2 -> BB3 --> Exit.
        // Should be merged into one BB (we don't check that specifically)
        // and then the whole CFG elided.
        let e = extension();
        let tst_op: OpType = e.instantiate_extension_op("Test", &[])?.into();
        let [res_t] = tst_op
            .dataflow_signature()
            .unwrap()
            .into_owned()
            .output
            .into_owned()
            .try_into()
            .unwrap();
        let mut h = CFGBuilder::new(inout_sig([qb_t()], [res_t.clone()]))?;
        let mut bb1 = h.simple_entry_builder(vec![usize_t(), qb_t()].into(), 1)?;
        let [inw] = bb1.input_wires_arr();
        let load_cst = bb1.add_load_value(ConstUsize::new(1));
        let pred = bb1.add_load_value(Value::unary_unit_sum());
        let bb1 = bb1.finish_with_outputs(pred, [load_cst, inw])?;

        let mut bb2 = h.block_builder(
            vec![usize_t(), qb_t()].into(),
            vec![type_row![]],
            vec![qb_t(), usize_t()].into(),
        )?;
        let [u, q] = bb2.input_wires_arr();
        let pred = bb2.add_load_value(Value::unary_unit_sum());
        let bb2 = bb2.finish_with_outputs(pred, [q, u])?;

        let mut bb3 = h.block_builder(
            vec![qb_t(), usize_t()].into(),
            vec![type_row![]],
            [res_t.clone()].into(),
        )?;
        let [q, u] = bb3.input_wires_arr();
        let tst = bb3.add_dataflow_op(tst_op, [q, u])?;
        let pred = bb3.add_load_value(Value::unary_unit_sum());
        let bb3 = bb3.finish_with_outputs(pred, tst.outputs())?;
        // Now add control-flow edges between basic blocks
        h.branch(&bb1, 0, &bb2)?;
        h.branch(&bb2, 0, &bb3)?;
        h.branch(&bb3, 0, &h.exit_block())?;

        let mut h = h.finish_hugr()?;
        let res = normalize_cfg(&mut h);
        assert_eq!(res, Ok(NormalizeCFGResult::CFGToDFG));
        h.validate()?;
        assert_eq!(h.entrypoint_optype().tag(), OpTag::Dfg);
        assert_eq!(
            h.entry_descendants().find(|n| matches!(
                h.get_optype(*n),
                OpType::DataflowBlock(_) | OpType::CFG(_) | OpType::ExitBlock(_)
            )),
            None
        );
        let tst = find_unique(
            h.entry_descendants(),
            |n| matches!(h.get_optype(*n), OpType::ExtensionOp(c) if c.def().extension_id() != &PRELUDE_ID),
        );
        assert_eq!(h.get_parent(tst), Some(h.entrypoint()));

        let inp = find_unique(h.entry_descendants(), |n| {
            matches!(h.get_optype(*n), OpType::Input(_))
        });
        let mut tst_inputs = h.input_neighbours(tst).collect::<Vec<_>>();
        tst_inputs.remove(tst_inputs.iter().find_position(|n| **n == inp).unwrap().0);
        let [other_input] = tst_inputs.try_into().unwrap();
        assert_eq!(
            h.get_optype(other_input),
            &(LoadConstant {
                datatype: usize_t()
            }
            .into())
        );
        Ok(())
    }

    fn find_unique<T>(items: impl Iterator<Item = T>, pred: impl Fn(&T) -> bool) -> T {
        items.filter(pred).exactly_one().ok().unwrap()
    }

    #[test]
    fn entry_before_loop() -> Result<(), Box<dyn std::error::Error>> {
        /* -> Noop --> Test -> Exit      -> Test --> Exit
                       |  |          =>     |  |
                       \<-/                 \<-/
        */
        let loop_variants: TypeRow = vec![qb_t()].into();
        let exit_types: TypeRow = vec![usize_t()].into();
        let e = extension();
        let tst_op = e.instantiate_extension_op("Test", [])?;
        let mut h = CFGBuilder::new(inout_sig([qb_t()], [usize_t()]))?;
        let mut nop_b = h.simple_entry_builder(loop_variants.clone(), 1)?;
        let n = nop_b.add_dataflow_op(Noop::new(qb_t()), nop_b.input_wires())?;
        let br = nop_b.add_load_value(Value::unary_unit_sum());
        let entry = nop_b.finish_with_outputs(br, n.outputs())?;

        let mut loop_b = h.block_builder(
            loop_variants.clone(),
            [loop_variants, exit_types],
            type_row![],
        )?;
        let [qb] = loop_b.input_wires_arr();
        let usz = loop_b.add_load_value(ConstUsize::new(3));
        let [tst] = loop_b.add_dataflow_op(tst_op, [qb, usz])?.outputs_arr();
        let loop_ = loop_b.finish_with_outputs(tst, [])?;
        h.branch(&entry, 0, &loop_)?;
        h.branch(&loop_, 0, &loop_)?;
        h.branch(&loop_, 1, &h.exit_block())?;

        let mut h = h.finish_hugr()?;

        let res = normalize_cfg(&mut h).unwrap();
        h.validate().unwrap();
        let NormalizeCFGResult::CFGPreserved {
            entry_nodes_moved,
            exit_dfg: None,
            num_merged: 0,
        } = res
        else {
            panic!("Unexpected result");
        };
        assert_eq!(entry_nodes_moved.len(), 4); // Noop, Const, LoadConstant, UnpackTuple
        assert_eq!(
            h.children(h.entrypoint())
                .map(|n| h.get_optype(n).tag())
                .collect_vec(),
            [OpTag::DataflowBlock, OpTag::BasicBlockExit]
        );
        let func = h.get_parent(h.entrypoint()).unwrap();
        let func_children = child_tags_ext_ids(&h, func);
        assert_eq!(
            func_children.into_iter().sorted().collect_vec(),
            [
                "Cfg",
                "Const",
                "Input",
                "LoadConst",
                "Noop",
                "Output",
                "UnpackTuple"
            ]
        );
        Ok(())
    }

    #[test]
    fn loop_before_exit() -> Result<(), Box<dyn std::error::Error>> {
        /* -> Test -> Noop -> Exit      -> Test --> Exit (then Noop)
              |  |                  =>    |  |
              \<-/                        \<-/
        */
        let loop_variants: TypeRow = vec![qb_t()].into();
        let exit_types: TypeRow = vec![usize_t()].into();
        let e = extension();
        let tst_op = e.instantiate_extension_op("Test", [])?;
        let mut h = CFGBuilder::new(inout_sig(loop_variants.clone(), exit_types.clone()))?;

        let mut loop_b = h.entry_builder(vec![loop_variants, exit_types.clone()], type_row![])?;
        let [qb] = loop_b.input_wires_arr();
        let usz = loop_b.add_load_value(ConstUsize::new(3));
        let [tst] = loop_b.add_dataflow_op(tst_op, [qb, usz])?.outputs_arr();
        let loop_ = loop_b.finish_with_outputs(tst, [])?;
        h.branch(&loop_, 0, &loop_)?;

        let mut nop_b = h.simple_block_builder(endo_sig(exit_types), 1)?;
        let n = nop_b.add_dataflow_op(Noop::new(usize_t()), nop_b.input_wires())?;
        let br = nop_b.add_load_value(Value::unary_unit_sum());
        let tail = nop_b.finish_with_outputs(br, n.outputs())?;

        h.branch(&loop_, 1, &tail)?;
        h.branch(&tail, 0, &h.exit_block())?;

        let mut h = h.finish_hugr()?;
        let res = normalize_cfg(&mut h).unwrap();
        h.validate().unwrap();
        let NormalizeCFGResult::CFGPreserved {
            entry_nodes_moved,
            exit_dfg: Some(dfg),
            num_merged: 0,
        } = res
        else {
            panic!("Unexpected result");
        };
        assert!(entry_nodes_moved.is_empty());
        assert_eq!(
            h.children(h.entrypoint())
                .map(|n| h.get_optype(n).tag())
                .collect_vec(),
            [OpTag::DataflowBlock, OpTag::BasicBlockExit]
        );
        let func = h.get_parent(h.entrypoint()).unwrap();
        assert_eq!(
            child_tags_ext_ids(&h, func),
            ["Input", "Output", "Cfg", "Dfg"]
        );

        assert_eq!(h.children(func).last(), Some(dfg));
        assert_eq!(
            child_tags_ext_ids(&h, dfg)
                .into_iter()
                .sorted()
                .collect_vec(),
            [
                "Const",
                "Input",
                "LoadConst",
                "Noop",
                "Output",
                "UnpackTuple"
            ]
        );

        Ok(())
    }

    #[rstest]
    fn nested_cfgs_pass(#[values(true, false)] nonlocal: bool) {
        //  --> Entry --> Loop --> Tail --> EXIT
        //        |       /  \
        //      (E->X)    \<-/
        let e = extension();
        let tst_op = e.instantiate_extension_op("Test", []).unwrap();
        let qqu = TypeRow::from(vec![qb_t(), qb_t(), usize_t()]);
        let qq = TypeRow::from(vec![qb_t(); 2]);
        let mut outer = CFGBuilder::new(inout_sig(qqu.clone(), vec![usize_t(), qb_t()])).unwrap();
        let mut entry = outer
            .entry_builder(vec![qq.clone()], [usize_t()].into())
            .unwrap();
        let [q1, q2, u] = entry.input_wires_arr();
        let (inner, inner_pred) = {
            let mut inner = entry
                .cfg_builder([(qb_t(), q1), (qb_t(), q2)], qq.clone())
                .unwrap();
            let mut entry = inner.entry_builder(vec![qq.clone()], type_row![]).unwrap();
            let [q1, q2] = entry.input_wires_arr();
            let [pred] = entry
                .add_dataflow_op(Tag::new(0, vec![qq.clone()]), [q1, q2])
                .unwrap()
                .outputs_arr();
            let entry = entry.finish_with_outputs(pred, []).unwrap();
            inner.branch(&entry, 0, &inner.exit_block()).unwrap();
            (inner.finish_sub_container().unwrap(), pred.node())
        };
        let [q1, q2] = inner.outputs_arr();
        let [entry_pred] = entry
            .add_dataflow_op(Tag::new(0, vec![qq.clone()]), [q1, q2])
            .unwrap()
            .outputs_arr();
        let entry = entry.finish_with_outputs(entry_pred, [u]).unwrap();

        let loop_b = {
            let qu = [qb_t(), usize_t()];
            let mut loop_b = outer
                .block_builder(qqu, qu.clone().map(|t| [t].into()), qu.into())
                .unwrap();
            let [q1, q2, u_local] = loop_b.input_wires_arr();
            // u here is `dom` edge from entry block
            let [pred] = loop_b
                .add_dataflow_op(tst_op, [q1, if nonlocal { u } else { u_local }])
                .unwrap()
                .outputs_arr();
            loop_b.finish_with_outputs(pred, [q2, u_local]).unwrap()
        };
        outer.branch(&entry, 0, &loop_b).unwrap();
        outer.branch(&loop_b, 0, &loop_b).unwrap();

        let (tail_b, tail_pred) = {
            let uq = TypeRow::from(vec![usize_t(), qb_t()]);
            let uqu = vec![usize_t(), qb_t(), usize_t()].into();
            let mut tail_b = outer
                .block_builder(uqu, vec![uq.clone()], type_row![])
                .unwrap();
            let [u, q, _] = tail_b.input_wires_arr();
            let [br] = tail_b
                .add_dataflow_op(Tag::new(0, vec![uq]), [u, q])
                .unwrap()
                .outputs_arr();
            (tail_b.finish_with_outputs(br, []).unwrap(), br.node())
        };
        outer.branch(&loop_b, 1, &tail_b).unwrap();
        outer.branch(&tail_b, 0, &outer.exit_block()).unwrap();
        let mut h = outer.finish_hugr().unwrap();
        // Sanity checks:
        assert_eq!(
            h.get_parent(h.get_parent(inner_pred).unwrap()),
            Some(inner.node())
        );
        assert_eq!(h.get_parent(entry_pred.node()), Some(entry.node()));
        assert_eq!(h.get_parent(tail_pred.node()), Some(tail_b.node()));

        let mut res = NormalizeCFGPass::default().run(&mut h).unwrap();

        h.validate().unwrap();
        assert_eq!(
            res.remove(&inner.node()),
            Some(NormalizeCFGResult::CFGToDFG)
        );
        let Some(NormalizeCFGResult::CFGPreserved {
            entry_nodes_moved,
            exit_dfg: Some(tail_dfg),
            num_merged: 0,
        }) = res.remove(&h.entrypoint())
        else {
            panic!("Unexpected result")
        };
        assert!(res.is_empty());
        assert_eq!(entry_nodes_moved.len(), 3);
        // Now contains only one CFG with one BB (self-loop)
        assert_eq!(
            h.nodes()
                .filter(|n| h.get_optype(*n).is_cfg())
                .exactly_one()
                .ok(),
            Some(h.entrypoint())
        );
        let [entry, exit] = h.children(h.entrypoint()).collect_array().unwrap();
        assert_eq!(h.output_neighbours(entry).collect_vec(), [entry, exit]);
        // Inner CFG is now a DFG (and still sibling of entry_pred)...
        assert_eq!(h.get_parent(inner_pred), Some(inner.node()));
        assert_eq!(h.get_optype(inner.node()).tag(), OpTag::Dfg);
        assert_eq!(h.get_parent(inner.node()), h.get_parent(entry_pred.node()));

        // Predicates lifted appropriately...
        let func = h.get_parent(h.entrypoint()).unwrap();
        assert_eq!(h.get_parent(entry_pred.node()), Some(func));

        assert_eq!(h.get_parent(tail_pred.node()), Some(tail_dfg));
        assert_eq!(h.get_optype(tail_dfg).tag(), OpTag::Dfg);
        assert_eq!(h.get_parent(tail_dfg), Some(func));

        // ...and followed by UnpackTuple's
        for n in [inner_pred, entry_pred.node(), tail_pred.node()] {
            let [unpack] = h.output_neighbours(n).collect_array().unwrap();
            assert!(
                h.get_optype(unpack)
                    .as_extension_op()
                    .and_then(|e| UnpackTuple::from_extension_op(e).ok())
                    .is_some()
            );
        }
    }

    fn child_tags_ext_ids<H: HugrView>(h: &H, n: H::Node) -> Vec<String> {
        h.children(n)
            .map(|n| match h.get_optype(n) {
                OpType::ExtensionOp(e) => e.unqualified_id().to_string(),
                op => format!("{:?}", op.tag()),
            })
            .collect()
    }

    // A local extension with a result op
    fn result_ext_and_op() -> (Arc<Extension>, Arc<OpDef>) {
        const EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("test.result");
        const RESULT_OP_ID: OpName = OpName::new_inline("result");
        let e = Extension::new_arc(EXTENSION_ID, Version::new(0, 0, 0), |ext, extension_ref| {
            let tv = Type::new_var_use(0, TypeBound::Linear);
            let signature = PolyFuncTypeRV::new(
                [TypeBound::Linear.into()],
                Signature::new(vec![tv], type_row![]),
            );
            ext.add_op(
                RESULT_OP_ID,
                "Record a value with type provided as a type argument".into(),
                signature,
                extension_ref,
            )
            .unwrap();
        });
        let result_opdef = e.get_op(&RESULT_OP_ID).unwrap().clone();
        (e, result_opdef)
    }

    #[test]
    fn order_edges() {
        // BB1 -> BB2 -> Exit    =>   DFG
        // ...where each BB has an Input->result->Output chain of order edges
        let (_e, result_opdef) = result_ext_and_op();

        let mut h = CFGBuilder::new(inout_sig([usize_t(), bool_t()], [])).unwrap();
        let res1_op: OpType = ExtensionOp::new(result_opdef.clone(), &[usize_t().into()])
            .unwrap()
            .into();
        let res2_op: OpType = ExtensionOp::new(result_opdef.clone(), &[bool_t().into()])
            .unwrap()
            .into();
        let [(bb1, res1), (bb2, res2)] = [
            (None, &res1_op, vec![bool_t()]),
            (Some([bool_t()]), &res2_op, vec![]),
        ]
        .map(|(input_tys, res_op, output_tys)| {
            let mut bb = match input_tys {
                Some(input_tys) => h
                    .simple_block_builder(inout_sig(input_tys, output_tys), 1)
                    .unwrap(),
                None => h.simple_entry_builder(output_tys.into(), 1).unwrap(),
            };
            let mut input_wires = bb.input_wires();
            let res = bb
                .add_dataflow_op(res_op.clone(), [input_wires.next().unwrap()])
                .unwrap()
                .node();
            let pred = bb.add_load_value(Value::unary_unit_sum());
            bb.add_other_wire(bb.input().node(), res);
            bb.add_other_wire(res, bb.output().node());
            (bb.finish_with_outputs(pred, input_wires).unwrap(), res)
        });
        let exit_bb = h.exit_block();
        h.branch(&bb1, 0, &bb2).unwrap();
        h.branch(&bb2, 0, &exit_bb).unwrap();
        let mut h = h.finish_hugr().unwrap();

        NormalizeCFGPass::default().run(&mut h).unwrap();
        h.validate().unwrap();
        // We should have Input -> result1 -> result2 -> Output with Order edges
        assert_eq!(h.get_optype(res1), &res1_op);
        assert_eq!(h.get_optype(res2), &res2_op);
        assert_eq!(h.get_parent(res1.node()), Some(h.entrypoint()));
        assert!(h.entrypoint_optype().is_dfg());
        let [(inp, _)] = h
            .linked_outputs(res1, res1_op.other_input_port().unwrap())
            .collect_array()
            .unwrap();
        let [(outp, _)] = h
            .linked_inputs(res2, res2_op.other_output_port().unwrap())
            .collect_array()
            .unwrap();
        assert_eq!(h.get_io(h.get_parent(res1).unwrap()), Some([inp, outp]));

        let res1_out = res1_op.other_output_port().unwrap();
        let res2_in = res2_op.other_input_port().unwrap();
        let inp_out = h.get_optype(inp).other_output_port().unwrap();
        let outp_in = h.get_optype(outp).other_input_port().unwrap();
        // Extra order edges Input->res2 and res1->Output are harmless
        // and would be removed by RedundantOrderEdgesPass, but we haven't run that
        assert_eq!(
            h.linked_inputs(res1, res1_out).collect_vec(),
            vec![(res2, res2_in), (outp, outp_in)]
        );
        assert_eq!(
            h.linked_outputs(res2, res2_in).collect_vec(),
            vec![(res1, res1_out), (inp, inp_out)]
        );
    }

    #[rstest]
    fn order_edges_loop_after() {
        // entry_bb -> loop_bb -> Exit
        //              /   \
        //              \-<-/
        // ...where entry_bb has a result op => moved before CFG
        let (_e, result_opdef) = result_ext_and_op();

        let mut h = CFGBuilder::new(inout_sig([usize_t(), bool_t()], [])).unwrap();
        let res_op: OpType = ExtensionOp::new(result_opdef.clone(), &[usize_t().into()])
            .unwrap()
            .into();
        let mut entry_bb = h.simple_entry_builder(type_row![], 1).unwrap();
        let [i, b] = entry_bb.input_wires_arr();
        let res = entry_bb
            .add_dataflow_op(res_op.clone(), [i])
            .unwrap()
            .node();
        let pred = entry_bb.add_load_value(Value::unary_unit_sum());
        entry_bb.add_other_wire(entry_bb.input().node(), res);
        entry_bb.add_other_wire(res, entry_bb.output().node());
        let entry_bb = entry_bb.finish_with_outputs(pred, []).unwrap();

        let loop_bb = h
            .simple_block_builder(endo_sig([]), 2)
            .unwrap()
            .finish_with_outputs(b, []) // `Dom` edge from entry block
            .unwrap();

        h.branch(&entry_bb, 0, &loop_bb).unwrap();
        h.branch(&loop_bb, 0, &h.exit_block()).unwrap();
        h.branch(&loop_bb, 1, &loop_bb).unwrap();
        let mut h = h.finish_hugr().unwrap();

        // Ensure the CFG is Ordered between Input/Output of containing FuncDefn
        let fd = h.get_parent(h.entrypoint()).unwrap();
        assert!(h.get_optype(fd).is_func_defn());
        let [inp, outp] = h.get_io(fd).unwrap();
        h.add_other_edge(inp, h.entrypoint());
        h.add_other_edge(h.entrypoint(), outp);
        h.validate().unwrap();

        NormalizeCFGPass::default().run(&mut h).unwrap();
        h.validate().unwrap();

        assert_eq!(h.get_optype(res), &res_op);
        assert!(h.entrypoint_optype().is_cfg()); // remains, but result node lifted outside:
        assert_eq!(h.get_parent(res.node()), Some(fd));

        assert_eq!(
            h.linked_outputs(res, res_op.other_input_port().unwrap())
                .collect_vec(),
            [(inp, h.get_optype(inp).other_output_port().unwrap())]
        );
        assert_eq!(
            h.linked_inputs(res, res_op.other_output_port().unwrap())
                .collect_vec(),
            [(h.entrypoint(), 0.into())]
        );
    }

    #[rstest]
    fn order_edges_loop_before() {
        // loop_bb -> BB1 -> Exit
        //  /   \
        //  \-<-/
        // ...where BB1 has a result op => moved into DFG after CFG
        let (_e, result_opdef) = result_ext_and_op();

        let mut h = CFGBuilder::new(inout_sig([usize_t(), bool_t()], [])).unwrap();
        let res_op: OpType = ExtensionOp::new(result_opdef.clone(), &[usize_t().into()])
            .unwrap()
            .into();
        let loop_bb = h
            .simple_entry_builder([usize_t(), bool_t()].into(), 2)
            .unwrap();
        let [i, b] = loop_bb.input_wires_arr();
        let loop_bb = loop_bb.finish_with_outputs(b, [i, b]).unwrap();

        let mut bb = h
            .simple_block_builder(inout_sig([usize_t(), bool_t()], []), 1)
            .unwrap();
        let [i, _] = bb.input_wires_arr();
        let res = bb.add_dataflow_op(res_op.clone(), [i]).unwrap().node();
        let pred = bb.add_load_value(Value::unary_unit_sum());
        bb.add_other_wire(bb.input().node(), res);
        bb.add_other_wire(res, bb.output().node());
        let bb1 = bb.finish_with_outputs(pred, []).unwrap();

        h.branch(&bb1, 0, &h.exit_block()).unwrap();
        h.branch(&loop_bb, 0, &bb1).unwrap();
        h.branch(&loop_bb, 1, &loop_bb).unwrap();
        let mut h = h.finish_hugr().unwrap();

        // Ensure the CFG is Ordered between Input/Output of containing FuncDefn
        let fd = h.get_parent(h.entrypoint()).unwrap();
        assert!(h.get_optype(fd).is_func_defn());
        let [inp, outp] = h.get_io(fd).unwrap();
        h.add_other_edge(inp, h.entrypoint());
        h.add_other_edge(h.entrypoint(), outp);
        h.validate().unwrap();

        NormalizeCFGPass::default().run(&mut h).unwrap();
        h.validate().unwrap();
        assert_eq!(h.get_optype(res), &res_op);
        assert!(h.entrypoint_optype().is_cfg()); // remains, but result nodes in DFG outside:
        let parent = h.get_parent(res.node()).unwrap();
        assert!(h.get_optype(parent).is_dfg());
        assert_order_single_chain(&h, [inp, h.entrypoint(), parent, outp]);
        let [inp, outp] = h.get_io(parent).unwrap();
        assert_order_single_chain(&h, [inp, res, outp]);
    }

    /// Assert that the given nodes are linked in a single chain of order edges
    /// (nodes[0] -> nodes[1] -> nodes[2], without e.g. nodes[0] -> nodes[2])
    fn assert_order_single_chain<H: HugrView>(h: &H, nodes: impl IntoIterator<Item = H::Node>) {
        for (prev, succ) in nodes.into_iter().tuple_windows() {
            let prev_out = h.get_optype(prev).other_output_port().unwrap();
            let succ_in = h.get_optype(succ).other_input_port().unwrap();
            assert_eq!(
                h.linked_inputs(prev, prev_out).collect_vec(),
                [(succ, succ_in)]
            );
            assert_eq!(
                h.linked_outputs(succ, succ_in).collect_vec(),
                [(prev, prev_out)]
            );
        }
    }
}
