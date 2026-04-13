//! Lowering for HUGR-specific structural regions.
//!
//! This module takes the structured metadata produced by `analyze` and rewrites
//! CFG roots into nested dataflow, `Conditional`, and `TailLoop` nodes.

use std::collections::HashSet;

use hugr::builder::{
    BuildError, Container, DFGBuilder, Dataflow, DataflowHugr, DataflowSubContainer, SubContainer,
};
use hugr::hugr::hugrmut::HugrMut;
use hugr::ops::{DFG, OpParent, OpType};
use hugr::types::{Signature, Type, TypeRow};
use hugr::{Hugr, HugrView, Node, Wire};
use hugr_core::hugr::internal::HugrMutInternals;
use itertools::Itertools;

use crate::control::{IdentityCfgMap, rvsdg};
use crate::passes::NormalizeCFGPass;
use crate::passes::composable::ComposablePass;

use super::analyze::analyze_cfg_region;
use super::types::{
    StructuralizationError, StructuralizationRewrite, StructuralizationRewriteReport,
    StructuralizationStrategy, StructuredBlock, StructuredLoopKind, StructuredNode,
    StructuredRegion, StructuredRegionBody, structured_node_contains_block,
};

/// Structuralize the specified CFGs and rewrite them in place.
pub fn structurize_cfgs<H: HugrMut<Node = Node>>(
    hugr: &mut H,
    cfgs: &[Node],
    strategy: StructuralizationStrategy,
) -> Result<StructuralizationRewriteReport, StructuralizationError> {
    match strategy {
        StructuralizationStrategy::Rvsdg => {}
        StructuralizationStrategy::BeyondRelooper => {
            return Err(StructuralizationError::UnsupportedStrategy { strategy });
        }
    }

    let cfgs = outermost_cfgs(hugr, cfgs);
    let mut prepared = Vec::with_capacity(cfgs.len());

    for &cfg in &cfgs {
        let cfg_view = hugr.with_entrypoint(cfg);
        let id_cfg = IdentityCfgMap::new(cfg_view.clone());
        let analyzed = analyze_cfg_region(
            &cfg_view,
            &id_cfg,
            rvsdg::build_control_tree(&id_cfg).map_err(StructuralizationError::Rvsdg)?,
        )?;
        let replacement = lower_cfg_region(&cfg_view, &analyzed)?;
        prepared.push((cfg, replacement));
    }

    let mut rewrites = Vec::with_capacity(prepared.len());
    for (cfg, replacement) in prepared {
        rewrite_cfg_as_dfg(hugr, cfg, replacement);
        NormalizeCFGPass::default().run(&mut hugr.with_entrypoint_mut(cfg))?;
        rewrites.push(StructuralizationRewrite {
            cfg,
            rewritten: true,
        });
    }

    Ok(StructuralizationRewriteReport { rewrites })
}

/// Filters a set of CFG roots down to the outermost ones.
///
/// Rewriting a parent CFG subsumes rewriting nested CFGs beneath it, so the
/// pass only prepares top-level rewrite targets.
fn outermost_cfgs<H: HugrView<Node = Node>>(hugr: &H, cfgs: &[Node]) -> Vec<Node> {
    let cfg_set = cfgs.iter().copied().collect::<HashSet<_>>();
    cfgs.iter()
        .copied()
        .filter(|cfg| {
            let mut parent = hugr.get_parent(*cfg);
            while let Some(node) = parent {
                if cfg_set.contains(&node) {
                    return false;
                }
                parent = hugr.get_parent(node);
            }
            true
        })
        .collect()
}

/// Borrowed lowering state for one analyzed loop region.
///
/// The lowering code for tail-controlled and header-controlled loops shares the
/// same typed metadata, so this struct keeps the dispatch and helper signatures
/// compact without losing meaning.
struct LoopLowering<'a> {
    header: &'a StructuredBlock,
    body: &'a [StructuredNode],
    backedge_source: Node,
    continue_inputs: &'a TypeRow,
    break_outputs: &'a TypeRow,
    continue_case: usize,
    break_case: usize,
}

impl<'a> LoopLowering<'a> {
    /// Packages the analyzed loop metadata needed by the lowering helpers.
    fn new(
        header: &'a StructuredBlock,
        body: &'a [StructuredNode],
        backedge_source: Node,
        continue_inputs: &'a TypeRow,
        break_outputs: &'a TypeRow,
        continue_case: usize,
        break_case: usize,
    ) -> Self {
        Self {
            header,
            body,
            backedge_source,
            continue_inputs,
            break_outputs,
            continue_case,
            break_case,
        }
    }

    /// Lowers a latch-at-end loop directly into one `TailLoop`.
    ///
    /// The loop body runs once, the latch decides between continue and break,
    /// and a small `Conditional` maps that control choice into the
    /// `TailLoop` continue/break sum.
    fn lower_tail_controlled<B, H>(
        &self,
        builder: &mut B,
        cfg_view: &H,
        current: Vec<Wire>,
    ) -> Result<Vec<Wire>, StructuralizationError>
    where
        B: Dataflow + Container,
        H: HugrView<Node = Node>,
    {
        let (latch, prefix) =
            self.body
                .split_last()
                .ok_or_else(|| StructuralizationError::UnsupportedLoop {
                    reason: "loop body is empty".into(),
                })?;
        let StructuredNode::Block(latch_block @ StructuredBlock::Dataflow { node, sum_rows, .. }) =
            latch
        else {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "loop latch is not a basic block".into(),
            });
        };
        if *node != self.backedge_source {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "loop latch does not match the analyzed backedge source".into(),
            });
        }

        let mut loop_builder = builder.tail_loop_builder(
            self.continue_inputs
                .iter()
                .cloned()
                .zip(current.iter().copied())
                .collect_vec(),
            [],
            self.break_outputs.clone(),
        )?;
        let loop_sig = loop_builder.loop_signature()?.clone();
        let loop_inputs = loop_builder.input_wires().collect_vec();
        let prefix_out = lower_sequence(&mut loop_builder, cfg_view, prefix, loop_inputs)?;
        let latch_out = lower_block(&mut loop_builder, cfg_view, latch_block, prefix_out)?;
        let [latch_control, latch_rest @ ..] = latch_out.as_slice() else {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "loop latch did not produce a control value".into(),
            });
        };

        if sum_rows.len() != 2 {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: format!(
                    "loop latch must have exactly two successors, found {}",
                    sum_rows.len()
                ),
            });
        }
        if self.continue_case >= sum_rows.len() || self.break_case >= sum_rows.len() {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "loop case index is out of range".into(),
            });
        }

        let mut cond = loop_builder.conditional_builder(
            (sum_rows.clone(), *latch_control),
            self.continue_inputs
                .iter()
                .cloned()
                .zip(latch_rest.iter().copied())
                .collect_vec(),
            [Type::new_sum([
                self.continue_inputs.clone(),
                self.break_outputs.clone(),
            ])]
            .into(),
        )?;
        for case_idx in 0..sum_rows.len() {
            let mut case = cond.case_builder(case_idx)?;
            let case_inputs = case.input_wires().collect_vec();
            let result = if case_idx == self.continue_case {
                case.make_continue(loop_sig.clone(), case_inputs)?
            } else if case_idx == self.break_case {
                case.make_break(loop_sig.clone(), case_inputs)?
            } else {
                return Err(StructuralizationError::UnsupportedLoop {
                    reason: format!("unexpected loop successor case {case_idx}"),
                });
            };
            case.finish_with_outputs([result])?;
        }
        let [loop_control] = cond.finish_sub_container()?.outputs_arr();
        let loop_handle = loop_builder.finish_with_outputs(loop_control, [])?;
        Ok(loop_handle.outputs().collect())
    }

    /// Lowers a header-tested loop as an outer `Conditional` plus inner `TailLoop`.
    ///
    /// The outer conditional preserves the zero-iteration path. The inner loop
    /// executes one iteration body and re-evaluates the header condition to
    /// choose between continue and break.
    fn lower_header_controlled<B, H>(
        &self,
        builder: &mut B,
        cfg_view: &H,
        current: Vec<Wire>,
    ) -> Result<Vec<Wire>, StructuralizationError>
    where
        B: Dataflow + Container,
        H: HugrView<Node = Node>,
    {
        let StructuredBlock::Dataflow {
            sum_rows, outputs, ..
        } = self.header
        else {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "header-controlled loop header is not a basic block".into(),
            });
        };
        if self.continue_case >= sum_rows.len() || self.break_case >= sum_rows.len() {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "loop case index is out of range".into(),
            });
        }

        let header_out = lower_block(builder, cfg_view, self.header, current)?;
        let [header_control, header_rest @ ..] = header_out.as_slice() else {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "loop header did not produce a control value".into(),
            });
        };
        if header_rest.len() != outputs.len() {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "loop header output arity does not match its analyzed row".into(),
            });
        }

        let mut cond = builder.conditional_builder(
            (sum_rows.clone(), *header_control),
            outputs
                .iter()
                .cloned()
                .zip(header_rest.iter().copied())
                .collect_vec(),
            self.break_outputs.clone(),
        )?;

        for case_idx in 0..sum_rows.len() {
            let mut case = cond.case_builder(case_idx)?;
            let case_inputs = case.input_wires().collect_vec();
            if case_idx == self.break_case {
                case.finish_with_outputs(case_inputs)?;
                continue;
            }
            if case_idx != self.continue_case {
                return Err(StructuralizationError::UnsupportedLoop {
                    reason: format!("unexpected loop successor case {case_idx}"),
                });
            }

            let mut loop_builder = case.tail_loop_builder(
                self.continue_inputs
                    .iter()
                    .cloned()
                    .zip(case_inputs.iter().copied())
                    .collect_vec(),
                [],
                self.break_outputs.clone(),
            )?;
            let loop_sig = loop_builder.loop_signature()?.clone();
            let loop_inputs = loop_builder.input_wires().collect_vec();
            let body_out = lower_sequence(&mut loop_builder, cfg_view, self.body, loop_inputs)?;
            let reevaluated = lower_block(&mut loop_builder, cfg_view, self.header, body_out)?;
            let [loop_control, loop_rest @ ..] = reevaluated.as_slice() else {
                return Err(StructuralizationError::UnsupportedLoop {
                    reason: "loop header reevaluation did not produce a control value".into(),
                });
            };
            if !self
                .body
                .iter()
                .any(|item| structured_node_contains_block(item, self.backedge_source))
            {
                return Err(StructuralizationError::UnsupportedLoop {
                    reason: "header-controlled loop body does not contain the backedge source"
                        .into(),
                });
            }

            let mut loop_cond = loop_builder.conditional_builder(
                (sum_rows.clone(), *loop_control),
                outputs
                    .iter()
                    .cloned()
                    .zip(loop_rest.iter().copied())
                    .collect_vec(),
                [Type::new_sum([
                    self.continue_inputs.clone(),
                    self.break_outputs.clone(),
                ])]
                .into(),
            )?;
            for inner_case_idx in 0..sum_rows.len() {
                let mut inner_case = loop_cond.case_builder(inner_case_idx)?;
                let inner_inputs = inner_case.input_wires().collect_vec();
                let result = if inner_case_idx == self.continue_case {
                    inner_case.make_continue(loop_sig.clone(), inner_inputs)?
                } else if inner_case_idx == self.break_case {
                    inner_case.make_break(loop_sig.clone(), inner_inputs)?
                } else {
                    return Err(StructuralizationError::UnsupportedLoop {
                        reason: format!("unexpected loop successor case {inner_case_idx}"),
                    });
                };
                inner_case.finish_with_outputs([result])?;
            }
            let [loop_control] = loop_cond.finish_sub_container()?.outputs_arr();
            let loop_handle = loop_builder.finish_with_outputs(loop_control, [])?;
            case.finish_with_outputs(loop_handle.outputs())?;
        }

        let cond_handle = cond.finish_sub_container()?;
        Ok(cond_handle.outputs().collect_vec())
    }
}

/// Lowers one analyzed CFG root into a replacement DFG HUGR.
fn lower_cfg_region<H: HugrView<Node = Node>>(
    cfg_view: &H,
    region: &StructuredRegion,
) -> Result<Hugr, StructuralizationError> {
    let mut builder = DFGBuilder::new(Signature::new(
        region.io.inputs.clone(),
        region.io.outputs.clone(),
    ))?;
    let inputs = builder.input_wires().collect_vec();
    let state = lower_sequence(&mut builder, cfg_view, region.body_sequence()?, inputs)?;
    Ok(builder.finish_hugr_with_outputs(state)?)
}

/// Lowers a linear sequence of structured nodes left-to-right.
fn lower_sequence<B, H>(
    builder: &mut B,
    cfg_view: &H,
    items: &[StructuredNode],
    mut current: Vec<Wire>,
) -> Result<Vec<Wire>, StructuralizationError>
where
    B: Dataflow + Container,
    H: HugrView<Node = Node>,
{
    for item in items {
        current = lower_node(builder, cfg_view, item, current)?;
    }
    Ok(current)
}

/// Dispatches lowering for either a single block or a nested region.
fn lower_node<B, H>(
    builder: &mut B,
    cfg_view: &H,
    node: &StructuredNode,
    current: Vec<Wire>,
) -> Result<Vec<Wire>, StructuralizationError>
where
    B: Dataflow + Container,
    H: HugrView<Node = Node>,
{
    match node {
        StructuredNode::Block(block) => lower_linear_block(builder, cfg_view, block, current),
        StructuredNode::Region(region) => lower_region(builder, cfg_view, region, current),
    }
}

/// Lowers one structured region according to its region kind.
///
/// Sequence regions recurse directly, branch regions become a `Conditional`
/// plus join block, and loop regions delegate to the specialized loop lowering
/// helpers above.
fn lower_region<B, H>(
    builder: &mut B,
    cfg_view: &H,
    region: &StructuredRegion,
    current: Vec<Wire>,
) -> Result<Vec<Wire>, StructuralizationError>
where
    B: Dataflow + Container,
    H: HugrView<Node = Node>,
{
    match &region.body {
        StructuredRegionBody::Sequence(items) => lower_sequence(builder, cfg_view, items, current),
        StructuredRegionBody::Branch { split, arms, join } => {
            let split_out = lower_block(builder, cfg_view, split, current)?;
            let StructuredBlock::Dataflow {
                sum_rows, outputs, ..
            } = split
            else {
                return Err(StructuralizationError::ExpectedDataflowBlock { node: split.node() });
            };
            let [control, rest @ ..] = split_out.as_slice() else {
                return Err(StructuralizationError::UnsupportedBranch {
                    reason: "split block did not produce a control output".into(),
                });
            };
            let mut cond = builder.conditional_builder(
                (sum_rows.clone(), *control),
                outputs
                    .iter()
                    .cloned()
                    .zip(rest.iter().copied())
                    .collect_vec(),
                join.inputs().clone(),
            )?;

            if arms.len() != sum_rows.len() {
                return Err(StructuralizationError::UnsupportedBranch {
                    reason: format!(
                        "split block exposes {} cases but region has {} arms",
                        sum_rows.len(),
                        arms.len()
                    ),
                });
            }

            for (case_idx, arm) in arms.iter().enumerate() {
                let mut case = cond.case_builder(case_idx)?;
                let case_inputs = case.input_wires().collect_vec();
                let case_outputs = lower_sequence(&mut case, cfg_view, arm, case_inputs)?;
                case.finish_with_outputs(case_outputs)?;
            }

            let cond_out = cond.finish_sub_container()?.outputs().collect_vec();
            let join_out = lower_block(builder, cfg_view, join, cond_out)?;
            Ok(join_out.into_iter().skip(1).collect())
        }
        StructuredRegionBody::Loop {
            kind,
            header,
            body,
            backedge_source,
            continue_inputs,
            break_outputs,
            continue_case,
            break_case,
        } => {
            let loop_lowering = LoopLowering::new(
                header,
                body,
                *backedge_source,
                continue_inputs,
                break_outputs,
                *continue_case,
                *break_case,
            );
            match kind {
                StructuredLoopKind::TailControlled => {
                    loop_lowering.lower_tail_controlled(builder, cfg_view, current)
                }
                StructuredLoopKind::HeaderControlled => {
                    loop_lowering.lower_header_controlled(builder, cfg_view, current)
                }
            }
        }
    }
}

/// Lowers a node that is expected to behave like straight-line dataflow.
///
/// Dataflow blocks yield their non-control outputs, while exit blocks simply
/// validate that the current wire row matches the expected exit row.
fn lower_linear_block<B, H>(
    builder: &mut B,
    cfg_view: &H,
    block: &StructuredBlock,
    current: Vec<Wire>,
) -> Result<Vec<Wire>, StructuralizationError>
where
    B: Dataflow + Container,
    H: HugrView<Node = Node>,
{
    match block {
        StructuredBlock::Dataflow { .. } => {
            let out = lower_block(builder, cfg_view, block, current)?;
            Ok(out.into_iter().skip(1).collect())
        }
        StructuredBlock::Exit { inputs, .. } => {
            let actual = current
                .iter()
                .map(|&wire| builder.get_wire_type(wire))
                .collect::<Result<Vec<_>, _>>()?;
            if TypeRow::from(actual) != *inputs {
                return Err(StructuralizationError::ExpectedExitBlock { node: block.node() });
            }
            Ok(current)
        }
    }
}

/// Copies one original CFG dataflow block subtree into the target container.
///
/// The copied subtree is wrapped as a DFG so it can be wired back into the
/// newly structured surrounding control flow.
fn lower_block<B, H>(
    builder: &mut B,
    cfg_view: &H,
    block: &StructuredBlock,
    block_inputs: Vec<Wire>,
) -> Result<Vec<Wire>, StructuralizationError>
where
    B: Dataflow + Container,
    H: HugrView<Node = Node>,
{
    let StructuredBlock::Dataflow { node, .. } = block else {
        return Err(StructuralizationError::ExpectedDataflowBlock { node: block.node() });
    };
    if block_inputs.len() != block.inputs().len() {
        return Err(StructuralizationError::Build(BuildError::InvalidHUGR(
            hugr::hugr::ValidationError::WrongNumberOfPorts {
                node: *node,
                optype: Box::new(cfg_view.get_optype(*node).clone()),
                expected: block.inputs().len(),
                actual: block_inputs.len(),
                dir: hugr::Direction::Incoming,
            },
        )));
    }
    let mut copied_nodes = cfg_view.descendants(*node).collect_vec();
    let mut copied_roots = vec![(*node, builder.container_node())];
    let mut copied_set = copied_nodes.iter().copied().collect::<HashSet<_>>();
    let mut extra_roots = Vec::new();
    for copied in copied_nodes.iter().copied() {
        if let Some(source) = cfg_view.static_source(copied) {
            if copied_set.contains(&source) {
                continue;
            }
            match cfg_view.get_optype(source) {
                OpType::Const(_) => {
                    extra_roots.push(source);
                }
                optype => {
                    return Err(StructuralizationError::Build(
                        BuildError::HugrViewInsertionError(format!(
                            "unsupported static dependency {source} ({optype}) while lowering block {node}"
                        )),
                    ));
                }
            }
        }
    }
    for root in extra_roots {
        for descendant in cfg_view.descendants(root) {
            if copied_set.insert(descendant) {
                copied_nodes.push(descendant);
            }
        }
        copied_roots.push((root, builder.container_node()));
    }
    let insertion = builder
        .hugr_mut()
        .insert_view_forest(cfg_view, copied_nodes.iter().copied(), copied_roots)
        .map_err(|err| BuildError::HugrViewInsertionError(err.to_string()))?;
    let root = insertion.node_map[node];
    let signature = builder
        .hugr()
        .get_optype(root)
        .inner_function_type()
        .expect("inserted block should be dataflow")
        .into_owned();
    let value_output_count = signature.output_count();
    builder.hugr_mut().replace_op(root, DFG { signature });
    let (input_count, output_count) = {
        let new_op = builder.hugr().get_optype(root);
        (new_op.input_count(), new_op.output_count())
    };
    builder
        .hugr_mut()
        .set_num_ports(root, input_count, output_count);
    let root_input_count = builder.hugr().get_optype(root).input_count();
    if block_inputs.len() > root_input_count {
        return Err(StructuralizationError::Build(BuildError::InvalidHUGR(
            hugr::hugr::ValidationError::WrongNumberOfPorts {
                node: *node,
                optype: Box::new(cfg_view.get_optype(*node).clone()),
                expected: root_input_count,
                actual: block_inputs.len(),
                dir: hugr::Direction::Incoming,
            },
        )));
    }
    for (dst_port, wire) in block_inputs.into_iter().enumerate() {
        builder
            .hugr_mut()
            .connect(wire.node(), wire.source(), root, dst_port);
    }
    Ok((0..value_output_count)
        .map(|offset| Wire::new(root, offset))
        .collect())
}

/// Replaces a CFG root with the newly lowered DFG subtree.
///
/// The original CFG children are removed, the root op is changed to `DFG`, and
/// the replacement HUGR is spliced into the original location.
fn rewrite_cfg_as_dfg<H: HugrMut<Node = Node>>(hugr: &mut H, cfg: Node, replacement: Hugr) {
    let signature = replacement
        .get_optype(replacement.entrypoint())
        .inner_function_type()
        .expect("replacement should be dataflow")
        .into_owned();

    for child in hugr.children(cfg).collect_vec() {
        hugr.remove_subtree(child);
    }
    hugr.replace_op(cfg, DFG { signature });
    let inserted = hugr.insert_hugr(cfg, replacement).inserted_entrypoint;
    while let Some(child) = hugr.first_child(inserted) {
        hugr.set_parent(child, cfg);
    }
    hugr.remove_node(inserted);
}
