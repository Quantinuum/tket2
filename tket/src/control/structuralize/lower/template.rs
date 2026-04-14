//! Detached template construction for structured CFG rewrites.
//!
//! The template phase builds a standalone structured HUGR using the shared
//! `Conditional` / `TailLoop` lowering logic. Original CFG blocks are not
//! cloned into this template. Instead, each block becomes a valid placeholder
//! `DFG` whose body immediately panics if it is ever executed before the
//! materialization phase swaps the real block subtree back in.

use hugr::builder::{
    BuildError, Container, DFGBuilder, Dataflow, DataflowHugr, DataflowSubContainer, SubContainer,
};
use hugr::extension::prelude::{ConstError, UnwrapBuilder};
use hugr::ops::handle::NodeHandle;
use hugr::ops::{OpParent, Tag, TailLoop};
use hugr::types::{Signature, Type, TypeRow};
use hugr::{Direction, Hugr, HugrView, Node, Wire};
use itertools::Itertools;
use std::collections::BTreeSet;

use super::super::types::{
    StructuralizationError, StructuredBlock, StructuredBranchJoinKind, StructuredLoopEdge,
    StructuredLoopExit, StructuredLoopKind, StructuredNode, StructuredRegion, StructuredRegionBody,
    structured_node_contains_block,
};

/// Detached replacement template for one CFG rewrite.
///
/// The `hugr` field holds the structured skeleton. Each entry in `placeholders`
/// records which placeholder `DFG` should be replaced with which original CFG
/// block once the template is inserted into the destination HUGR.
#[derive(Debug)]
pub(crate) struct LoweredCfgTemplate {
    /// Structured replacement skeleton built in isolation.
    pub(super) hugr: Hugr,
    /// Placeholder-to-original block mapping for materialization.
    pub(super) placeholders: Vec<BlockPlaceholder>,
}

/// Mapping from a template placeholder node back to the original CFG block.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct BlockPlaceholder {
    /// Original CFG basic block node.
    pub(super) original: Node,
    /// Placeholder `DFG` node inside the detached template.
    pub(super) placeholder: Node,
    /// Whether this placeholder should move the source subtree or clone it.
    pub(super) materialization: BlockMaterialization,
}

/// How one placeholder should obtain its CFG block subtree.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum BlockMaterialization {
    /// Move the original CFG block subtree into the structured replacement.
    Move,
    /// Clone the already-existing block subtree for a duplicated CFG node.
    Clone,
}

/// Borrowed lowering state for one analyzed loop region.
///
/// The lowering code for tail-controlled and header-controlled loops shares the
/// same typed metadata, so this struct keeps the dispatch and helper signatures
/// compact without losing meaning.
struct LoopLowering<'a> {
    /// The analyzed loop header block.
    header: &'a StructuredBlock,
    /// One-iteration body items.
    body: &'a [StructuredNode],
    /// CFG blocks that return control to the header.
    backedge_sources: &'a [Node],
    /// Continue edges routed back to the loop header.
    continue_edges: &'a [StructuredLoopEdge],
    /// Distinct exits routed out of the loop.
    exits: &'a [StructuredLoopExit],
    /// Payload row consumed by the `TailLoop` break path.
    break_outputs: &'a TypeRow,
}

/// Lowered fragment together with the sibling nodes that carry its order
/// dependencies inside the enclosing container.
struct LoweredFragment {
    /// Value wires produced by the fragment.
    outputs: Vec<Wire>,
    /// Nodes in the fragment that should be ordered after the enclosing
    /// container input or a previous sibling.
    entry_order_edges: Vec<Node>,
    /// Nodes in the fragment that should be ordered before a following sibling
    /// or the enclosing container output.
    exit_order_edges: Vec<Node>,
}

impl LoweredFragment {
    /// Creates a fragment for one lowered sibling node.
    fn ordered(outputs: Vec<Wire>, node: Node) -> Self {
        Self {
            outputs,
            entry_order_edges: vec![node],
            exit_order_edges: vec![node],
        }
    }

    /// Creates an order-less fragment, used for exit blocks that only validate
    /// an existing row of wires.
    fn passthrough(outputs: Vec<Wire>) -> Self {
        Self {
            outputs,
            entry_order_edges: Vec::new(),
            exit_order_edges: Vec::new(),
        }
    }
}

impl<'a> LoopLowering<'a> {
    /// Packages the analyzed loop metadata needed by the lowering helpers.
    fn new(
        header: &'a StructuredBlock,
        body: &'a [StructuredNode],
        backedge_sources: &'a [Node],
        continue_edges: &'a [StructuredLoopEdge],
        exits: &'a [StructuredLoopExit],
        break_outputs: &'a TypeRow,
    ) -> Self {
        Self {
            header,
            body,
            backedge_sources,
            continue_edges,
            exits,
            break_outputs,
        }
    }

    /// Returns the payload row carried on the loop's continue edge.
    fn continue_inputs(&self) -> &TypeRow {
        &self
            .continue_edges
            .first()
            .expect("loop lowering requires at least one continue edge")
            .payload
    }

    /// Returns whether a specific block/case pair continues the loop.
    fn is_continue_case(&self, node: Node, case: usize) -> bool {
        self.continue_edges
            .iter()
            .any(|edge| edge.source == node && edge.case == case)
    }

    /// Returns whether a specific block/case pair exits the loop.
    fn break_exit_index(&self, node: Node, case: usize) -> Option<usize> {
        self.exits.iter().position(|exit| {
            exit.edges
                .iter()
                .any(|edge| edge.source == node && edge.case == case)
        })
    }

    /// Returns whether a structured item contains a block with loop control edges.
    fn contains_loop_control(&self, node: &StructuredNode) -> bool {
        let mut targets = self.continue_edges.iter().map(|edge| edge.source).chain(
            self.exits
                .iter()
                .flat_map(|exit| exit.edges.iter().map(|edge| edge.source)),
        );
        targets.any(|target| structured_node_contains_block(node, target))
    }

    /// Returns the unique header continue edge expected by the current
    /// header-controlled lowerer.
    fn header_continue_edge(&self) -> Result<&StructuredLoopEdge, StructuralizationError> {
        self.continue_edges.iter().exactly_one().map_err(|_| {
            StructuralizationError::UnsupportedLoop {
                reason: "header-controlled loop currently requires exactly one continue edge"
                    .into(),
            }
        })
    }
}

/// Builds one detached template from the analyzed structured region.
///
/// # Errors
///
/// Returns an error when the analyzed region cannot be lowered into the shared
/// structured HUGR template.
pub(crate) fn prepare_cfg_replacement<H: HugrView<Node = Node>>(
    cfg_view: &H,
    region: &StructuredRegion,
) -> Result<LoweredCfgTemplate, StructuralizationError> {
    TemplateLowerer::new(cfg_view).lower_cfg_region(region)
}

/// Stateful detached lowerer for one CFG template.
///
/// The lowerer holds the source CFG view plus the placeholder mapping that will
/// later drive in-place block materialization after the template is inserted.
struct TemplateLowerer<'a, H> {
    /// Immutable view of the original CFG being rewritten.
    cfg_view: &'a H,
    /// Placeholder mapping accumulated while lowering blocks.
    placeholders: Vec<BlockPlaceholder>,
    /// Original blocks already assigned a move-based placeholder.
    seen_blocks: BTreeSet<Node>,
}

impl<'a, H: HugrView<Node = Node>> TemplateLowerer<'a, H> {
    /// Creates a lowerer for one CFG template.
    fn new(cfg_view: &'a H) -> Self {
        Self {
            cfg_view,
            placeholders: Vec::new(),
            seen_blocks: BTreeSet::new(),
        }
    }

    /// Lowers one analyzed CFG root into a detached template HUGR.
    fn lower_cfg_region(
        mut self,
        region: &StructuredRegion,
    ) -> Result<LoweredCfgTemplate, StructuralizationError> {
        let mut builder = DFGBuilder::new(Signature::new(
            region.io.inputs.clone(),
            region.io.outputs.clone(),
        ))?;
        let inputs = builder.input_wires().collect_vec();
        let fragment = self.lower_sequence(&mut builder, region.body_sequence()?, inputs)?;
        self.close_fragment(&mut builder, &fragment, None);
        let hugr = builder.finish_hugr_with_outputs(fragment.outputs)?;
        Ok(LoweredCfgTemplate {
            hugr,
            placeholders: self.placeholders,
        })
    }

    /// Lowers a linear sequence of structured nodes left-to-right.
    fn lower_sequence<B>(
        &mut self,
        builder: &mut B,
        items: &[StructuredNode],
        mut current: Vec<Wire>,
    ) -> Result<LoweredFragment, StructuralizationError>
    where
        B: Dataflow + Container,
    {
        let mut entry_order_edges = Vec::new();
        let mut previous_exit_order_edges = Vec::new();
        for item in items {
            let fragment = self.lower_node(builder, item, current)?;
            self.connect_order_edges(
                builder,
                &previous_exit_order_edges,
                &fragment.entry_order_edges,
            );
            if entry_order_edges.is_empty() {
                entry_order_edges = fragment.entry_order_edges.clone();
            }
            if !fragment.exit_order_edges.is_empty() {
                previous_exit_order_edges = fragment.exit_order_edges.clone();
            }
            current = fragment.outputs;
        }
        Ok(LoweredFragment {
            outputs: current,
            entry_order_edges,
            exit_order_edges: previous_exit_order_edges,
        })
    }

    /// Dispatches lowering for either a single block or a nested region.
    fn lower_node<B>(
        &mut self,
        builder: &mut B,
        node: &StructuredNode,
        current: Vec<Wire>,
    ) -> Result<LoweredFragment, StructuralizationError>
    where
        B: Dataflow + Container,
    {
        match node {
            StructuredNode::Block(block) => self.lower_linear_block(builder, block, current),
            StructuredNode::Region(region) => self.lower_region(builder, region, current),
        }
    }

    /// Lowers one structured region according to its region kind.
    ///
    /// Sequence regions recurse directly, branch regions become a
    /// `Conditional` plus join block, and loop regions delegate to the
    /// specialized loop lowering helpers below.
    fn lower_region<B>(
        &mut self,
        builder: &mut B,
        region: &StructuredRegion,
        current: Vec<Wire>,
    ) -> Result<LoweredFragment, StructuralizationError>
    where
        B: Dataflow + Container,
    {
        match &region.body {
            StructuredRegionBody::Sequence(items) => self.lower_sequence(builder, items, current),
            StructuredRegionBody::Branch {
                split,
                arms,
                join,
                join_kind,
            } => {
                let (split_out, _) = self.lower_block(builder, split, current)?;
                let StructuredBlock::Dataflow {
                    sum_rows, outputs, ..
                } = split
                else {
                    return Err(StructuralizationError::ExpectedDataflowBlock {
                        node: split.node(),
                    });
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

                if arms.len() != sum_rows.len()
                    || arms
                        .iter()
                        .map(|arm| arm.case)
                        .sorted()
                        .ne(0..sum_rows.len())
                {
                    return Err(StructuralizationError::UnsupportedBranch {
                        reason: format!(
                            "split block exposes {} cases but region has {} visible arms",
                            sum_rows.len(),
                            arms.len()
                        ),
                    });
                }

                for arm in arms {
                    let mut case = cond.case_builder(arm.case)?;
                    let case_inputs = case.input_wires().collect_vec();
                    let case_fragment = self.lower_sequence(&mut case, &arm.body, case_inputs)?;
                    self.close_fragment(&mut case, &case_fragment, None);
                    case.finish_with_outputs(case_fragment.outputs)?;
                }

                let cond_handle = cond.finish_sub_container()?;
                let cond_node = cond_handle.node();
                let cond_out = cond_handle.outputs().collect_vec();
                match join_kind {
                    StructuredBranchJoinKind::Inline => match join {
                        StructuredBlock::Dataflow { .. } => {
                            let join_fragment = self.lower_linear_block(builder, join, cond_out)?;
                            self.connect_order_edges(
                                builder,
                                &[cond_node],
                                &join_fragment.entry_order_edges,
                            );
                            Ok(LoweredFragment {
                                outputs: join_fragment.outputs,
                                entry_order_edges: vec![cond_node],
                                exit_order_edges: if join_fragment.exit_order_edges.is_empty() {
                                    vec![cond_node]
                                } else {
                                    join_fragment.exit_order_edges
                                },
                            })
                        }
                        StructuredBlock::Exit { .. } => {
                            let join_fragment = self.lower_linear_block(builder, join, cond_out)?;
                            Ok(LoweredFragment {
                                outputs: join_fragment.outputs,
                                entry_order_edges: vec![cond_node],
                                exit_order_edges: vec![cond_node],
                            })
                        }
                    },
                    StructuredBranchJoinKind::Deferred => {
                        Ok(LoweredFragment::ordered(cond_out, cond_node))
                    }
                }
            }
            StructuredRegionBody::Loop {
                kind,
                header,
                body,
                backedge_sources,
                continue_edges,
                exits,
                break_outputs,
            } => {
                let loop_lowering = LoopLowering::new(
                    header,
                    body,
                    backedge_sources,
                    continue_edges,
                    exits,
                    break_outputs,
                );
                match kind {
                    StructuredLoopKind::TailControlled => {
                        self.lower_tail_controlled_loop(builder, loop_lowering, current)
                    }
                    StructuredLoopKind::HeaderControlled => {
                        self.lower_header_controlled_loop(builder, loop_lowering, current)
                    }
                }
            }
        }
    }

    /// Lowers a latch-at-end loop directly into one `TailLoop`.
    ///
    /// The loop body runs once, the latch decides between continue and break,
    /// and a small `Conditional` maps that control choice into the
    /// `TailLoop` continue/break sum.
    fn lower_tail_controlled_loop<B>(
        &mut self,
        builder: &mut B,
        loop_lowering: LoopLowering<'_>,
        current: Vec<Wire>,
    ) -> Result<LoweredFragment, StructuralizationError>
    where
        B: Dataflow + Container,
    {
        let mut loop_builder = builder.tail_loop_builder(
            loop_lowering
                .continue_inputs()
                .iter()
                .cloned()
                .zip(current.iter().copied())
                .collect_vec(),
            [],
            loop_lowering.break_outputs.clone(),
        )?;
        let loop_sig = loop_builder.loop_signature()?.clone();
        let loop_inputs = loop_builder.input_wires().collect_vec();
        let loop_control = self.lower_tail_loop_items(
            &mut loop_builder,
            loop_lowering.body,
            loop_inputs,
            &loop_lowering,
            &loop_sig,
        )?;
        let loop_handle = loop_builder.finish_with_outputs(loop_control, [])?;
        self.lower_loop_exits(
            builder,
            &loop_lowering,
            loop_handle.outputs().collect(),
            loop_handle.node(),
        )
    }

    /// Lowers one loop-body sequence into a `TailLoop` continue/break sum.
    fn lower_tail_loop_items<B>(
        &mut self,
        builder: &mut B,
        items: &[StructuredNode],
        current: Vec<Wire>,
        loop_lowering: &LoopLowering<'_>,
        loop_sig: &TailLoop,
    ) -> Result<Wire, StructuralizationError>
    where
        B: Dataflow + Container,
    {
        let Some((first, rest)) = items.split_first() else {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "loop body does not reach a continue or break edge".into(),
            });
        };

        match first {
            StructuredNode::Block(block @ StructuredBlock::Dataflow { node, sum_rows, .. }) => {
                let has_special_case = loop_lowering
                    .continue_edges
                    .iter()
                    .any(|edge| edge.source == *node)
                    || loop_lowering
                        .exits
                        .iter()
                        .flat_map(|exit| exit.edges.iter())
                        .any(|edge| edge.source == *node);
                if !has_special_case {
                    let next = self.lower_linear_block(builder, block, current)?;
                    return self.lower_tail_loop_items(
                        builder,
                        rest,
                        next.outputs,
                        loop_lowering,
                        loop_sig,
                    );
                }

                let (block_out, _) = self.lower_block(builder, block, current)?;
                let [control, payload @ ..] = block_out.as_slice() else {
                    return Err(StructuralizationError::UnsupportedLoop {
                        reason: format!(
                            "loop control block {node} did not produce a control value"
                        ),
                    });
                };

                let ordinary_cases = (0..sum_rows.len())
                    .filter(|&case_idx| {
                        !loop_lowering.is_continue_case(*node, case_idx)
                            && loop_lowering.break_exit_index(*node, case_idx).is_none()
                    })
                    .collect_vec();
                if ordinary_cases.len() > 1 {
                    return Err(StructuralizationError::UnsupportedLoop {
                        reason: format!(
                            "loop control block {node} has multiple in-loop successor cases"
                        ),
                    });
                }

                let mut cond = builder.conditional_builder(
                    (sum_rows.clone(), *control),
                    loop_lowering
                        .continue_inputs()
                        .iter()
                        .cloned()
                        .zip(payload.iter().copied())
                        .collect_vec(),
                    [Type::new_sum([
                        loop_lowering.continue_inputs().clone(),
                        loop_lowering.break_outputs.clone(),
                    ])]
                    .into(),
                )?;
                for case_idx in 0..sum_rows.len() {
                    let mut case = cond.case_builder(case_idx)?;
                    let case_inputs = case.input_wires().collect_vec();
                    let result = if loop_lowering.is_continue_case(*node, case_idx) {
                        case.make_continue(loop_sig.clone(), case_inputs)?
                    } else if let Some(exit_idx) = loop_lowering.break_exit_index(*node, case_idx) {
                        let break_inputs = self.build_loop_break_inputs(
                            &mut case,
                            loop_lowering,
                            exit_idx,
                            case_inputs,
                        )?;
                        case.make_break(loop_sig.clone(), break_inputs)?
                    } else if ordinary_cases.contains(&case_idx) {
                        self.lower_tail_loop_items(
                            &mut case,
                            rest,
                            case_inputs,
                            loop_lowering,
                            loop_sig,
                        )?
                    } else {
                        return Err(StructuralizationError::UnsupportedLoop {
                            reason: format!("unexpected loop successor case {case_idx}"),
                        });
                    };
                    case.finish_with_outputs([result])?;
                }
                let [result] = cond.finish_sub_container()?.outputs_arr();
                Ok(result)
            }
            StructuredNode::Block(StructuredBlock::Exit { node, .. }) => {
                Err(StructuralizationError::UnsupportedLoop {
                    reason: format!("loop body unexpectedly reaches exit block {node}"),
                })
            }
            StructuredNode::Region(region) => match &region.body {
                StructuredRegionBody::Branch {
                    split,
                    arms,
                    join,
                    join_kind,
                } if loop_lowering.contains_loop_control(first) => {
                    let (split_out, _) = self.lower_block(builder, split, current)?;
                    let StructuredBlock::Dataflow {
                        sum_rows, outputs, ..
                    } = split
                    else {
                        return Err(StructuralizationError::ExpectedDataflowBlock {
                            node: split.node(),
                        });
                    };
                    let [control, rest_outputs @ ..] = split_out.as_slice() else {
                        return Err(StructuralizationError::UnsupportedBranch {
                            reason: "split block did not produce a control output".into(),
                        });
                    };
                    let mut cond = builder.conditional_builder(
                        (sum_rows.clone(), *control),
                        outputs
                            .iter()
                            .cloned()
                            .zip(rest_outputs.iter().copied())
                            .collect_vec(),
                        [Type::new_sum([
                            loop_lowering.continue_inputs().clone(),
                            loop_lowering.break_outputs.clone(),
                        ])]
                        .into(),
                    )?;
                    let rejoins_at_header = join.node() == loop_lowering.header.node();
                    for case_idx in 0..sum_rows.len() {
                        let mut case = cond.case_builder(case_idx)?;
                        let case_inputs = case.input_wires().collect_vec();
                        let Some(arm) = arms.iter().find(|arm| arm.case == case_idx) else {
                            let result = if loop_lowering.is_continue_case(split.node(), case_idx) {
                                case.make_continue(loop_sig.clone(), case_inputs)?
                            } else if let Some(exit_idx) =
                                loop_lowering.break_exit_index(split.node(), case_idx)
                            {
                                let break_inputs = self.build_loop_break_inputs(
                                    &mut case,
                                    loop_lowering,
                                    exit_idx,
                                    case_inputs,
                                )?;
                                case.make_break(loop_sig.clone(), break_inputs)?
                            } else {
                                return Err(StructuralizationError::UnsupportedLoop {
                                    reason: format!("unexpected loop successor case {case_idx}"),
                                });
                            };
                            case.finish_with_outputs([result])?;
                            continue;
                        };
                        let arm_fragment =
                            self.lower_sequence(&mut case, &arm.body, case_inputs)?;
                        let result = if rejoins_at_header {
                            if !rest.is_empty() {
                                return Err(StructuralizationError::UnsupportedLoop {
                                    reason: "loop branch rejoins at the header before remaining body items"
                                        .into(),
                                });
                            }
                            case.make_continue(loop_sig.clone(), arm_fragment.outputs)?
                        } else {
                            match join_kind {
                                StructuredBranchJoinKind::Inline
                                    if loop_lowering.contains_loop_control(
                                        &StructuredNode::Block(join.clone()),
                                    ) =>
                                {
                                    let mut remaining = Vec::with_capacity(1 + rest.len());
                                    remaining.push(StructuredNode::Block(join.clone()));
                                    remaining.extend(rest.iter().cloned());
                                    self.lower_tail_loop_items(
                                        &mut case,
                                        &remaining,
                                        arm_fragment.outputs,
                                        loop_lowering,
                                        loop_sig,
                                    )?
                                }
                                _ => {
                                    let after_join = match join_kind {
                                        StructuredBranchJoinKind::Inline => match join {
                                            StructuredBlock::Dataflow { .. } => {
                                                let (join_out, _) = self.lower_block(
                                                    &mut case,
                                                    join,
                                                    arm_fragment.outputs,
                                                )?;
                                                join_out.into_iter().skip(1).collect()
                                            }
                                            StructuredBlock::Exit { .. } => {
                                                return Err(
                                                    StructuralizationError::UnsupportedLoop {
                                                        reason:
                                                            "loop branch join cannot be an exit block"
                                                                .into(),
                                                    },
                                                );
                                            }
                                        },
                                        StructuredBranchJoinKind::Deferred => arm_fragment.outputs,
                                    };
                                    self.lower_tail_loop_items(
                                        &mut case,
                                        rest,
                                        after_join,
                                        loop_lowering,
                                        loop_sig,
                                    )?
                                }
                            }
                        };
                        case.finish_with_outputs([result])?;
                    }
                    let [result] = cond.finish_sub_container()?.outputs_arr();
                    Ok(result)
                }
                _ => {
                    let next = self.lower_region(builder, region, current)?;
                    self.lower_tail_loop_items(builder, rest, next.outputs, loop_lowering, loop_sig)
                }
            },
        }
    }

    /// Lowers a header-tested loop as an outer `Conditional` plus inner `TailLoop`.
    ///
    /// The outer conditional preserves the zero-iteration path. The inner loop
    /// executes one iteration body and re-evaluates the header condition to
    /// choose between continue and break.
    fn lower_header_controlled_loop<B>(
        &mut self,
        builder: &mut B,
        loop_lowering: LoopLowering<'_>,
        current: Vec<Wire>,
    ) -> Result<LoweredFragment, StructuralizationError>
    where
        B: Dataflow + Container,
    {
        let StructuredBlock::Dataflow {
            sum_rows, outputs, ..
        } = loop_lowering.header
        else {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "header-controlled loop header is not a basic block".into(),
            });
        };
        let continue_edge = loop_lowering.header_continue_edge()?;
        if continue_edge.case >= sum_rows.len() {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "loop case index is out of range".into(),
            });
        }

        let (header_out, _) = self.lower_block(builder, loop_lowering.header, current)?;
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
            loop_lowering.break_outputs.clone(),
        )?;

        for case_idx in 0..sum_rows.len() {
            let mut case = cond.case_builder(case_idx)?;
            let case_inputs = case.input_wires().collect_vec();
            if let Some(exit_idx) =
                loop_lowering.break_exit_index(loop_lowering.header.node(), case_idx)
            {
                let break_inputs =
                    self.build_loop_break_inputs(&mut case, &loop_lowering, exit_idx, case_inputs)?;
                case.finish_with_outputs(break_inputs)?;
                continue;
            }
            if case_idx != continue_edge.case {
                return Err(StructuralizationError::UnsupportedLoop {
                    reason: format!("unexpected loop successor case {case_idx}"),
                });
            }

            let mut loop_builder = case.tail_loop_builder(
                loop_lowering
                    .continue_inputs()
                    .iter()
                    .cloned()
                    .zip(case_inputs.iter().copied())
                    .collect_vec(),
                [],
                loop_lowering.break_outputs.clone(),
            )?;
            let loop_sig = loop_builder.loop_signature()?.clone();
            let loop_inputs = loop_builder.input_wires().collect_vec();
            let body_fragment =
                self.lower_sequence(&mut loop_builder, loop_lowering.body, loop_inputs)?;
            let (reevaluated, _) = self.lower_block(
                &mut loop_builder,
                loop_lowering.header,
                body_fragment.outputs,
            )?;
            let [loop_control, loop_rest @ ..] = reevaluated.as_slice() else {
                return Err(StructuralizationError::UnsupportedLoop {
                    reason: "loop header reevaluation did not produce a control value".into(),
                });
            };
            if !loop_lowering.body.iter().any(|item| {
                loop_lowering
                    .backedge_sources
                    .iter()
                    .any(|source| structured_node_contains_block(item, *source))
            }) {
                return Err(StructuralizationError::UnsupportedLoop {
                    reason: "header-controlled loop body does not contain any backedge source"
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
                    loop_lowering.continue_inputs().clone(),
                    loop_lowering.break_outputs.clone(),
                ])]
                .into(),
            )?;
            for inner_case_idx in 0..sum_rows.len() {
                let mut inner_case = loop_cond.case_builder(inner_case_idx)?;
                let inner_inputs = inner_case.input_wires().collect_vec();
                let result = if inner_case_idx == continue_edge.case {
                    inner_case.make_continue(loop_sig.clone(), inner_inputs)?
                } else if let Some(exit_idx) =
                    loop_lowering.break_exit_index(loop_lowering.header.node(), inner_case_idx)
                {
                    let break_inputs = self.build_loop_break_inputs(
                        &mut inner_case,
                        &loop_lowering,
                        exit_idx,
                        inner_inputs,
                    )?;
                    inner_case.make_break(loop_sig.clone(), break_inputs)?
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
        self.lower_loop_exits(
            builder,
            &loop_lowering,
            cond_handle.outputs().collect_vec(),
            cond_handle.node(),
        )
    }

    /// Builds the `TailLoop` break payload for one selected loop exit.
    fn build_loop_break_inputs<B>(
        &mut self,
        builder: &mut B,
        loop_lowering: &LoopLowering<'_>,
        exit_idx: usize,
        exit_inputs: Vec<Wire>,
    ) -> Result<Vec<Wire>, StructuralizationError>
    where
        B: Dataflow + Container,
    {
        if loop_lowering.exits.len() == 1 {
            return Ok(exit_inputs);
        }
        let rows = loop_lowering
            .exits
            .iter()
            .map(|exit| exit.outputs.clone())
            .collect_vec();
        let [tagged] = builder
            .add_dataflow_op(Tag::new(exit_idx, rows), exit_inputs)?
            .outputs_arr();
        Ok(vec![tagged])
    }

    /// Lowers any continuation that runs after a loop break.
    fn lower_loop_exits<B>(
        &mut self,
        builder: &mut B,
        loop_lowering: &LoopLowering<'_>,
        current: Vec<Wire>,
        loop_anchor: Node,
    ) -> Result<LoweredFragment, StructuralizationError>
    where
        B: Dataflow + Container,
    {
        match loop_lowering.exits {
            [] => Err(StructuralizationError::UnsupportedLoop {
                reason: "loop has no exits".into(),
            }),
            [exit] if exit.continuation.is_empty() => {
                Ok(LoweredFragment::ordered(current, loop_anchor))
            }
            [exit] => {
                let fragment = self.lower_sequence(builder, &exit.continuation, current)?;
                self.connect_order_edges(builder, &[loop_anchor], &fragment.entry_order_edges);
                Ok(LoweredFragment {
                    outputs: fragment.outputs,
                    entry_order_edges: vec![loop_anchor],
                    exit_order_edges: if fragment.exit_order_edges.is_empty() {
                        vec![loop_anchor]
                    } else {
                        fragment.exit_order_edges
                    },
                })
            }
            exits => {
                let [control] = current.as_slice() else {
                    return Err(StructuralizationError::UnsupportedLoop {
                        reason: "multi-exit loop did not produce a tagged break value".into(),
                    });
                };
                let expected_outputs = loop_lowering
                    .exits
                    .iter()
                    .map(|exit| {
                        if exit.continuation.is_empty() {
                            exit.outputs.clone()
                        } else {
                            continuation_output_row(&exit.continuation)
                        }
                    })
                    .dedup()
                    .exactly_one()
                    .map_err(|_| StructuralizationError::UnsupportedLoop {
                        reason: "loop exits do not agree on their continuation outputs".into(),
                    })?;
                let mut cond = builder.conditional_builder(
                    (
                        exits.iter().map(|exit| exit.outputs.clone()).collect_vec(),
                        *control,
                    ),
                    [],
                    expected_outputs,
                )?;
                for (case_idx, exit) in exits.iter().enumerate() {
                    let mut case = cond.case_builder(case_idx)?;
                    let case_inputs = case.input_wires().collect_vec();
                    let case_fragment = if exit.continuation.is_empty() {
                        LoweredFragment::passthrough(case_inputs)
                    } else {
                        self.lower_sequence(&mut case, &exit.continuation, case_inputs)?
                    };
                    self.close_fragment(&mut case, &case_fragment, None);
                    case.finish_with_outputs(case_fragment.outputs)?;
                }
                let cond_handle = cond.finish_sub_container()?;
                builder.add_other_wire(loop_anchor, cond_handle.node());
                Ok(LoweredFragment::ordered(
                    cond_handle.outputs().collect(),
                    cond_handle.node(),
                ))
            }
        }
    }

    /// Lowers a node that is expected to behave like straight-line dataflow.
    ///
    /// Dataflow blocks become placeholder `DFG`s whose outputs are rewired to
    /// the real block during materialization. Exit blocks simply validate that
    /// the current wire row matches the expected exit row.
    fn lower_linear_block<B>(
        &mut self,
        builder: &mut B,
        block: &StructuredBlock,
        current: Vec<Wire>,
    ) -> Result<LoweredFragment, StructuralizationError>
    where
        B: Dataflow + Container,
    {
        match block {
            StructuredBlock::Dataflow { .. } => {
                let (out, node) = self.lower_block(builder, block, current)?;
                Ok(LoweredFragment::ordered(
                    out.into_iter().skip(1).collect(),
                    node,
                ))
            }
            StructuredBlock::Exit { inputs, .. } => {
                let actual = current
                    .iter()
                    .map(|&wire| builder.get_wire_type(wire))
                    .collect::<Result<Vec<_>, _>>()?;
                if TypeRow::from(actual) != *inputs {
                    return Err(StructuralizationError::ExpectedExitBlock { node: block.node() });
                }
                Ok(LoweredFragment::passthrough(current))
            }
        }
    }

    /// Builds a valid placeholder `DFG` for one original CFG dataflow block.
    ///
    /// The placeholder preserves the original block signature and produces its
    /// outputs via a `panic` op. After insertion into the destination HUGR, the
    /// materialization phase rewires the original block subtree into the
    /// placeholder's location and removes the placeholder entirely.
    fn lower_block<B>(
        &mut self,
        builder: &mut B,
        block: &StructuredBlock,
        block_inputs: Vec<Wire>,
    ) -> Result<(Vec<Wire>, Node), StructuralizationError>
    where
        B: Dataflow + Container,
    {
        let StructuredBlock::Dataflow { node, .. } = block else {
            return Err(StructuralizationError::ExpectedDataflowBlock { node: block.node() });
        };
        if block_inputs.len() != block.inputs().len() {
            return Err(StructuralizationError::Build(BuildError::InvalidHUGR(
                hugr::hugr::ValidationError::WrongNumberOfPorts {
                    node: *node,
                    optype: Box::new(self.cfg_view.get_optype(*node).clone()),
                    expected: block.inputs().len(),
                    actual: block_inputs.len(),
                    dir: Direction::Incoming,
                },
            )));
        }

        let signature = self
            .cfg_view
            .get_optype(*node)
            .inner_function_type()
            .ok_or(StructuralizationError::ExpectedDataflowBlock { node: *node })?
            .into_owned();
        let mut placeholder = builder.dfg_builder(signature.clone(), block_inputs)?;
        let placeholder_node = placeholder.container_node();
        let placeholder_inputs = placeholder
            .input_wires()
            .zip(signature.input().iter().cloned())
            .collect_vec();
        let panic = placeholder.add_panic(
            ConstError::new(
                1,
                format!("unmaterialized structuralization placeholder for {node}"),
            ),
            signature.output().iter().cloned(),
            placeholder_inputs,
        )?;
        let handle = placeholder.finish_with_outputs(panic.outputs())?;
        let materialization = if self.seen_blocks.insert(*node) {
            BlockMaterialization::Move
        } else {
            BlockMaterialization::Clone
        };
        self.placeholders.push(BlockPlaceholder {
            original: *node,
            placeholder: placeholder_node,
            materialization,
        });
        Ok((handle.outputs().collect(), handle.node()))
    }

    /// Closes a fragment within the current container by connecting its
    /// explicit order edges to the container `Input`/`Output` nodes.
    fn close_fragment<B>(
        &mut self,
        builder: &mut B,
        fragment: &LoweredFragment,
        start_order_edge: Option<Node>,
    ) where
        B: Dataflow + Container,
    {
        let start = start_order_edge.unwrap_or(builder.input().node());
        self.connect_order_edges(builder, &[start], &fragment.entry_order_edges);
        self.connect_order_edges(
            builder,
            &fragment.exit_order_edges,
            &[builder.output().node()],
        );
    }

    /// Connects each source order edge to each destination order edge.
    fn connect_order_edges<B>(&mut self, builder: &mut B, sources: &[Node], targets: &[Node])
    where
        B: Dataflow + Container,
    {
        for &source in sources {
            for &target in targets {
                builder.add_other_wire(source, target);
            }
        }
    }
}

/// Returns the output row produced by a continuation sequence.
fn continuation_output_row(items: &[StructuredNode]) -> TypeRow {
    match items.last() {
        Some(StructuredNode::Block(block)) => match block {
            StructuredBlock::Dataflow { outputs, .. }
            | StructuredBlock::Exit {
                inputs: outputs, ..
            } => outputs.clone(),
        },
        Some(StructuredNode::Region(region)) => region.io.outputs.clone(),
        None => TypeRow::default(),
    }
}
