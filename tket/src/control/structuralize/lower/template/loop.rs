//! Loop-specific lowering for detached structuralization templates.

use hugr::builder::{Container, Dataflow, DataflowSubContainer, SubContainer};
use hugr::ops::TailLoop;
use hugr::ops::handle::NodeHandle;
use hugr::types::{Type, TypeRow};
use hugr::{Node, Wire};
use itertools::Itertools;

use super::{
    LoopLowering, LoweredFragment, StructuredBlock, StructuredCfgNode, StructuredNode,
    StructuredRegionBody, TemplateLowerer, structured_node_contains_block,
};
use crate::control::structuralize::StructuralizationError;

impl<'a> LoopLowering<'a> {
    /// Packages the analyzed loop metadata needed by the lowering helpers.
    pub(super) fn new(
        header: &'a StructuredBlock,
        body: &'a [StructuredNode],
        continue_edges: &'a [super::StructuredLoopEdge],
        exits: &'a [super::StructuredLoopExit],
        break_outputs: &'a TypeRow,
    ) -> Self {
        Self {
            header,
            body,
            continue_edges,
            exits,
            break_outputs,
        }
    }

    /// Returns the payload row carried on the loop's continue edge.
    pub(super) fn continue_inputs(&self) -> &TypeRow {
        &self
            .continue_edges
            .first()
            .expect("loop lowering requires at least one continue edge")
            .payload
    }

    /// Returns whether a specific block/case pair continues the loop.
    pub(super) fn is_continue_case(&self, node: StructuredCfgNode, case: usize) -> bool {
        self.continue_edges
            .iter()
            .any(|edge| edge.source == node && edge.case == case)
    }

    /// Returns whether a specific block/case pair exits the loop.
    pub(super) fn break_exit_index(&self, node: StructuredCfgNode, case: usize) -> Option<usize> {
        self.exits.iter().position(|exit| {
            exit.edges
                .iter()
                .any(|edge| edge.source == node && edge.case == case)
        })
    }

    /// Returns whether a structured item contains a block with loop control edges.
    pub(super) fn contains_loop_control(&self, node: &StructuredNode) -> bool {
        structured_node_contains_block(node, self.header.node())
            || self
                .continue_edges
                .iter()
                .map(|edge| edge.source)
                .chain(
                    self.exits
                        .iter()
                        .flat_map(|exit| exit.edges.iter().map(|edge| edge.source)),
                )
                .any(|target| structured_node_contains_block(node, original_cfg_node(target)))
    }

    /// Returns the unique header continue edge expected by the current
    /// header-controlled lowerer.
    pub(super) fn header_continue_edge(
        &self,
    ) -> Result<&super::StructuredLoopEdge, StructuralizationError> {
        self.continue_edges.iter().exactly_one().map_err(|_| {
            StructuralizationError::UnsupportedLoop {
                reason: "header-controlled loop currently requires exactly one continue edge"
                    .into(),
            }
        })
    }
}

/// Returns the original HUGR block represented by one structured CFG node.
pub(super) fn original_cfg_node(node: StructuredCfgNode) -> Node {
    match node {
        StructuredCfgNode::Original(node) | StructuredCfgNode::Duplicate { original: node, .. } => {
            node
        }
    }
}

impl<'a, H: hugr::HugrView<Node = Node>> TemplateLowerer<'a, H> {
    /// Lowers a latch-at-end loop directly into one `TailLoop`.
    pub(super) fn lower_tail_controlled_loop<B>(
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
    pub(super) fn lower_tail_loop_items<B>(
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
            StructuredNode::Block(
                block @ StructuredBlock::Dataflow {
                    node,
                    sum_rows,
                    outputs,
                    ..
                },
            ) => {
                let has_special_case = loop_lowering
                    .continue_edges
                    .iter()
                    .any(|edge| edge.source == block.cfg_node())
                    || loop_lowering
                        .exits
                        .iter()
                        .flat_map(|exit| exit.edges.iter())
                        .any(|edge| edge.source == block.cfg_node());
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
                        !loop_lowering.is_continue_case(block.cfg_node(), case_idx)
                            && loop_lowering
                                .break_exit_index(block.cfg_node(), case_idx)
                                .is_none()
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
                    outputs
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
                    let result = if loop_lowering.is_continue_case(block.cfg_node(), case_idx) {
                        case.make_continue(loop_sig.clone(), case_inputs)?
                    } else if let Some(exit_idx) =
                        loop_lowering.break_exit_index(block.cfg_node(), case_idx)
                    {
                        let break_inputs = self.build_loop_break_inputs(
                            &mut case,
                            loop_lowering,
                            block.cfg_node(),
                            case_idx,
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
                StructuredRegionBody::Sequence(items) => {
                    let mut remaining = Vec::with_capacity(items.len() + rest.len());
                    remaining.extend(items.iter().cloned());
                    remaining.extend(rest.iter().cloned());
                    self.lower_header_loop_items(
                        builder,
                        &remaining,
                        current,
                        loop_lowering,
                        loop_sig,
                    )
                }
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
                            let result = if loop_lowering
                                .is_continue_case(split.cfg_node(), case_idx)
                            {
                                case.make_continue(loop_sig.clone(), case_inputs)?
                            } else if let Some(exit_idx) =
                                loop_lowering.break_exit_index(split.cfg_node(), case_idx)
                            {
                                let break_inputs = self.build_loop_break_inputs(
                                    &mut case,
                                    loop_lowering,
                                    split.cfg_node(),
                                    case_idx,
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
                                super::StructuredBranchJoinKind::Inline
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
                                        super::StructuredBranchJoinKind::Inline => match join {
                                            StructuredBlock::Dataflow { .. } => {
                                                self.lower_linear_block(
                                                    &mut case,
                                                    join,
                                                    arm_fragment.outputs,
                                                )?
                                                .outputs
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
                                        super::StructuredBranchJoinKind::Deferred => {
                                            arm_fragment.outputs
                                        }
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
                    if let Some(dispatch) = &region.multilevel_exit_dispatch {
                        self.lower_multilevel_dispatch(
                            builder,
                            dispatch,
                            next.outputs,
                            rest,
                            loop_lowering,
                            loop_sig,
                        )
                    } else {
                        self.lower_tail_loop_items(
                            builder,
                            rest,
                            next.outputs,
                            loop_lowering,
                            loop_sig,
                        )
                    }
                }
            },
        }
    }

    /// Dispatches a multilevel-exit Sum produced by an inner loop region.
    ///
    /// The inner loop region output is `[Sum(variant_rows)]`.  Each variant
    /// corresponds to one inner-loop exit. Local variants continue the outer
    /// loop body; non-local variants (return / branch to outer loop) cause the
    /// outer loop to break or continue immediately.
    fn lower_multilevel_dispatch<B>(
        &mut self,
        builder: &mut B,
        dispatch: &super::MultilevelExitDispatch,
        sum_wires: Vec<Wire>,
        rest: &[StructuredNode],
        loop_lowering: &LoopLowering<'_>,
        loop_sig: &TailLoop,
    ) -> Result<Wire, StructuralizationError>
    where
        B: Dataflow + Container,
    {
        let [sum_wire] = sum_wires.as_slice() else {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: format!(
                    "multilevel exit dispatch expected 1 Sum wire, got {}",
                    sum_wires.len()
                ),
            });
        };
        let variant_rows: Vec<TypeRow> = dispatch
            .variants
            .iter()
            .map(|v| v.continuation_outputs.clone())
            .collect_vec();

        // All cases produce the outer loop's continue/break Sum.
        let loop_control_type: TypeRow = [Type::new_sum([
            loop_lowering.continue_inputs().clone(),
            loop_lowering.break_outputs.clone(),
        ])]
        .into();

        let mut cond = builder.conditional_builder(
            (variant_rows.clone(), *sum_wire),
            [],
            loop_control_type,
        )?;

        for (case_idx, variant) in dispatch.variants.iter().enumerate() {
            let mut case = cond.case_builder(case_idx)?;
            let case_inputs = case.input_wires().collect_vec();

            // Determine whether this variant continues the outer loop body
            // (its continuation outputs match the outer loop's continue
            // inputs) or breaks the outer loop.
            let continues_outer = variant.continuation_outputs == *loop_lowering.continue_inputs();

            let result = if continues_outer {
                // This exit produced values matching the outer loop's continue
                // payload.  Continue processing the outer loop body, or
                // make_continue if there are no remaining items.
                if rest.is_empty() {
                    case.make_continue(loop_sig.clone(), case_inputs)?
                } else {
                    self.lower_tail_loop_items(
                        &mut case,
                        rest,
                        case_inputs,
                        loop_lowering,
                        loop_sig,
                    )?
                }
            } else {
                // This exit produced values that don't match the outer loop's
                // continue payload.  Break the outer loop immediately.
                // Find which outer exit this variant maps to by matching edges.
                let break_inputs = if loop_lowering.exits.len() <= 1 {
                    case_inputs
                } else {
                    let outer_exit_idx = loop_lowering
                        .exits
                        .iter()
                        .position(|outer_exit| {
                            variant.edges.iter().any(|inner_edge| {
                                outer_exit.edges.iter().any(|outer_edge| {
                                    inner_edge.source == outer_edge.source
                                        && inner_edge.case == outer_edge.case
                                })
                            })
                        })
                        .ok_or_else(|| StructuralizationError::UnsupportedLoop {
                            reason: "multilevel dispatch variant has no matching outer exit".into(),
                        })?;
                    let rows = loop_lowering
                        .exits
                        .iter()
                        .map(|exit| exit.outputs.clone())
                        .collect_vec();
                    let [tagged] = case
                        .add_dataflow_op(hugr::ops::Tag::new(outer_exit_idx, rows), case_inputs)?
                        .outputs_arr();
                    vec![tagged]
                };
                case.make_break(loop_sig.clone(), break_inputs)?
            };
            case.finish_with_outputs([result])?;
        }

        let [result] = cond.finish_sub_container()?.outputs_arr();
        Ok(result)
    }

    /// Re-evaluates a header-controlled loop header after one body path reaches
    /// the header again.
    fn finish_header_loop_iteration<B>(
        &mut self,
        builder: &mut B,
        current: Vec<Wire>,
        loop_lowering: &LoopLowering<'_>,
        loop_sig: &TailLoop,
    ) -> Result<Wire, StructuralizationError>
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
        let (reevaluated, _) = self.lower_block(builder, loop_lowering.header, current)?;
        let [loop_control, loop_rest @ ..] = reevaluated.as_slice() else {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "loop header reevaluation did not produce a control value".into(),
            });
        };

        let mut loop_cond = builder.conditional_builder(
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
                loop_lowering.break_exit_index(loop_lowering.header.cfg_node(), inner_case_idx)
            {
                let break_inputs = self.build_loop_break_inputs(
                    &mut inner_case,
                    loop_lowering,
                    loop_lowering.header.cfg_node(),
                    inner_case_idx,
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
        Ok(loop_control)
    }

    /// Lowers the body of a header-controlled loop while preserving jumps back
    /// to the header and multilevel inner-loop exits.
    fn lower_header_loop_items<B>(
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
            return self.finish_header_loop_iteration(builder, current, loop_lowering, loop_sig);
        };

        match first {
            StructuredNode::Block(
                block @ StructuredBlock::Dataflow {
                    node,
                    sum_rows,
                    outputs,
                    ..
                },
            ) => {
                let has_special_case = loop_lowering
                    .continue_edges
                    .iter()
                    .any(|edge| edge.source == block.cfg_node())
                    || loop_lowering
                        .exits
                        .iter()
                        .flat_map(|exit| exit.edges.iter())
                        .any(|edge| edge.source == block.cfg_node());
                if !has_special_case {
                    let next = self.lower_linear_block(builder, block, current)?;
                    return self.lower_header_loop_items(
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
                        !loop_lowering.is_continue_case(block.cfg_node(), case_idx)
                            && loop_lowering
                                .break_exit_index(block.cfg_node(), case_idx)
                                .is_none()
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
                    outputs
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
                    let result = if loop_lowering.is_continue_case(block.cfg_node(), case_idx) {
                        self.finish_header_loop_iteration(
                            &mut case,
                            case_inputs,
                            loop_lowering,
                            loop_sig,
                        )?
                    } else if let Some(exit_idx) =
                        loop_lowering.break_exit_index(block.cfg_node(), case_idx)
                    {
                        let break_inputs = self.build_loop_break_inputs(
                            &mut case,
                            loop_lowering,
                            block.cfg_node(),
                            case_idx,
                            exit_idx,
                            case_inputs,
                        )?;
                        case.make_break(loop_sig.clone(), break_inputs)?
                    } else if ordinary_cases.contains(&case_idx) {
                        self.lower_header_loop_items(
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
                StructuredRegionBody::Sequence(items) => {
                    let mut remaining = Vec::with_capacity(items.len() + rest.len());
                    remaining.extend(items.iter().cloned());
                    remaining.extend(rest.iter().cloned());
                    self.lower_header_loop_items(
                        builder,
                        &remaining,
                        current,
                        loop_lowering,
                        loop_sig,
                    )
                }
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
                            let result = if loop_lowering
                                .is_continue_case(split.cfg_node(), case_idx)
                            {
                                self.finish_header_loop_iteration(
                                    &mut case,
                                    case_inputs,
                                    loop_lowering,
                                    loop_sig,
                                )?
                            } else if let Some(exit_idx) =
                                loop_lowering.break_exit_index(split.cfg_node(), case_idx)
                            {
                                let break_inputs = self.build_loop_break_inputs(
                                    &mut case,
                                    loop_lowering,
                                    split.cfg_node(),
                                    case_idx,
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
                            self.finish_header_loop_iteration(
                                &mut case,
                                arm_fragment.outputs,
                                loop_lowering,
                                loop_sig,
                            )?
                        } else {
                            match join_kind {
                                super::StructuredBranchJoinKind::Inline
                                    if loop_lowering.contains_loop_control(
                                        &StructuredNode::Block(join.clone()),
                                    ) =>
                                {
                                    let mut remaining = Vec::with_capacity(1 + rest.len());
                                    remaining.push(StructuredNode::Block(join.clone()));
                                    remaining.extend(rest.iter().cloned());
                                    self.lower_header_loop_items(
                                        &mut case,
                                        &remaining,
                                        arm_fragment.outputs,
                                        loop_lowering,
                                        loop_sig,
                                    )?
                                }
                                _ => {
                                    let after_join = match join_kind {
                                        super::StructuredBranchJoinKind::Inline => match join {
                                            StructuredBlock::Dataflow { .. } => {
                                                self.lower_linear_block(
                                                    &mut case,
                                                    join,
                                                    arm_fragment.outputs,
                                                )?
                                                .outputs
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
                                        super::StructuredBranchJoinKind::Deferred => {
                                            arm_fragment.outputs
                                        }
                                    };
                                    self.lower_header_loop_items(
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
                    if let Some(dispatch) = &region.multilevel_exit_dispatch {
                        self.lower_multilevel_dispatch_to_header(
                            builder,
                            dispatch,
                            next.outputs,
                            rest,
                            loop_lowering,
                            loop_sig,
                        )
                    } else {
                        self.lower_header_loop_items(
                            builder,
                            rest,
                            next.outputs,
                            loop_lowering,
                            loop_sig,
                        )
                    }
                }
            },
        }
    }

    /// Dispatches a multilevel-exit Sum inside a header-controlled outer loop.
    fn lower_multilevel_dispatch_to_header<B>(
        &mut self,
        builder: &mut B,
        dispatch: &super::MultilevelExitDispatch,
        sum_wires: Vec<Wire>,
        rest: &[StructuredNode],
        loop_lowering: &LoopLowering<'_>,
        loop_sig: &TailLoop,
    ) -> Result<Wire, StructuralizationError>
    where
        B: Dataflow + Container,
    {
        let [sum_wire] = sum_wires.as_slice() else {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: format!(
                    "multilevel exit dispatch expected 1 Sum wire, got {}",
                    sum_wires.len()
                ),
            });
        };
        let variant_rows = dispatch
            .variants
            .iter()
            .map(|variant| variant.continuation_outputs.clone())
            .collect_vec();
        let loop_control_type: TypeRow = [Type::new_sum([
            loop_lowering.continue_inputs().clone(),
            loop_lowering.break_outputs.clone(),
        ])]
        .into();
        let mut cond =
            builder.conditional_builder((variant_rows, *sum_wire), [], loop_control_type)?;

        for (case_idx, variant) in dispatch.variants.iter().enumerate() {
            let mut case = cond.case_builder(case_idx)?;
            let case_inputs = case.input_wires().collect_vec();
            let continues_outer = variant.continuation_outputs == *loop_lowering.continue_inputs();
            let result = if continues_outer {
                if rest.is_empty() {
                    self.finish_header_loop_iteration(
                        &mut case,
                        case_inputs,
                        loop_lowering,
                        loop_sig,
                    )?
                } else {
                    self.lower_header_loop_items(
                        &mut case,
                        rest,
                        case_inputs,
                        loop_lowering,
                        loop_sig,
                    )?
                }
            } else {
                let break_inputs = if loop_lowering.exits.len() <= 1 {
                    case_inputs
                } else {
                    let outer_exit_idx = loop_lowering
                        .exits
                        .iter()
                        .position(|outer_exit| {
                            variant.edges.iter().any(|inner_edge| {
                                outer_exit.edges.iter().any(|outer_edge| {
                                    inner_edge.source == outer_edge.source
                                        && inner_edge.case == outer_edge.case
                                })
                            })
                        })
                        .ok_or_else(|| StructuralizationError::UnsupportedLoop {
                            reason: "multilevel dispatch variant has no matching outer exit".into(),
                        })?;
                    let rows = loop_lowering
                        .exits
                        .iter()
                        .map(|exit| exit.outputs.clone())
                        .collect_vec();
                    let [tagged] = case
                        .add_dataflow_op(hugr::ops::Tag::new(outer_exit_idx, rows), case_inputs)?
                        .outputs_arr();
                    vec![tagged]
                };
                case.make_break(loop_sig.clone(), break_inputs)?
            };
            case.finish_with_outputs([result])?;
        }
        let [result] = cond.finish_sub_container()?.outputs_arr();
        Ok(result)
    }

    /// Lowers a header-tested loop as an outer `Conditional` plus inner `TailLoop`.
    pub(super) fn lower_header_controlled_loop<B>(
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
                loop_lowering.break_exit_index(loop_lowering.header.cfg_node(), case_idx)
            {
                let break_inputs = self.build_loop_break_inputs(
                    &mut case,
                    &loop_lowering,
                    loop_lowering.header.cfg_node(),
                    case_idx,
                    exit_idx,
                    case_inputs,
                )?;
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
            let loop_control = self.lower_header_loop_items(
                &mut loop_builder,
                loop_lowering.body,
                loop_inputs,
                &loop_lowering,
                &loop_sig,
            )?;
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
    pub(super) fn build_loop_break_inputs<B>(
        &mut self,
        builder: &mut B,
        loop_lowering: &LoopLowering<'_>,
        current_node: StructuredCfgNode,
        current_case: usize,
        exit_idx: usize,
        exit_inputs: Vec<Wire>,
    ) -> Result<Vec<Wire>, StructuralizationError>
    where
        B: Dataflow + Container,
    {
        let expected_outputs = loop_lowering.exits[exit_idx].outputs.clone();
        let actual_outputs: TypeRow = exit_inputs
            .iter()
            .map(|&wire| builder.get_wire_type(wire))
            .collect::<Result<Vec<_>, _>>()?
            .into();
        if exit_inputs.len() != expected_outputs.len() {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: format!(
                    "loop break inputs had the wrong arity: got {}, expected {}",
                    exit_inputs.len(),
                    expected_outputs.len()
                ),
            });
        }
        if loop_lowering.exits.len() == 1 {
            if actual_outputs != expected_outputs {
                return Err(StructuralizationError::UnsupportedLoop {
                    reason: format!(
                        "single-exit loop break payload mismatch at header {} from control {:?}/case {}: actual {actual_outputs:?}, expected {expected_outputs:?}, exit edge payload {:?}, break outputs {:?}",
                        loop_lowering.header.node(),
                        current_node,
                        current_case,
                        loop_lowering.exits[exit_idx]
                            .edges
                            .first()
                            .map(|edge| edge.payload.clone()),
                        loop_lowering.break_outputs,
                    ),
                });
            }
            return Ok(exit_inputs);
        }
        let rows = loop_lowering
            .exits
            .iter()
            .map(|exit| exit.outputs.clone())
            .collect_vec();
        let [tagged] = builder
            .add_dataflow_op(hugr::ops::Tag::new(exit_idx, rows), exit_inputs)?
            .outputs_arr();
        Ok(vec![tagged])
    }

    /// Lowers one exit continuation while treating a propagated branch target
    /// as the terminal boundary of the sequence.
    fn lower_exit_continuation<B>(
        &mut self,
        builder: &mut B,
        items: &[StructuredNode],
        mut current: Vec<Wire>,
        terminal_target: StructuredCfgNode,
    ) -> Result<LoweredFragment, StructuralizationError>
    where
        B: Dataflow + Container,
    {
        let target_node = original_cfg_node(terminal_target);
        let mut entry_order_edges = Vec::new();
        let mut previous_exit_order_edges = Vec::new();

        for item in items {
            let fragment = match item {
                StructuredNode::Block(block) if block.node() == target_node => {
                    break;
                }
                StructuredNode::Region(region) => match &region.body {
                    StructuredRegionBody::Sequence(nested) => self.lower_exit_continuation(
                        builder,
                        nested,
                        current.clone(),
                        terminal_target,
                    )?,
                    StructuredRegionBody::Branch {
                        split, arms, join, ..
                    } if join.node() == target_node => {
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
                            let case_fragment = self.lower_exit_continuation(
                                &mut case,
                                &arm.body,
                                case_inputs,
                                terminal_target,
                            )?;
                            self.close_fragment(&mut case, &case_fragment, None);
                            if case_fragment.outputs.len() != join.inputs().len() {
                                return Err(StructuralizationError::UnsupportedBranch {
                                    reason: format!(
                                        "branch arm {} produced {} outputs but join expects {}",
                                        arm.case,
                                        case_fragment.outputs.len(),
                                        join.inputs().len()
                                    ),
                                });
                            }
                            case.finish_with_outputs(case_fragment.outputs)?;
                        }

                        let cond_handle = cond.finish_sub_container()?;
                        LoweredFragment::ordered(
                            cond_handle.outputs().collect_vec(),
                            cond_handle.node(),
                        )
                    }
                    _ => self.lower_region(builder, region, current.clone())?,
                },
                _ => self.lower_node(builder, item, current.clone())?,
            };

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

    /// Lowers any continuation that runs after a loop break.
    pub(super) fn lower_loop_exits<B>(
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
                let expected_outputs: Vec<TypeRow> = loop_lowering
                    .exits
                    .iter()
                    .map(|exit| exit.effect.outputs().clone())
                    .collect_vec();
                let unique_expected_outputs =
                    expected_outputs.iter().cloned().dedup().collect_vec();
                // Use heterogeneous handling when exits produce different
                // continuation output rows, requiring a Sum-tagged break value
                // and multilevel dispatch at the enclosing loop level.
                let cont_rows: Vec<TypeRow> = exits
                    .iter()
                    .map(|exit| exit.continuation_outputs.clone())
                    .collect_vec();
                let heterogeneous = cont_rows.windows(2).any(|w| w[0] != w[1]);

                if heterogeneous {
                    // Heterogeneous exit effects: use continuation_outputs
                    // to build a Sum type, wrapping each case in Tag.
                    let actual_output_rows: Vec<TypeRow> = exits
                        .iter()
                        .map(|exit| exit.continuation_outputs.clone())
                        .collect_vec();

                    let cond_output_row: TypeRow =
                        vec![Type::new_sum(actual_output_rows.clone())].into();

                    let mut cond = builder.conditional_builder(
                        (
                            exits.iter().map(|exit| exit.outputs.clone()).collect_vec(),
                            *control,
                        ),
                        [],
                        cond_output_row.clone(),
                    )?;
                    for (case_idx, exit) in exits.iter().enumerate() {
                        let mut case = cond.case_builder(case_idx)?;
                        let case_inputs = case.input_wires().collect_vec();
                        let case_fragment = if exit.continuation.is_empty() {
                            LoweredFragment::passthrough(case_inputs)
                        } else if let crate::control::structuralize::StructuredExitEffect::Branch {
                            target,
                            ..
                        } = exit.effect
                        {
                            self.lower_exit_continuation(
                                &mut case,
                                &exit.continuation,
                                case_inputs,
                                target,
                            )?
                        } else {
                            self.lower_sequence(&mut case, &exit.continuation, case_inputs)?
                        };
                        self.close_fragment(&mut case, &case_fragment, None);
                        // Wrap in Tag to produce the Sum output
                        let [tagged] = case
                            .add_dataflow_op(
                                hugr::ops::Tag::new(case_idx, actual_output_rows.clone()),
                                case_fragment.outputs,
                            )?
                            .outputs_arr();
                        case.finish_with_outputs([tagged])?;
                    }
                    let cond_handle = cond.finish_sub_container()?;
                    builder.add_other_wire(loop_anchor, cond_handle.node());
                    Ok(LoweredFragment::ordered(
                        cond_handle.outputs().collect(),
                        cond_handle.node(),
                    ))
                } else {
                    // Homogeneous exit effects: all cases produce the same row.
                    let cond_output_row: TypeRow = unique_expected_outputs[0].clone();

                    let mut cond = builder.conditional_builder(
                        (
                            exits.iter().map(|exit| exit.outputs.clone()).collect_vec(),
                            *control,
                        ),
                        [],
                        cond_output_row.clone(),
                    )?;
                    for (case_idx, exit) in exits.iter().enumerate() {
                        let mut case = cond.case_builder(case_idx)?;
                        let case_inputs = case.input_wires().collect_vec();
                        let case_fragment = if exit.continuation.is_empty() {
                            LoweredFragment::passthrough(case_inputs)
                        } else if let crate::control::structuralize::StructuredExitEffect::Branch {
                            target,
                            ..
                        } = exit.effect
                        {
                            self.lower_exit_continuation(
                                &mut case,
                                &exit.continuation,
                                case_inputs,
                                target,
                            )?
                        } else {
                            self.lower_sequence(&mut case, &exit.continuation, case_inputs)?
                        };
                        self.close_fragment(&mut case, &case_fragment, None);
                        let actual_outputs = TypeRow::from(
                            case_fragment
                                .outputs
                                .iter()
                                .map(|wire| case.get_wire_type(*wire))
                                .collect::<Result<Vec<_>, _>>()?,
                        );
                        let expected_case_outputs = exit.effect.outputs().clone();
                        if actual_outputs != expected_case_outputs {
                            return Err(StructuralizationError::UnsupportedLoop {
                                reason: format!(
                                    "loop exit continuation for effect {:?} produced {:?} but expects {:?}",
                                    exit.effect, actual_outputs, expected_case_outputs,
                                ),
                            });
                        }
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
    }
}
