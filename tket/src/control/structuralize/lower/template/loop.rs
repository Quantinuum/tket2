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
        backedge_sources: &'a [Node],
        continue_edges: &'a [super::StructuredLoopEdge],
        exits: &'a [super::StructuredLoopExit],
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
        let mut targets = self.continue_edges.iter().map(|edge| edge.source).chain(
            self.exits
                .iter()
                .flat_map(|exit| exit.edges.iter().map(|edge| edge.source)),
        );
        targets.any(|target| structured_node_contains_block(node, original_cfg_node(target)))
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

/// Returns the output row produced by a continuation sequence.
pub(super) fn continuation_output_row(items: &[StructuredNode]) -> TypeRow {
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
                    self.lower_tail_loop_items(builder, rest, next.outputs, loop_lowering, loop_sig)
                }
            },
        }
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
                    loop_lowering.break_exit_index(loop_lowering.header.cfg_node(), inner_case_idx)
                {
                    let break_inputs = self.build_loop_break_inputs(
                        &mut inner_case,
                        &loop_lowering,
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
}
