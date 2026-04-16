//! Loop-region construction for the Beyond-Relooper strategy.

use std::collections::BTreeSet;

use hugr::core::HugrNode;
use hugr::{HugrView, Node};
use itertools::Itertools;

use crate::control::CfgBlockMap;
use crate::control::cfg::CfgFacts;
use crate::control::relooper::ast::{
    RelooperBlockLowering, RelooperBranchTarget, RelooperContextFrame, RelooperLoopLowering,
    RelooperRegion, RelooperStmt,
};
use crate::control::relooper::block::{block_input_row, block_successor_payload, cfg_output_row};
use crate::control::relooper::construct::context::{
    append_branch_to_target, append_exit_from_context, context_branch_target, is_terminal_stmt,
    push_context,
};
use crate::control::structuralize::IntoStructuredCfgNode;
use crate::control::structuralize::{
    RegionIo, StructuralizationError, StructuredBranchJoinKind, StructuredLoopEdge,
    StructuredLoopKind,
};

use super::{IntoRelooperLabel, ScopeBuild, ScopeFrame, analyze_block};

impl<T> CfgFacts<T>
where
    T: IntoRelooperLabel + IntoStructuredCfgNode,
{
    /// Structures one reducible loop rooted at its unique header.
    pub(super) fn build_loop_region<H, C>(
        &self,
        cfg_view: &H,
        cfg: &C,
        header: T,
        frame: ScopeFrame<'_, T>,
    ) -> Result<(RelooperRegion, Option<T>), StructuralizationError>
    where
        H: HugrView<Node = Node>,
        C: CfgBlockMap<T>,
    {
        let loop_blocks = self
            .loop_blocks
            .get(&header)
            .ok_or(StructuralizationError::Relooper {
                reason: format!("missing loop blocks for header {header}"),
            })?
            .intersection(frame.scope)
            .copied()
            .collect::<BTreeSet<_>>();
        let exit_edges = loop_blocks
            .iter()
            .copied()
            .flat_map(|src| {
                self.succs
                    .get(&src)
                    .into_iter()
                    .flatten()
                    .copied()
                    .map(move |dst| (src, dst))
            })
            .filter(|(src, dst)| {
                !loop_blocks.contains(dst) && !self.is_loop_backedge(*src, *dst, header)
            })
            .collect_vec();
        let backedge_sources = self
            .backedges
            .get(&header)
            .into_iter()
            .flatten()
            .copied()
            .filter(|source| loop_blocks.contains(source))
            .collect_vec();
        if backedge_sources.is_empty() {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: format!("loop headed by {header} has no backedge source"),
            });
        }
        let header_block = analyze_block(
            cfg_view,
            header.into_structured_cfg_node(),
            cfg.hugr_node(header),
        )?;
        let header_succs = self.succs.get(&header).cloned().unwrap_or_default();
        let in_loop_succs = header_succs
            .iter()
            .copied()
            .filter(|succ| loop_blocks.contains(succ))
            .collect_vec();
        let out_of_loop_succs = header_succs
            .iter()
            .copied()
            .filter(|succ| !loop_blocks.contains(succ))
            .collect_vec();

        let exit_variants = build_loop_exit_variants(
            cfg_view,
            cfg,
            &exit_edges,
            "loop exit source {src} has no edge to the exit target",
            "tail-controlled loop exit case is out of range",
        )?;
        let multi_exit = exit_variants.len() > 1;
        let io = RegionIo {
            inputs: header_block.inputs().clone(),
            outputs: if multi_exit {
                loop_continuation_outputs(cfg_view, cfg, frame.stop)?
            } else {
                block_input_row(cfg_view, cfg.hugr_node(exit_variants[0].0))?
            },
        };

        let loop_label = header.into_relooper_label();
        let use_header_controlled = !out_of_loop_succs.is_empty() && in_loop_succs.len() == 1;
        let (body, next) = if use_header_controlled {
            let continue_target = in_loop_succs.into_iter().exactly_one().map_err(|_| {
                StructuralizationError::UnsupportedLoop {
                    reason: "header-controlled loop must have exactly one in-loop successor".into(),
                }
            })?;
            let continue_case = header_succs
                .iter()
                .position(|succ| *succ == continue_target)
                .ok_or(StructuralizationError::UnsupportedLoop {
                    reason: "header-controlled loop continue edge is missing".into(),
                })?;
            let continue_payload = block_successor_payload(
                cfg_view,
                cfg.hugr_node(header),
                continue_case,
                "header-controlled loop continue case is out of range",
            )?;
            let exit_continuations = exit_variants
                .iter()
                .map(|(target, _)| {
                    if multi_exit {
                        let mut continuation = self.build_scope(
                            cfg_view,
                            cfg,
                            ScopeBuild {
                                start: *target,
                                scope: frame.scope,
                                stop: frame.stop,
                                active_loop: frame.active_loop,
                                context: frame.context,
                            },
                        )?;
                        // If the target is outside the enclosing scope, this
                        // exit leaves all enclosing loops entirely rather than
                        // branching back to the nearest one.
                        if frame.scope.contains(target) {
                            let exit_payload = context_branch_target(frame.context)
                                .map(|target| branch_target_payload_row(cfg_view, target))
                                .transpose()?
                                .unwrap_or_else(|| io.outputs.clone());
                            append_exit_from_context(
                                &mut continuation,
                                frame.context,
                                exit_payload,
                            );
                        } else if !continuation.last().is_some_and(is_terminal_stmt) {
                            continuation.push(RelooperStmt::Return(io.outputs.clone()));
                        }
                        Ok(continuation)
                    } else {
                        Ok(Vec::new())
                    }
                })
                .collect::<Result<Vec<Vec<RelooperStmt>>, StructuralizationError>>()?;
            let exit_edges = exit_variants
                .iter()
                .map(|(_, edges)| edges.clone())
                .collect_vec();
            let exit_targets = exit_variants
                .iter()
                .map(|(target, _)| *target)
                .collect_vec();
            let loop_context = push_context(
                frame.context,
                RelooperContextFrame::LoopHeadedBy(loop_label),
            );
            let body = self.build_scope(
                cfg_view,
                cfg,
                ScopeBuild {
                    start: continue_target,
                    scope: &loop_blocks,
                    stop: None,
                    active_loop: Some(header),
                    context: &loop_context,
                },
            )?;
            let mut body = body;
            append_branch_to_target(
                &mut body,
                RelooperBranchTarget::LoopHeadedBy(loop_label),
                continue_payload.clone(),
            );
            (
                wrap_loop_exits(
                    &io,
                    RelooperStmt::Loop {
                        label: loop_label,
                        lowering: RelooperLoopLowering {
                            kind: StructuredLoopKind::HeaderControlled,
                            header: header_block,
                            backedge_sources: backedge_sources
                                .iter()
                                .map(|source| cfg.hugr_node(*source))
                                .collect(),
                            continue_edges: vec![StructuredLoopEdge {
                                source: header.into_structured_cfg_node(),
                                case: continue_case,
                                payload: continue_payload,
                            }],
                        },
                        body: Box::new(RelooperStmt::Seq(body)),
                    },
                    &exit_targets,
                    &exit_continuations,
                    &exit_edges,
                )?,
                if multi_exit {
                    frame.stop
                } else {
                    Some(exit_targets[0])
                },
            )
        } else {
            let continue_edges = backedge_sources
                .iter()
                .copied()
                .map(|backedge_source| {
                    let latch_succs = self
                        .succs
                        .get(&backedge_source)
                        .cloned()
                        .unwrap_or_default();
                    let continue_case = latch_succs.iter().position(|succ| *succ == header).ok_or(
                        StructuralizationError::UnsupportedLoop {
                            reason: format!(
                                "loop latch {backedge_source} has no backedge to the header"
                            ),
                        },
                    )?;
                    let continue_payload = block_successor_payload(
                        cfg_view,
                        cfg.hugr_node(backedge_source),
                        continue_case,
                        "tail-controlled loop continue case is out of range",
                    )?;
                    Ok(StructuredLoopEdge {
                        source: backedge_source.into_structured_cfg_node(),
                        case: continue_case,
                        payload: continue_payload,
                    })
                })
                .collect::<Result<Vec<_>, StructuralizationError>>()?;
            let continue_payload = continue_edges[0].payload.clone();
            let exit_continuations = exit_variants
                .iter()
                .map(|(target, _)| {
                    if multi_exit {
                        let mut continuation = self.build_scope(
                            cfg_view,
                            cfg,
                            ScopeBuild {
                                start: *target,
                                scope: frame.scope,
                                stop: frame.stop,
                                active_loop: frame.active_loop,
                                context: frame.context,
                            },
                        )?;
                        if frame.scope.contains(target) {
                            let exit_payload = context_branch_target(frame.context)
                                .map(|target| branch_target_payload_row(cfg_view, target))
                                .transpose()?
                                .unwrap_or_else(|| io.outputs.clone());
                            append_exit_from_context(
                                &mut continuation,
                                frame.context,
                                exit_payload,
                            );
                        } else if !continuation.last().is_some_and(is_terminal_stmt) {
                            continuation.push(RelooperStmt::Return(io.outputs.clone()));
                        }
                        Ok(continuation)
                    } else {
                        Ok(Vec::new())
                    }
                })
                .collect::<Result<Vec<Vec<RelooperStmt>>, StructuralizationError>>()?;
            let exit_edges = exit_variants
                .iter()
                .map(|(_, edges)| edges.clone())
                .collect_vec();
            let exit_targets = exit_variants
                .iter()
                .map(|(target, _)| *target)
                .collect_vec();
            let loop_context = push_context(
                frame.context,
                RelooperContextFrame::LoopHeadedBy(loop_label),
            );
            let mut body = self.build_scope(
                cfg_view,
                cfg,
                ScopeBuild {
                    start: header,
                    scope: &loop_blocks,
                    stop: None,
                    active_loop: Some(header),
                    context: &loop_context,
                },
            )?;
            append_branch_to_target(
                &mut body,
                RelooperBranchTarget::LoopHeadedBy(loop_label),
                continue_payload.clone(),
            );
            (
                wrap_loop_exits(
                    &io,
                    RelooperStmt::Loop {
                        label: loop_label,
                        lowering: RelooperLoopLowering {
                            kind: StructuredLoopKind::TailControlled,
                            header: header_block,
                            backedge_sources: backedge_sources
                                .iter()
                                .map(|source| cfg.hugr_node(*source))
                                .collect(),
                            continue_edges: continue_edges.clone(),
                        },
                        body: Box::new(RelooperStmt::Seq(body)),
                    },
                    &exit_targets,
                    &exit_continuations,
                    &exit_edges,
                )?,
                if multi_exit {
                    frame.stop
                } else {
                    Some(exit_targets[0])
                },
            )
        };

        Ok((RelooperRegion { io, body }, next))
    }
}

/// Returns the input row expected by one explicit branch target in the
/// enclosing Beyond-Relooper context.
fn branch_target_payload_row<H: HugrView<Node = Node>>(
    cfg_view: &H,
    target: RelooperBranchTarget,
) -> Result<hugr::types::TypeRow, StructuralizationError> {
    let node = match target {
        RelooperBranchTarget::BlockFollowedBy(label)
        | RelooperBranchTarget::LoopHeadedBy(label) => match label {
            super::super::ast::RelooperLabel::Original(node)
            | super::super::ast::RelooperLabel::Duplicate { original: node, .. } => node,
        },
    };
    block_input_row(cfg_view, node)
}

/// Wraps a loop in labelled blocks that encode multi-exit continuations.
fn wrap_loop_exits<T>(
    region_io: &RegionIo,
    loop_stmt: RelooperStmt,
    exit_targets: &[T],
    continuations: &[Vec<RelooperStmt>],
    exit_edges: &[Vec<StructuredLoopEdge>],
) -> Result<RelooperStmt, StructuralizationError>
where
    T: IntoRelooperLabel,
{
    if exit_edges.is_empty() {
        return Ok(loop_stmt);
    }

    (0..exit_targets.len())
        .rev()
        .try_fold(loop_stmt, |inner, idx| {
            let target = exit_targets[idx];
            let continuation = continuations.get(idx).cloned().unwrap_or_default();
            let exit_edges = &exit_edges[idx];
            let mut items = Vec::with_capacity(1 + continuation.len());
            items.push(RelooperStmt::Block {
                label: target.into_relooper_label(),
                lowering: RelooperBlockLowering {
                    join_kind: StructuredBranchJoinKind::Deferred,
                    loop_exit_edges: exit_edges.clone(),
                },
                body: Box::new(RelooperStmt::Region(Box::new(RelooperRegion {
                    io: region_io.clone(),
                    body: inner,
                }))),
            });
            items.extend(continuation);
            Ok(RelooperStmt::Seq(items))
        })
}

/// Builds loop-exit selectors grouped by target and immediate payload row.
fn build_loop_exit_variants<T, H, C>(
    cfg_view: &H,
    cfg: &C,
    exit_edges: &[(T, T)],
    missing_edge_reason: &str,
    payload_reason: &str,
) -> Result<Vec<(T, Vec<StructuredLoopEdge>)>, StructuralizationError>
where
    T: IntoRelooperLabel + IntoStructuredCfgNode + Eq,
    H: HugrView<Node = Node>,
    C: CfgBlockMap<T>,
{
    let mut variants: Vec<(T, Vec<StructuredLoopEdge>)> = Vec::new();
    for &(src, target) in exit_edges {
        let source = cfg.hugr_node(src);
        let succs = cfg.successors(src).collect_vec();
        let break_case = succs.iter().position(|succ| *succ == target).ok_or(
            StructuralizationError::UnsupportedLoop {
                reason: missing_edge_reason.replace("{src}", &src.to_string()),
            },
        )?;
        let payload = block_successor_payload(cfg_view, source, break_case, payload_reason)?;
        let edge = StructuredLoopEdge {
            source: src.into_structured_cfg_node(),
            case: break_case,
            payload: payload.clone(),
        };
        if let Some((_, edges)) = variants.iter_mut().find(|(variant_target, edges)| {
            *variant_target == target && edges[0].payload == payload
        }) {
            edges.push(edge);
        } else {
            variants.push((target, vec![edge]));
        }
    }
    Ok(variants)
}

/// Returns the output row visible after a loop continuation reaches its stop.
fn loop_continuation_outputs<T, H, C>(
    cfg_view: &H,
    cfg: &C,
    stop: Option<T>,
) -> Result<hugr::types::TypeRow, StructuralizationError>
where
    T: HugrNode,
    H: HugrView<Node = Node>,
    C: CfgBlockMap<T>,
{
    match stop {
        Some(stop) => block_input_row(cfg_view, cfg.hugr_node(stop)),
        None => cfg_output_row(cfg_view),
    }
}
