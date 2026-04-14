//! CFG-to-AST construction for the Beyond-Relooper strategy.
//!
//! This module owns the control reconstruction step for Beyond Relooper. It
//! consumes deterministic CFG facts and produces the strategy-local AST used by
//! the lowering stage.

use std::collections::BTreeSet;

use hugr::core::HugrNode;
use hugr::{HugrView, Node};
use itertools::Itertools;

use crate::control::cfg::{CfgFacts, CfgFactsError, PreprocessedNode};
use crate::control::structuralize::shared::{
    analyze_block, block_input_row, block_successor_payload, cfg_input_row, cfg_output_row,
};
use crate::control::structuralize::{
    RegionIo, StructuralizationError, StructuredBlock, StructuredBranchJoinKind,
    StructuredLoopEdge, StructuredLoopKind,
};
use crate::control::{CfgBlockMap, IdentityCfgMap};

use super::ast::{
    RelooperBlockLowering, RelooperContext, RelooperContextFrame, RelooperLabel,
    RelooperLoopLowering, RelooperRegion, RelooperStmt,
};

/// One recursive scope walk in the Beyond-Relooper constructor.
struct ScopeBuild<'a, T> {
    start: T,
    scope: &'a BTreeSet<T>,
    stop: Option<T>,
    active_loop: Option<T>,
    context: &'a RelooperContext,
}

/// Shared recursive environment for nested branch and loop construction.
struct ScopeFrame<'a, T> {
    scope: &'a BTreeSet<T>,
    stop: Option<T>,
    active_loop: Option<T>,
    context: &'a RelooperContext,
}

/// Converts one CFG-graph node into the label used by the Beyond-Relooper AST.
///
/// Labels follow CFG nodes so debugging the reconstructed control stays close
/// to the source graph. Preprocessing may duplicate nodes, so duplicated
/// entries carry their stable clone identifiers into the label.
pub(crate) trait IntoRelooperLabel: HugrNode {
    /// Returns the strategy-local label for this CFG node.
    fn into_relooper_label(self) -> RelooperLabel;
}

impl IntoRelooperLabel for Node {
    fn into_relooper_label(self) -> RelooperLabel {
        RelooperLabel::Original(self)
    }
}

impl IntoRelooperLabel for PreprocessedNode<Node> {
    fn into_relooper_label(self) -> RelooperLabel {
        match self {
            PreprocessedNode::Original(node) => RelooperLabel::Original(node),
            PreprocessedNode::Duplicate { original, clone_id } => {
                RelooperLabel::Duplicate { original, clone_id }
            }
        }
    }
}

#[cfg(test)]
/// Builds the Beyond-Relooper AST for one CFG.
///
/// # Errors
///
/// Returns an error when shared CFG facts cannot be computed, when a branch or
/// loop shape is outside the currently supported control families, or when a
/// required HUGR block summary cannot be derived.
pub(super) fn build_cfg_ast<H: HugrView<Node = Node>>(
    cfg_view: &H,
    cfg: &IdentityCfgMap<H>,
) -> Result<RelooperRegion, StructuralizationError> {
    build_cfg_program(cfg_view, cfg)
}

/// Builds the Beyond-Relooper AST for one CFG-like graph view.
///
/// The graph view may introduce synthetic nodes during preprocessing, but it
/// must still map each graph node back to one original HUGR block so block
/// summaries and lowering metadata can be recovered from the immutable HUGR
/// view.
pub(super) fn build_cfg_program<H: HugrView<Node = Node>>(
    cfg_view: &H,
    cfg: &IdentityCfgMap<H>,
) -> Result<RelooperRegion, StructuralizationError> {
    build_cfg_program_with_map(cfg_view, cfg)
}

/// Builds the Beyond-Relooper program bundle for one CFG-like graph view.
pub(super) fn build_cfg_program_with_map<H, T, C>(
    cfg_view: &H,
    cfg: &C,
) -> Result<RelooperRegion, StructuralizationError>
where
    H: HugrView<Node = Node>,
    T: IntoRelooperLabel,
    C: CfgBlockMap<T>,
{
    let info = CfgFacts::<T>::new(cfg.entry_node(), cfg)
        .map_err(|err| map_cfg_facts_error(cfg_view.entrypoint(), err))?;
    let body = RelooperStmt::Seq(info.build_scope(
        cfg_view,
        cfg,
        ScopeBuild {
            start: cfg.entry_node(),
            scope: &info.scope,
            stop: None,
            active_loop: None,
            context: &RelooperContext::new(),
        },
    )?);
    Ok(RelooperRegion {
        io: RegionIo {
            inputs: cfg_input_row(cfg_view)?,
            outputs: cfg_output_row(cfg_view)?,
        },
        body,
    })
}

impl<T> CfgFacts<T>
where
    T: IntoRelooperLabel,
{
    /// Structures one linear scope until it reaches an explicit stop node.
    ///
    /// The walker emits straight-line blocks, nested branch regions, and nested
    /// loop regions while preserving CFG successor order. Nested loops are
    /// carved out before generic branching so loop headers are always lowered as
    /// loops rather than accidental multi-way branches.
    fn build_scope<H, C>(
        &self,
        cfg_view: &H,
        cfg: &C,
        request: ScopeBuild<'_, T>,
    ) -> Result<Vec<RelooperStmt>, StructuralizationError>
    where
        H: HugrView<Node = Node>,
        C: CfgBlockMap<T>,
    {
        let mut items = Vec::new();
        let mut current = Some(request.start);
        let mut seen = BTreeSet::new();

        while let Some(node) = current {
            if Some(node) == request.stop || !request.scope.contains(&node) {
                break;
            }
            if !seen.insert(node) {
                return Err(StructuralizationError::Relooper {
                    reason: format!("scope walk revisited node {node}"),
                });
            }

            if self.is_nested_loop_header(node, request.scope, request.active_loop) {
                let (region, next) = self.build_loop_region(
                    cfg_view,
                    cfg,
                    node,
                    ScopeFrame {
                        scope: request.scope,
                        stop: request.stop,
                        active_loop: request.active_loop,
                        context: request.context,
                    },
                )?;
                items.push(RelooperStmt::Region(Box::new(region)));
                current = next;
                continue;
            }

            let succs = self.scope_successors(node, request.scope, request.active_loop);
            if succs.len() > 1 {
                let (region, next) = self.build_branch_region(
                    cfg_view,
                    cfg,
                    node,
                    ScopeFrame {
                        scope: request.scope,
                        stop: request.stop,
                        active_loop: request.active_loop,
                        context: request.context,
                    },
                )?;
                items.push(RelooperStmt::Region(Box::new(region)));
                current = next;
                continue;
            }

            let block = analyze_block(cfg_view, cfg.hugr_node(node))?;
            match block {
                StructuredBlock::Exit { inputs, .. } => {
                    items.push(RelooperStmt::Return(inputs));
                    break;
                }
                block => {
                    items.push(RelooperStmt::Exec(block));
                    current = succs.into_iter().next();
                }
            }
        }

        Ok(items)
    }

    /// Structures one reducible branch region rooted at a CFG split.
    fn build_branch_region<H, C>(
        &self,
        cfg_view: &H,
        cfg: &C,
        split_node: T,
        frame: ScopeFrame<'_, T>,
    ) -> Result<(RelooperRegion, Option<T>), StructuralizationError>
    where
        H: HugrView<Node = Node>,
        C: CfgBlockMap<T>,
    {
        let split = analyze_block(cfg_view, cfg.hugr_node(split_node))?;
        let join_node = self
            .branch_join(split_node, frame.scope, frame.stop)
            .map_err(|reason| StructuralizationError::Relooper {
                reason: format!("branch at node {split_node} {reason}"),
            })?;
        let join = analyze_block(cfg_view, cfg.hugr_node(join_node))?;
        let arms = self
            .scope_successors(split_node, frame.scope, frame.active_loop)
            .into_iter()
            .map(|succ| {
                let arm_context = push_context(
                    &push_context(
                        frame.context,
                        RelooperContextFrame::BlockFollowedBy(join_node.into_relooper_label()),
                    ),
                    RelooperContextFrame::Case,
                );
                let mut arm = self.build_scope(
                    cfg_view,
                    cfg,
                    ScopeBuild {
                        start: succ,
                        scope: frame.scope,
                        stop: Some(join_node),
                        active_loop: frame.active_loop,
                        context: &arm_context,
                    },
                )?;
                append_branch_to_label(
                    &mut arm,
                    join_node.into_relooper_label(),
                    join.inputs().clone(),
                );
                Ok::<Vec<RelooperStmt>, StructuralizationError>(arm)
            })
            .collect::<Result<Vec<_>, _>>()?;
        let (join_kind, next) = if frame.active_loop == Some(join_node) {
            (StructuredBranchJoinKind::Deferred, None)
        } else {
            branch_continuation(self, join_node, frame.scope, frame.active_loop)
        };

        Ok((
            RelooperRegion {
                io: RegionIo {
                    inputs: split.inputs().clone(),
                    outputs: join.inputs().clone(),
                },
                body: RelooperStmt::Block {
                    label: join_node.into_relooper_label(),
                    lowering: RelooperBlockLowering {
                        join_kind,
                        loop_exit_edges: Vec::new(),
                    },
                    body: Box::new(RelooperStmt::Case {
                        split,
                        arms: arms.into_iter().map(RelooperStmt::Seq).collect(),
                    }),
                },
            },
            next,
        ))
    }

    /// Structures one reducible loop rooted at its unique header.
    fn build_loop_region<H, C>(
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
        let exit_targets = exit_edges
            .iter()
            .map(|(_, dst)| *dst)
            .unique()
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
        let header_block = analyze_block(cfg_view, cfg.hugr_node(header))?;
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

        let multi_exit = exit_targets.len() > 1;
        let io = RegionIo {
            inputs: header_block.inputs().clone(),
            outputs: if multi_exit {
                loop_continuation_outputs(cfg_view, cfg, frame.stop)?
            } else {
                block_input_row(cfg_view, cfg.hugr_node(exit_targets[0]))?
            },
        };

        let loop_label = header.into_relooper_label();
        let (body, next) = if !out_of_loop_succs.is_empty() {
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
            let exit_continuations = exit_targets
                .iter()
                .copied()
                .map(|target| {
                    if multi_exit {
                        let mut continuation = self.build_scope(
                            cfg_view,
                            cfg,
                            ScopeBuild {
                                start: target,
                                scope: frame.scope,
                                stop: frame.stop,
                                active_loop: frame.active_loop,
                                context: frame.context,
                            },
                        )?;
                        append_exit_from_context(
                            &mut continuation,
                            frame.context,
                            io.outputs.clone(),
                        );
                        Ok(continuation)
                    } else {
                        Ok(Vec::new())
                    }
                })
                .collect::<Result<Vec<Vec<RelooperStmt>>, StructuralizationError>>()?;
            let exit_edges = build_loop_exit_plans(
                cfg_view,
                cfg,
                &exit_edges,
                &exit_targets,
                "header-controlled loop exit source {src} has no edge to target",
                "header-controlled loop exit case is out of range",
            )?;
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
            append_branch_to_label(&mut body, loop_label, continue_payload.clone());
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
                                source: cfg.hugr_node(header),
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
                        source: cfg.hugr_node(backedge_source),
                        case: continue_case,
                        payload: continue_payload,
                    })
                })
                .collect::<Result<Vec<_>, StructuralizationError>>()?;
            let continue_payload = continue_edges[0].payload.clone();
            let exit_continuations = exit_targets
                .iter()
                .copied()
                .map(|target| {
                    if multi_exit {
                        let mut continuation = self.build_scope(
                            cfg_view,
                            cfg,
                            ScopeBuild {
                                start: target,
                                scope: frame.scope,
                                stop: frame.stop,
                                active_loop: frame.active_loop,
                                context: frame.context,
                            },
                        )?;
                        append_exit_from_context(
                            &mut continuation,
                            frame.context,
                            io.outputs.clone(),
                        );
                        Ok(continuation)
                    } else {
                        Ok(Vec::new())
                    }
                })
                .collect::<Result<Vec<Vec<RelooperStmt>>, StructuralizationError>>()?;
            let exit_edges = build_loop_exit_plans(
                cfg_view,
                cfg,
                &exit_edges,
                &exit_targets,
                "loop exit source {src} has no edge to the exit target",
                "tail-controlled loop exit case is out of range",
            )?;
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
            append_branch_to_label(&mut body, loop_label, continue_payload.clone());
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

/// Wraps a loop in labelled blocks that encode multi-exit continuations.
///
/// The paper models exits as branches to enclosing labels. The current
/// constructor still computes typed exit summaries for the lowerer, but this
/// helper also reifies the continuation shape in the AST by nesting labelled
/// blocks around the loop and placing each exit continuation structurally after
/// the corresponding block.
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

/// Builds loop-exit selectors keyed by their target labels.
fn build_loop_exit_plans<T, H, C>(
    cfg_view: &H,
    cfg: &C,
    exit_edges: &[(T, T)],
    exit_targets: &[T],
    missing_edge_reason: &str,
    payload_reason: &str,
) -> Result<Vec<Vec<StructuredLoopEdge>>, StructuralizationError>
where
    T: IntoRelooperLabel,
    H: HugrView<Node = Node>,
    C: CfgBlockMap<T>,
{
    exit_targets
        .iter()
        .copied()
        .map(|target| {
            let edges = exit_edges
                .iter()
                .filter(|(_, dst)| *dst == target)
                .map(|(src, _)| {
                    let source = cfg.hugr_node(*src);
                    let succs = cfg.successors(*src).collect_vec();
                    let break_case = succs.iter().position(|succ| *succ == target).ok_or(
                        StructuralizationError::UnsupportedLoop {
                            reason: missing_edge_reason.replace("{src}", &src.to_string()),
                        },
                    )?;
                    let payload =
                        block_successor_payload(cfg_view, source, break_case, payload_reason)?;
                    Ok(StructuredLoopEdge {
                        source,
                        case: break_case,
                        payload,
                    })
                })
                .collect::<Result<Vec<_>, StructuralizationError>>()?;
            Ok(edges)
        })
        .collect()
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

/// Returns a new context with one innermost frame pushed on the front.
fn push_context(context: &RelooperContext, frame: RelooperContextFrame) -> RelooperContext {
    let mut nested = Vec::with_capacity(context.len() + 1);
    nested.push(frame);
    nested.extend(context.iter().copied());
    nested
}

/// Returns the innermost enclosing block-followed-by label, if any.
fn context_follow_label(context: &RelooperContext) -> Option<RelooperLabel> {
    context.iter().find_map(|frame| match frame {
        RelooperContextFrame::BlockFollowedBy(label) => Some(*label),
        _ => None,
    })
}

/// Appends an explicit branch to the target label unless the sequence already terminates.
fn append_branch_to_label(
    items: &mut Vec<RelooperStmt>,
    target: RelooperLabel,
    _payload: hugr::types::TypeRow,
) {
    if items.last().is_some_and(is_terminal_stmt) {
        return;
    }
    items.push(RelooperStmt::Br(target));
}

/// Appends the explicit exit selected by the active Beyond-Relooper context.
///
/// The paper phrases non-local control in terms of exits interpreted relative
/// to the enclosing context. The current implementation keeps labels explicit,
/// so this helper chooses the innermost block-followed-by target when one is
/// available and otherwise returns from the enclosing region.
fn append_exit_from_context(
    items: &mut Vec<RelooperStmt>,
    context: &RelooperContext,
    payload: hugr::types::TypeRow,
) {
    if items.last().is_some_and(is_terminal_stmt) {
        return;
    }
    match context_follow_label(context) {
        Some(label) => append_branch_to_label(items, label, payload),
        None => items.push(RelooperStmt::Return(payload)),
    }
}

/// Returns whether a statement already terminates control flow in its sequence.
fn is_terminal_stmt(stmt: &RelooperStmt) -> bool {
    match stmt {
        RelooperStmt::Br(_) | RelooperStmt::Return(_) => true,
        RelooperStmt::Seq(items) => items.last().is_some_and(is_terminal_stmt),
        RelooperStmt::Region(region) => is_terminal_stmt(&region.body),
        _ => false,
    }
}

/// Classifies how control should continue after a branch join.
fn branch_continuation<T: HugrNode>(
    info: &CfgFacts<T>,
    node: T,
    scope: &BTreeSet<T>,
    active_loop: Option<T>,
) -> (StructuredBranchJoinKind, Option<T>) {
    let successors = info.scope_successors(node, scope, active_loop);
    match successors.as_slice() {
        [] => (StructuredBranchJoinKind::Inline, None),
        [next] => (StructuredBranchJoinKind::Inline, Some(*next)),
        _ => (StructuredBranchJoinKind::Deferred, Some(node)),
    }
}

/// Maps shared CFG-facts failures into Beyond-Relooper analysis errors.
fn map_cfg_facts_error<T: HugrNode>(
    cfg_root: Node,
    err: CfgFactsError<T>,
) -> StructuralizationError {
    match err {
        CfgFactsError::NoEntryExitPath => StructuralizationError::Relooper {
            reason: "cfg has no entry-to-exit path".into(),
        },
        CfgFactsError::ReachableNodesDoNotReachExit { dropped } => {
            StructuralizationError::Relooper {
                reason: format!(
                    "reachable nodes do not all reach the CFG exit: {:?}",
                    dropped
                ),
            }
        }
        CfgFactsError::Irreducible { entries, .. } => {
            StructuralizationError::UnsupportedIrreducibleCfg {
                cfg: cfg_root,
                reason: format!("cyclic SCC has multiple entries: {:?}", entries),
            }
        }
    }
}
