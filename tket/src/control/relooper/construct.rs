//! CFG-to-AST construction for the Beyond-Relooper strategy.
//!
//! This module owns the control reconstruction step for Beyond Relooper. It
//! consumes deterministic CFG facts and produces the strategy-local AST used by
//! the lowering stage.

use std::collections::BTreeSet;

use hugr::core::HugrNode;
use hugr::types::Type;
use hugr::{HugrView, Node};
use itertools::Itertools;

use crate::control::cfg::{CfgFacts, CfgFactsError};
use crate::control::structuralize::shared::{
    analyze_block, block_input_row, block_successor_payload, cfg_input_row, cfg_output_row,
};
use crate::control::structuralize::{
    RegionIo, StructuralizationError, StructuredBlock, StructuredBranchJoinKind,
    StructuredLoopEdge, StructuredLoopKind,
};
use crate::control::{CfgBlockMap, IdentityCfgMap};

use super::ast::{RelooperBody, RelooperLoopExit, RelooperNode, RelooperRegion};

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
    build_cfg_ast_with_map(cfg_view, cfg)
}

/// Builds the Beyond-Relooper AST for one CFG-like graph view.
///
/// The graph view may introduce synthetic nodes during preprocessing, but it
/// must still map each graph node back to one original HUGR block so block
/// summaries and lowering metadata can be recovered from the immutable HUGR
/// view.
pub(super) fn build_cfg_ast_with_map<H, T, C>(
    cfg_view: &H,
    cfg: &C,
) -> Result<RelooperRegion, StructuralizationError>
where
    H: HugrView<Node = Node>,
    T: HugrNode,
    C: CfgBlockMap<T>,
{
    let info = CfgFacts::<T>::new(cfg.entry_node(), cfg)
        .map_err(|err| map_cfg_facts_error(cfg_view.entrypoint(), err))?;
    let body = RelooperBody::Sequence(info.build_scope(
        cfg_view,
        cfg,
        cfg.entry_node(),
        &info.scope,
        None,
        None,
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
    T: HugrNode,
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
        start: T,
        scope: &BTreeSet<T>,
        stop: Option<T>,
        active_loop: Option<T>,
    ) -> Result<Vec<RelooperNode>, StructuralizationError>
    where
        H: HugrView<Node = Node>,
        C: CfgBlockMap<T>,
    {
        let mut items = Vec::new();
        let mut current = Some(start);
        let mut seen = BTreeSet::new();

        while let Some(node) = current {
            if Some(node) == stop || !scope.contains(&node) {
                break;
            }
            if !seen.insert(node) {
                return Err(StructuralizationError::Relooper {
                    reason: format!("scope walk revisited node {node}"),
                });
            }

            if self.is_nested_loop_header(node, scope, active_loop) {
                let (region, next) =
                    self.build_loop_region(cfg_view, cfg, node, scope, stop, active_loop)?;
                items.push(RelooperNode::Region(Box::new(region)));
                current = next;
                continue;
            }

            let succs = self.scope_successors(node, scope, active_loop);
            if succs.len() > 1 {
                let (region, next) =
                    self.build_branch_region(cfg_view, cfg, node, scope, stop, active_loop)?;
                items.push(RelooperNode::Region(Box::new(region)));
                current = next;
                continue;
            }

            let block = analyze_block(cfg_view, cfg.hugr_node(node))?;
            let is_exit = matches!(block, StructuredBlock::Exit { .. });
            items.push(RelooperNode::Block(block));
            current = succs.into_iter().next();
            if is_exit {
                break;
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
        scope: &BTreeSet<T>,
        stop: Option<T>,
        active_loop: Option<T>,
    ) -> Result<(RelooperRegion, Option<T>), StructuralizationError>
    where
        H: HugrView<Node = Node>,
        C: CfgBlockMap<T>,
    {
        let split = analyze_block(cfg_view, cfg.hugr_node(split_node))?;
        let join_node = self
            .branch_join(split_node, scope, stop)
            .map_err(|reason| StructuralizationError::Relooper {
                reason: format!("branch at node {split_node} {reason}"),
            })?;
        let join = analyze_block(cfg_view, cfg.hugr_node(join_node))?;
        let arms = self
            .scope_successors(split_node, scope, active_loop)
            .into_iter()
            .map(|succ| self.build_scope(cfg_view, cfg, succ, scope, Some(join_node), active_loop))
            .collect::<Result<Vec<_>, _>>()?;
        let (join_kind, next) = branch_continuation(self, join_node, scope, active_loop);

        Ok((
            RelooperRegion {
                io: RegionIo {
                    inputs: split.inputs().clone(),
                    outputs: join.inputs().clone(),
                },
                body: RelooperBody::Branch {
                    split,
                    arms,
                    join,
                    join_kind,
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
        scope: &BTreeSet<T>,
        stop: Option<T>,
        active_loop: Option<T>,
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
            .intersection(scope)
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
        let backedge_source = self
            .backedges
            .get(&header)
            .into_iter()
            .flatten()
            .copied()
            .filter(|source| loop_blocks.contains(source))
            .exactly_one()
            .map_err(|_| StructuralizationError::Relooper {
                reason: format!("loop headed by {header} does not have a unique backedge source"),
            })?;
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
                loop_continuation_outputs(cfg_view, cfg, stop)?
            } else {
                block_input_row(cfg_view, cfg.hugr_node(exit_targets[0]))?
            },
        };

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
            let exits = exit_targets
                .iter()
                .copied()
                .map(|target| {
                    let edges = exit_edges
                        .iter()
                        .filter(|(_, dst)| *dst == target)
                        .map(|(src, _)| {
                            let source = cfg.hugr_node(*src);
                            let succs = self.succs.get(src).cloned().unwrap_or_default();
                            let break_case = succs.iter().position(|succ| *succ == target).ok_or(
                                StructuralizationError::UnsupportedLoop {
                                    reason: format!(
                                        "header-controlled loop exit source {src} has no edge to target"
                                    ),
                                },
                            )?;
                            let payload = block_successor_payload(
                                cfg_view,
                                source,
                                break_case,
                                "header-controlled loop exit case is out of range",
                            )?;
                            Ok(StructuredLoopEdge {
                                source,
                                case: break_case,
                                payload,
                            })
                        })
                        .collect::<Result<Vec<_>, StructuralizationError>>()?;
                    let outputs = edges.first().map(|edge| edge.payload.clone()).ok_or(
                        StructuralizationError::UnsupportedLoop {
                            reason: "header-controlled loop has no exit edges".into(),
                        },
                    )?;
                    Ok(RelooperLoopExit {
                        edges,
                        outputs,
                        continuation: if multi_exit {
                            self.build_scope(cfg_view, cfg, target, scope, stop, active_loop)?
                        } else {
                            Vec::new()
                        },
                    })
                })
                .collect::<Result<Vec<RelooperLoopExit>, StructuralizationError>>()?;
            let break_outputs = loop_break_outputs(&exits);
            let body = self.build_scope(
                cfg_view,
                cfg,
                continue_target,
                &loop_blocks,
                None,
                Some(header),
            )?;
            (
                RelooperBody::Loop {
                    kind: StructuredLoopKind::HeaderControlled,
                    header: header_block,
                    body,
                    backedge_source: cfg.hugr_node(backedge_source),
                    continue_edge: StructuredLoopEdge {
                        source: cfg.hugr_node(header),
                        case: continue_case,
                        payload: continue_payload,
                    },
                    exits,
                    break_outputs,
                },
                if multi_exit {
                    stop
                } else {
                    Some(exit_targets[0])
                },
            )
        } else {
            let latch_succs = self
                .succs
                .get(&backedge_source)
                .cloned()
                .unwrap_or_default();
            let continue_case = latch_succs.iter().position(|succ| *succ == header).ok_or(
                StructuralizationError::UnsupportedLoop {
                    reason: "loop latch has no backedge to the header".into(),
                },
            )?;
            let continue_payload = block_successor_payload(
                cfg_view,
                cfg.hugr_node(backedge_source),
                continue_case,
                "tail-controlled loop continue case is out of range",
            )?;
            let exits = exit_targets
                .iter()
                .copied()
                .map(|target| {
                    let edges = exit_edges
                        .iter()
                        .filter(|(_, dst)| *dst == target)
                        .map(|(src, _)| {
                            let source = cfg.hugr_node(*src);
                            let succs = self.succs.get(src).cloned().unwrap_or_default();
                            let break_case = succs.iter().position(|succ| *succ == target).ok_or(
                                StructuralizationError::UnsupportedLoop {
                                    reason: format!(
                                        "loop exit source {src} has no edge to the exit target"
                                    ),
                                },
                            )?;
                            let payload = block_successor_payload(
                                cfg_view,
                                source,
                                break_case,
                                "tail-controlled loop exit case is out of range",
                            )?;
                            Ok(StructuredLoopEdge {
                                source,
                                case: break_case,
                                payload,
                            })
                        })
                        .collect::<Result<Vec<_>, StructuralizationError>>()?;
                    let outputs = edges.first().map(|edge| edge.payload.clone()).ok_or(
                        StructuralizationError::UnsupportedLoop {
                            reason: "tail-controlled loop has no exit edges".into(),
                        },
                    )?;
                    Ok(RelooperLoopExit {
                        edges,
                        outputs,
                        continuation: if multi_exit {
                            self.build_scope(cfg_view, cfg, target, scope, stop, active_loop)?
                        } else {
                            Vec::new()
                        },
                    })
                })
                .collect::<Result<Vec<RelooperLoopExit>, StructuralizationError>>()?;
            let break_outputs = loop_break_outputs(&exits);
            let body = self.build_scope(cfg_view, cfg, header, &loop_blocks, None, Some(header))?;
            (
                RelooperBody::Loop {
                    kind: StructuredLoopKind::TailControlled,
                    header: header_block,
                    body,
                    backedge_source: cfg.hugr_node(backedge_source),
                    continue_edge: StructuredLoopEdge {
                        source: cfg.hugr_node(backedge_source),
                        case: continue_case,
                        payload: continue_payload,
                    },
                    exits,
                    break_outputs,
                },
                if multi_exit {
                    stop
                } else {
                    Some(exit_targets[0])
                },
            )
        };

        Ok((RelooperRegion { io, body }, next))
    }
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

/// Returns the immediate `TailLoop` break row for one analyzed exit set.
fn loop_break_outputs(exits: &[RelooperLoopExit]) -> hugr::types::TypeRow {
    match exits {
        [exit] => exit.outputs.clone(),
        _ => vec![Type::new_sum(
            exits.iter().map(|exit| exit.outputs.clone()).collect_vec(),
        )]
        .into(),
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
