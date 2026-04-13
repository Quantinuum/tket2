//! CFG-to-AST construction for the Beyond-Relooper strategy.
//!
//! This module owns the control reconstruction step for Beyond Relooper. It
//! consumes deterministic CFG facts and produces the strategy-local AST used by
//! the lowering stage.

use std::collections::BTreeSet;

use hugr::{HugrView, Node};
use itertools::Itertools;

use crate::control::cfg::{CfgFacts, CfgFactsError};
use crate::control::structuralize::shared::{
    analyze_block, block_input_row, block_successor_payload, cfg_input_row, cfg_output_row,
};
use crate::control::structuralize::{
    RegionIo, StructuralizationError, StructuredBlock, StructuredBranchJoinKind, StructuredLoopKind,
};
use crate::control::{CfgNodeMap, IdentityCfgMap};

use super::ast::{RelooperBody, RelooperNode, RelooperRegion};

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
    let info = CfgFacts::new(cfg_view.entrypoint(), cfg)
        .map_err(|err| map_cfg_facts_error(cfg_view.entrypoint(), err))?;
    let body = RelooperBody::Sequence(info.build_scope(
        cfg_view,
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

impl CfgFacts {
    /// Structures one linear scope until it reaches an explicit stop node.
    ///
    /// The walker emits straight-line blocks, nested branch regions, and nested
    /// loop regions while preserving CFG successor order. Nested loops are
    /// carved out before generic branching so loop headers are always lowered as
    /// loops rather than accidental multi-way branches.
    fn build_scope<H: HugrView<Node = Node>>(
        &self,
        cfg_view: &H,
        start: Node,
        scope: &BTreeSet<Node>,
        stop: Option<Node>,
        active_loop: Option<Node>,
    ) -> Result<Vec<RelooperNode>, StructuralizationError> {
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
                let (region, next) = self.build_loop_region(cfg_view, node, scope)?;
                items.push(RelooperNode::Region(Box::new(region)));
                current = next;
                continue;
            }

            let succs = self.scope_successors(node, scope, active_loop);
            if succs.len() > 1 {
                let (region, next) =
                    self.build_branch_region(cfg_view, node, scope, stop, active_loop)?;
                items.push(RelooperNode::Region(Box::new(region)));
                current = next;
                continue;
            }

            let block = analyze_block(cfg_view, node)?;
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
    fn build_branch_region<H: HugrView<Node = Node>>(
        &self,
        cfg_view: &H,
        split_node: Node,
        scope: &BTreeSet<Node>,
        stop: Option<Node>,
        active_loop: Option<Node>,
    ) -> Result<(RelooperRegion, Option<Node>), StructuralizationError> {
        let split = analyze_block(cfg_view, split_node)?;
        let join_node = self
            .branch_join(split_node, scope, stop)
            .map_err(|reason| StructuralizationError::Relooper {
                reason: format!("branch at node {split_node} {reason}"),
            })?;
        let join = analyze_block(cfg_view, join_node)?;
        let arms = self
            .scope_successors(split_node, scope, active_loop)
            .into_iter()
            .map(|succ| self.build_scope(cfg_view, succ, scope, Some(join_node), active_loop))
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
    fn build_loop_region<H: HugrView<Node = Node>>(
        &self,
        cfg_view: &H,
        header: Node,
        scope: &BTreeSet<Node>,
    ) -> Result<(RelooperRegion, Option<Node>), StructuralizationError> {
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
        let exit_target = exit_edges
            .iter()
            .map(|(_, dst)| *dst)
            .dedup()
            .exactly_one()
            .map_err(|_| StructuralizationError::Relooper {
                reason: format!("loop headed by {header} does not have a unique exit target"),
            })?;
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
        let header_block = analyze_block(cfg_view, header)?;
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

        let io = RegionIo {
            inputs: header_block.inputs().clone(),
            outputs: block_input_row(cfg_view, exit_target)?,
        };

        let body = if !out_of_loop_succs.is_empty() {
            let continue_target = in_loop_succs.into_iter().exactly_one().map_err(|_| {
                StructuralizationError::UnsupportedLoop {
                    reason: "header-controlled loop must have exactly one in-loop successor".into(),
                }
            })?;
            let break_target = out_of_loop_succs.into_iter().exactly_one().map_err(|_| {
                StructuralizationError::UnsupportedLoop {
                    reason: "header-controlled loop must have exactly one exit successor".into(),
                }
            })?;
            let continue_case = header_succs
                .iter()
                .position(|succ| *succ == continue_target)
                .ok_or(StructuralizationError::UnsupportedLoop {
                    reason: "header-controlled loop continue edge is missing".into(),
                })?;
            let break_case = header_succs
                .iter()
                .position(|succ| *succ == break_target)
                .ok_or(StructuralizationError::UnsupportedLoop {
                    reason: "header-controlled loop exit edge is missing".into(),
                })?;
            let continue_inputs = block_successor_payload(
                cfg_view,
                header,
                continue_case,
                "header-controlled loop continue case is out of range",
            )?;
            let break_outputs = block_successor_payload(
                cfg_view,
                header,
                break_case,
                "header-controlled loop exit case is out of range",
            )?;
            let body =
                self.build_scope(cfg_view, continue_target, &loop_blocks, None, Some(header))?;
            RelooperBody::Loop {
                kind: StructuredLoopKind::HeaderControlled,
                header: header_block,
                body,
                backedge_source,
                continue_inputs,
                break_outputs,
                continue_case,
                break_case,
            }
        } else {
            let exit_source = exit_edges
                .iter()
                .map(|(src, _)| *src)
                .dedup()
                .exactly_one()
                .map_err(|_| StructuralizationError::UnsupportedLoop {
                    reason: "tail-controlled loop must have exactly one exit source".into(),
                })?;
            let latch_succs = self.succs.get(&exit_source).cloned().unwrap_or_default();
            let continue_case = latch_succs.iter().position(|succ| *succ == header).ok_or(
                StructuralizationError::UnsupportedLoop {
                    reason: "loop latch has no backedge to the header".into(),
                },
            )?;
            let break_case = latch_succs
                .iter()
                .position(|succ| *succ == exit_target)
                .ok_or(StructuralizationError::UnsupportedLoop {
                    reason: "loop latch has no exit edge".into(),
                })?;
            let continue_inputs = block_successor_payload(
                cfg_view,
                exit_source,
                continue_case,
                "tail-controlled loop continue case is out of range",
            )?;
            let break_outputs = block_successor_payload(
                cfg_view,
                exit_source,
                break_case,
                "tail-controlled loop exit case is out of range",
            )?;
            let body = self.build_scope(cfg_view, header, &loop_blocks, None, Some(header))?;
            RelooperBody::Loop {
                kind: StructuredLoopKind::TailControlled,
                header: header_block,
                body,
                backedge_source,
                continue_inputs,
                break_outputs,
                continue_case,
                break_case,
            }
        };

        Ok((RelooperRegion { io, body }, Some(exit_target)))
    }
}

/// Classifies how control should continue after a branch join.
fn branch_continuation(
    info: &CfgFacts,
    node: Node,
    scope: &BTreeSet<Node>,
    active_loop: Option<Node>,
) -> (StructuredBranchJoinKind, Option<Node>) {
    let successors = info.scope_successors(node, scope, active_loop);
    match successors.as_slice() {
        [] => (StructuredBranchJoinKind::Inline, None),
        [next] => (StructuredBranchJoinKind::Inline, Some(*next)),
        _ => (StructuredBranchJoinKind::Deferred, Some(node)),
    }
}

/// Maps shared CFG-facts failures into Beyond-Relooper analysis errors.
fn map_cfg_facts_error(cfg_root: Node, err: CfgFactsError) -> StructuralizationError {
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
