//! Branch-region construction for the Beyond-Relooper strategy.

use std::collections::BTreeSet;

use hugr::core::HugrNode;
use hugr::{HugrView, Node};

use crate::control::CfgBlockMap;
use crate::control::cfg::CfgFacts;
use crate::control::relooper::construct::context::{append_branch_to_target, push_context};
use crate::control::structuralize::{StructuralizationError, StructuredBranchJoinKind};

use super::{IntoRelooperLabel, ScopeFrame, analyze_block, analyze_block_with_linear_successor};
use crate::control::relooper::ast::{
    RelooperBlockLowering, RelooperBranchTarget, RelooperCaseArm, RelooperContextFrame,
    RelooperRegion, RelooperStmt,
};
use crate::control::structuralize::IntoStructuredCfgNode;
use crate::control::structuralize::RegionIo;

impl<T> CfgFacts<T>
where
    T: IntoRelooperLabel + IntoStructuredCfgNode,
{
    /// Structures one reducible branch region rooted at a CFG split.
    pub(super) fn build_branch_region<H, C>(
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
        let split = analyze_block(
            cfg_view,
            split_node.into_structured_cfg_node(),
            cfg.hugr_node(split_node),
        )?;
        let join_node = self
            .branch_join(split_node, frame.scope, frame.stop, frame.active_loop)
            .map_err(|reason| StructuralizationError::Relooper {
                reason: format!("branch at node {split_node} {reason}"),
            })?;
        let join = analyze_block_with_linear_successor(
            cfg_view,
            join_node.into_structured_cfg_node(),
            cfg.hugr_node(join_node),
            self.scope_linear_successor_case(join_node, frame.scope, frame.active_loop),
        )?;
        let arms = self
            .scope_successor_cases(split_node, frame.scope, frame.active_loop)
            .into_iter()
            .map(|(case_idx, succ)| {
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
                    super::ScopeBuild {
                        start: succ,
                        scope: frame.scope,
                        stop: Some(join_node),
                        active_loop: frame.active_loop,
                        context: &arm_context,
                    },
                )?;
                append_branch_to_target(
                    &mut arm,
                    RelooperBranchTarget::BlockFollowedBy(join_node.into_relooper_label()),
                    join.inputs().clone(),
                );
                Ok::<RelooperCaseArm, StructuralizationError>(RelooperCaseArm {
                    case: case_idx,
                    body: RelooperStmt::Seq(arm),
                })
            })
            .collect::<Result<Vec<_>, _>>()?;
        let (join_kind, next) = if frame.active_loop == Some(join_node) {
            (StructuredBranchJoinKind::Deferred, None)
        } else if frame.stop == Some(join_node) {
            (StructuredBranchJoinKind::Deferred, Some(join_node))
        } else {
            branch_continuation(self, join_node, frame.scope, frame.stop, frame.active_loop)
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
                    body: Box::new(RelooperStmt::Case { split, arms }),
                },
            },
            next,
        ))
    }
}

/// Classifies how control should continue after a branch join.
pub(super) fn branch_continuation<T: HugrNode>(
    info: &CfgFacts<T>,
    node: T,
    scope: &BTreeSet<T>,
    stop: Option<T>,
    active_loop: Option<T>,
) -> (StructuredBranchJoinKind, Option<T>) {
    if stop == Some(node) {
        return (StructuredBranchJoinKind::Deferred, Some(node));
    }
    let successors = info.scope_successors(node, scope, active_loop);
    match successors.as_slice() {
        [] => (StructuredBranchJoinKind::Inline, None),
        [next] => (StructuredBranchJoinKind::Inline, Some(*next)),
        _ => (StructuredBranchJoinKind::Deferred, Some(node)),
    }
}
