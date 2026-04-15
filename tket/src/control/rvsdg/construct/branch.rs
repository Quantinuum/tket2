//! Branch (`gamma`) construction for the RVSDG structuralizer.

use std::collections::BTreeSet;

use hugr::core::HugrNode;
use hugr::{HugrView, Node};
use itertools::Itertools;

use crate::control::CfgBlockMap;
use crate::control::cfg::CfgFacts;
use crate::control::structuralize::IntoStructuredCfgNode;

use super::super::error::RvsdgBuildError;
use super::super::ir::{
    BlockNode, BranchJoinKind, GammaBranch, GammaEntryVar, GammaNode, GammaOutputVar, Region,
    RegionVar,
};
use super::RvsdgBuilder;

impl<'a, H: HugrView<Node = Node>> RvsdgBuilder<'a, H> {
    /// Structures one reducible branch into a `gamma` node.
    pub(super) fn build_gamma<T, C>(
        &mut self,
        split_node: T,
        scope: &BTreeSet<T>,
        stop: Option<T>,
        active_loop: Option<T>,
        info: &CfgFacts<T>,
        cfg: &C,
    ) -> Result<(GammaNode, Option<T>), RvsdgBuildError<Node>>
    where
        T: HugrNode + IntoStructuredCfgNode,
        C: CfgBlockMap<T>,
    {
        let split = self.build_block(split_node, cfg.hugr_node(split_node))?;
        let (inputs, sum_rows, outputs) = match &split {
            BlockNode::Dataflow {
                inputs,
                sum_rows,
                outputs,
                ..
            } => (inputs.clone(), sum_rows.clone(), outputs.clone()),
            _ => {
                return Err(RvsdgBuildError::ExpectedBlock {
                    node: cfg.hugr_node(split_node),
                });
            }
        };
        if sum_rows.is_empty() {
            return Err(RvsdgBuildError::ExpectedBlock {
                node: cfg.hugr_node(split_node),
            });
        }
        let join_node = info
            .branch_join(split_node, scope, stop, active_loop)
            .map_err(|reason| RvsdgBuildError::UnsupportedBranch {
                split: cfg.hugr_node(split_node),
                reason,
            })?;
        let join = self.build_block_with_linear_successor(
            join_node,
            cfg.hugr_node(join_node),
            info.scope_linear_successor_case(join_node, scope, active_loop),
        )?;
        let join_inputs = join.inputs().to_vec();

        let arms = info.scope_successor_cases(split_node, scope, active_loop);
        let mut branches = Vec::with_capacity(arms.len());
        for (case_idx, succ) in arms.iter().copied() {
            let branch_arguments = branch_arguments(&sum_rows, &outputs, case_idx);
            let branch_results = join_inputs
                .iter()
                .map(|var| self.fresh_var(var.ty.clone()))
                .collect_vec();
            let body = self.build_scope(succ, scope, Some(join_node), active_loop, info, cfg)?;
            branches.push(GammaBranch {
                case: case_idx,
                region: Region {
                    arguments: branch_arguments,
                    body,
                    results: branch_results,
                },
            });
        }

        let entry_vars = outputs
            .iter()
            .cloned()
            .enumerate()
            .map(|(idx, input)| GammaEntryVar {
                input,
                branch_arguments: branches
                    .iter()
                    .map(|branch| branch.region.arguments[sum_rows[0].len() + idx].clone())
                    .collect(),
            })
            .collect_vec();
        let outputs = join_inputs
            .iter()
            .cloned()
            .enumerate()
            .map(|(idx, output)| GammaOutputVar {
                branch_results: branches
                    .iter()
                    .map(|branch| branch.region.results[idx].clone())
                    .collect(),
                output,
            })
            .collect_vec();
        let (join_kind, next) = branch_continuation(info, join_node, scope, active_loop);

        Ok((
            GammaNode {
                inputs,
                split,
                entry_vars,
                match_rows: sum_rows,
                branches,
                outputs,
                join,
                join_kind,
            },
            next,
        ))
    }
}

/// Ordered shared branch arguments for one branch case.
pub(super) fn branch_arguments(
    match_rows: &[Vec<RegionVar>],
    entry_outputs: &[RegionVar],
    case_idx: usize,
) -> Vec<RegionVar> {
    match_rows[case_idx]
        .iter()
        .cloned()
        .chain(entry_outputs.iter().cloned())
        .collect()
}

/// Classifies how control should continue after a branch join.
pub(super) fn branch_continuation<T: HugrNode>(
    info: &CfgFacts<T>,
    node: T,
    scope: &BTreeSet<T>,
    active_loop: Option<T>,
) -> (BranchJoinKind, Option<T>) {
    let successors = info.scope_successors(node, scope, active_loop);
    match successors.as_slice() {
        [] => (BranchJoinKind::Inline, None),
        [next] => (BranchJoinKind::Inline, Some(*next)),
        _ => (BranchJoinKind::Deferred, Some(node)),
    }
}
