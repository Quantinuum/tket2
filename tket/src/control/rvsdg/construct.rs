//! Reducible-CFG to RVSDG construction.
//!
//! The builder uses shared CFG facts to structure a reducible CFG into nested
//! RVSDG regions with explicit `gamma` and `theta` nodes, ordered
//! arguments/results, and bundled control variables.

use std::collections::BTreeSet;

use hugr::core::HugrNode;
use hugr::ops::OpType;
use hugr::types::{Type, TypeRow};
use hugr::{HugrView, Node};
use itertools::Itertools;

use crate::control::CfgBlockMap;
use crate::control::cfg::{CfgFacts, CfgFactsError};

use super::error::RvsdgBuildError;
use super::ir::{
    BlockNode, BranchJoinKind, GammaEntryVar, GammaNode, GammaOutputVar, LoopKind, Region,
    RegionVar, Rvsdg, RvsdgNode, ThetaEdge, ThetaLoopVar, ThetaNode, VarId,
};

/// Builds an RVSDG for one reducible CFG.
///
/// # Errors
///
/// Returns an error when the CFG is irreducible, when a required branch/loop
/// fact cannot be derived, or when a node expected to be a CFG block is not a
/// dataflow/exit block.
/// Builds an RVSDG for one CFG-like graph view.
///
/// Preprocessing may later introduce synthetic graph nodes that still map back
/// to original HUGR blocks. This helper keeps the graph walk generic over that
/// node type while preserving the RVSDG's references to original block nodes.
pub(super) fn build_cfg_rvsdg_with_map<H, T, C>(
    cfg_view: &H,
    cfg: &C,
) -> Result<Rvsdg, RvsdgBuildError<Node>>
where
    H: HugrView<Node = Node>,
    T: HugrNode,
    C: CfgBlockMap<T>,
{
    let cfg_root = cfg_view.entrypoint();
    let info = CfgFacts::<T>::new(cfg.entry_node(), cfg)
        .map_err(|err| map_cfg_facts_error(cfg_root, err))?;
    let mut builder = RvsdgBuilder::new(cfg_view);
    let root = builder.build_root(cfg.entry_node(), &info.scope, &info, cfg)?;
    Ok(Rvsdg { root })
}

/// Stateful builder allocating deterministic RVSDG variables.
struct RvsdgBuilder<'a, H> {
    /// Immutable view of the CFG being structured.
    cfg_view: &'a H,
    /// Monotonic variable allocator.
    next_var: usize,
}

impl<'a, H: HugrView<Node = Node>> RvsdgBuilder<'a, H> {
    /// Creates a builder for one CFG.
    fn new(cfg_view: &'a H) -> Self {
        Self {
            cfg_view,
            next_var: 0,
        }
    }

    /// Allocates one ordered variable of the given type.
    fn fresh_var(&mut self, ty: Type) -> RegionVar {
        let id = VarId(self.next_var);
        self.next_var += 1;
        RegionVar { id, ty }
    }

    /// Allocates one ordered variable per element in a row.
    fn fresh_vars(&mut self, row: &TypeRow) -> Vec<RegionVar> {
        row.iter().cloned().map(|ty| self.fresh_var(ty)).collect()
    }

    /// Builds the root region for the whole CFG.
    fn build_root<T, C>(
        &mut self,
        start: T,
        scope: &BTreeSet<T>,
        info: &CfgFacts<T>,
        cfg: &C,
    ) -> Result<Region, RvsdgBuildError<Node>>
    where
        T: HugrNode,
        C: CfgBlockMap<T>,
    {
        let arguments = self.cfg_signature_inputs()?;
        let results = self.cfg_signature_outputs()?;
        let body = self.build_scope(start, scope, None, None, info, cfg)?;
        Ok(Region {
            arguments,
            body,
            results,
        })
    }

    /// Returns the ordered CFG signature inputs as region arguments.
    fn cfg_signature_inputs(&mut self) -> Result<Vec<RegionVar>, RvsdgBuildError<Node>> {
        let cfg = self
            .cfg_view
            .get_optype(self.cfg_view.entrypoint())
            .as_cfg()
            .ok_or(RvsdgBuildError::ExpectedBlock {
                node: self.cfg_view.entrypoint(),
            })?;
        Ok(self.fresh_vars(&cfg.signature.input))
    }

    /// Returns the ordered CFG signature outputs as region results.
    fn cfg_signature_outputs(&mut self) -> Result<Vec<RegionVar>, RvsdgBuildError<Node>> {
        let cfg = self
            .cfg_view
            .get_optype(self.cfg_view.entrypoint())
            .as_cfg()
            .ok_or(RvsdgBuildError::ExpectedBlock {
                node: self.cfg_view.entrypoint(),
            })?;
        Ok(self.fresh_vars(&cfg.signature.output))
    }

    /// Structures one linear scope until an explicit stop node is reached.
    fn build_scope<T, C>(
        &mut self,
        start: T,
        scope: &BTreeSet<T>,
        stop: Option<T>,
        active_loop: Option<T>,
        info: &CfgFacts<T>,
        cfg: &C,
    ) -> Result<Vec<RvsdgNode>, RvsdgBuildError<Node>>
    where
        T: HugrNode,
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
                return Err(RvsdgBuildError::MalformedScope {
                    start: cfg.hugr_node(start),
                    reason: format!("scope walk revisited node {node}"),
                });
            }

            if info.is_nested_loop_header(node, scope, active_loop) {
                let (theta, next) = self.build_theta(node, scope, info, cfg)?;
                items.push(RvsdgNode::Theta(Box::new(theta)));
                current = next;
                continue;
            }

            let succs = info.scope_successors(node, scope, active_loop);
            if succs.len() > 1 {
                let (gamma, next) = self.build_gamma(node, scope, stop, active_loop, info, cfg)?;
                items.push(RvsdgNode::Gamma(Box::new(gamma)));
                current = next;
                continue;
            }

            let block = self.build_block(cfg.hugr_node(node))?;
            let is_exit = matches!(block, BlockNode::Exit { .. });
            items.push(RvsdgNode::Block(block));
            current = succs.into_iter().next();
            if is_exit {
                break;
            }
        }

        Ok(items)
    }

    /// Converts one CFG block into a typed RVSDG leaf.
    fn build_block(&mut self, node: Node) -> Result<BlockNode, RvsdgBuildError<Node>> {
        match self.cfg_view.get_optype(node) {
            OpType::DataflowBlock(block) => Ok(BlockNode::Dataflow {
                node,
                inputs: self.fresh_vars(&block.inputs),
                sum_rows: block
                    .sum_rows
                    .iter()
                    .map(|row| self.fresh_vars(row))
                    .collect(),
                outputs: self.fresh_vars(&block.other_outputs),
            }),
            OpType::ExitBlock(exit) => Ok(BlockNode::Exit {
                node,
                inputs: self.fresh_vars(&exit.cfg_outputs),
            }),
            _ => Err(RvsdgBuildError::ExpectedBlock { node }),
        }
    }

    /// Structures one reducible branch into a `gamma` node.
    fn build_gamma<T, C>(
        &mut self,
        split_node: T,
        scope: &BTreeSet<T>,
        stop: Option<T>,
        active_loop: Option<T>,
        info: &CfgFacts<T>,
        cfg: &C,
    ) -> Result<(GammaNode, Option<T>), RvsdgBuildError<Node>>
    where
        T: HugrNode,
        C: CfgBlockMap<T>,
    {
        let split = self.build_block(cfg.hugr_node(split_node))?;
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
            .branch_join(split_node, scope, stop)
            .map_err(|reason| RvsdgBuildError::UnsupportedBranch {
                split: cfg.hugr_node(split_node),
                reason,
            })?;
        let join = self.build_block(cfg.hugr_node(join_node))?;
        let join_inputs = join.inputs().to_vec();

        let arms = info.scope_successors(split_node, scope, active_loop);
        let mut branches = Vec::with_capacity(arms.len());
        for (case_idx, succ) in arms.iter().copied().enumerate() {
            let branch_arguments = branch_arguments(&sum_rows, &outputs, case_idx);
            let branch_results = join_inputs
                .iter()
                .map(|var| self.fresh_var(var.ty.clone()))
                .collect_vec();
            let body = self.build_scope(succ, scope, Some(join_node), active_loop, info, cfg)?;
            branches.push(Region {
                arguments: branch_arguments,
                body,
                results: branch_results,
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
                    .map(|branch| branch.arguments[sum_rows[0].len() + idx].clone())
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
                    .map(|branch| branch.results[idx].clone())
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

    /// Structures one reducible loop into a `theta` node.
    fn build_theta<T, C>(
        &mut self,
        header: T,
        scope: &BTreeSet<T>,
        info: &CfgFacts<T>,
        cfg: &C,
    ) -> Result<(ThetaNode, Option<T>), RvsdgBuildError<Node>>
    where
        T: HugrNode,
        C: CfgBlockMap<T>,
    {
        let loop_blocks = info
            .loop_blocks
            .get(&header)
            .ok_or_else(|| RvsdgBuildError::UnsupportedLoop {
                header: cfg.hugr_node(header),
                reason: "missing loop block set".into(),
            })?
            .intersection(scope)
            .copied()
            .collect::<BTreeSet<_>>();
        let exit_edges = loop_blocks
            .iter()
            .copied()
            .flat_map(|src| {
                info.succs
                    .get(&src)
                    .into_iter()
                    .flatten()
                    .copied()
                    .map(move |dst| (src, dst))
            })
            .filter(|(src, dst)| {
                !loop_blocks.contains(dst) && !info.is_loop_backedge(*src, *dst, header)
            })
            .collect_vec();
        let exit_target = exit_edges
            .iter()
            .map(|(_, dst)| *dst)
            .dedup()
            .exactly_one()
            .map_err(|_| RvsdgBuildError::UnsupportedLoop {
                header: cfg.hugr_node(header),
                reason: "loop does not have a unique exit target".into(),
            })?;
        let backedge_source = info
            .backedges
            .get(&header)
            .into_iter()
            .flatten()
            .copied()
            .filter(|source| loop_blocks.contains(source))
            .exactly_one()
            .map_err(|_| RvsdgBuildError::UnsupportedLoop {
                header: cfg.hugr_node(header),
                reason: "loop does not have a unique backedge source".into(),
            })?;

        let header_block = self.build_block(cfg.hugr_node(header))?;
        let header_inputs = header_block.inputs().to_vec();
        let header_succs = info.succs.get(&header).cloned().unwrap_or_default();
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

        let (kind, continue_edge, break_edges, body_start) = if !out_of_loop_succs.is_empty() {
            let continue_target = in_loop_succs.into_iter().exactly_one().map_err(|_| {
                RvsdgBuildError::UnsupportedLoop {
                    header: cfg.hugr_node(header),
                    reason: "header-controlled loop must have one in-loop successor".into(),
                }
            })?;
            let break_target = out_of_loop_succs.into_iter().exactly_one().map_err(|_| {
                RvsdgBuildError::UnsupportedLoop {
                    header: cfg.hugr_node(header),
                    reason: "header-controlled loop must have one exit successor".into(),
                }
            })?;
            let continue_case = header_succs
                .iter()
                .position(|succ| *succ == continue_target)
                .ok_or_else(|| RvsdgBuildError::UnsupportedLoop {
                    header: cfg.hugr_node(header),
                    reason: "header-controlled loop is missing the continue edge".into(),
                })?;
            let break_case = header_succs
                .iter()
                .position(|succ| *succ == break_target)
                .ok_or_else(|| RvsdgBuildError::UnsupportedLoop {
                    header: cfg.hugr_node(header),
                    reason: "header-controlled loop is missing the break edge".into(),
                })?;
            let continue_payload = self.fresh_vars(
                &block_successor_row(self.cfg_view, cfg.hugr_node(header), continue_case).map_err(
                    |reason| RvsdgBuildError::UnsupportedLoop {
                        header: cfg.hugr_node(header),
                        reason,
                    },
                )?,
            );
            let break_payload = self.fresh_vars(
                &block_successor_row(self.cfg_view, cfg.hugr_node(header), break_case).map_err(
                    |reason| RvsdgBuildError::UnsupportedLoop {
                        header: cfg.hugr_node(header),
                        reason,
                    },
                )?,
            );
            (
                LoopKind::HeaderControlled,
                ThetaEdge {
                    source: cfg.hugr_node(header),
                    case: continue_case,
                    payload: continue_payload,
                },
                vec![ThetaEdge {
                    source: cfg.hugr_node(header),
                    case: break_case,
                    payload: break_payload,
                }],
                continue_target,
            )
        } else {
            let latch_succs = info
                .succs
                .get(&backedge_source)
                .cloned()
                .unwrap_or_default();
            let continue_case = latch_succs
                .iter()
                .position(|succ| *succ == header)
                .ok_or_else(|| RvsdgBuildError::UnsupportedLoop {
                    header: cfg.hugr_node(header),
                    reason: "loop latch is missing the backedge".into(),
                })?;
            let continue_payload = self.fresh_vars(
                &block_successor_row(self.cfg_view, cfg.hugr_node(backedge_source), continue_case)
                    .map_err(|reason| RvsdgBuildError::UnsupportedLoop {
                        header: cfg.hugr_node(header),
                        reason,
                    })?,
            );
            let break_edges = exit_edges
                .iter()
                .copied()
                .map(|(exit_source, _)| {
                    let succs = info.succs.get(&exit_source).cloned().unwrap_or_default();
                    let break_case = succs
                        .iter()
                        .position(|succ| *succ == exit_target)
                        .ok_or_else(|| RvsdgBuildError::UnsupportedLoop {
                            header: cfg.hugr_node(header),
                            reason: format!(
                                "loop exit source {} is missing the exit edge",
                                cfg.hugr_node(exit_source)
                            ),
                        })?;
                    let break_payload =
                        block_successor_row(self.cfg_view, cfg.hugr_node(exit_source), break_case)
                            .map_err(|reason| RvsdgBuildError::UnsupportedLoop {
                                header: cfg.hugr_node(header),
                                reason,
                            })?;
                    Ok(ThetaEdge {
                        source: cfg.hugr_node(exit_source),
                        case: break_case,
                        payload: self.fresh_vars(&break_payload),
                    })
                })
                .collect::<Result<Vec<_>, _>>()?;
            (
                LoopKind::TailControlled,
                ThetaEdge {
                    source: cfg.hugr_node(backedge_source),
                    case: continue_case,
                    payload: continue_payload,
                },
                break_edges,
                header,
            )
        };

        let loop_vars = continue_edge
            .payload
            .iter()
            .cloned()
            .map(|continue_ty| ThetaLoopVar {
                input: self.fresh_var(continue_ty.ty.clone()),
                pre: self.fresh_var(continue_ty.ty.clone()),
                post: self.fresh_var(continue_ty.ty),
            })
            .collect_vec();
        let break_outputs: TypeRow = break_edges
            .first()
            .map(|edge| {
                edge.payload
                    .iter()
                    .map(|var| var.ty.clone())
                    .collect::<Vec<_>>()
                    .into()
            })
            .ok_or_else(|| RvsdgBuildError::UnsupportedLoop {
                header: cfg.hugr_node(header),
                reason: "loop must have at least one break edge".into(),
            })?;
        let outputs = break_outputs
            .iter()
            .cloned()
            .map(|ty| self.fresh_var(ty))
            .collect_vec();
        let body = Region {
            arguments: loop_vars.iter().map(|var| var.pre.clone()).collect(),
            body: self.build_scope(body_start, &loop_blocks, None, Some(header), info, cfg)?,
            results: loop_vars.iter().map(|var| var.post.clone()).collect(),
        };

        Ok((
            ThetaNode {
                inputs: header_inputs,
                outputs,
                kind,
                header: header_block,
                body,
                backedge_source: cfg.hugr_node(backedge_source),
                continue_edge,
                break_edges,
                loop_vars,
            },
            Some(exit_target),
        ))
    }
}

/// Ordered shared branch arguments for one branch case.
fn branch_arguments(
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

/// Returns the typed payload row emitted along one successor edge.
fn block_successor_row<H: HugrView<Node = Node>>(
    cfg_view: &H,
    node: Node,
    case_idx: usize,
) -> Result<TypeRow, String> {
    cfg_view
        .get_optype(node)
        .as_dataflow_block()
        .ok_or_else(|| format!("node {node} is not a dataflow block"))?
        .successor_input(case_idx)
        .ok_or_else(|| format!("successor case {case_idx} is out of range for node {node}"))
}

/// Classifies how control should continue after a branch join.
fn branch_continuation<T: HugrNode>(
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

/// Maps shared CFG-facts failures into RVSDG construction errors.
fn map_cfg_facts_error<T: HugrNode>(
    cfg_root: Node,
    err: CfgFactsError<T>,
) -> RvsdgBuildError<Node> {
    match err {
        CfgFactsError::NoEntryExitPath => RvsdgBuildError::MalformedScope {
            start: cfg_root,
            reason: "cfg has no entry-to-exit path".into(),
        },
        CfgFactsError::ReachableNodesDoNotReachExit { dropped } => {
            RvsdgBuildError::MalformedScope {
                start: cfg_root,
                reason: format!(
                    "reachable nodes do not all reach the CFG exit: {:?}",
                    dropped
                ),
            }
        }
        CfgFactsError::Irreducible { entries, .. } => RvsdgBuildError::IrreducibleCfg {
            cfg: cfg_root,
            reason: format!("cyclic SCC has multiple entries: {:?}", entries),
        },
    }
}
