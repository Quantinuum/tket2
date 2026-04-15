//! Tail loop (`theta`) construction for the RVSDG structuralizer.

use std::collections::BTreeSet;

use hugr::core::HugrNode;
use hugr::ops::OpType;
use hugr::types::TypeRow;
use hugr::{HugrView, Node};
use itertools::Itertools;

use crate::control::CfgBlockMap;
use crate::control::cfg::CfgFacts;
use crate::control::structuralize::IntoStructuredCfgNode;

use super::super::error::RvsdgBuildError;
use super::super::ir::{
    LoopKind, Region, RegionVar, ThetaEdge, ThetaExit, ThetaLoopVar, ThetaNode,
};
use super::RvsdgBuilder;

impl<'a, H: HugrView<Node = Node>> RvsdgBuilder<'a, H> {
    /// Structures one reducible loop into a `theta` node.
    pub(super) fn build_theta<T, C>(
        &mut self,
        header: T,
        scope: &BTreeSet<T>,
        stop: Option<T>,
        active_loop: Option<T>,
        info: &CfgFacts<T>,
        cfg: &C,
    ) -> Result<(ThetaNode, Option<T>), RvsdgBuildError<Node>>
    where
        T: HugrNode + IntoStructuredCfgNode,
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

        let header_block = self.build_block(header, cfg.hugr_node(header))?;
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

        let exit_variants =
            build_theta_exit_variants(self, cfg, info, cfg.hugr_node(header), &exit_edges)?;
        let multi_exit = exit_variants.len() > 1;
        let (kind, continue_edge, exits, body_start, next) = if !out_of_loop_succs.is_empty() {
            let continue_target = in_loop_succs.into_iter().exactly_one().map_err(|_| {
                RvsdgBuildError::UnsupportedLoop {
                    header: cfg.hugr_node(header),
                    reason: "header-controlled loop must have one in-loop successor".into(),
                }
            })?;
            let continue_case = header_succs
                .iter()
                .position(|succ| *succ == continue_target)
                .ok_or_else(|| RvsdgBuildError::UnsupportedLoop {
                    header: cfg.hugr_node(header),
                    reason: "header-controlled loop is missing the continue edge".into(),
                })?;
            let continue_payload = self.fresh_vars(
                &block_successor_row(self.cfg_view, cfg.hugr_node(header), continue_case).map_err(
                    |reason| RvsdgBuildError::UnsupportedLoop {
                        header: cfg.hugr_node(header),
                        reason,
                    },
                )?,
            );
            let exits = exit_variants
                .iter()
                .map(|(target, edges)| {
                    let target = *target;
                    let outputs =
                        edges
                            .first()
                            .map(|edge| edge.payload.clone())
                            .ok_or_else(|| RvsdgBuildError::UnsupportedLoop {
                                header: cfg.hugr_node(header),
                                reason: "loop must have at least one break edge".into(),
                            })?;
                    let continuation = if multi_exit {
                        let final_results = loop_continuation_results(
                            self,
                            cfg_view_output_row(self.cfg_view, cfg, stop)?,
                        );
                        Region {
                            arguments: outputs.clone(),
                            body: self.build_scope(target, scope, stop, active_loop, info, cfg)?,
                            results: final_results,
                        }
                    } else {
                        Region {
                            arguments: outputs.clone(),
                            body: Vec::new(),
                            results: outputs.clone(),
                        }
                    };
                    Ok(ThetaExit {
                        edges: edges.clone(),
                        outputs,
                        continuation,
                    })
                })
                .collect::<Result<Vec<_>, _>>()?;
            (
                LoopKind::HeaderControlled,
                ThetaEdge {
                    source: header.into_structured_cfg_node(),
                    case: continue_case,
                    payload: continue_payload,
                },
                exits,
                continue_target,
                if multi_exit {
                    stop
                } else {
                    Some(exit_variants[0].0)
                },
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
            let exits = exit_variants
                .iter()
                .map(|(target, edges)| {
                    let target = *target;
                    let outputs =
                        edges
                            .first()
                            .map(|edge| edge.payload.clone())
                            .ok_or_else(|| RvsdgBuildError::UnsupportedLoop {
                                header: cfg.hugr_node(header),
                                reason: "loop must have at least one break edge".into(),
                            })?;
                    let continuation = if multi_exit {
                        let final_results = loop_continuation_results(
                            self,
                            cfg_view_output_row(self.cfg_view, cfg, stop)?,
                        );
                        Region {
                            arguments: outputs.clone(),
                            body: self.build_scope(target, scope, stop, active_loop, info, cfg)?,
                            results: final_results,
                        }
                    } else {
                        Region {
                            arguments: outputs.clone(),
                            body: Vec::new(),
                            results: outputs.clone(),
                        }
                    };
                    Ok(ThetaExit {
                        edges: edges.clone(),
                        outputs,
                        continuation,
                    })
                })
                .collect::<Result<Vec<_>, _>>()?;
            (
                LoopKind::TailControlled,
                ThetaEdge {
                    source: backedge_source.into_structured_cfg_node(),
                    case: continue_case,
                    payload: continue_payload,
                },
                exits,
                header,
                if multi_exit {
                    stop
                } else {
                    Some(exit_variants[0].0)
                },
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
        let outputs = if multi_exit {
            cfg_view_output_row(self.cfg_view, cfg, stop)?
                .iter()
                .cloned()
                .map(|ty| self.fresh_var(ty))
                .collect_vec()
        } else {
            exits[0].outputs.clone()
        };
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
                exits,
                loop_vars,
            },
            next,
        ))
    }
}

/// Returns the final output row visible after a loop continuation reaches stop.
pub(super) fn cfg_view_output_row<H, T, C>(
    cfg_view: &H,
    cfg: &C,
    stop: Option<T>,
) -> Result<TypeRow, RvsdgBuildError<Node>>
where
    H: HugrView<Node = Node>,
    T: HugrNode + IntoStructuredCfgNode,
    C: CfgBlockMap<T>,
{
    match stop {
        Some(stop) => match cfg_view.get_optype(cfg.hugr_node(stop)) {
            OpType::DataflowBlock(block) => Ok(block.inputs.clone()),
            OpType::ExitBlock(exit) => Ok(exit.cfg_outputs.clone()),
            _ => Err(RvsdgBuildError::ExpectedBlock {
                node: cfg.hugr_node(stop),
            }),
        },
        None => Ok(cfg_view
            .get_optype(cfg_view.entrypoint())
            .as_cfg()
            .ok_or(RvsdgBuildError::ExpectedBlock {
                node: cfg_view.entrypoint(),
            })?
            .signature
            .output
            .clone()),
    }
}

/// Allocates ordered RVSDG variables for one loop-continuation output row.
pub(super) fn loop_continuation_results<H: HugrView<Node = Node>>(
    builder: &mut RvsdgBuilder<'_, H>,
    row: TypeRow,
) -> Vec<RegionVar> {
    builder.fresh_vars(&row)
}

/// Builds deterministic loop-exit variants grouped by target and payload row.
pub(super) fn build_theta_exit_variants<H, T, C>(
    builder: &mut RvsdgBuilder<'_, H>,
    cfg: &C,
    info: &CfgFacts<T>,
    header: Node,
    exit_edges: &[(T, T)],
) -> Result<Vec<(T, Vec<ThetaEdge>)>, RvsdgBuildError<Node>>
where
    H: HugrView<Node = Node>,
    T: HugrNode + IntoStructuredCfgNode + Eq,
    C: CfgBlockMap<T>,
{
    let mut variants: Vec<(T, Vec<ThetaEdge>)> = Vec::new();
    for &(exit_source, target) in exit_edges {
        let succs = info.succs.get(&exit_source).cloned().unwrap_or_default();
        let break_case = succs
            .iter()
            .position(|succ| *succ == target)
            .ok_or_else(|| RvsdgBuildError::UnsupportedLoop {
                header,
                reason: format!(
                    "loop exit source {} is missing the exit edge",
                    cfg.hugr_node(exit_source)
                ),
            })?;
        let break_payload =
            block_successor_row(builder.cfg_view, cfg.hugr_node(exit_source), break_case)
                .map_err(|reason| RvsdgBuildError::UnsupportedLoop { header, reason })?;
        let edge = ThetaEdge {
            source: exit_source.into_structured_cfg_node(),
            case: break_case,
            payload: builder.fresh_vars(&break_payload),
        };
        if let Some((_, edges)) = variants.iter_mut().find(|(variant_target, edges)| {
            *variant_target == target && edges[0].payload == edge.payload
        }) {
            edges.push(edge);
        } else {
            variants.push((target, vec![edge]));
        }
    }
    Ok(variants)
}

/// Returns the typed payload row emitted along one successor edge.
pub(super) fn block_successor_row<H: HugrView<Node = Node>>(
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
