//! RVSDG-to-structured lowering for CFG structuralization.
//!
//! Structuralization rewrites CFGs through a HUGR-oriented region summary. This
//! module owns the RVSDG-specific translation from the private RVSDG nodes into
//! that structured summary so the strategy boundary stays within
//! `crate::control::rvsdg`.

use hugr::{HugrView, Node};

use crate::control::IdentityCfgMap;
use crate::control::structuralize::{
    RegionIo, StructuralizationError, StructuredBlock, StructuredBranchJoinKind, StructuredCaseArm,
    StructuredLoopEdge, StructuredLoopExit, StructuredLoopKind, StructuredNode, StructuredRegion,
    StructuredRegionBody,
};

use super::build_cfg_rvsdg;
use super::ir::{
    BlockNode, BranchJoinKind, GammaNode, LoopKind, Region, RvsdgNode, ThetaNode, vars_to_row,
};

/// Analyzes one CFG through the RVSDG strategy and lowers it into the shared
/// structuralization region form consumed by the rewrite pipeline.
pub(crate) fn analyze_cfg<H: HugrView<Node = Node>>(
    cfg_view: &H,
    cfg: &IdentityCfgMap<H>,
) -> Result<StructuredRegion, StructuralizationError> {
    let rvsdg = build_cfg_rvsdg(cfg_view, cfg).map_err(StructuralizationError::Rvsdg)?;
    lower_region(&rvsdg.root)
}

/// Lowers one RVSDG region into the shared structuralization region form.
fn lower_region(region: &Region) -> Result<StructuredRegion, StructuralizationError> {
    Ok(StructuredRegion {
        io: RegionIo {
            inputs: vars_to_row(&region.arguments),
            outputs: vars_to_row(&region.results),
        },
        body: StructuredRegionBody::Sequence(
            region
                .body
                .iter()
                .map(lower_node)
                .collect::<Result<Vec<_>, _>>()?,
        ),
    })
}

/// Lowers one RVSDG node into a shared structuralization node.
fn lower_node(node: &RvsdgNode) -> Result<StructuredNode, StructuralizationError> {
    match node {
        RvsdgNode::Block(block) => Ok(StructuredNode::Block(lower_block(block))),
        RvsdgNode::Gamma(gamma) => Ok(StructuredNode::Region(Box::new(lower_gamma(gamma)))),
        RvsdgNode::Theta(theta) => Ok(StructuredNode::Region(Box::new(lower_theta(theta)?))),
    }
}

/// Lowers one RVSDG block summary into the shared structural block summary.
fn lower_block(block: &BlockNode) -> StructuredBlock {
    match block {
        BlockNode::Dataflow {
            cfg_node,
            node,
            inputs,
            sum_rows,
            outputs,
        } => StructuredBlock::Dataflow {
            cfg_node: *cfg_node,
            node: *node,
            inputs: vars_to_row(inputs),
            sum_rows: sum_rows.iter().map(|row| vars_to_row(row)).collect(),
            outputs: vars_to_row(outputs),
        },
        BlockNode::Exit {
            cfg_node,
            node,
            inputs,
        } => StructuredBlock::Exit {
            cfg_node: *cfg_node,
            node: *node,
            inputs: vars_to_row(inputs),
        },
    }
}

/// Lowers one RVSDG `gamma` node into a structured branch region.
fn lower_gamma(gamma: &GammaNode) -> StructuredRegion {
    StructuredRegion {
        io: RegionIo {
            inputs: vars_to_row(&gamma.inputs),
            outputs: vars_to_row(
                &gamma
                    .outputs
                    .iter()
                    .map(|var| var.output.clone())
                    .collect::<Vec<_>>(),
            ),
        },
        body: StructuredRegionBody::Branch {
            split: lower_block(&gamma.split),
            arms: gamma
                .branches
                .iter()
                .map(|branch| {
                    Ok::<StructuredCaseArm, StructuralizationError>(StructuredCaseArm {
                        case: branch.case,
                        body: branch
                            .region
                            .body
                            .iter()
                            .map(lower_node)
                            .collect::<Result<Vec<_>, _>>()?,
                    })
                })
                .collect::<Result<Vec<_>, _>>()
                .expect("rvsdg branch lowering is infallible"),
            join: lower_block(&gamma.join),
            join_kind: match gamma.join_kind {
                BranchJoinKind::Inline => StructuredBranchJoinKind::Inline,
                BranchJoinKind::Deferred => StructuredBranchJoinKind::Deferred,
            },
        },
    }
}

/// Lowers one RVSDG `theta` node into a structured loop region.
fn lower_theta(theta: &ThetaNode) -> Result<StructuredRegion, StructuralizationError> {
    Ok(StructuredRegion {
        io: RegionIo {
            inputs: vars_to_row(&theta.inputs),
            outputs: vars_to_row(&theta.outputs),
        },
        body: StructuredRegionBody::Loop {
            kind: match theta.kind {
                LoopKind::TailControlled => StructuredLoopKind::TailControlled,
                LoopKind::HeaderControlled => StructuredLoopKind::HeaderControlled,
            },
            header: lower_block(&theta.header),
            body: theta
                .body
                .body
                .iter()
                .map(lower_node)
                .collect::<Result<Vec<_>, _>>()?,
            backedge_sources: vec![theta.backedge_source],
            continue_edges: vec![StructuredLoopEdge {
                source: theta.continue_edge.source,
                case: theta.continue_edge.case,
                payload: vars_to_row(&theta.continue_edge.payload),
            }],
            exits: theta
                .exits
                .iter()
                .map(|exit| StructuredLoopExit {
                    edges: exit
                        .edges
                        .iter()
                        .map(|edge| StructuredLoopEdge {
                            source: edge.source,
                            case: edge.case,
                            payload: vars_to_row(&edge.payload),
                        })
                        .collect(),
                    outputs: vars_to_row(&exit.outputs),
                    continuation: exit
                        .continuation
                        .body
                        .iter()
                        .map(lower_node)
                        .collect::<Result<Vec<_>, _>>()
                        .expect("rvsdg exit continuation lowering is infallible"),
                })
                .collect(),
            break_outputs: if theta.exits.len() == 1 {
                vars_to_row(&theta.exits[0].outputs)
            } else {
                vec![hugr::types::Type::new_sum(
                    theta
                        .exits
                        .iter()
                        .map(|exit| vars_to_row(&exit.outputs))
                        .collect::<Vec<_>>(),
                )]
                .into()
            },
        },
    })
}
