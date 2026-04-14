//! RVSDG-to-structured lowering for CFG structuralization.
//!
//! Structuralization rewrites CFGs through a HUGR-oriented region summary. This
//! module owns the RVSDG-specific translation from the private RVSDG nodes into
//! that structured summary so the strategy boundary stays within
//! `crate::control::rvsdg`.

use hugr::{HugrView, Node};

use crate::control::IdentityCfgMap;
use crate::control::structuralize::{
    RegionIo, StructuralizationError, StructuredBlock, StructuredBranchJoinKind,
    StructuredLoopEdge, StructuredLoopKind, StructuredNode, StructuredRegion, StructuredRegionBody,
};

use super::{
    BlockNode, BranchJoinKind, GammaNode, LoopKind, Region, RvsdgNode, ThetaNode, build_cfg_rvsdg,
    vars_to_row,
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
            node,
            inputs,
            sum_rows,
            outputs,
        } => StructuredBlock::Dataflow {
            node: *node,
            inputs: vars_to_row(inputs),
            sum_rows: sum_rows.iter().map(|row| vars_to_row(row)).collect(),
            outputs: vars_to_row(outputs),
        },
        BlockNode::Exit { node, inputs } => StructuredBlock::Exit {
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
                    branch
                        .body
                        .iter()
                        .map(lower_node)
                        .collect::<Result<Vec<_>, _>>()
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
            backedge_source: theta.backedge_source,
            continue_edge: StructuredLoopEdge {
                source: theta.continue_edge.source,
                case: theta.continue_edge.case,
                payload: vars_to_row(&theta.continue_edge.payload),
            },
            break_edges: theta
                .break_edges
                .iter()
                .map(|edge| StructuredLoopEdge {
                    source: edge.source,
                    case: edge.case,
                    payload: vars_to_row(&edge.payload),
                })
                .collect(),
            break_outputs: vars_to_row(&theta.outputs),
        },
    })
}
