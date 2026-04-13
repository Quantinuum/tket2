//! RVSDG-backed structural analysis for CFG structuralization.
//!
//! The shared lowerer consumes a private structured IR built around linear
//! blocks, branches, and loops. This module translates the RVSDG produced by
//! [`crate::control::rvsdg`] into that shared lowering-oriented representation.

use hugr::{HugrView, Node};

use crate::control::IdentityCfgMap;
use crate::control::rvsdg as control_rvsdg;

use super::types::{
    RegionIo, StructuralizationError, StructuredBlock, StructuredBranchJoinKind,
    StructuredLoopKind, StructuredNode, StructuredRegion, StructuredRegionBody,
};

/// Analyzes one CFG through the RVSDG strategy and translates it into the
/// shared lowering-oriented structural IR.
pub(super) fn analyze_cfg<H: HugrView<Node = Node>>(
    cfg_view: &H,
    cfg: &IdentityCfgMap<H>,
) -> Result<StructuredRegion, StructuralizationError> {
    let rvsdg =
        control_rvsdg::build_cfg_rvsdg(cfg_view, cfg).map_err(StructuralizationError::Rvsdg)?;
    translate_region(&rvsdg.root)
}

/// Translates one RVSDG region into the shared structured region form.
fn translate_region(
    region: &control_rvsdg::Region,
) -> Result<StructuredRegion, StructuralizationError> {
    Ok(StructuredRegion {
        io: RegionIo {
            inputs: control_rvsdg::vars_to_row(&region.arguments),
            outputs: control_rvsdg::vars_to_row(&region.results),
        },
        body: StructuredRegionBody::Sequence(
            region
                .body
                .iter()
                .map(translate_node)
                .collect::<Result<Vec<_>, _>>()?,
        ),
    })
}

/// Translates one RVSDG body node into the shared structured node form.
fn translate_node(
    node: &control_rvsdg::RvsdgNode,
) -> Result<StructuredNode, StructuralizationError> {
    match node {
        control_rvsdg::RvsdgNode::Block(block) => Ok(StructuredNode::Block(translate_block(block))),
        control_rvsdg::RvsdgNode::Gamma(gamma) => {
            Ok(StructuredNode::Region(Box::new(translate_gamma(gamma))))
        }
        control_rvsdg::RvsdgNode::Theta(theta) => {
            Ok(StructuredNode::Region(Box::new(translate_theta(theta)?)))
        }
    }
}

/// Translates one RVSDG block summary into the shared lowering block summary.
fn translate_block(block: &control_rvsdg::BlockNode) -> StructuredBlock {
    match block {
        control_rvsdg::BlockNode::Dataflow {
            node,
            inputs,
            sum_rows,
            outputs,
        } => StructuredBlock::Dataflow {
            node: *node,
            inputs: control_rvsdg::vars_to_row(inputs),
            sum_rows: sum_rows
                .iter()
                .map(|row| control_rvsdg::vars_to_row(row))
                .collect(),
            outputs: control_rvsdg::vars_to_row(outputs),
        },
        control_rvsdg::BlockNode::Exit { node, inputs } => StructuredBlock::Exit {
            node: *node,
            inputs: control_rvsdg::vars_to_row(inputs),
        },
    }
}

/// Translates one RVSDG `gamma` node into a shared branch region.
fn translate_gamma(gamma: &control_rvsdg::GammaNode) -> StructuredRegion {
    StructuredRegion {
        io: RegionIo {
            inputs: control_rvsdg::vars_to_row(&gamma.inputs),
            outputs: control_rvsdg::vars_to_row(
                &gamma
                    .outputs
                    .iter()
                    .map(|var| var.output.clone())
                    .collect::<Vec<_>>(),
            ),
        },
        body: StructuredRegionBody::Branch {
            split: translate_block(&gamma.split),
            arms: gamma
                .branches
                .iter()
                .map(|branch| {
                    branch
                        .body
                        .iter()
                        .map(translate_node)
                        .collect::<Result<Vec<_>, _>>()
                })
                .collect::<Result<Vec<_>, _>>()
                .expect("rvsdg branch translation is infallible"),
            join: translate_block(&gamma.join),
            join_kind: match gamma.join_kind {
                control_rvsdg::BranchJoinKind::Inline => StructuredBranchJoinKind::Inline,
                control_rvsdg::BranchJoinKind::Deferred => StructuredBranchJoinKind::Deferred,
            },
        },
    }
}

/// Translates one RVSDG `theta` node into a shared loop region.
fn translate_theta(
    theta: &control_rvsdg::ThetaNode,
) -> Result<StructuredRegion, StructuralizationError> {
    Ok(StructuredRegion {
        io: RegionIo {
            inputs: control_rvsdg::vars_to_row(&theta.inputs),
            outputs: control_rvsdg::vars_to_row(&theta.outputs),
        },
        body: StructuredRegionBody::Loop {
            kind: match theta.kind {
                control_rvsdg::LoopKind::TailControlled => StructuredLoopKind::TailControlled,
                control_rvsdg::LoopKind::HeaderControlled => StructuredLoopKind::HeaderControlled,
            },
            header: translate_block(&theta.header),
            body: theta
                .body
                .body
                .iter()
                .map(translate_node)
                .collect::<Result<Vec<_>, _>>()?,
            backedge_source: theta.backedge_source,
            continue_inputs: control_rvsdg::vars_to_row(
                &theta
                    .loop_vars
                    .iter()
                    .map(|var| var.post.clone())
                    .collect::<Vec<_>>(),
            ),
            break_outputs: control_rvsdg::vars_to_row(&theta.outputs),
            continue_case: theta.continue_case,
            break_case: theta.break_case,
        },
    })
}
