//! RVSDG-backed structural analysis for CFG structuralization.
//!
//! This module owns the strategy-specific step that turns the generic
//! [`crate::control::rvsdg`] control tree into the shared HUGR-oriented
//! structural IR used by lowering. Keeping this translation here keeps the
//! shared `analyze` entry points free of RVSDG-specific region and loop logic.

use hugr::types::TypeRow;
use hugr::{HugrView, Node};
use itertools::Itertools;

use crate::control::rvsdg as control_rvsdg;
use crate::control::{CfgNodeMap, IdentityCfgMap};

use super::shared::{
    analyze_block, block_input_row, block_successor_payload, cfg_input_row, cfg_output_row,
};
use super::types::{
    RegionIo, StructuralizationError, StructuredBlock, StructuredLoopKind, StructuredNode,
    StructuredRegion, StructuredRegionBody, structured_node_contains_block,
};

/// Analyzes one CFG through the RVSDG structural tree and enriches the result
/// with HUGR-specific lowering metadata.
pub(super) fn analyze_cfg<H: HugrView<Node = Node>>(
    cfg_view: &H,
    cfg: &IdentityCfgMap<H>,
) -> Result<StructuredRegion, StructuralizationError> {
    let region = control_rvsdg::build_control_tree(cfg).map_err(StructuralizationError::Rvsdg)?;
    analyze_region(cfg_view, cfg, region)
}

/// Converts one RVSDG region into a HUGR-aware structured region.
///
/// The result preserves the generic control tree while attaching block typing
/// information and the lowering metadata needed by `lower`.
fn analyze_region<H: HugrView<Node = Node>>(
    cfg_view: &H,
    cfg: &IdentityCfgMap<H>,
    region: control_rvsdg::ControlRegion<Node>,
) -> Result<StructuredRegion, StructuralizationError> {
    let io = RegionIo {
        inputs: region_input_row(cfg_view, &region)?,
        outputs: region_output_row(cfg_view, &region)?,
    };

    let body = match &region.body {
        control_rvsdg::ControlRegionBody::Sequence(items) => StructuredRegionBody::Sequence(
            items
                .iter()
                .cloned()
                .map(|item| analyze_node(cfg_view, cfg, item))
                .collect::<Result<Vec<_>, _>>()?,
        ),
        control_rvsdg::ControlRegionBody::Branch { split, arms, join } => {
            let [split] = split.as_slice() else {
                return Err(StructuralizationError::UnsupportedBranch {
                    reason: format!("expected exactly one split block, found {}", split.len()),
                });
            };
            let [join] = join.as_slice() else {
                return Err(StructuralizationError::UnsupportedBranch {
                    reason: format!("expected exactly one join block, found {}", join.len()),
                });
            };
            StructuredRegionBody::Branch {
                split: analyze_block(cfg_view, *split)?,
                arms: arms
                    .iter()
                    .map(|arm| {
                        arm.iter()
                            .cloned()
                            .map(|item| analyze_node(cfg_view, cfg, item))
                            .collect::<Result<Vec<_>, _>>()
                    })
                    .collect::<Result<Vec<_>, _>>()?,
                join: analyze_block(cfg_view, *join)?,
            }
        }
        control_rvsdg::ControlRegionBody::Loop { body } => {
            let structured_body = body
                .iter()
                .cloned()
                .map(|item| analyze_node(cfg_view, cfg, item))
                .collect::<Result<Vec<_>, _>>()?;

            let header = region
                .boundary
                .incoming
                .first()
                .map(|(_, dst)| *dst)
                .ok_or_else(|| StructuralizationError::UnsupportedLoop {
                    reason: "loop has no entry boundary".into(),
                })?;
            let header_block = analyze_block(cfg_view, header)?;
            let exit_target = region
                .boundary
                .outgoing
                .first()
                .map(|(_, dst)| *dst)
                .ok_or_else(|| StructuralizationError::UnsupportedLoop {
                    reason: "loop has no exit boundary".into(),
                })?;
            let exit_source = region
                .boundary
                .outgoing
                .first()
                .map(|(src, _)| *src)
                .ok_or_else(|| StructuralizationError::UnsupportedLoop {
                    reason: "loop has no exit boundary".into(),
                })?;
            let backedge_source = unique_backedge_source(cfg, &region, header)?;

            let loop_body = analyze_loop_body(
                cfg_view,
                cfg,
                LoopAnalysisInput {
                    region: &region,
                    boundary: LoopBoundary {
                        header,
                        backedge_source,
                        exit_source,
                        exit_target,
                    },
                    header: header_block,
                    body: structured_body,
                    io: &io,
                },
            )?;

            StructuredRegionBody::Loop {
                kind: loop_body.kind,
                header: loop_body.header,
                body: loop_body.body,
                backedge_source: loop_body.backedge_source,
                continue_inputs: loop_body.continue_inputs,
                break_outputs: loop_body.break_outputs,
                continue_case: loop_body.continue_case,
                break_case: loop_body.break_case,
            }
        }
    };

    Ok(StructuredRegion { io, body })
}

/// Converts a single RVSDG control-tree node into the shared structured form.
fn analyze_node<H: HugrView<Node = Node>>(
    cfg_view: &H,
    cfg: &IdentityCfgMap<H>,
    node: control_rvsdg::ControlTree<Node>,
) -> Result<StructuredNode, StructuralizationError> {
    match node {
        control_rvsdg::ControlTree::Block(block) => {
            Ok(StructuredNode::Block(analyze_block(cfg_view, block)?))
        }
        control_rvsdg::ControlTree::Region(region) => Ok(StructuredNode::Region(Box::new(
            analyze_region(cfg_view, cfg, *region)?,
        ))),
    }
}

/// Final lowering-oriented summary for a loop region.
///
/// This is an analysis-only helper used to normalize both tail-controlled and
/// header-controlled loops into one shape before lowering stores the data in
/// `StructuredRegionBody::Loop`.
struct LoopAnalysis {
    kind: StructuredLoopKind,
    header: StructuredBlock,
    body: Vec<StructuredNode>,
    backedge_source: Node,
    continue_inputs: TypeRow,
    break_outputs: TypeRow,
    continue_case: usize,
    break_case: usize,
}

/// Concrete CFG edges that define one loop's control boundary.
///
/// Grouping these nodes keeps the loop-shape analysis readable and avoids
/// threading raw node tuples through every helper.
#[derive(Clone, Copy)]
struct LoopBoundary {
    header: Node,
    backedge_source: Node,
    exit_source: Node,
    exit_target: Node,
}

/// All HUGR-specific inputs required to classify a loop region.
///
/// The loop analyzer combines region topology, typed block summaries, and
/// computed region I/O, so this struct keeps the handoff explicit.
struct LoopAnalysisInput<'a> {
    region: &'a control_rvsdg::ControlRegion<Node>,
    boundary: LoopBoundary,
    header: StructuredBlock,
    body: Vec<StructuredNode>,
    io: &'a RegionIo,
}

/// Finds the unique in-region predecessor that acts as the loop backedge.
///
/// The current structuralizer only supports reducible loops with a single
/// logical backedge source.
fn unique_backedge_source<H: HugrView<Node = Node>>(
    cfg: &IdentityCfgMap<H>,
    region: &control_rvsdg::ControlRegion<Node>,
    header: Node,
) -> Result<Node, StructuralizationError> {
    cfg.predecessors(header)
        .filter(|pred| region.blocks.contains(pred))
        .exactly_one()
        .map_err(|_| StructuralizationError::UnsupportedLoop {
            reason: "loop does not have a unique in-region backedge to the header".into(),
        })
}

/// Classifies a loop region into the specific lowering shape we support.
///
/// This function is where generic RVSDG region information becomes HUGR-specific
/// lowering metadata. It validates the loop invariants we rely on and reports
/// targeted errors when a reducible region still does not match a supported
/// loop family.
fn analyze_loop_body<H: HugrView<Node = Node>>(
    cfg_view: &H,
    cfg: &IdentityCfgMap<H>,
    input: LoopAnalysisInput<'_>,
) -> Result<LoopAnalysis, StructuralizationError> {
    if input.body.is_empty() {
        return Err(StructuralizationError::UnsupportedLoop {
            reason: "loop body is empty".into(),
        });
    }

    if input.boundary.backedge_source == input.boundary.exit_source {
        let structured_tail =
            input
                .body
                .last()
                .ok_or_else(|| StructuralizationError::UnsupportedLoop {
                    reason: "loop body is empty".into(),
                })?;
        let StructuredNode::Block(StructuredBlock::Dataflow {
            node: tail_node, ..
        }) = structured_tail
        else {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "tail-controlled loop body does not end in a basic block".into(),
            });
        };
        if *tail_node != input.boundary.backedge_source {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "tail-controlled loop body does not end at the backedge source".into(),
            });
        }

        let successors = cfg.successors(input.boundary.backedge_source).collect_vec();
        let continue_case = successors
            .iter()
            .position(|&succ| succ == input.boundary.header)
            .ok_or_else(|| StructuralizationError::UnsupportedLoop {
                reason: "loop latch has no backedge to the header".into(),
            })?;
        let break_case = successors
            .iter()
            .position(|&succ| succ == input.boundary.exit_target)
            .ok_or_else(|| StructuralizationError::UnsupportedLoop {
                reason: "loop latch has no exit edge".into(),
            })?;
        let continue_inputs = block_successor_payload(
            cfg_view,
            input.boundary.backedge_source,
            continue_case,
            "loop continue case is out of range",
        )
        .map_err(|err| match err {
            StructuralizationError::Relooper { reason } => {
                StructuralizationError::UnsupportedLoop { reason }
            }
            other => other,
        })?;
        let break_outputs = block_successor_payload(
            cfg_view,
            input.boundary.backedge_source,
            break_case,
            "loop break case is out of range",
        )
        .map_err(|err| match err {
            StructuralizationError::Relooper { reason } => {
                StructuralizationError::UnsupportedLoop { reason }
            }
            other => other,
        })?;

        if continue_inputs != input.io.inputs {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "loop continue payload does not match the loop entry row".into(),
            });
        }
        if break_outputs != input.io.outputs {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "loop break payload does not match the loop exit row".into(),
            });
        }

        return Ok(LoopAnalysis {
            kind: StructuredLoopKind::TailControlled,
            header: input.header,
            body: input.body,
            backedge_source: input.boundary.backedge_source,
            continue_inputs,
            break_outputs,
            continue_case,
            break_case,
        });
    }

    if input.boundary.exit_source != input.boundary.header {
        return Err(StructuralizationError::UnsupportedLoop {
            reason: "header-controlled loops must exit directly from the header".into(),
        });
    }

    let StructuredNode::Block(StructuredBlock::Dataflow {
        node: first_node,
        sum_rows,
        outputs,
        ..
    }) = input
        .body
        .first()
        .ok_or_else(|| StructuralizationError::UnsupportedLoop {
            reason: "loop body is empty".into(),
        })?
    else {
        return Err(StructuralizationError::UnsupportedLoop {
            reason: "header-controlled loop header is not a basic block".into(),
        });
    };
    if *first_node != input.boundary.header {
        return Err(StructuralizationError::UnsupportedLoop {
            reason: "header-controlled loop body does not begin at the header".into(),
        });
    }

    let successors = cfg.successors(input.boundary.header).collect_vec();
    let continue_case = successors
        .iter()
        .position(|succ| input.region.blocks.contains(succ) && *succ != input.boundary.header)
        .ok_or_else(|| StructuralizationError::UnsupportedLoop {
            reason: "header-controlled loop header has no in-region continue edge".into(),
        })?;
    let break_case = successors
        .iter()
        .position(|&succ| succ == input.boundary.exit_target)
        .ok_or_else(|| StructuralizationError::UnsupportedLoop {
            reason: "header-controlled loop header has no exit edge".into(),
        })?;
    let continue_inputs = block_successor_payload(
        cfg_view,
        input.boundary.header,
        continue_case,
        "header-controlled loop continue case is out of range",
    )
    .map_err(|err| match err {
        StructuralizationError::Relooper { reason } => {
            StructuralizationError::UnsupportedLoop { reason }
        }
        other => other,
    })?;
    let break_outputs = block_successor_payload(
        cfg_view,
        input.boundary.header,
        break_case,
        "header-controlled loop break case is out of range",
    )
    .map_err(|err| match err {
        StructuralizationError::Relooper { reason } => {
            StructuralizationError::UnsupportedLoop { reason }
        }
        other => other,
    })?;

    if break_outputs != input.io.outputs {
        return Err(StructuralizationError::UnsupportedLoop {
            reason: "loop break payload does not match the loop exit row".into(),
        });
    }
    if sum_rows.len() != 2 {
        return Err(StructuralizationError::UnsupportedLoop {
            reason: format!(
                "header-controlled loop header must have exactly two successors, found {}",
                sum_rows.len()
            ),
        });
    }
    if outputs != &continue_inputs || continue_inputs != break_outputs {
        return Err(StructuralizationError::UnsupportedLoop {
            reason:
                "header-controlled loop requires identical continue, break, and header output rows"
                    .into(),
        });
    }
    if !input
        .body
        .iter()
        .skip(1)
        .any(|node| structured_node_contains_block(node, input.boundary.backedge_source))
    {
        return Err(StructuralizationError::UnsupportedLoop {
            reason: "header-controlled loop body does not contain the backedge source".into(),
        });
    }

    Ok(LoopAnalysis {
        kind: StructuredLoopKind::HeaderControlled,
        header: input.header,
        body: input.body.into_iter().skip(1).collect(),
        backedge_source: input.boundary.backedge_source,
        continue_inputs,
        break_outputs,
        continue_case,
        break_case,
    })
}

/// Computes the ordered live-in row for a structured region.
///
/// For nested regions, this is derived from the entry boundary block. For the
/// root region, it comes directly from the CFG signature.
fn region_input_row<H: HugrView<Node = Node>>(
    cfg_view: &H,
    region: &control_rvsdg::ControlRegion<Node>,
) -> Result<TypeRow, StructuralizationError> {
    if region.kind == control_rvsdg::RegionKind::Root {
        return cfg_input_row(cfg_view);
    }

    let entry_block = region
        .boundary
        .incoming
        .first()
        .map(|(_, dst)| *dst)
        .ok_or_else(|| StructuralizationError::UnsupportedBranch {
            reason: "region has no entry boundary".into(),
        })?;
    block_input_row(cfg_view, entry_block)
}

/// Computes the ordered live-out row for a structured region.
///
/// For nested regions, this is derived from the exit boundary target. For the
/// root region, it comes directly from the CFG signature.
fn region_output_row<H: HugrView<Node = Node>>(
    cfg_view: &H,
    region: &control_rvsdg::ControlRegion<Node>,
) -> Result<TypeRow, StructuralizationError> {
    if region.kind == control_rvsdg::RegionKind::Root {
        return cfg_output_row(cfg_view);
    }

    let exit_target = region
        .boundary
        .outgoing
        .first()
        .map(|(_, dst)| *dst)
        .ok_or_else(|| StructuralizationError::UnsupportedBranch {
            reason: "region has no exit boundary".into(),
        })?;
    block_input_row(cfg_view, exit_target).map_err(|err| match err {
        StructuralizationError::ExpectedDataflowBlock { node } => {
            StructuralizationError::ExpectedExitBlock { node }
        }
        other => other,
    })
}
