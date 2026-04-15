//! Lowering from the Beyond-Relooper AST into a detached rewrite template.
//!
//! Beyond Relooper now prepares rewrites from its own control AST directly.
//! The current lowerer still reuses the shared detached-template builder, but
//! loop-exit continuations and selectors are recovered from the AST-wrapped
//! labelled blocks rather than from a side table external to the AST.

use hugr::{HugrView, Node};

use crate::control::structuralize::lower::{LoweredCfgTemplate, prepare_cfg_replacement};
use crate::control::structuralize::{
    StructuralizationError, StructuredBlock, StructuredCaseArm, StructuredLoopEdge,
    StructuredLoopExit, StructuredNode, StructuredRegion, StructuredRegionBody,
};

use super::ast::{RelooperLabel, RelooperRegion, RelooperStmt};
use super::block::analyze_block;

/// Lowered loop-exit continuations in wrapped-block order.
type ExitContinuations = Vec<Vec<StructuredNode>>;
/// Raw AST loop-exit continuations in wrapped-block order.
type RawExitContinuations<'a> = Vec<(Vec<StructuredLoopEdge>, &'a [RelooperStmt])>;

/// One explicit non-local exit propagated while lowering a sequence.
#[derive(Clone, Debug, PartialEq, Eq)]
enum LoweredExit {
    /// Branch to an enclosing labelled construct.
    Branch(RelooperLabel),
    /// Return from the enclosing region with the given payload row.
    Return(hugr::types::TypeRow),
}

/// Lowered straight-line items plus an optional propagated explicit exit.
#[derive(Clone, Debug, PartialEq, Eq)]
struct LoweredSequence {
    /// Lowered structured nodes emitted before the explicit exit.
    nodes: Vec<StructuredNode>,
    /// Explicit exit that still needs to be interpreted by an enclosing scope.
    exit: Option<LoweredExit>,
}

/// Builds one detached rewrite template from a Beyond-Relooper AST.
///
/// # Errors
///
/// Returns an error when the AST cannot be translated into the current HUGR
/// rewrite template.
pub(super) fn prepare_cfg_rewrite<H: HugrView<Node = Node>>(
    cfg_view: &H,
    region: &RelooperRegion,
) -> Result<LoweredCfgTemplate, StructuralizationError> {
    let region = lower_region(cfg_view, region)?;
    prepare_cfg_replacement(cfg_view, &region)
}

/// Lowers one Beyond-Relooper region into the shared structuralization region.
pub(super) fn lower_region<H: HugrView<Node = Node>>(
    cfg_view: &H,
    region: &RelooperRegion,
) -> Result<StructuredRegion, StructuralizationError> {
    Ok(StructuredRegion {
        io: region.io.clone(),
        body: lower_stmt_as_body(cfg_view, &region.body)?,
    })
}

/// Lowers one top-level statement into the shared structuralization body.
fn lower_stmt_as_body<H: HugrView<Node = Node>>(
    cfg_view: &H,
    stmt: &RelooperStmt,
) -> Result<StructuredRegionBody, StructuralizationError> {
    Ok(match stmt {
        RelooperStmt::Seq(_) => StructuredRegionBody::Sequence(require_closed_sequence(
            lower_stmt_as_sequence_in_context(cfg_view, stmt, None)?,
            "top-level sequence",
        )?),
        RelooperStmt::Region(region) => return Ok(lower_region(cfg_view, region)?.body),
        RelooperStmt::Block {
            label,
            lowering,
            body,
        } => match body.as_ref() {
            RelooperStmt::Case { split, arms } => StructuredRegionBody::Branch {
                split: split.clone(),
                arms: arms
                    .iter()
                    .map(|arm| {
                        Ok::<StructuredCaseArm, StructuralizationError>(StructuredCaseArm {
                            case: arm.case,
                            body: lower_stmt_as_sequence_in_context(
                                cfg_view,
                                &arm.body,
                                Some(*label),
                            )?
                            .nodes,
                        })
                    })
                    .collect::<Result<Vec<_>, _>>()?,
                join: label_target_block(cfg_view, *label)?,
                join_kind: lowering.join_kind,
            },
            _ => {
                return Ok(StructuredRegionBody::Sequence(require_closed_sequence(
                    lower_stmt_as_sequence_in_context(cfg_view, body, Some(*label))?,
                    "labelled block body",
                )?));
            }
        },
        RelooperStmt::Case { .. } => {
            return Err(StructuralizationError::Relooper {
                reason: "case statements must be enclosed by a labelled block".into(),
            });
        }
        RelooperStmt::Loop {
            label,
            lowering,
            body,
        } => lower_loop_body(cfg_view, label, lowering, body, &[], &[])?,
        RelooperStmt::Br(_) | RelooperStmt::Return(_) => {
            return Err(StructuralizationError::Relooper {
                reason: "explicit labelled exits are not lowered yet".into(),
            });
        }
        RelooperStmt::Exec(_) => StructuredRegionBody::Sequence(vec![lower_stmt(cfg_view, stmt)?]),
    })
}

/// Lowers one statement into a sequence, optionally consuming one matching
/// terminal branch to the current enclosing label.
fn lower_stmt_as_sequence_in_context<H: HugrView<Node = Node>>(
    cfg_view: &H,
    stmt: &RelooperStmt,
    terminal_label: Option<RelooperLabel>,
) -> Result<LoweredSequence, StructuralizationError> {
    let mut lowered = lower_stmt_as_sequence(cfg_view, stmt)?;
    match (&terminal_label, &lowered.exit) {
        (Some(label), Some(LoweredExit::Branch(target))) if target == label => {
            lowered.exit = None;
        }
        (None, Some(LoweredExit::Return(_))) => {
            lowered.exit = None;
        }
        _ => {}
    }

    if lowered.exit.is_some()
        && matches!(
            stmt,
            RelooperStmt::Case { .. }
                | RelooperStmt::Loop { .. }
                | RelooperStmt::Region(_)
                | RelooperStmt::Exec(_)
        )
    {
        return Err(StructuralizationError::Relooper {
            reason: "explicit labelled exits cannot yet cross this structured boundary".into(),
        });
    }

    Ok(lowered)
}

/// Lowers one statement into a straight-line sequence, propagating any explicit exit.
fn lower_stmt_as_sequence<H: HugrView<Node = Node>>(
    cfg_view: &H,
    stmt: &RelooperStmt,
) -> Result<LoweredSequence, StructuralizationError> {
    match stmt {
        RelooperStmt::Seq(items) => {
            if let Some((region, raw_continuations)) = extract_wrapped_loop_region(items) {
                return Ok(LoweredSequence {
                    nodes: vec![StructuredNode::Region(Box::new(lower_wrapped_loop_region(
                        cfg_view,
                        region,
                        &raw_continuations,
                    )?))],
                    exit: None,
                });
            }
            let mut lowered = Vec::new();
            for (idx, item) in items.iter().enumerate() {
                let item = lower_stmt_as_sequence(cfg_view, item)?;
                lowered.extend(item.nodes);
                if let Some(exit) = item.exit {
                    if idx + 1 != items.len() {
                        return Err(StructuralizationError::Relooper {
                            reason: "sequence contains statements after an explicit exit".into(),
                        });
                    }
                    return Ok(LoweredSequence {
                        nodes: lowered,
                        exit: Some(exit),
                    });
                }
            }
            Ok(LoweredSequence {
                nodes: lowered,
                exit: None,
            })
        }
        RelooperStmt::Region(region) => Ok(LoweredSequence {
            nodes: vec![StructuredNode::Region(Box::new(lower_region(
                cfg_view, region,
            )?))],
            exit: None,
        }),
        RelooperStmt::Exec(block) => Ok(LoweredSequence {
            nodes: vec![StructuredNode::Block(block.clone())],
            exit: None,
        }),
        RelooperStmt::Block { label, body, .. } => {
            lower_stmt_as_sequence_in_context(cfg_view, body, Some(*label))
        }
        RelooperStmt::Case { .. } | RelooperStmt::Loop { .. } => {
            Err(StructuralizationError::Relooper {
                reason: "nested control statements must be wrapped in an analyzed region".into(),
            })
        }
        RelooperStmt::Br(label) => Ok(LoweredSequence {
            nodes: Vec::new(),
            exit: Some(LoweredExit::Branch(*label)),
        }),
        RelooperStmt::Return(payload) => Ok(LoweredSequence {
            nodes: Vec::new(),
            exit: Some(LoweredExit::Return(payload.clone())),
        }),
    }
}

/// Lowers one loop statement with explicit per-exit continuations.
fn lower_loop_body<H: HugrView<Node = Node>>(
    cfg_view: &H,
    label: &RelooperLabel,
    lowering: &super::ast::RelooperLoopLowering,
    body: &RelooperStmt,
    exit_edges: &[Vec<StructuredLoopEdge>],
    exit_continuations: &[Vec<StructuredNode>],
) -> Result<StructuredRegionBody, StructuralizationError> {
    Ok(StructuredRegionBody::Loop {
        kind: lowering.kind,
        header: lowering.header.clone(),
        body: require_closed_sequence(
            lower_stmt_as_sequence_in_context(cfg_view, body, Some(*label))?,
            "loop body",
        )?,
        backedge_sources: lowering.backedge_sources.clone(),
        continue_edges: lowering.continue_edges.clone(),
        exits: exit_edges
            .iter()
            .cloned()
            .zip(
                exit_continuations
                    .iter()
                    .cloned()
                    .chain(std::iter::repeat_with(Vec::new)),
            )
            .map(|(edges, continuation)| lower_exit(edges, continuation))
            .collect::<Result<Vec<_>, _>>()?,
        break_outputs: loop_break_outputs(exit_edges)?,
    })
}

/// Lowers one wrapped loop region after extracting AST-side exit data.
fn lower_wrapped_loop_region<H: HugrView<Node = Node>>(
    cfg_view: &H,
    region: &RelooperRegion,
    raw_continuations: &RawExitContinuations<'_>,
) -> Result<StructuredRegion, StructuralizationError> {
    let exit_continuations = raw_continuations
        .iter()
        .map(|(_, items)| lower_exit_sequence(cfg_view, items))
        .collect::<Result<ExitContinuations, StructuralizationError>>()?;
    let exit_edges = raw_continuations
        .iter()
        .map(|(edges, _)| edges.clone())
        .collect::<Vec<_>>();
    let RelooperStmt::Loop {
        label,
        lowering,
        body,
    } = &region.body
    else {
        return Err(StructuralizationError::Relooper {
            reason: "wrapped loop extraction did not produce a loop region".into(),
        });
    };
    Ok(StructuredRegion {
        io: region.io.clone(),
        body: lower_loop_body(
            cfg_view,
            label,
            lowering,
            body,
            &exit_edges,
            &exit_continuations,
        )?,
    })
}

/// Extracts one loop region plus its AST-side exit selectors/continuations.
fn extract_wrapped_loop_region(
    items: &[RelooperStmt],
) -> Option<(&RelooperRegion, RawExitContinuations<'_>)> {
    fn peel<'a>(
        items: &'a [RelooperStmt],
        continuations: &mut RawExitContinuations<'a>,
    ) -> Option<&'a RelooperRegion> {
        match items {
            [RelooperStmt::Region(region)] => match &region.body {
                RelooperStmt::Loop { .. } => Some(region.as_ref()),
                RelooperStmt::Seq(inner) => peel(inner, continuations),
                _ => None,
            },
            [RelooperStmt::Block { lowering, body, .. }, tail @ ..]
                if !lowering.loop_exit_edges.is_empty() =>
            {
                continuations.push((lowering.loop_exit_edges.clone(), tail));
                match body.as_ref() {
                    RelooperStmt::Region(region) => match &region.body {
                        RelooperStmt::Loop { .. } => Some(region.as_ref()),
                        RelooperStmt::Seq(inner) => peel(inner, continuations),
                        _ => None,
                    },
                    _ => None,
                }
            }
            [RelooperStmt::Seq(inner)] => peel(inner, continuations),
            _ => None,
        }
    }

    let mut continuations = RawExitContinuations::new();
    peel(items, &mut continuations).map(|region| (region, continuations))
}

/// Lowers a continuation sequence while consuming one terminal explicit exit.
fn lower_exit_sequence<H: HugrView<Node = Node>>(
    cfg_view: &H,
    items: &[RelooperStmt],
) -> Result<Vec<StructuredNode>, StructuralizationError> {
    Ok(lower_stmt_as_sequence(cfg_view, &RelooperStmt::Seq(items.to_vec()))?.nodes)
}

/// Ensures a lowered sequence has no remaining explicit exit.
fn require_closed_sequence(
    lowered: LoweredSequence,
    context: &str,
) -> Result<Vec<StructuredNode>, StructuralizationError> {
    match lowered.exit {
        None => Ok(lowered.nodes),
        Some(LoweredExit::Branch(_)) => Err(StructuralizationError::Relooper {
            reason: format!("{context} still contains an explicit branch to an outer label"),
        }),
        Some(LoweredExit::Return(_)) => Err(StructuralizationError::Relooper {
            reason: format!("{context} still contains an explicit return"),
        }),
    }
}

/// Lowers one Beyond-Relooper statement into one structured node.
fn lower_stmt<H: HugrView<Node = Node>>(
    cfg_view: &H,
    stmt: &RelooperStmt,
) -> Result<StructuredNode, StructuralizationError> {
    Ok(match stmt {
        RelooperStmt::Region(region) => {
            StructuredNode::Region(Box::new(lower_region(cfg_view, region)?))
        }
        RelooperStmt::Exec(block) => StructuredNode::Block(block.clone()),
        RelooperStmt::Seq(_)
        | RelooperStmt::Block { .. }
        | RelooperStmt::Case { .. }
        | RelooperStmt::Loop { .. }
        | RelooperStmt::Br(_)
        | RelooperStmt::Return(_) => {
            return Err(StructuralizationError::Relooper {
                reason: "nested control statements must be wrapped in an analyzed region".into(),
            });
        }
    })
}

/// Lowers one Beyond-Relooper loop exit into the shared structural form.
fn lower_exit(
    edges: Vec<StructuredLoopEdge>,
    continuation: Vec<StructuredNode>,
) -> Result<StructuredLoopExit, StructuralizationError> {
    Ok(StructuredLoopExit {
        outputs: exit_output_row(&edges)?,
        edges,
        continuation,
    })
}

/// Returns the immediate `TailLoop` break row for one analyzed exit set.
fn loop_break_outputs(
    exits: &[Vec<StructuredLoopEdge>],
) -> Result<hugr::types::TypeRow, StructuralizationError> {
    match exits {
        [exit] => exit_output_row(exit),
        _ => Ok(vec![hugr::types::Type::new_sum(
            exits
                .iter()
                .map(|exit| exit_output_row(exit))
                .collect::<Result<Vec<_>, _>>()?,
        )]
        .into()),
    }
}

/// Returns the immediate payload row carried by one analyzed loop exit.
fn exit_output_row(
    exit: &[StructuredLoopEdge],
) -> Result<hugr::types::TypeRow, StructuralizationError> {
    exit.first()
        .map(|edge| edge.payload.clone())
        .ok_or(StructuralizationError::UnsupportedLoop {
            reason: "loop exit has no selecting edges".into(),
        })
}

/// Resolves the block shape named by one Beyond-Relooper label.
///
/// Duplicated labels created during preprocessing still lower to the same
/// original HUGR block shape, so the lowerer can recover the followed-by block
/// from the label instead of storing it in the AST.
fn label_target_block<H: HugrView<Node = Node>>(
    cfg_view: &H,
    label: RelooperLabel,
) -> Result<StructuredBlock, StructuralizationError> {
    let (cfg_node, node) = match label {
        RelooperLabel::Original(node) => {
            (crate::control::cfg::PreprocessedNode::Original(node), node)
        }
        RelooperLabel::Duplicate { original, clone_id } => (
            crate::control::cfg::PreprocessedNode::Duplicate { original, clone_id },
            original,
        ),
    };
    analyze_block(cfg_view, cfg_node, node)
}
