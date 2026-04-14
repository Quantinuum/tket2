//! Lowering from the Beyond-Relooper AST into the shared rewrite-oriented form.
//!
//! The pass pipeline currently rewrites CFGs through the structured HUGR IR
//! used by `control::structuralize`. This module isolates the translation from
//! the Beyond-Relooper AST into that representation so the analysis phase does
//! not depend on the shared lowering types directly.
//!
//! The current lowerer consumes the paper-oriented statement AST and maps it to
//! the existing shared region representation. Labels are preserved in the AST
//! so later work can interpret explicit multilevel exits directly in this
//! module without changing the public pass-facing API.

use itertools::Itertools;

use crate::control::structuralize::{
    StructuralizationError, StructuredLoopExit, StructuredNode, StructuredRegion,
    StructuredRegionBody,
};

use super::ast::{RelooperExit, RelooperRegion, RelooperStmt};

/// Lowers one Beyond-Relooper region into the shared structuralization region.
pub(super) fn lower_region(
    region: &RelooperRegion,
) -> Result<StructuredRegion, StructuralizationError> {
    Ok(StructuredRegion {
        io: region.io.clone(),
        body: lower_stmt_as_body(&region.body)?,
    })
}

/// Lowers one top-level statement into the shared structuralization body.
fn lower_stmt_as_body(stmt: &RelooperStmt) -> Result<StructuredRegionBody, StructuralizationError> {
    Ok(match stmt {
        RelooperStmt::Seq(_) => {
            StructuredRegionBody::Sequence(lower_stmt_as_sequence_in_context(stmt, None)?)
        }
        RelooperStmt::Region(region) => return Ok(lower_region(region)?.body),
        RelooperStmt::Block {
            label,
            lowering,
            body,
        } => match body.as_ref() {
            RelooperStmt::Case { split, arms } => StructuredRegionBody::Branch {
                split: split.clone(),
                arms: arms
                    .iter()
                    .map(|arm| lower_stmt_as_sequence_in_context(arm, Some(*label)))
                    .collect::<Result<Vec<_>, _>>()?,
                join: lowering.follow.clone(),
                join_kind: lowering.join_kind,
            },
            _ => return lower_stmt_as_body(body),
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
        } => StructuredRegionBody::Loop {
            kind: lowering.kind,
            header: lowering.header.clone(),
            body: lower_stmt_as_sequence_in_context(body, Some(*label))?,
            backedge_source: lowering.backedge_source,
            continue_edge: lowering.continue_edge.clone(),
            exits: lowering
                .exits
                .iter()
                .map(lower_exit)
                .collect::<Result<Vec<_>, _>>()?,
            break_outputs: loop_break_outputs(&lowering.exits),
        },
        RelooperStmt::Br(_) | RelooperStmt::Return(_) => {
            return Err(StructuralizationError::Relooper {
                reason: "explicit labelled exits are not lowered yet".into(),
            });
        }
        RelooperStmt::Exec(_) => StructuredRegionBody::Sequence(vec![lower_stmt(stmt)?]),
    })
}

/// Lowers one statement into a sequence, optionally consuming one matching
/// terminal branch to the current enclosing label.
fn lower_stmt_as_sequence_in_context(
    stmt: &RelooperStmt,
    terminal_label: Option<super::ast::RelooperLabel>,
) -> Result<Vec<StructuredNode>, StructuralizationError> {
    match stmt {
        RelooperStmt::Seq(items) => {
            let mut items = items.as_slice();
            match (terminal_label, items.last()) {
                (Some(label), Some(RelooperStmt::Br(exit))) if exit.target == label => {
                    items = &items[..items.len() - 1];
                }
                (None, Some(RelooperStmt::Return(_))) => {
                    items = &items[..items.len() - 1];
                }
                _ => {}
            }
            items.iter().map(lower_stmt).collect()
        }
        _ => Ok(vec![lower_stmt(stmt)?]),
    }
}

/// Lowers a continuation sequence while consuming one terminal explicit exit.
///
/// Exit continuations live beneath the paper-style loop-exit records and are
/// allowed to end in a branch to the enclosing labelled block or a return from
/// the enclosing region. The shared structural lowering IR represents only the
/// straight-line continuation after the exit is taken, so the terminal explicit
/// exit is consumed here.
fn lower_exit_sequence(stmt: &RelooperStmt) -> Result<Vec<StructuredNode>, StructuralizationError> {
    match stmt {
        RelooperStmt::Seq(items) => {
            let mut items = items.as_slice();
            if matches!(
                items.last(),
                Some(RelooperStmt::Br(_) | RelooperStmt::Return(_))
            ) {
                items = &items[..items.len() - 1];
            }
            items.iter().map(lower_stmt).collect()
        }
        _ => Ok(vec![lower_stmt(stmt)?]),
    }
}

/// Lowers one Beyond-Relooper statement into the shared structuralization node.
fn lower_stmt(stmt: &RelooperStmt) -> Result<StructuredNode, StructuralizationError> {
    Ok(match stmt {
        RelooperStmt::Region(region) => StructuredNode::Region(Box::new(lower_region(region)?)),
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
fn lower_exit(exit: &RelooperExit) -> Result<StructuredLoopExit, StructuralizationError> {
    Ok(StructuredLoopExit {
        edges: exit.edges.clone(),
        outputs: exit.payload.clone(),
        continuation: lower_exit_sequence(exit.continuation.as_ref())?,
    })
}

/// Returns the immediate `TailLoop` break row for one analyzed exit set.
fn loop_break_outputs(exits: &[RelooperExit]) -> hugr::types::TypeRow {
    match exits {
        [exit] => exit.payload.clone(),
        _ => vec![hugr::types::Type::new_sum(
            exits.iter().map(|exit| exit.payload.clone()).collect_vec(),
        )]
        .into(),
    }
}
