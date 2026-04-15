//! Context-sensitive exit helpers for the Beyond-Relooper constructor.

use crate::control::relooper::ast::{
    RelooperContext, RelooperContextFrame, RelooperLabel, RelooperStmt,
};

/// Returns a new context with one innermost frame pushed on the front.
pub(super) fn push_context(
    context: &RelooperContext,
    frame: RelooperContextFrame,
) -> RelooperContext {
    let mut nested = Vec::with_capacity(context.len() + 1);
    nested.push(frame);
    nested.extend(context.iter().copied());
    nested
}

/// Returns the innermost enclosing block-followed-by label, if any.
fn context_follow_label(context: &RelooperContext) -> Option<RelooperLabel> {
    context.iter().find_map(|frame| match frame {
        RelooperContextFrame::BlockFollowedBy(label) => Some(*label),
        _ => None,
    })
}

/// Appends an explicit branch to the target label unless the sequence already terminates.
pub(super) fn append_branch_to_label(
    items: &mut Vec<RelooperStmt>,
    target: RelooperLabel,
    _payload: hugr::types::TypeRow,
) {
    if items.last().is_some_and(is_terminal_stmt) {
        return;
    }
    items.push(RelooperStmt::Br(target));
}

/// Appends the explicit exit selected by the active Beyond-Relooper context.
pub(super) fn append_exit_from_context(
    items: &mut Vec<RelooperStmt>,
    context: &RelooperContext,
    payload: hugr::types::TypeRow,
) {
    if items.last().is_some_and(is_terminal_stmt) {
        return;
    }
    match context_follow_label(context) {
        Some(label) => append_branch_to_label(items, label, payload),
        None => items.push(RelooperStmt::Return(payload)),
    }
}

/// Returns whether a statement already terminates control flow in its sequence.
pub(super) fn is_terminal_stmt(stmt: &RelooperStmt) -> bool {
    match stmt {
        RelooperStmt::Br(_) | RelooperStmt::Return(_) => true,
        RelooperStmt::Seq(items) => items.last().is_some_and(is_terminal_stmt),
        RelooperStmt::Region(region) => is_terminal_stmt(&region.body),
        _ => false,
    }
}
