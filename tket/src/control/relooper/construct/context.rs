//! Context-sensitive exit helpers for the Beyond-Relooper constructor.

use crate::control::relooper::ast::{
    RelooperBranch, RelooperBranchTarget, RelooperContext, RelooperContextFrame, RelooperStmt,
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

/// Returns the innermost enclosing structured target that should consume one
/// propagated branch.
pub(super) fn context_branch_target(context: &RelooperContext) -> Option<RelooperBranchTarget> {
    context.iter().find_map(|frame| match frame {
        RelooperContextFrame::BlockFollowedBy(label) => {
            Some(RelooperBranchTarget::BlockFollowedBy(*label))
        }
        RelooperContextFrame::LoopHeadedBy(label) => {
            Some(RelooperBranchTarget::LoopHeadedBy(*label))
        }
        _ => None,
    })
}

/// Appends an explicit branch to the target unless the sequence already
/// terminates.
pub(super) fn append_branch_to_target(
    items: &mut Vec<RelooperStmt>,
    target: RelooperBranchTarget,
    payload: hugr::types::TypeRow,
) {
    if items.last().is_some_and(is_terminal_stmt) {
        return;
    }
    items.push(RelooperStmt::Br(RelooperBranch {
        target,
        outputs: payload,
    }));
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
    match context_branch_target(context) {
        Some(target) => append_branch_to_target(items, target, payload),
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
