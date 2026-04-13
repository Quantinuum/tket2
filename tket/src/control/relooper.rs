//! Beyond-Relooper-style structural analysis for CFGs.
//!
//! This module implements the structured-control reconstruction algorithm
//! described in "Beyond Relooper" (OOPSLA 2022) and informed by GHC's
//! `FromCmm` implementation. The implementation targets HUGR CFGs and is split
//! into three internal stages:
//!
//! 1. CFG reconstruction into a Beyond-Relooper-specific control AST,
//! 2. lowering from that AST into the HUGR-oriented structural form consumed by
//!    the shared rewrite pipeline, and
//! 3. HUGR rewriting performed by [`crate::control::structuralize`].
//!
//! Keeping the AST and its lowering local to this module lets the Beyond path
//! evolve independently from the RVSDG implementation while the pass-facing API
//! remains stable.

mod ast;
mod construct;
mod lower;
#[cfg(test)]
mod test;

use hugr::{HugrView, Node};

use crate::control::IdentityCfgMap;
use crate::control::structuralize::{StructuralizationError, StructuredRegion};

/// Analyze one CFG using the Beyond-Relooper strategy.
///
/// The public structuralization entry point asks each strategy for one
/// lowering-ready region rooted at the CFG entry. Beyond Relooper therefore
/// constructs its private AST first, then lowers that AST into the shared
/// HUGR-oriented region form used by the current rewrite backend.
pub(crate) fn analyze_cfg<H: HugrView<Node = Node>>(
    cfg_view: &H,
    cfg: &IdentityCfgMap<H>,
) -> Result<StructuredRegion, StructuralizationError> {
    let ast = construct::build_cfg_ast(cfg_view, cfg)?;
    lower::lower_region(&ast)
}
