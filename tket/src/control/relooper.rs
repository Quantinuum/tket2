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
use crate::control::cfg::{CfgFactsError, PreprocessedCfg};
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
    let ast = match construct::build_cfg_ast(cfg_view, cfg) {
        Ok(ast) => ast,
        Err(StructuralizationError::UnsupportedIrreducibleCfg { .. }) => {
            let preprocessed = PreprocessedCfg::new(cfg_view.entrypoint(), cfg)
                .map_err(map_cfg_facts_error(cfg_view.entrypoint()))?;
            construct::build_cfg_ast_with_map(cfg_view, &preprocessed)?
        }
        Err(err) => return Err(err),
    };
    lower::lower_region(&ast)
}

/// Converts shared CFG-fact failures into Beyond-Relooper entry-point errors.
fn map_cfg_facts_error(
    cfg_root: Node,
) -> impl FnOnce(CfgFactsError<Node>) -> StructuralizationError {
    move |err| match err {
        CfgFactsError::NoEntryExitPath => StructuralizationError::Relooper {
            reason: "cfg has no entry-to-exit path".into(),
        },
        CfgFactsError::ReachableNodesDoNotReachExit { dropped } => {
            StructuralizationError::Relooper {
                reason: format!(
                    "reachable nodes do not all reach the CFG exit: {:?}",
                    dropped
                ),
            }
        }
        CfgFactsError::Irreducible { entries, .. } => {
            StructuralizationError::UnsupportedIrreducibleCfg {
                cfg: cfg_root,
                reason: format!("cyclic SCC has multiple entries: {:?}", entries),
            }
        }
    }
}
