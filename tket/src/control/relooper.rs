//! Beyond-Relooper-style structural analysis for CFGs.
//!
//! This module implements the structured-control reconstruction algorithm
//! described in "Beyond Relooper" (OOPSLA 2022) and informed by GHC's
//! `FromCmm` implementation. The implementation targets HUGR CFGs and is split
//! into three internal stages:
//!
//! 1. CFG reconstruction into a Beyond-Relooper-specific control AST, and
//! 2. direct lowering from that AST into a detached HUGR rewrite template.
//!
//! The in-place materialization and CFG-root replacement still live in
//! [`crate::control::structuralize`], but the Beyond-Relooper strategy no
//! longer routes through the shared structured-region IR used by RVSDG.
//! Keeping the AST and its lowering local to this module lets the Beyond path
//! evolve independently from the RVSDG implementation while the pass-facing API
//! remains stable.

mod ast;
mod block;
mod construct;
mod lower;
#[cfg(test)]
mod test;

use hugr::{HugrView, Node};

use crate::control::IdentityCfgMap;
use crate::control::cfg::{CfgFactsError, PreprocessedCfg};
use crate::control::structuralize::{LoweredCfgTemplate, StructuralizationError};

/// Build a detached rewrite template for one CFG using Beyond Relooper.
///
/// The public structuralization entry point asks each strategy to prepare the
/// replacement it wants to materialize. Beyond Relooper therefore constructs
/// its private AST first, then lowers that AST directly into a detached HUGR
/// template with block placeholders.
pub(crate) fn prepare_cfg_rewrite<H: HugrView<Node = Node>>(
    cfg_view: &H,
    cfg: &IdentityCfgMap<H>,
) -> Result<LoweredCfgTemplate, StructuralizationError> {
    let program = match construct::build_cfg_program(cfg_view, cfg) {
        Ok(ast) => ast,
        Err(StructuralizationError::UnsupportedIrreducibleCfg { .. }) => {
            let preprocessed = PreprocessedCfg::new(cfg_view.entrypoint(), cfg)
                .map_err(map_cfg_facts_error(cfg_view.entrypoint()))?;
            construct::build_cfg_program(cfg_view, &preprocessed)?
        }
        Err(err) => return Err(err),
    };
    lower::prepare_cfg_rewrite(cfg_view, &program)
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
