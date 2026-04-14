//! Private RVSDG construction for reducible CFG structuralization.
//!
//! This module builds the RVSDG representation used by the structuralization
//! pass. The implementation focuses on the control constructs needed for
//! reducible CFG structuralization:
//!
//! - regions with ordered arguments and results,
//! - leaf references to original CFG blocks,
//! - `gamma`-style branch nodes with explicit entry/exit variables, and
//! - `theta`-style loop nodes with explicit loop-carried variables.
//!
//! It is responsible for constructing a structured control representation of
//! reducible CFGs before they are translated into the shared HUGR-lowering IR
//! used by [`crate::control::structuralize`].
//!
//! Relevant references:
//!
//! - N. Reissmann, J. C. Meyer, H. Bahmann, and M. Sj\"alander, "RVSDG: An
//!   Intermediate Representation for Optimizing Compilers", ACM TECS 19(6),
//!   2020. <https://doi.org/10.1145/3391902>
//! - JLM's RVSDG implementation, especially `RvsdgModule`, `graph`, `gamma`,
//!   and `theta` under `jlm/rvsdg/`:
//!   <https://github.com/phate/jlm/blob/aa379ad2afa2c6a5ce5805417d5c76fed8417460/jlm/rvsdg/RvsdgModule.hpp>

mod construct;
mod error;
mod ir;
mod lower;
#[cfg(test)]
mod test;

use hugr::{HugrView, Node};

use crate::control::IdentityCfgMap;
use crate::control::cfg::{CfgFactsError, PreprocessedCfg};

pub(crate) fn build_cfg_rvsdg<H: HugrView<Node = Node>>(
    cfg_view: &H,
    cfg: &IdentityCfgMap<H>,
) -> Result<ir::Rvsdg, RvsdgBuildError<Node>> {
    match construct::build_cfg_rvsdg_with_map(cfg_view, cfg) {
        Ok(rvsdg) => Ok(rvsdg),
        Err(RvsdgBuildError::IrreducibleCfg { .. }) => {
            let preprocessed = PreprocessedCfg::new(cfg_view.entrypoint(), cfg)
                .map_err(map_cfg_facts_error(cfg_view.entrypoint()))?;
            construct::build_cfg_rvsdg_with_map(cfg_view, &preprocessed)
        }
        Err(err) => Err(err),
    }
}

pub(crate) use error::RvsdgBuildError;
pub(crate) use ir::{
    BlockNode, BranchJoinKind, GammaNode, LoopKind, Region, RvsdgNode, ThetaNode, vars_to_row,
};
pub(crate) use lower::analyze_cfg;

/// Converts shared CFG-fact failures into RVSDG entry-point errors.
fn map_cfg_facts_error(
    cfg_root: Node,
) -> impl FnOnce(CfgFactsError<Node>) -> RvsdgBuildError<Node> {
    move |err| match err {
        CfgFactsError::NoEntryExitPath => RvsdgBuildError::MalformedScope {
            start: cfg_root,
            reason: "cfg has no entry-to-exit path".into(),
        },
        CfgFactsError::ReachableNodesDoNotReachExit { dropped } => {
            RvsdgBuildError::MalformedScope {
                start: cfg_root,
                reason: format!(
                    "reachable nodes do not all reach the CFG exit: {:?}",
                    dropped
                ),
            }
        }
        CfgFactsError::Irreducible { entries, .. } => RvsdgBuildError::IrreducibleCfg {
            cfg: cfg_root,
            reason: format!("cyclic SCC has multiple entries: {:?}", entries),
        },
    }
}
