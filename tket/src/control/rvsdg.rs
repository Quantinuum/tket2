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

pub(crate) use construct::build_cfg_rvsdg;
pub(crate) use error::RvsdgBuildError;
pub(crate) use ir::{
    BlockNode, BranchJoinKind, GammaNode, LoopKind, Region, RvsdgNode, ThetaNode, vars_to_row,
};
pub(crate) use lower::analyze_cfg;
