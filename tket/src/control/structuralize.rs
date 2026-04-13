//! HUGR-specific entry points for CFG structuralization analysis and lowering.
//!
//! This module keeps HUGR-facing concerns out of `control::rvsdg`: typed block
//! interfaces, region I/O summaries, and lowering from strategy-specific
//! structured analyses into nested `DFG` / `Conditional` / `TailLoop` nodes.
//!
//! The public API is intentionally small:
//! `analyze_hugr_cfgs` produces a lowering-ready structural summary, while
//! `structurize_cfgs` performs the rewrite. Strategy-specific analyzers live in
//! their own submodules and all converge on one private shared lowering IR.
//! Keeping the implementation split this way lets the RVSDG and Beyond-
//! Relooper paths evolve independently without one giant file.

mod analyze;
mod lower;
pub(crate) mod shared;
#[cfg(test)]
mod test;
mod types;

pub use analyze::analyze_hugr_cfgs;
pub use lower::structurize_cfgs;
pub(crate) use types::{
    RegionIo, StructuredBlock, StructuredBranchJoinKind, StructuredLoopKind, StructuredNode,
    StructuredRegion, StructuredRegionBody,
};
pub use types::{
    StructuralizationAnalysisReport, StructuralizationError, StructuralizationRewriteReport,
    StructuralizationStrategy,
};
