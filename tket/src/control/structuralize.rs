//! HUGR-specific entry points for CFG structuralization analysis and lowering.
//!
//! This module keeps HUGR-facing concerns out of `control::rvsdg`: typed block
//! interfaces, region I/O summaries, and lowering from the structural control
//! tree into nested `DFG` / `Conditional` / `TailLoop` nodes.
//!
//! The public API is intentionally small:
//! `analyze_hugr_cfgs` produces a lowering-ready structural summary, while
//! `structurize_cfgs` performs the rewrite. The implementation lives in
//! submodules so analysis, lowering, shared data types, and regression tests
//! can evolve independently without one giant file.

mod analyze;
mod lower;
#[cfg(test)]
mod test;
mod types;

pub use analyze::analyze_hugr_cfgs;
pub use lower::structurize_cfgs;
pub use types::{
    RegionIo, StructuralizationAnalysisReport, StructuralizationError,
    StructuralizationRewriteReport, StructuralizationStrategy, StructuredBlock, StructuredLoopKind,
    StructuredNode, StructuredRegion, StructuredRegionBody,
};
