//! HUGR-facing entry points for CFG structuralization.
//!
//! The pass-oriented public interface lives here, together with the
//! HUGR-specific structured-control types consumed by the rewrite pipeline.
//! Strategy-specific reconstruction stays in `control::rvsdg` and
//! `control::relooper`; only the lowering-oriented data that is genuinely
//! shared between strategies belongs in this module.

mod analyze;
pub(crate) mod lower;
#[cfg(test)]
mod test;
mod types;

#[cfg(test)]
pub(crate) use analyze::analyze_hugr_cfgs;
pub(crate) use lower::LoweredCfgTemplate;
pub(crate) use lower::structurize_cfg;
#[cfg(test)]
pub(crate) use types::StructuralizationAnalysisReport;
pub(crate) use types::{
    IntoStructuredCfgNode, MultilevelExitDispatch, MultilevelExitVariant, RegionIo,
    StructuredBlock, StructuredBranchJoinKind, StructuredBranchTargetKind, StructuredCaseArm,
    StructuredCfgNode, StructuredExitEffect, StructuredLoopEdge, StructuredLoopExit,
    StructuredLoopKind, StructuredNode, StructuredRegion, StructuredRegionBody,
};
pub use types::{
    StructuralizationError, StructuralizationRewriteReport, StructuralizationStrategy,
};
