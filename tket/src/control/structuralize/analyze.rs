//! Strategy dispatch for CFG structuralization analysis.
//!
//! This module is intentionally small: it walks CFG roots in a HUGR and routes
//! each one to the selected strategy-specific analysis module. The concrete
//! RVSDG and Beyond-Relooper logic lives in their own modules so the shared
//! entry points stay free of strategy-specific region details.

use std::collections::BTreeMap;

use hugr::ops::OpTrait;
use hugr::{HugrView, Node};
use hugr_core::ops::OpTag;

use crate::control::{IdentityCfgMap, relooper};

use super::rvsdg;
use super::types::{
    StructuralizationAnalysisReport, StructuralizationError, StructuralizationStrategy,
    StructuredRegion,
};

/// Analyze all CFGs in a HUGR using the requested strategy.
///
/// # Errors
///
/// Returns an error when strategy-specific CFG analysis fails for any CFG or
/// when a lowering-oriented structural region cannot be derived
/// deterministically.
pub fn analyze_hugr_cfgs<H: HugrView<Node = Node>>(
    hugr: &H,
    strategy: StructuralizationStrategy,
) -> Result<StructuralizationAnalysisReport, StructuralizationError> {
    let mut cfg_regions = BTreeMap::new();
    for cfg in hugr
        .nodes()
        .filter(|n| hugr.get_optype(*n).tag() == OpTag::Cfg)
    {
        let cfg_view = hugr.with_entrypoint(cfg);
        cfg_regions.insert(cfg, analyze_cfg(cfg_view, strategy)?);
    }
    Ok(StructuralizationAnalysisReport { cfg_regions })
}

/// Analyzes one CFG into the shared lowering-oriented structural region form.
///
/// This is the strategy dispatch point used by both the public analysis entry
/// point and the rewrite path. It keeps lowerer code free from strategy-
/// specific RVSDG or Beyond-Relooper details.
pub(super) fn analyze_cfg<H: HugrView<Node = Node> + Clone>(
    cfg_view: H,
    strategy: StructuralizationStrategy,
) -> Result<StructuredRegion, StructuralizationError> {
    let id_cfg = IdentityCfgMap::new(cfg_view.clone());
    match strategy {
        StructuralizationStrategy::Rvsdg => rvsdg::analyze_cfg(&cfg_view, &id_cfg),
        StructuralizationStrategy::BeyondRelooper => relooper::analyze_cfg(&cfg_view, &id_cfg),
    }
}
