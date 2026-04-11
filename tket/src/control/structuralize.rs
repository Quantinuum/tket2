//! HUGR-specific entry points for CFG structuralization analysis.
//!
//! This module bridges generic CFG structural analysis (`control::rvsdg`) with
//! whole-HUGR traversal so pass-layer code can request structured CFG regions
//! without depending on low-level CFG view wiring.

use std::collections::HashMap;

use derive_more::{Display, Error};
use hugr::core::HugrNode;
use hugr::ops::OpTrait;
use hugr::{HugrView, Node};
use hugr_core::ops::OpTag;

use crate::control::{IdentityCfgMap, rvsdg};

/// Strategy selector for control-flow structuralization.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum StructuralizationStrategy {
    /// Build the current RVSDG-style structured control tree.
    #[default]
    Rvsdg,
    /// Placeholder for a future Beyond-Relooper implementation.
    BeyondRelooper,
}

/// Errors returned by HUGR-level structuralization entry points.
#[derive(Clone, Debug, Display, Error)]
#[non_exhaustive]
pub enum StructuralizationError<T: HugrNode> {
    /// Error while building an RVSDG structural tree for a CFG.
    #[display("failed to build RVSDG control tree: {_0}")]
    Rvsdg(rvsdg::RvsdgBuildError<T>),
    /// The selected strategy is not implemented yet.
    #[display("structuralization strategy {strategy:?} is not implemented yet")]
    UnsupportedStrategy {
        /// Strategy requested by the caller.
        #[error(not(source))]
        strategy: StructuralizationStrategy,
    },
}

/// Structured CFG analysis output for one HUGR.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct StructuralizationReport<T: HugrNode> {
    /// Structured control tree for each CFG node in the HUGR.
    pub cfg_regions: HashMap<T, rvsdg::ControlRegion<T>>,
}

/// Analyze all CFGs in a HUGR using the requested strategy.
pub fn analyze_hugr_cfgs<H: HugrView<Node = Node>>(
    hugr: &H,
    strategy: StructuralizationStrategy,
) -> Result<StructuralizationReport<Node>, StructuralizationError<Node>> {
    match strategy {
        StructuralizationStrategy::Rvsdg => {
            let mut cfg_regions = HashMap::new();
            for cfg in hugr
                .nodes()
                .filter(|n| hugr.get_optype(*n).tag() == OpTag::Cfg)
            {
                let cfg_view = hugr.with_entrypoint(cfg);
                let tree = rvsdg::build_control_tree(&IdentityCfgMap::new(cfg_view))
                    .map_err(StructuralizationError::Rvsdg)?;
                cfg_regions.insert(cfg, tree);
            }
            Ok(StructuralizationReport { cfg_regions })
        }
        StructuralizationStrategy::BeyondRelooper => {
            Err(StructuralizationError::UnsupportedStrategy { strategy })
        }
    }
}
