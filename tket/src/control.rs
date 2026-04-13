//! Common control-flow analysis utilities.
//!
//! This module collects reusable control-flow-graph analysis helpers that are
//! useful outside the pass pipeline.
//!
//! For now, these helpers are thin wrappers around the generic CFG-view and
//! SESE-analysis code used by the existing nesting passes. Keeping the public
//! entry points here means new control-flow tooling can depend on
//! `tket::control` instead of reaching into `tket::passes`.

use hugr::core::HugrNode;
use std::collections::HashMap;

pub(crate) mod cfg;
pub(crate) mod half_node;
pub mod nest_cfgs;
pub(crate) mod relooper;
pub(crate) mod rvsdg;
pub mod structuralize;

pub use nest_cfgs::{CfgBlockMap, CfgNodeMap, IdentityCfgMap, RegionBlocksError, region_blocks};

use nest_cfgs::EdgeClassifier;

/// A directed control-flow edge.
pub type CfgEdge<T> = (T, T);

/// Returns cycle-equivalence classes for the edges in a CFG.
pub fn edge_classes<T: HugrNode>(cfg: &impl CfgNodeMap<T>) -> HashMap<CfgEdge<T>, usize> {
    EdgeClassifier::get_edge_classes(cfg)
}
