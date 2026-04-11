//! Interfaces for structured control regions.
//!
//! The RVSDG/control-structuralization pipeline keeps multiple interface views:
//! edge-level boundaries for exact control-flow provenance, and canonical block
//! sets for deterministic downstream consumption.

use hugr::core::HugrNode;

use crate::control::rvsdg::RegionBoundary;

/// Canonical region-level control interface for structured regions.
///
/// [`RegionBoundary`] preserves full edge-level detail, but later RVSDG/HUGR
/// lowering often needs deterministic entry/exit block sets first.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RegionInterface<T> {
    /// Blocks inside the region that receive control from outside it.
    pub entry_blocks: Vec<T>,
    /// Blocks inside the region that can transfer control outside it.
    pub exit_blocks: Vec<T>,
}

impl<T: HugrNode> RegionInterface<T> {
    /// Builds a deterministic block-level interface from region boundary edges.
    pub fn from_boundary(boundary: &RegionBoundary<T>) -> Self {
        let mut entry_blocks = boundary
            .incoming
            .iter()
            .map(|(_, dst)| *dst)
            .collect::<Vec<_>>();
        let mut exit_blocks = boundary
            .outgoing
            .iter()
            .map(|(src, _)| *src)
            .collect::<Vec<_>>();
        entry_blocks.sort();
        entry_blocks.dedup();
        exit_blocks.sort();
        exit_blocks.dedup();
        Self {
            entry_blocks,
            exit_blocks,
        }
    }
}
