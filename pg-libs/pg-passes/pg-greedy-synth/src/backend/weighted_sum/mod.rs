//! Strategies for computing weighted sums of packed cost masks.

mod expanded;
mod grouped;

#[cfg(feature = "simd")]
pub(crate) use expanded::ExpandedSimdWeightedSum;
pub(crate) use expanded::ExpandedSparseWeightedSum;
pub(crate) use grouped::GroupedWeightedSum;

use crate::packed_pg_slice::PackedPGSlice;

pub(crate) trait WeightedSumStrategy: Sync {
    /// Rebuilds weights for the current visible slice layout, which is rebased
    /// whenever the front set changes.
    fn prepare(&mut self, slice: &PackedPGSlice);

    /// Computes weighted increases minus weighted decreases from cost masks.
    fn weighted_delta(&self, increases: &[u64], decreases: &[u64]) -> f64;
}

#[cfg(test)]
mod tests;
