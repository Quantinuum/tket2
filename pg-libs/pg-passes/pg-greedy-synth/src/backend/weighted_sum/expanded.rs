use super::WeightedSumStrategy;
use crate::packed_pg_slice::{PackedOpMeta, PackedPGSlice, SliceBitPosition};

#[cfg(feature = "simd")]
use std::simd::{LaneCount, Simd, SupportedLaneCount, num::SimdFloat, num::SimdUint};

#[cfg(feature = "simd")]
const SIMD_LANES: usize = 64;

#[derive(Default)]
struct ExpandedWeights {
    values: Vec<f64>,
}

impl ExpandedWeights {
    /// Stores one weight for each packed position and discounts each later
    /// commuting set by `ALPHA`.
    fn prepare(&mut self, slice: &PackedPGSlice) {
        assert!(!slice.is_empty(), "cannot build weights for empty slice");

        self.values.clear();
        let mut current_weight = 1.0;

        for (set_offset, set) in slice.op_sets().enumerate() {
            if set_offset != 0 {
                current_weight *= super::super::ALPHA;
            }

            for (_, view) in set {
                let start = bit_index(view.bit_range.0);
                let end = bit_index(view.bit_range.1);
                self.values.resize(end, 0.0);
                match view.meta {
                    PackedOpMeta::Rotation(_)
                    | PackedOpMeta::Measure(_)
                    | PackedOpMeta::Reset(_) => self.values[start] = current_weight,
                    PackedOpMeta::ConditionalBox(_) | PackedOpMeta::BlackBox(_) => {}
                }
            }
        }
    }
}

fn bit_index(position: SliceBitPosition) -> usize {
    position.chunk * 64 + position.bit
}

/// Sums the expanded weights by visiting only the set bits in each mask.
fn sparse_weight_sum(masks: &[u64], weights: &[f64]) -> f64 {
    let mut sum = 0.0;

    for (word_index, word) in masks.iter().copied().enumerate() {
        let base = word_index * 64;
        if base >= weights.len() {
            break;
        }

        let mut remaining = word;
        while remaining != 0 {
            let bit = remaining.trailing_zeros() as usize;
            let weight_index = base + bit;
            if weight_index < weights.len() {
                sum += weights[weight_index];
            }
            remaining &= remaining - 1;
        }
    }

    sum
}

/// A `WeightedSumStrategy` that uses Kernighan's algorithm to visit the set
/// bits and sum their corresponding weights.
#[derive(Default)]
pub(crate) struct ExpandedSparseWeightedSum {
    weights: ExpandedWeights,
}

impl WeightedSumStrategy for ExpandedSparseWeightedSum {
    fn prepare(&mut self, slice: &PackedPGSlice) {
        self.weights.prepare(slice);
    }

    fn weighted_delta(&self, increases: &[u64], decreases: &[u64]) -> f64 {
        debug_assert_eq!(increases.len(), decreases.len());
        let weight_len = self.weights.values.len().min(increases.len() * 64);
        let weights = &self.weights.values[..weight_len];
        sparse_weight_sum(increases, weights) - sparse_weight_sum(decreases, weights)
    }
}

#[cfg(feature = "simd")]
fn simd_weight_tail<const N: usize>(mask: u64, weights: &[f64]) -> f64
where
    LaneCount<N>: SupportedLaneCount,
{
    let shifts = Simd::<u64, N>::from_array(std::array::from_fn(|i| i as u64));
    let bits = (Simd::splat(mask) >> shifts) & Simd::splat(1);
    (bits.cast::<f64>() * Simd::<f64, N>::from_slice(weights)).reduce_sum()
}

/// Sums the expanded weights using SIMD for dense masks and sparse iteration
/// when fewer than 25 percent of the mask bits are set.
///
/// The SIMD path expands each `u64` mask into 64 lanes, multiplies each lane by
/// its corresponding weight, and sums the results.
///
/// The cutoff is an inherited performance heuristic and should only change
/// after representative benchmarks.
#[cfg(feature = "simd")]
fn simd_weight_sum(masks: &[u64], weights: &[f64]) -> f64 {
    if masks.is_empty() {
        return 0.0;
    }
    let weights = &weights[..weights.len().min(masks.len() * 64)];
    let density = (masks.iter().map(|word| word.count_ones()).sum::<u32>() as f64
        / masks.len() as f64)
        / 64.0;
    if density < 0.25 {
        return sparse_weight_sum(masks, weights);
    }

    let shifts = Simd::<u64, SIMD_LANES>::from_array(std::array::from_fn(|i| i as u64));
    let mut sum = Simd::<f64, SIMD_LANES>::splat(0.0);
    let complete_words = weights.len() / SIMD_LANES;
    for (word_index, word) in masks.iter().enumerate().take(complete_words) {
        let bits = (Simd::splat(*word) >> shifts) & Simd::splat(1);
        let base = word_index * SIMD_LANES;
        let chunk = Simd::<f64, SIMD_LANES>::from_slice(&weights[base..base + SIMD_LANES]);
        sum += bits.cast::<f64>() * chunk;
    }

    let remainder = weights.len() % SIMD_LANES;
    let tail = if remainder == 0 {
        0.0
    } else {
        let base = complete_words * SIMD_LANES;
        dispatch_simd_tail!(
            remainder,
            simd_weight_tail(masks[complete_words], &weights[base..])
        )
    };
    sum.reduce_sum() + tail
}

/// A `WeightedSumStrategy` that selects sparse iteration or SIMD based on mask
/// density.
///
/// For dense masks, it expands each `u64` mask into 64 SIMD lanes, multiplies
/// each lane by its corresponding weight, and sums the results.
#[cfg(feature = "simd")]
#[derive(Default)]
pub(crate) struct ExpandedSimdWeightedSum {
    weights: ExpandedWeights,
}

#[cfg(feature = "simd")]
impl WeightedSumStrategy for ExpandedSimdWeightedSum {
    fn prepare(&mut self, slice: &PackedPGSlice) {
        self.weights.prepare(slice);
    }

    fn weighted_delta(&self, increases: &[u64], decreases: &[u64]) -> f64 {
        debug_assert_eq!(increases.len(), decreases.len());
        let weight_len = self.weights.values.len().min(increases.len() * 64);
        let weights = &self.weights.values[..weight_len];
        simd_weight_sum(increases, weights) - simd_weight_sum(decreases, weights)
    }
}
