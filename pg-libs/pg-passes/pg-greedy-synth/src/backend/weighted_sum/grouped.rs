use super::WeightedSumStrategy;
use crate::packed_pg_slice::{PackedOpMeta, PackedPGSlice, SliceBitPosition, SliceBitRange};

struct WeightRun {
    bit_range: SliceBitRange,
    weight: f64,
}

/// A `WeightedSumStrategy` that groups contiguous positions with the same
/// weight and counts the set bits in each group. This approach is efficient
/// when groups contain many positions.
#[derive(Default)]
pub(crate) struct GroupedWeightedSum {
    weight_runs: Vec<WeightRun>,
}

impl WeightedSumStrategy for GroupedWeightedSum {
    /// Stores contiguous equal weight ranges, discounting each later commuting
    /// set by `ALPHA`.
    fn prepare(&mut self, slice: &PackedPGSlice) {
        assert!(!slice.is_empty(), "cannot build weights for empty slice");

        self.weight_runs.clear();
        let mut current_weight = 1.0;

        for (set_offset, set) in slice.op_sets().enumerate() {
            if set_offset != 0 {
                current_weight *= super::super::ALPHA;
            }
            for (_, view) in set {
                let bit_range = match view.meta {
                    PackedOpMeta::Rotation(_) | PackedOpMeta::Measure(_) => view.bit_range,
                    PackedOpMeta::Reset(_) => (view.bit_range.0, next_position(view.bit_range.0)),
                    PackedOpMeta::ConditionalBox(_) | PackedOpMeta::BlackBox(_) => continue,
                };
                if let Some(previous) = self.weight_runs.last_mut()
                    && previous.bit_range.1 == bit_range.0
                    && previous.weight == current_weight
                {
                    previous.bit_range.1 = bit_range.1;
                } else {
                    self.weight_runs.push(WeightRun {
                        bit_range,
                        weight: current_weight,
                    });
                }
            }
        }
    }

    fn weighted_delta(&self, increases: &[u64], decreases: &[u64]) -> f64 {
        debug_assert_eq!(increases.len(), decreases.len());
        grouped_weight_sum(increases, &self.weight_runs)
            - grouped_weight_sum(decreases, &self.weight_runs)
    }
}

fn next_position(position: SliceBitPosition) -> SliceBitPosition {
    if position.bit == 63 {
        SliceBitPosition {
            chunk: position.chunk + 1,
            bit: 0,
        }
    } else {
        SliceBitPosition {
            chunk: position.chunk,
            bit: position.bit + 1,
        }
    }
}

fn low_bits(bits: usize) -> u64 {
    if bits == 64 {
        u64::MAX
    } else {
        (1u64 << bits) - 1
    }
}

fn count_mask_bits_in_range(masks: &[u64], (start, end): SliceBitRange) -> u64 {
    debug_assert!(start < end);
    debug_assert!(start.bit < 64 && end.bit < 64);

    if start.chunk >= masks.len() {
        return 0;
    }
    let (last_chunk, last_end) = if end.chunk >= masks.len() {
        (masks.len() - 1, 64)
    } else if end.bit == 0 {
        (end.chunk - 1, 64)
    } else {
        (end.chunk, end.bit)
    };

    if start.chunk == last_chunk {
        return u64::from(
            (masks[start.chunk] & low_bits(last_end) & !low_bits(start.bit)).count_ones(),
        );
    }

    let mut count = u64::from((masks[start.chunk] & !low_bits(start.bit)).count_ones());
    count += masks[start.chunk + 1..last_chunk]
        .iter()
        .map(|word| u64::from(word.count_ones()))
        .sum::<u64>();
    count + u64::from((masks[last_chunk] & low_bits(last_end)).count_ones())
}

/// Computes a weighted sum by counting set mask bits within each weight range.
fn grouped_weight_sum(masks: &[u64], weight_runs: &[WeightRun]) -> f64 {
    weight_runs
        .iter()
        .map(|run| count_mask_bits_in_range(masks, run.bit_range) as f64 * run.weight)
        .sum()
}
