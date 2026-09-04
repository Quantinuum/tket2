//! A greedy heuristic implementation of the `TQECostBackend` trait.

use super::{CostKernel, TQECostBackend, WeightedSumStrategy};
use crate::packed_pg_slice::{Packed2QSlice, PackedOpMeta, PackedPGSlice, SliceIndex};
use crate::tqe::TQE;
use pg_bitpacked::u8_tqe_to_u64;
use pg_qm_tableau::Tableau;
use std::collections::HashMap;

/// Stores cost metadata for a slice.
struct GreedyCostMetadata {
    black_box_weights: HashMap<SliceIndex, f64>,
    tableau_weight: f64,
    has_pair_ops: bool,
}

impl GreedyCostMetadata {
    fn new() -> Self {
        Self {
            black_box_weights: HashMap::new(),
            tableau_weight: 0.0,
            has_pair_ops: false,
        }
    }

    /// Rebuilds the black box weights and trailing tableau weight for the
    /// current slice.
    ///
    /// Each later commuting set is discounted by `ALPHA = 0.588`. This quality
    /// tuning heuristic is inherited from the original implementation and
    /// should only change with representative benchmarks and an output quality
    /// comparison.
    fn rebuild(&mut self, slice: &PackedPGSlice) {
        assert!(!slice.is_empty(), "cannot build weights for empty slice");

        self.black_box_weights.clear();
        self.has_pair_ops = false;

        let mut current_weight = 1.0;
        let mut final_op_has_tableau_weight = false;

        for (set_offset, set) in slice.op_sets().enumerate() {
            if set_offset != 0 {
                current_weight *= super::ALPHA;
            }

            for (index, view) in set {
                final_op_has_tableau_weight = matches!(
                    view.meta,
                    PackedOpMeta::Rotation(_) | PackedOpMeta::Measure(_)
                );
                match view.meta {
                    PackedOpMeta::Rotation(_) | PackedOpMeta::Measure(_) => {}
                    PackedOpMeta::Reset(_) => self.has_pair_ops = true,
                    PackedOpMeta::BlackBox(_) => {
                        self.black_box_weights.insert(index, current_weight);
                    }
                    PackedOpMeta::ConditionalBox(_) => {
                        self.has_pair_ops = true;
                    }
                }
            }
        }

        self.tableau_weight = if final_op_has_tableau_weight {
            current_weight * super::ALPHA
        } else {
            0.0
        };
    }
}

/// A greedy heuristic implementation of the `TQECostBackend` trait.
pub(crate) struct GreedyCostBackend<K, W>
where
    K: CostKernel,
    W: WeightedSumStrategy,
{
    kernel: K,
    weighted_sum: W,
    metadata: GreedyCostMetadata,
}

impl<K, W> GreedyCostBackend<K, W>
where
    K: CostKernel,
    W: WeightedSumStrategy,
{
    pub(crate) fn new(kernel: K, weighted_sum: W) -> Self {
        Self {
            kernel,
            weighted_sum,
            metadata: GreedyCostMetadata::new(),
        }
    }

    /// Computes weighted support cost changes for single operations and, when
    /// present, aligned pair operations.
    fn packed_cost(
        &self,
        view: Packed2QSlice<'_>,
        gate_z_q0: u64,
        gate_x_q0: u64,
        gate_z_q1: u64,
        gate_x_q1: u64,
    ) -> f64 {
        let mut increases = vec![0; view.zb_q0.len()];
        let mut decreases = vec![0; view.zb_q0.len()];
        self.kernel.single_cost_masks(
            view.zb_q0,
            view.xb_q0,
            view.zb_q1,
            view.xb_q1,
            gate_z_q0,
            gate_x_q0,
            gate_z_q1,
            gate_x_q1,
            view.single_mask,
            &mut increases,
            &mut decreases,
        );
        clip_last_words(&mut increases, &mut decreases, view.last_chunk_mask);
        let mut cost = self.weighted_sum.weighted_delta(&increases, &decreases);
        if self.metadata.has_pair_ops {
            self.kernel.pair_cost_masks(
                view.zb_q0,
                view.xb_q0,
                view.zb_q1,
                view.xb_q1,
                gate_z_q0,
                gate_x_q0,
                gate_z_q1,
                gate_x_q1,
                view.pair_mask,
                &mut increases,
                &mut decreases,
            );
            clip_last_words(&mut increases, &mut decreases, view.last_chunk_mask);
            cost += self.weighted_sum.weighted_delta(&increases, &decreases);
        }
        cost
    }
}

fn clip_last_words(increases: &mut [u64], decreases: &mut [u64], last_chunk_mask: u64) {
    let last_increase = increases
        .last_mut()
        .expect("packed cost mask must contain at least one word");
    let last_decrease = decreases
        .last_mut()
        .expect("packed cost mask must contain at least one word");
    *last_increase &= last_chunk_mask;
    *last_decrease &= last_chunk_mask;
}

impl<K, W> TQECostBackend for GreedyCostBackend<K, W>
where
    K: CostKernel,
    W: WeightedSumStrategy,
{
    fn update(&mut self, slice: &PackedPGSlice) {
        self.weighted_sum.prepare(slice);
        self.metadata.rebuild(slice);
    }

    fn tqe_cost(&self, slice: &PackedPGSlice, tqe: TQE, stop: SliceIndex) -> f64 {
        let (gate_z_q0, gate_x_q0, gate_z_q1, gate_x_q1) = u8_tqe_to_u64(tqe.gate_type);
        let mut cost = self.packed_cost(
            slice.split_ref(tqe.q0, tqe.q1, stop),
            gate_z_q0,
            gate_x_q0,
            gate_z_q1,
            gate_x_q1,
        );
        let (tableau, weight) = if stop.is_end() {
            (slice.trailing_tableau(), self.metadata.tableau_weight)
        } else {
            let PackedOpMeta::BlackBox(black_box) = slice.op(stop) else {
                panic!("cost stop does not identify a black box")
            };
            (black_box.tableau(), self.metadata.black_box_weights[&stop])
        };
        cost += weight * self.tqe_tableau_cost(tableau, tqe);
        cost
    }

    fn tqe_tableau_cost(&self, tableau: &Tableau, tqe: TQE) -> f64 {
        let (gate_z_q0, gate_x_q0, gate_z_q1, gate_x_q1) = u8_tqe_to_u64(tqe.gate_type);
        self.kernel.unweighted_pair_cost(
            &tableau.qubit_slices_z_bits()[tqe.q0],
            &tableau.qubit_slices_x_bits()[tqe.q0],
            &tableau.qubit_slices_z_bits()[tqe.q1],
            &tableau.qubit_slices_x_bits()[tqe.q1],
            gate_z_q0,
            gate_x_q0,
            gate_z_q1,
            gate_x_q1,
        ) as f64
    }
}
