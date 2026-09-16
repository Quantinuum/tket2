//! Packed conjugation and candidate cost backends.
//!
//! `PackedBackend` provides scalar and SIMD implementations for tableau and
//! packed bit operations.
//! `CostKernel` handles the intermediate cost calculations.
//! `WeightedSumStrategy` combines the cost changes into the final TQE cost
//! using a weighted sum.
//!
//! The backend data flow is:
//!
//! ```text
//! Reducer
//! ├── PackedBackend
//! │   └── applies the selected TQE
//! │
//! └── TQECostBackend
//!     └── GreedyCostBackend<K, W>
//!         ├── CostKernel K
//!         │   └── produces increase and decrease masks
//!         └── WeightedSumStrategy W
//!             └── computes the final TQE cost
//! ```

#[cfg(feature = "simd")]
macro_rules! dispatch_simd_tail {
    ($lanes:expr, $function:ident($($argument:expr),* $(,)?)) => {
        match $lanes {
            1 => $function::<1>($($argument),*),
            2 => $function::<2>($($argument),*),
            3 => $function::<3>($($argument),*),
            4 => $function::<4>($($argument),*),
            5 => $function::<5>($($argument),*),
            6 => $function::<6>($($argument),*),
            7 => $function::<7>($($argument),*),
            8 => $function::<8>($($argument),*),
            9 => $function::<9>($($argument),*),
            10 => $function::<10>($($argument),*),
            11 => $function::<11>($($argument),*),
            12 => $function::<12>($($argument),*),
            13 => $function::<13>($($argument),*),
            14 => $function::<14>($($argument),*),
            15 => $function::<15>($($argument),*),
            16 => $function::<16>($($argument),*),
            17 => $function::<17>($($argument),*),
            18 => $function::<18>($($argument),*),
            19 => $function::<19>($($argument),*),
            20 => $function::<20>($($argument),*),
            21 => $function::<21>($($argument),*),
            22 => $function::<22>($($argument),*),
            23 => $function::<23>($($argument),*),
            24 => $function::<24>($($argument),*),
            25 => $function::<25>($($argument),*),
            26 => $function::<26>($($argument),*),
            27 => $function::<27>($($argument),*),
            28 => $function::<28>($($argument),*),
            29 => $function::<29>($($argument),*),
            30 => $function::<30>($($argument),*),
            31 => $function::<31>($($argument),*),
            32 => $function::<32>($($argument),*),
            33 => $function::<33>($($argument),*),
            34 => $function::<34>($($argument),*),
            35 => $function::<35>($($argument),*),
            36 => $function::<36>($($argument),*),
            37 => $function::<37>($($argument),*),
            38 => $function::<38>($($argument),*),
            39 => $function::<39>($($argument),*),
            40 => $function::<40>($($argument),*),
            41 => $function::<41>($($argument),*),
            42 => $function::<42>($($argument),*),
            43 => $function::<43>($($argument),*),
            44 => $function::<44>($($argument),*),
            45 => $function::<45>($($argument),*),
            46 => $function::<46>($($argument),*),
            47 => $function::<47>($($argument),*),
            48 => $function::<48>($($argument),*),
            49 => $function::<49>($($argument),*),
            50 => $function::<50>($($argument),*),
            51 => $function::<51>($($argument),*),
            52 => $function::<52>($($argument),*),
            53 => $function::<53>($($argument),*),
            54 => $function::<54>($($argument),*),
            55 => $function::<55>($($argument),*),
            56 => $function::<56>($($argument),*),
            57 => $function::<57>($($argument),*),
            58 => $function::<58>($($argument),*),
            59 => $function::<59>($($argument),*),
            60 => $function::<60>($($argument),*),
            61 => $function::<61>($($argument),*),
            62 => $function::<62>($($argument),*),
            63 => $function::<63>($($argument),*),
            _ => unreachable!("SIMD tail must contain between 1 and 63 items"),
        }
    };
}

mod greedy;
mod scalar;
#[cfg(feature = "simd")]
mod simd;
mod tqe_cost;
mod weighted_sum;

pub(crate) use greedy::GreedyCostBackend;
pub(crate) use scalar::ScalarBackend;
#[cfg(feature = "simd")]
pub(crate) use simd::SimdBackend;
#[cfg(feature = "simd")]
pub(crate) use weighted_sum::ExpandedSimdWeightedSum;
pub(crate) use weighted_sum::GroupedWeightedSum;
pub(crate) use weighted_sum::{ExpandedSparseWeightedSum, WeightedSumStrategy};

const ALPHA: f64 = 0.588;

/// Raw operations for computing intermediate costs.
pub(crate) trait CostKernel {
    /// Marks single string columns whose support cost increases or decreases
    /// after a TQE. Each cost can change only by +1, 0, or -1. The four Pauli
    /// indicators `gate_z_q0`, `gate_x_q0`, `gate_z_q1`, and `gate_x_q1`
    /// specify the TQE.
    fn single_cost_masks(
        &self,
        zb_q0: &[u64],
        xb_q0: &[u64],
        zb_q1: &[u64],
        xb_q1: &[u64],
        gate_z_q0: u64,
        gate_x_q0: u64,
        gate_z_q1: u64,
        gate_x_q1: u64,
        mask: &[u64],
        increases: &mut [u64],
        decreases: &mut [u64],
    );

    /// Marks aligned pairs of columns whose reduction cost increases or
    /// decreases after a TQE. Each cost can change only by +1, 0, or -1. The
    /// four Pauli indicators `gate_z_q0`, `gate_x_q0`, `gate_z_q1`, and
    /// `gate_x_q1` specify the TQE.
    fn pair_cost_masks(
        &self,
        zb_q0: &[u64],
        xb_q0: &[u64],
        zb_q1: &[u64],
        xb_q1: &[u64],
        gate_z_q0: u64,
        gate_x_q0: u64,
        gate_z_q1: u64,
        gate_x_q1: u64,
        mask: &[u64],
        increases: &mut [u64],
        decreases: &mut [u64],
    );

    /// Computes the signed unweighted cost change for paired strings.
    fn unweighted_pair_cost(
        &self,
        zb_q0: &[u64],
        xb_q0: &[u64],
        zb_q1: &[u64],
        xb_q1: &[u64],
        gate_z_q0: u64,
        gate_x_q0: u64,
        gate_z_q1: u64,
        gate_x_q1: u64,
    ) -> i64;
}

use crate::packed_pg_slice::{PackedPGSlice, SliceIndex};
use crate::tqe::TQE;
use pg_qm_tableau::Tableau;

/// Interface for a TQE cost backend.
pub(crate) trait TQECostBackend {
    /// Updates the backend state for the given packed slice.
    fn update(&mut self, slice: &PackedPGSlice);

    /// Computes the TQE cost for the part of `slice` up to `stop`.
    fn tqe_cost(&self, slice: &PackedPGSlice, tqe: TQE, stop: SliceIndex) -> f64;

    /// Computes the TQE cost for the given tableau.
    fn tqe_tableau_cost(&self, tableau: &Tableau, tqe: TQE) -> f64;
}

#[cfg(test)]
mod tests;
