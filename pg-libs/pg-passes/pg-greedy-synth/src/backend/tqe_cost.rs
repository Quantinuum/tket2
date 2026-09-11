//! TQE cost changes computed with scalar and SIMD operations.

use std::ops::{BitAnd, BitOr, BitXor, Not, Shr};

#[cfg(feature = "simd")]
use super::SimdBackend;
use super::{CostKernel, ScalarBackend};
#[cfg(feature = "simd")]
use std::simd::{LaneCount, Simd, SupportedLaneCount, num::SimdUint};

// Each pair stores its left string first and its right string in the following
// packed position.
const LEFT_STRING_MASK: u64 = 0x5555_5555_5555_5555;
const RIGHT_STRING_MASK: u64 = 0xAAAA_AAAA_AAAA_AAAA;

#[cfg(feature = "simd")]
const SIMD_LANES: usize = 64;

/// Computes masks for single strings whose support cost increases or decreases
/// after a TQE.
#[inline(always)]
fn single_cost_changes<T>(
    zb_q0: T,
    xb_q0: T,
    zb_q1: T,
    xb_q1: T,
    gate_z_q0: T,
    gate_x_q0: T,
    gate_z_q1: T,
    gate_x_q1: T,
) -> (T, T)
where
    T: BitXor<Output = T> + BitAnd<Output = T> + Not<Output = T> + BitOr<Output = T> + Copy,
{
    let anticommutes_q0 = (gate_z_q0 & xb_q0) ^ (gate_x_q0 & zb_q0);
    let anticommutes_q1 = (gate_z_q1 & xb_q1) ^ (gate_x_q1 & zb_q1);

    let new_zb_q0 = (gate_z_q0 & anticommutes_q1) ^ zb_q0;
    let new_xb_q0 = (gate_x_q0 & anticommutes_q1) ^ xb_q0;
    let new_zb_q1 = (gate_z_q1 & anticommutes_q0) ^ zb_q1;
    let new_xb_q1 = (gate_x_q1 & anticommutes_q0) ^ xb_q1;

    let occupied_q0 = zb_q0 | xb_q0;
    let occupied_q1 = zb_q1 | xb_q1;
    let new_occupied_q0 = new_zb_q0 | new_xb_q0;
    let new_occupied_q1 = new_zb_q1 | new_xb_q1;

    let increase = (new_occupied_q0 & !occupied_q0) | (new_occupied_q1 & !occupied_q1);
    let decrease = (!new_occupied_q0 & occupied_q0) | (!new_occupied_q1 & occupied_q1);

    (increase & !decrease, !increase & decrease)
}

/// Computes masks that show whether the left and right Pauli strings
/// anticommute or form a commuting nonidentity pair on each qubit.
#[inline(always)]
fn pair_support_masks<T>(
    zb_q0: T,
    xb_q0: T,
    zb_q1: T,
    xb_q1: T,
    left_string_mask: T,
    right_string_mask: T,
) -> (T, T, T, T)
where
    T: BitXor<Output = T>
        + BitAnd<Output = T>
        + Not<Output = T>
        + BitOr<Output = T>
        + Copy
        + Shr<u64, Output = T>,
{
    let left_z_q0 = zb_q0 & left_string_mask;
    let right_z_q0 = (zb_q0 & right_string_mask) >> 1;
    let left_x_q0 = xb_q0 & left_string_mask;
    let right_x_q0 = (xb_q0 & right_string_mask) >> 1;
    let left_z_q1 = zb_q1 & left_string_mask;
    let right_z_q1 = (zb_q1 & right_string_mask) >> 1;
    let left_x_q1 = xb_q1 & left_string_mask;
    let right_x_q1 = (xb_q1 & right_string_mask) >> 1;

    let anticommuting_q0 = (left_z_q0 & right_x_q0) ^ (right_z_q0 & left_x_q0);
    let anticommuting_q1 = (left_z_q1 & right_x_q1) ^ (right_z_q1 & left_x_q1);
    let occupied_q0 = left_z_q0 | left_x_q0 | right_z_q0 | right_x_q0;
    let occupied_q1 = left_z_q1 | left_x_q1 | right_z_q1 | right_x_q1;
    let commuting_nonidentity_q0 = !anticommuting_q0 & occupied_q0;
    let commuting_nonidentity_q1 = !anticommuting_q1 & occupied_q1;

    (
        anticommuting_q0,
        anticommuting_q1,
        commuting_nonidentity_q0,
        commuting_nonidentity_q1,
    )
}

/// Computes masks for paired strings whose reduction cost increases or
/// decreases after a TQE.
#[inline(always)]
fn pair_cost_changes<T>(
    zb_q0: T,
    xb_q0: T,
    zb_q1: T,
    xb_q1: T,
    gate_z_q0: T,
    gate_x_q0: T,
    gate_z_q1: T,
    gate_x_q1: T,
    left_string_mask: T,
    right_string_mask: T,
) -> (T, T)
where
    T: BitXor<Output = T>
        + BitAnd<Output = T>
        + Not<Output = T>
        + BitOr<Output = T>
        + Copy
        + Shr<u64, Output = T>,
{
    let anticommutes_q0 = (gate_z_q0 & xb_q0) ^ (gate_x_q0 & zb_q0);
    let anticommutes_q1 = (gate_z_q1 & xb_q1) ^ (gate_x_q1 & zb_q1);

    let new_zb_q0 = (gate_z_q0 & anticommutes_q1) ^ zb_q0;
    let new_xb_q0 = (gate_x_q0 & anticommutes_q1) ^ xb_q0;
    let new_zb_q1 = (gate_z_q1 & anticommutes_q0) ^ zb_q1;
    let new_xb_q1 = (gate_x_q1 & anticommutes_q0) ^ xb_q1;

    let (anticommuting_q0, anticommuting_q1, commuting_nonidentity_q0, commuting_nonidentity_q1) =
        pair_support_masks(
            zb_q0,
            xb_q0,
            zb_q1,
            xb_q1,
            left_string_mask,
            right_string_mask,
        );
    let (
        new_anticommuting_q0,
        new_anticommuting_q1,
        new_commuting_nonidentity_q0,
        new_commuting_nonidentity_q1,
    ) = pair_support_masks(
        new_zb_q0,
        new_xb_q0,
        new_zb_q1,
        new_xb_q1,
        left_string_mask,
        right_string_mask,
    );

    // The cost increases when both local pairs become anticommuting or when
    // one local pair becomes commuting and nonidentity. Reversing either
    // change decreases the cost. Clifford conjugation preserves
    // anticommutation parity, so these are the only possible changes.
    let both_become_anticommuting =
        new_anticommuting_q0 & !anticommuting_q0 & new_anticommuting_q1 & !anticommuting_q1;
    let both_stop_being_anticommuting =
        !new_anticommuting_q0 & anticommuting_q0 & !new_anticommuting_q1 & anticommuting_q1;
    let one_becomes_commuting_nonidentity = (new_commuting_nonidentity_q0
        & !commuting_nonidentity_q0
        & !(new_commuting_nonidentity_q1 ^ commuting_nonidentity_q1))
        | (new_commuting_nonidentity_q1
            & !commuting_nonidentity_q1
            & !(new_commuting_nonidentity_q0 ^ commuting_nonidentity_q0));
    let one_stops_being_commuting_nonidentity = (!new_commuting_nonidentity_q0
        & commuting_nonidentity_q0
        & !(new_commuting_nonidentity_q1 ^ commuting_nonidentity_q1))
        | (!new_commuting_nonidentity_q1
            & commuting_nonidentity_q1
            & !(new_commuting_nonidentity_q0 ^ commuting_nonidentity_q0));

    (
        both_become_anticommuting | one_becomes_commuting_nonidentity,
        both_stop_being_anticommuting | one_stops_being_commuting_nonidentity,
    )
}

/// Computes single string cost masks.
fn single_cost_masks(
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
) {
    for index in 0..zb_q0.len() {
        let (increase, decrease) = single_cost_changes(
            zb_q0[index],
            xb_q0[index],
            zb_q1[index],
            xb_q1[index],
            gate_z_q0,
            gate_x_q0,
            gate_z_q1,
            gate_x_q1,
        );
        increases[index] = increase & mask[index];
        decreases[index] = decrease & mask[index];
    }
}

/// Computes paired string cost masks.
fn pair_cost_masks(
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
) {
    for index in 0..zb_q0.len() {
        let (increase, decrease) = pair_cost_changes(
            zb_q0[index],
            xb_q0[index],
            zb_q1[index],
            xb_q1[index],
            gate_z_q0,
            gate_x_q0,
            gate_z_q1,
            gate_x_q1,
            LEFT_STRING_MASK,
            RIGHT_STRING_MASK,
        );
        increases[index] = increase & mask[index];
        decreases[index] = decrease & mask[index];
    }
}

/// Computes the total unweighted cost change for paired strings.
fn unweighted_pair_cost(
    zb_q0: &[u64],
    xb_q0: &[u64],
    zb_q1: &[u64],
    xb_q1: &[u64],
    gate_z_q0: u64,
    gate_x_q0: u64,
    gate_z_q1: u64,
    gate_x_q1: u64,
) -> i64 {
    let mut increases = 0;
    let mut decreases = 0;

    for index in 0..zb_q0.len() {
        let (increase, decrease) = pair_cost_changes(
            zb_q0[index],
            xb_q0[index],
            zb_q1[index],
            xb_q1[index],
            gate_z_q0,
            gate_x_q0,
            gate_z_q1,
            gate_x_q1,
            LEFT_STRING_MASK,
            RIGHT_STRING_MASK,
        );
        increases += increase.count_ones() as i64;
        decreases += decrease.count_ones() as i64;
    }

    increases - decreases
}

/// Computes single string cost masks for one SIMD chunk.
#[cfg(feature = "simd")]
fn single_cost_chunk<const N: usize>(
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
) where
    LaneCount<N>: SupportedLaneCount,
{
    let (increase, decrease) = single_cost_changes(
        Simd::<u64, N>::from_slice(zb_q0),
        Simd::<u64, N>::from_slice(xb_q0),
        Simd::<u64, N>::from_slice(zb_q1),
        Simd::<u64, N>::from_slice(xb_q1),
        Simd::splat(gate_z_q0),
        Simd::splat(gate_x_q0),
        Simd::splat(gate_z_q1),
        Simd::splat(gate_x_q1),
    );
    let mask = Simd::<u64, N>::from_slice(mask);
    (increase & mask).copy_to_slice(increases);
    (decrease & mask).copy_to_slice(decreases);
}

/// Computes paired string cost masks for one SIMD chunk.
#[cfg(feature = "simd")]
fn pair_cost_chunk<const N: usize>(
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
) where
    LaneCount<N>: SupportedLaneCount,
{
    let (increase, decrease) = pair_cost_changes(
        Simd::<u64, N>::from_slice(zb_q0),
        Simd::<u64, N>::from_slice(xb_q0),
        Simd::<u64, N>::from_slice(zb_q1),
        Simd::<u64, N>::from_slice(xb_q1),
        Simd::splat(gate_z_q0),
        Simd::splat(gate_x_q0),
        Simd::splat(gate_z_q1),
        Simd::splat(gate_x_q1),
        Simd::splat(LEFT_STRING_MASK),
        Simd::splat(RIGHT_STRING_MASK),
    );
    let mask = Simd::<u64, N>::from_slice(mask);
    (increase & mask).copy_to_slice(increases);
    (decrease & mask).copy_to_slice(decreases);
}

/// Computes the unweighted cost change for paired strings in one SIMD chunk.
#[cfg(feature = "simd")]
fn unweighted_pair_cost_chunk<const N: usize>(
    zb_q0: &[u64],
    xb_q0: &[u64],
    zb_q1: &[u64],
    xb_q1: &[u64],
    gate_z_q0: u64,
    gate_x_q0: u64,
    gate_z_q1: u64,
    gate_x_q1: u64,
) -> i64
where
    LaneCount<N>: SupportedLaneCount,
{
    let (increase, decrease) = pair_cost_changes(
        Simd::<u64, N>::from_slice(zb_q0),
        Simd::<u64, N>::from_slice(xb_q0),
        Simd::<u64, N>::from_slice(zb_q1),
        Simd::<u64, N>::from_slice(xb_q1),
        Simd::splat(gate_z_q0),
        Simd::splat(gate_x_q0),
        Simd::splat(gate_z_q1),
        Simd::splat(gate_x_q1),
        Simd::splat(LEFT_STRING_MASK),
        Simd::splat(RIGHT_STRING_MASK),
    );
    increase.count_ones().reduce_sum() as i64 - decrease.count_ones().reduce_sum() as i64
}

/// Computes single string cost masks across all SIMD chunks.
#[cfg(feature = "simd")]
fn single_cost_masks_simd(
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
) {
    let complete_chunks = zb_q0.len() / SIMD_LANES;
    for chunk in 0..complete_chunks {
        let start = chunk * SIMD_LANES;
        let end = start + SIMD_LANES;
        single_cost_chunk::<SIMD_LANES>(
            &zb_q0[start..end],
            &xb_q0[start..end],
            &zb_q1[start..end],
            &xb_q1[start..end],
            gate_z_q0,
            gate_x_q0,
            gate_z_q1,
            gate_x_q1,
            &mask[start..end],
            &mut increases[start..end],
            &mut decreases[start..end],
        );
    }
    let base = complete_chunks * SIMD_LANES;
    let remainder = zb_q0.len() - base;
    if remainder != 0 {
        dispatch_simd_tail!(
            remainder,
            single_cost_chunk(
                &zb_q0[base..],
                &xb_q0[base..],
                &zb_q1[base..],
                &xb_q1[base..],
                gate_z_q0,
                gate_x_q0,
                gate_z_q1,
                gate_x_q1,
                &mask[base..],
                &mut increases[base..],
                &mut decreases[base..],
            )
        );
    }
}

/// Computes paired string cost masks across all SIMD chunks.
#[cfg(feature = "simd")]
fn pair_cost_masks_simd(
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
) {
    let complete_chunks = zb_q0.len() / SIMD_LANES;
    for chunk in 0..complete_chunks {
        let start = chunk * SIMD_LANES;
        let end = start + SIMD_LANES;
        pair_cost_chunk::<SIMD_LANES>(
            &zb_q0[start..end],
            &xb_q0[start..end],
            &zb_q1[start..end],
            &xb_q1[start..end],
            gate_z_q0,
            gate_x_q0,
            gate_z_q1,
            gate_x_q1,
            &mask[start..end],
            &mut increases[start..end],
            &mut decreases[start..end],
        );
    }
    let base = complete_chunks * SIMD_LANES;
    let remainder = zb_q0.len() - base;
    if remainder != 0 {
        dispatch_simd_tail!(
            remainder,
            pair_cost_chunk(
                &zb_q0[base..],
                &xb_q0[base..],
                &zb_q1[base..],
                &xb_q1[base..],
                gate_z_q0,
                gate_x_q0,
                gate_z_q1,
                gate_x_q1,
                &mask[base..],
                &mut increases[base..],
                &mut decreases[base..],
            )
        );
    }
}

/// Computes the total unweighted cost change for paired strings across all
/// SIMD chunks.
#[cfg(feature = "simd")]
fn unweighted_pair_cost_simd(
    zb_q0: &[u64],
    xb_q0: &[u64],
    zb_q1: &[u64],
    xb_q1: &[u64],
    gate_z_q0: u64,
    gate_x_q0: u64,
    gate_z_q1: u64,
    gate_x_q1: u64,
) -> i64 {
    let complete_chunks = zb_q0.len() / SIMD_LANES;
    let mut cost = 0;
    for chunk in 0..complete_chunks {
        let start = chunk * SIMD_LANES;
        let end = start + SIMD_LANES;
        cost += unweighted_pair_cost_chunk::<SIMD_LANES>(
            &zb_q0[start..end],
            &xb_q0[start..end],
            &zb_q1[start..end],
            &xb_q1[start..end],
            gate_z_q0,
            gate_x_q0,
            gate_z_q1,
            gate_x_q1,
        );
    }
    let base = complete_chunks * SIMD_LANES;
    let remainder = zb_q0.len() - base;
    if remainder != 0 {
        cost += dispatch_simd_tail!(
            remainder,
            unweighted_pair_cost_chunk(
                &zb_q0[base..],
                &xb_q0[base..],
                &zb_q1[base..],
                &xb_q1[base..],
                gate_z_q0,
                gate_x_q0,
                gate_z_q1,
                gate_x_q1,
            )
        );
    }
    cost
}

impl CostKernel for ScalarBackend {
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
    ) {
        single_cost_masks(
            zb_q0, xb_q0, zb_q1, xb_q1, gate_z_q0, gate_x_q0, gate_z_q1, gate_x_q1, mask,
            increases, decreases,
        )
    }

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
    ) {
        pair_cost_masks(
            zb_q0, xb_q0, zb_q1, xb_q1, gate_z_q0, gate_x_q0, gate_z_q1, gate_x_q1, mask,
            increases, decreases,
        )
    }

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
    ) -> i64 {
        unweighted_pair_cost(
            zb_q0, xb_q0, zb_q1, xb_q1, gate_z_q0, gate_x_q0, gate_z_q1, gate_x_q1,
        )
    }
}

#[cfg(feature = "simd")]
impl CostKernel for SimdBackend {
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
    ) {
        single_cost_masks_simd(
            zb_q0, xb_q0, zb_q1, xb_q1, gate_z_q0, gate_x_q0, gate_z_q1, gate_x_q1, mask,
            increases, decreases,
        )
    }

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
    ) {
        pair_cost_masks_simd(
            zb_q0, xb_q0, zb_q1, xb_q1, gate_z_q0, gate_x_q0, gate_z_q1, gate_x_q1, mask,
            increases, decreases,
        )
    }

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
    ) -> i64 {
        unweighted_pair_cost_simd(
            zb_q0, xb_q0, zb_q1, xb_q1, gate_z_q0, gate_x_q0, gate_z_q1, gate_x_q1,
        )
    }
}
