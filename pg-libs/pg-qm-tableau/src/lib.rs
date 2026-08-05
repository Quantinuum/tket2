//! Clifford tableau
#![cfg_attr(feature = "simd", feature(portable_simd))]
#[cfg(feature = "simd")]
use pg_bitpacked::{
    apply_enum_tqe_simd, apply_half_pi_gate_simd, simd_h_gate, simd_s_gate, simd_sdg_gate,
    simd_v_gate, simd_vdg_gate, simd_x_gate, simd_xx_gate, simd_xy_gate, simd_xz_gate, simd_y_gate,
    simd_yy_gate, simd_yz_gate, simd_z_gate, simd_zz_gate,
};
use pg_bitpacked::{
    apply_enum_tqe_slice, apply_half_pi_gate_slice, paulis_to_u64s, slice_h_gate, slice_s_gate,
    slice_sdg_gate, slice_v_gate, slice_vdg_gate, slice_x_gate, slice_xx_gate, slice_xy_gate,
    slice_xz_gate, slice_y_gate, slice_yy_gate, slice_yz_gate, slice_z_gate, slice_zz_gate,
    u64s_to_paulis,
};
use pg_core::Pauli;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use std::fmt;
#[cfg(feature = "simd")]
use std::simd::num::SimdUint;
#[cfg(feature = "simd")]
use std::simd::{LaneCount, Simd, SupportedLaneCount};

mod converter;

/// Hard-coded n-qubit threshold for when to use multi-threading.
/// This value was chosen based on earlier benchmarking and may need re-tuning.
const MT_THRESH: usize = 500;

// Type aliases for function pointers to various gate application functions
// These are used to allow for different implementations (e.g., SIMD vs. non-SIMD) to be used interchangeably.
type TqeGateFn = fn(&mut [u64], &mut [u64], &mut [u64], &mut [u64], &mut [u64]);
type SqGateFn = fn(&mut [u64], &mut [u64], &mut [u64]);
type PauliGateFn = fn(&[u64], &[u64], &mut [u64]);
type ApplyEnumTqeFn = fn(&mut [u64], &mut [u64], &mut [u64], &mut [u64], &mut [u64], Pauli, Pauli);
type ApplyHalfPiFn = fn(&mut [u64], &mut [u64], &mut [u64], Pauli, bool);
type StringMulFn = fn(&mut [u64], &mut [u64], &[u64], &[u64]) -> u8;
type CountYFn = fn(&[u64], &[u64]) -> u64;

/// Returns mutable references to the elements at indices `i` and `j`.
///
/// # Arguments
///
/// * `v` - A mutable slice.
/// * `i` - The first index.
/// * `j` - The second index.
///
/// # Returns
///
/// A tuple containing mutable references to the elements at indices `i` and `j`.
fn get_two_mut<T>(v: &mut [T], i: usize, j: usize) -> (&mut T, &mut T) {
    let (a, b) = if i < j {
        let (left, right) = v.split_at_mut(j);
        (&mut left[i], &mut right[0])
    } else {
        let (left, right) = v.split_at_mut(i);
        (&mut right[0], &mut left[j])
    };
    (a, b)
}

/// Panics with a clear message if `q0` and `q1` are not distinct qubit indices.
fn check_distinct_qubits(q0: usize, q1: usize) {
    if q0 == q1 {
        panic!("Expected two distinct qubit indices, but both q0 and q1 are {q0}");
    }
}

/// Multiplies two bit-packed Pauli strings, computing the phase.
/// We count the number of anti-commuting sites and the number of anti-cyclic sites (e.g. YX = -iZ)
/// to determine the phase factor.
///
/// # Arguments
///
/// * `lhs_z_bits` - Mutable Z-bits of the first Pauli string (modified in-place)
/// * `lhs_x_bits` - Mutable X-bits of the first Pauli string (modified in-place)
/// * `rhs_z_bits` - Z-bits of the second Pauli string
/// * `rhs_x_bits` - X-bits of the second Pauli string
///
/// # Returns
///
/// The phase factor as the position in [1, i, -1, -i]
///
fn string_mul(
    lhs_z_bits: &mut [u64],
    lhs_x_bits: &mut [u64],
    rhs_z_bits: &[u64],
    rhs_x_bits: &[u64],
) -> u8 {
    let len = lhs_z_bits.len();
    let mut n_anti_commute_sites: u64 = 0;
    let mut n_anti_cyclic_sites: u64 = 0;
    for idx in 0..len {
        let lhs_z_word = lhs_z_bits[idx];
        let lhs_x_word = lhs_x_bits[idx];
        let rhs_z_word = rhs_z_bits[idx];
        let rhs_x_word = rhs_x_bits[idx];
        lhs_z_bits[idx] ^= rhs_z_word;
        lhs_x_bits[idx] ^= rhs_x_word;
        let lhs_z_rhs_x_word = lhs_z_word & rhs_x_word;
        let rhs_z_lhs_x_word = rhs_z_word & lhs_x_word;
        let anti_commute_word = lhs_z_rhs_x_word ^ rhs_z_lhs_x_word;
        let anti_cyclic_word = (rhs_z_lhs_x_word & !lhs_z_word & !rhs_x_word)
            | (lhs_z_rhs_x_word & (lhs_x_word ^ rhs_z_word));
        n_anti_commute_sites += anti_commute_word.count_ones() as u64;
        n_anti_cyclic_sites += anti_cyclic_word.count_ones() as u64;
    }
    ((n_anti_commute_sites as i64 - 2 * n_anti_cyclic_sites as i64).rem_euclid(4)) as u8
}

/// Given a Pauli string, count the number of Ys
fn count_y(input_z_bits: &[u64], input_x_bits: &[u64]) -> u64 {
    let len = input_z_bits.len();
    let mut n_ys = 0;
    for idx in 0..len {
        n_ys += (input_z_bits[idx] & input_x_bits[idx]).count_ones() as u64;
    }
    n_ys
}

/// Multiplies two bit-packed Pauli strings using SIMD operations, computing the phase.
#[cfg(feature = "simd")]
fn simd_string_mul<const N: usize>(
    lhs_z_bits: &mut [u64],
    lhs_x_bits: &mut [u64],
    rhs_z_bits: &[u64],
    rhs_x_bits: &[u64],
) -> u8
where
    LaneCount<N>: SupportedLaneCount,
{
    let len = lhs_z_bits.len();
    let chunks = len / N;
    let mut n_anti_commute_sites: u64 = 0;
    let mut n_anti_cyclic_sites: u64 = 0;
    for i in 0..chunks {
        let lhs_z_chunk = &mut lhs_z_bits[i * N..(i + 1) * N];
        let lhs_x_chunk = &mut lhs_x_bits[i * N..(i + 1) * N];
        let rhs_z_chunk = &rhs_z_bits[i * N..(i + 1) * N];
        let rhs_x_chunk = &rhs_x_bits[i * N..(i + 1) * N];
        let lhs_z_simd = Simd::<u64, N>::from_slice(lhs_z_chunk);
        let lhs_x_simd = Simd::<u64, N>::from_slice(lhs_x_chunk);
        let rhs_z_simd = Simd::<u64, N>::from_slice(rhs_z_chunk);
        let rhs_x_simd = Simd::<u64, N>::from_slice(rhs_x_chunk);
        let result_z_simd = lhs_z_simd ^ rhs_z_simd;
        let result_x_simd = lhs_x_simd ^ rhs_x_simd;
        result_z_simd.copy_to_slice(lhs_z_chunk);
        result_x_simd.copy_to_slice(lhs_x_chunk);
        let lhs_z_rhs_x_simd = lhs_z_simd & rhs_x_simd;
        let rhs_z_lhs_x_simd = rhs_z_simd & lhs_x_simd;
        let anti_commute_simd = lhs_z_rhs_x_simd ^ rhs_z_lhs_x_simd;
        let anti_cyclic_simd = (rhs_z_lhs_x_simd & !lhs_z_simd & !rhs_x_simd)
            | (lhs_z_rhs_x_simd & (lhs_x_simd ^ rhs_z_simd));
        n_anti_commute_sites += anti_commute_simd.count_ones().reduce_sum();
        n_anti_cyclic_sites += anti_cyclic_simd.count_ones().reduce_sum();
    }
    // TODO: Use SIMD for the remaining elements.
    let start = chunks * N;
    for idx in start..len {
        let lhs_z_word = lhs_z_bits[idx];
        let lhs_x_word = lhs_x_bits[idx];
        let rhs_z_word = rhs_z_bits[idx];
        let rhs_x_word = rhs_x_bits[idx];
        lhs_z_bits[idx] ^= rhs_z_word;
        lhs_x_bits[idx] ^= rhs_x_word;
        let lhs_z_rhs_x_word = lhs_z_word & rhs_x_word;
        let rhs_z_lhs_x_word = rhs_z_word & lhs_x_word;
        let anti_commute_word = lhs_z_rhs_x_word ^ rhs_z_lhs_x_word;
        let anti_cyclic_word = (rhs_z_lhs_x_word & !lhs_z_word & !rhs_x_word)
            | (lhs_z_rhs_x_word & (lhs_x_word ^ rhs_z_word));
        n_anti_commute_sites += anti_commute_word.count_ones() as u64;
        n_anti_cyclic_sites += anti_cyclic_word.count_ones() as u64;
    }
    ((n_anti_commute_sites as i64 - 2 * n_anti_cyclic_sites as i64).rem_euclid(4)) as u8
}

/// Given a Pauli string, count the number of Ys using SIMD
#[cfg(feature = "simd")]
fn simd_count_y<const N: usize>(input_z_bits: &[u64], input_x_bits: &[u64]) -> u64
where
    LaneCount<N>: SupportedLaneCount,
{
    let len = input_z_bits.len();
    let chunks = len / N;

    let mut n_ys = 0;
    for i in 0..chunks {
        let z_chunk = &input_z_bits[i * N..(i + 1) * N];
        let x_chunk = &input_x_bits[i * N..(i + 1) * N];
        let z_simd = Simd::<u64, N>::from_slice(z_chunk);
        let x_simd = Simd::<u64, N>::from_slice(x_chunk);
        n_ys += (z_simd & x_simd).count_ones().reduce_sum();
    }
    // TODO: Use SIMD for the remaining elements.
    let start = chunks * N;
    for idx in start..len {
        let input_z_word = input_z_bits[idx];
        let input_x_word = input_x_bits[idx];
        n_ys += (input_z_word & input_x_word).count_ones() as u64;
    }
    n_ys
}

/// A qubit-major implementation of a unitary tableau.
/// # Example
/// |     | Z0 | X0 | Z1 | X1 |
/// | -------- | - | - | - | - |
/// | \(Q_0\)  | Z | X | Y | I |
/// | \(Q_1\)  | X | X | Z | X |
/// | Sign |+|+|+|-|
///
/// The X0 image, represented by a column, says that an X operator at qubit 0
/// on the input will be transformed into XX over the output.
///
/// Our tableau implementation is qubit(row)-major, and the Z bits and X bits of the Pauli letters
/// are stored separately. The tableau above is therefore stored, before bit packing, as:
/// qubit_slices_z_bits = \[
/// \[1,0,1,0\],
/// \[0,0,1,0\]
/// \]
/// qubit_slices_x_bits = \[
/// \[0,1,1,0\],
/// \[1,1,0,1\]
/// \]
/// sign_bits = \[0, 0, 0, 1\]
/// n_qubits = 2
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Tableau {
    qubit_slices_z_bits: Vec<Vec<u64>>,
    qubit_slices_x_bits: Vec<Vec<u64>>,
    sign_bits: Vec<u64>,
    n_qubits: usize,
}

impl fmt::Display for Tableau {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        for i in 0..self.n_qubits {
            let paulis = u64s_to_paulis(
                &self.qubit_slices_z_bits[i],
                &self.qubit_slices_x_bits[i],
                self.n_qubits * 2,
            );
            for p in paulis {
                write!(f, "{p} ")?;
            }
            writeln!(f)?;
        }
        let total_sign_bits = 2 * self.n_qubits;
        let mut bits_printed = 0;
        for s in &self.sign_bits {
            for i in 0..64 {
                if bits_printed == total_sign_bits {
                    break;
                }
                let bit = (s >> i) & 1;
                write!(f, "{} ", if bit == 1 { '-' } else { '+' })?;
                bits_printed += 1;
            }
            if bits_printed == total_sign_bits {
                break;
            }
        }
        writeln!(f)
    }
}

impl Tableau {
    /// Get the number of qubits in the tableau.
    pub fn get_n_qubits(&self) -> usize {
        self.n_qubits
    }
    /// Get the packed Z bits for each qubit slice.
    pub fn qubit_slices_z_bits(&self) -> &Vec<Vec<u64>> {
        &self.qubit_slices_z_bits
    }
    /// Get the packed X bits for each qubit slice.
    pub fn qubit_slices_x_bits(&self) -> &Vec<Vec<u64>> {
        &self.qubit_slices_x_bits
    }
    /// Get the sign bits of the tableau rows. 1 represents a negative sign, and 0 represents a positive sign.
    pub fn get_sign_bits(&self) -> &Vec<u64> {
        &self.sign_bits
    }

    /// Split the tableau into mutable references for two qubits.
    ///
    /// # Panics
    ///
    /// Panics if `q0 == q1`.
    pub fn split_qubit_slices_mut(
        &mut self,
        q0: usize,
        q1: usize,
    ) -> (
        &mut Vec<u64>,
        &mut Vec<u64>,
        &mut Vec<u64>,
        &mut Vec<u64>,
        &mut Vec<u64>,
    ) {
        check_distinct_qubits(q0, q1);
        let (q0_slice_z_bits, q1_slice_z_bits) = get_two_mut(&mut self.qubit_slices_z_bits, q0, q1);
        let (q0_slice_x_bits, q1_slice_x_bits) = get_two_mut(&mut self.qubit_slices_x_bits, q0, q1);
        (
            q0_slice_z_bits,
            q1_slice_z_bits,
            q0_slice_x_bits,
            q1_slice_x_bits,
            &mut self.sign_bits,
        )
    }

    /// Post-compose a TQE gate.
    fn postcompose_tqe_with(
        &mut self,
        g0: Pauli,
        g1: Pauli,
        q0: usize,
        q1: usize,
        xx_gate: TqeGateFn,
        xy_gate: TqeGateFn,
        xz_gate: TqeGateFn,
        yy_gate: TqeGateFn,
        yz_gate: TqeGateFn,
        zz_gate: TqeGateFn,
    ) {
        let (q0_slice_z_bits, q1_slice_z_bits) = get_two_mut(&mut self.qubit_slices_z_bits, q0, q1);
        let (q0_slice_x_bits, q1_slice_x_bits) = get_two_mut(&mut self.qubit_slices_x_bits, q0, q1);
        match (g0, g1) {
            (Pauli::X, Pauli::X) => xx_gate(
                q0_slice_z_bits,
                q0_slice_x_bits,
                q1_slice_z_bits,
                q1_slice_x_bits,
                &mut self.sign_bits,
            ),
            (Pauli::X, Pauli::Y) => xy_gate(
                q0_slice_z_bits,
                q0_slice_x_bits,
                q1_slice_z_bits,
                q1_slice_x_bits,
                &mut self.sign_bits,
            ),
            (Pauli::X, Pauli::Z) => xz_gate(
                q0_slice_z_bits,
                q0_slice_x_bits,
                q1_slice_z_bits,
                q1_slice_x_bits,
                &mut self.sign_bits,
            ),
            (Pauli::Y, Pauli::X) => xy_gate(
                q1_slice_z_bits,
                q1_slice_x_bits,
                q0_slice_z_bits,
                q0_slice_x_bits,
                &mut self.sign_bits,
            ),
            (Pauli::Y, Pauli::Y) => yy_gate(
                q0_slice_z_bits,
                q0_slice_x_bits,
                q1_slice_z_bits,
                q1_slice_x_bits,
                &mut self.sign_bits,
            ),
            (Pauli::Y, Pauli::Z) => yz_gate(
                q0_slice_z_bits,
                q0_slice_x_bits,
                q1_slice_z_bits,
                q1_slice_x_bits,
                &mut self.sign_bits,
            ),
            (Pauli::Z, Pauli::X) => xz_gate(
                q1_slice_z_bits,
                q1_slice_x_bits,
                q0_slice_z_bits,
                q0_slice_x_bits,
                &mut self.sign_bits,
            ),
            (Pauli::Z, Pauli::Y) => yz_gate(
                q1_slice_z_bits,
                q1_slice_x_bits,
                q0_slice_z_bits,
                q0_slice_x_bits,
                &mut self.sign_bits,
            ),
            (Pauli::Z, Pauli::Z) => zz_gate(
                q0_slice_z_bits,
                q0_slice_x_bits,
                q1_slice_z_bits,
                q1_slice_x_bits,
                &mut self.sign_bits,
            ),
            _ => panic!("Unexpected TQE gate!"),
        }
    }

    /// Pre-compose a TQE gate.
    fn precompose_tqe_with(
        &mut self,
        g0: Pauli,
        g1: Pauli,
        q0: usize,
        q1: usize,
        string_mul: StringMulFn,
    ) {
        let (g0, g1, q0, q1) = {
            if g0 <= g1 {
                (g0, g1, q0, q1)
            } else {
                (g1, g0, q1, q0)
            }
        };
        let z0_img_idx = 2 * q0;
        let x0_img_idx = 2 * q0 + 1;
        let z1_img_idx = 2 * q1;
        let x1_img_idx = 2 * q1 + 1;
        let (mut z0_img_z_bits, mut z0_img_x_bits, z0_img_sign_bit) = self.packed_image(z0_img_idx);
        let (mut x0_img_z_bits, mut x0_img_x_bits, x0_img_sign_bit) = self.packed_image(x0_img_idx);
        let (mut z1_img_z_bits, mut z1_img_x_bits, z1_img_sign_bit) = self.packed_image(z1_img_idx);
        let (mut x1_img_z_bits, mut x1_img_x_bits, x1_img_sign_bit) = self.packed_image(x1_img_idx);

        match (g0, g1) {
            (Pauli::X, Pauli::X) => {
                // z0 <- z0 * x1
                let phase = string_mul(
                    &mut z0_img_z_bits,
                    &mut z0_img_x_bits,
                    &x1_img_z_bits,
                    &x1_img_x_bits,
                );
                self.set_packed_image(
                    z0_img_idx,
                    &z0_img_z_bits,
                    &z0_img_x_bits,
                    z0_img_sign_bit ^ x1_img_sign_bit ^ (phase == 2),
                );
                // z1 <- z1 * x0
                let phase = string_mul(
                    &mut z1_img_z_bits,
                    &mut z1_img_x_bits,
                    &x0_img_z_bits,
                    &x0_img_x_bits,
                );
                self.set_packed_image(
                    z1_img_idx,
                    &z1_img_z_bits,
                    &z1_img_x_bits,
                    z1_img_sign_bit ^ x0_img_sign_bit ^ (phase == 2),
                );
            }
            (Pauli::X, Pauli::Y) => {
                // z0 <- z0 * y1 = i * z0 * x1 * z1
                let mut phase = string_mul(
                    &mut z0_img_z_bits,
                    &mut z0_img_x_bits,
                    &x1_img_z_bits,
                    &x1_img_x_bits,
                );
                phase += string_mul(
                    &mut z0_img_z_bits,
                    &mut z0_img_x_bits,
                    &z1_img_z_bits,
                    &z1_img_x_bits,
                );
                phase = (phase + 1) % 4;
                self.set_packed_image(
                    z0_img_idx,
                    &z0_img_z_bits,
                    &z0_img_x_bits,
                    z0_img_sign_bit ^ z1_img_sign_bit ^ x1_img_sign_bit ^ (phase == 2),
                );
                // z1 <- z1 * x0
                let phase = string_mul(
                    &mut z1_img_z_bits,
                    &mut z1_img_x_bits,
                    &x0_img_z_bits,
                    &x0_img_x_bits,
                );
                self.set_packed_image(
                    z1_img_idx,
                    &z1_img_z_bits,
                    &z1_img_x_bits,
                    z1_img_sign_bit ^ x0_img_sign_bit ^ (phase == 2),
                );
                // x1 <- x1 * x0
                let phase = string_mul(
                    &mut x1_img_z_bits,
                    &mut x1_img_x_bits,
                    &x0_img_z_bits,
                    &x0_img_x_bits,
                );
                self.set_packed_image(
                    x1_img_idx,
                    &x1_img_z_bits,
                    &x1_img_x_bits,
                    x1_img_sign_bit ^ x0_img_sign_bit ^ (phase == 2),
                );
            }
            (Pauli::X, Pauli::Z) => {
                // z0 <- z0 * z1
                let phase = string_mul(
                    &mut z0_img_z_bits,
                    &mut z0_img_x_bits,
                    &z1_img_z_bits,
                    &z1_img_x_bits,
                );
                self.set_packed_image(
                    z0_img_idx,
                    &z0_img_z_bits,
                    &z0_img_x_bits,
                    z0_img_sign_bit ^ z1_img_sign_bit ^ (phase == 2),
                );
                // x1 <- x1 * x0
                let phase = string_mul(
                    &mut x1_img_z_bits,
                    &mut x1_img_x_bits,
                    &x0_img_z_bits,
                    &x0_img_x_bits,
                );
                self.set_packed_image(
                    x1_img_idx,
                    &x1_img_z_bits,
                    &x1_img_x_bits,
                    x0_img_sign_bit ^ x1_img_sign_bit ^ (phase == 2),
                );
            }
            (Pauli::Y, Pauli::Y) => {
                // convert sign to phase for readability
                let z0_img_phase = (z0_img_sign_bit as u8) << 1;
                let x0_img_phase = (x0_img_sign_bit as u8) << 1;
                let z1_img_phase = (z1_img_sign_bit as u8) << 1;
                let x1_img_phase = (x1_img_sign_bit as u8) << 1;
                // y1 = i * x1 * z1
                let mut y1_img_z_bits = x1_img_z_bits;
                let mut y1_img_x_bits = x1_img_x_bits;
                let y1_img_phase = string_mul(
                    &mut y1_img_z_bits,
                    &mut y1_img_x_bits,
                    &z1_img_z_bits,
                    &z1_img_x_bits,
                ) + 1
                    + z1_img_phase
                    + x1_img_phase;
                // z0 <- z0 * y1
                let z0_img_phase_new = (string_mul(
                    &mut z0_img_z_bits,
                    &mut z0_img_x_bits,
                    &y1_img_z_bits,
                    &y1_img_x_bits,
                ) + z0_img_phase
                    + y1_img_phase)
                    % 4;
                self.set_packed_image(
                    z0_img_idx,
                    &z0_img_z_bits,
                    &z0_img_x_bits,
                    z0_img_phase_new == 2,
                );
                // x0 <- x0 * y1
                let x0_img_phase_new = (string_mul(
                    &mut x0_img_z_bits,
                    &mut x0_img_x_bits,
                    &y1_img_z_bits,
                    &y1_img_x_bits,
                ) + x0_img_phase
                    + y1_img_phase)
                    % 4;
                self.set_packed_image(
                    x0_img_idx,
                    &x0_img_z_bits,
                    &x0_img_x_bits,
                    x0_img_phase_new == 2,
                );
                // y0 = i * x0 * z0 = i * (x0 * y1) * (z0 * y1)
                let mut y0_img_z_bits = x0_img_z_bits;
                let mut y0_img_x_bits = x0_img_x_bits;
                let y0_img_phase = string_mul(
                    &mut y0_img_z_bits,
                    &mut y0_img_x_bits,
                    &z0_img_z_bits,
                    &z0_img_x_bits,
                ) + z0_img_phase_new
                    + x0_img_phase_new
                    + 1;
                // z1 <- z1 * y0
                let z1_img_phase_new = (string_mul(
                    &mut z1_img_z_bits,
                    &mut z1_img_x_bits,
                    &y0_img_z_bits,
                    &y0_img_x_bits,
                ) + z1_img_phase
                    + y0_img_phase)
                    % 4;
                self.set_packed_image(
                    z1_img_idx,
                    &z1_img_z_bits,
                    &z1_img_x_bits,
                    z1_img_phase_new == 2,
                );
                // x1 <- x1 * y0 = (-i * y1 * z1) * y0
                let mut x1_img_z_bits = y1_img_z_bits;
                let mut x1_img_x_bits = y1_img_x_bits;
                let x1_img_phase_new = (string_mul(
                    &mut x1_img_z_bits,
                    &mut x1_img_x_bits,
                    &z1_img_z_bits,
                    &z1_img_x_bits,
                ) + y1_img_phase
                    + z1_img_phase_new
                    + 3)
                    % 4;
                self.set_packed_image(
                    x1_img_idx,
                    &x1_img_z_bits,
                    &x1_img_x_bits,
                    x1_img_phase_new == 2,
                );
            }
            (Pauli::Y, Pauli::Z) => {
                // x1 <- x1 * y0 = i * x1 * x0 * z0
                let mut phase = string_mul(
                    &mut x1_img_z_bits,
                    &mut x1_img_x_bits,
                    &x0_img_z_bits,
                    &x0_img_x_bits,
                );
                phase += string_mul(
                    &mut x1_img_z_bits,
                    &mut x1_img_x_bits,
                    &z0_img_z_bits,
                    &z0_img_x_bits,
                );
                phase = (phase + 1) % 4;
                self.set_packed_image(
                    x1_img_idx,
                    &x1_img_z_bits,
                    &x1_img_x_bits,
                    x1_img_sign_bit ^ z0_img_sign_bit ^ x0_img_sign_bit ^ (phase == 2),
                );
                // z0 <- z0 * z1
                let phase = string_mul(
                    &mut z0_img_z_bits,
                    &mut z0_img_x_bits,
                    &z1_img_z_bits,
                    &z1_img_x_bits,
                );
                self.set_packed_image(
                    z0_img_idx,
                    &z0_img_z_bits,
                    &z0_img_x_bits,
                    z0_img_sign_bit ^ z1_img_sign_bit ^ (phase == 2),
                );
                // x0 <- x0 * z1
                let phase = string_mul(
                    &mut x0_img_z_bits,
                    &mut x0_img_x_bits,
                    &z1_img_z_bits,
                    &z1_img_x_bits,
                );
                self.set_packed_image(
                    x0_img_idx,
                    &x0_img_z_bits,
                    &x0_img_x_bits,
                    x0_img_sign_bit ^ z1_img_sign_bit ^ (phase == 2),
                );
            }
            (Pauli::Z, Pauli::Z) => {
                // x0 <- x0 * z1
                let phase = string_mul(
                    &mut x0_img_z_bits,
                    &mut x0_img_x_bits,
                    &z1_img_z_bits,
                    &z1_img_x_bits,
                );
                self.set_packed_image(
                    x0_img_idx,
                    &x0_img_z_bits,
                    &x0_img_x_bits,
                    x0_img_sign_bit ^ z1_img_sign_bit ^ (phase == 2),
                );
                // x1 <- x1 * z0
                let phase = string_mul(
                    &mut x1_img_z_bits,
                    &mut x1_img_x_bits,
                    &z0_img_z_bits,
                    &z0_img_x_bits,
                );
                self.set_packed_image(
                    x1_img_idx,
                    &x1_img_z_bits,
                    &x1_img_x_bits,
                    x1_img_sign_bit ^ z0_img_sign_bit ^ (phase == 2),
                );
            }
            _ => panic!("Unexpected TQE gate!"),
        }
    }

    /// Post compose a basis change gate.
    fn postcompose_basis_change_with(
        &mut self,
        axis: Pauli,
        q: usize,
        v_gate: SqGateFn,
        h_gate: SqGateFn,
        s_gate: SqGateFn,
    ) {
        match axis {
            Pauli::X => v_gate(
                &mut self.qubit_slices_z_bits[q],
                &mut self.qubit_slices_x_bits[q],
                &mut self.sign_bits,
            ),
            Pauli::Y => h_gate(
                &mut self.qubit_slices_z_bits[q],
                &mut self.qubit_slices_x_bits[q],
                &mut self.sign_bits,
            ),
            Pauli::Z => s_gate(
                &mut self.qubit_slices_z_bits[q],
                &mut self.qubit_slices_x_bits[q],
                &mut self.sign_bits,
            ),
            _ => panic!("Unexpected basis change gate!"),
        }
    }

    /// Pre compose a basis change gate.
    fn precompose_basis_change_with(
        &mut self,
        axis: Pauli,
        q: usize,
        dagger: bool,
        string_mul: StringMulFn,
    ) {
        let z_img_idx = 2 * q;
        let x_img_idx = 2 * q + 1;
        let (mut z_img_z_bits, mut z_img_x_bits, z_img_sign_bit) = self.packed_image(z_img_idx);
        let (mut x_img_z_bits, mut x_img_x_bits, x_img_sign_bit) = self.packed_image(x_img_idx);
        match axis {
            Pauli::X => {
                // v gate
                // x <- x
                // z <- -y = iz*x
                let mut phase = string_mul(
                    &mut z_img_z_bits,
                    &mut z_img_x_bits,
                    &x_img_z_bits,
                    &x_img_x_bits,
                );
                // 1 is i
                phase = (phase + 1) % 4;
                self.set_packed_image(
                    z_img_idx,
                    &z_img_z_bits,
                    &z_img_x_bits,
                    z_img_sign_bit ^ x_img_sign_bit ^ (phase == 2) ^ dagger,
                );
            }
            Pauli::Y => {
                // h gate
                self.set_packed_image(z_img_idx, &x_img_z_bits, &x_img_x_bits, x_img_sign_bit);
                self.set_packed_image(x_img_idx, &z_img_z_bits, &z_img_x_bits, z_img_sign_bit);
            }
            Pauli::Z => {
                // s gate
                // x <- y = ix*z
                // z <- z
                let mut phase = string_mul(
                    &mut x_img_z_bits,
                    &mut x_img_x_bits,
                    &z_img_z_bits,
                    &z_img_x_bits,
                );
                // 1 is i
                phase = (phase + 1) % 4;
                self.set_packed_image(
                    x_img_idx,
                    &x_img_z_bits,
                    &x_img_x_bits,
                    z_img_sign_bit ^ x_img_sign_bit ^ (phase == 2) ^ dagger,
                );
            }
            _ => panic!("Unexpected basis gate!"),
        }
    }

    /// Post compose a half pi Pauli rotation gate.
    fn postcompose_half_pi_with(
        &mut self,
        q: usize,
        axis: Pauli,
        neg: bool,
        apply_half_pi: ApplyHalfPiFn,
    ) {
        apply_half_pi(
            &mut self.qubit_slices_z_bits[q],
            &mut self.qubit_slices_x_bits[q],
            &mut self.sign_bits,
            axis,
            neg,
        );
    }

    /// Post compose a Pauli gate.
    fn postcompose_pauli_with(
        &mut self,
        pauli: Pauli,
        q: usize,
        x_gate: PauliGateFn,
        y_gate: PauliGateFn,
        z_gate: PauliGateFn,
    ) {
        match pauli {
            Pauli::X => x_gate(
                &self.qubit_slices_z_bits[q],
                &self.qubit_slices_x_bits[q],
                &mut self.sign_bits,
            ),
            Pauli::Y => y_gate(
                &self.qubit_slices_z_bits[q],
                &self.qubit_slices_x_bits[q],
                &mut self.sign_bits,
            ),
            Pauli::Z => z_gate(
                &self.qubit_slices_z_bits[q],
                &self.qubit_slices_x_bits[q],
                &mut self.sign_bits,
            ),
            _ => panic!("Unexpected basis gate!"),
        }
    }

    /// Post compose a Pauli gadget.
    fn postcompose_pauli_gadget_with(
        &mut self,
        pauli_z_bits: &[u64],
        pauli_x_bits: &[u64],
        half_pis: u8,
        apply_enum_tqe: ApplyEnumTqeFn,
        apply_half_pi: ApplyHalfPiFn,
        x_gate: PauliGateFn,
        y_gate: PauliGateFn,
        z_gate: PauliGateFn,
    ) {
        let half_pis = half_pis % 4;
        if half_pis == 0 {
            return;
        }
        if half_pis == 2 {
            for q in 0..self.n_qubits {
                let word_idx = q / 64;
                let bit_offset = q % 64;
                let pauli_z_bit = (pauli_z_bits[word_idx] >> bit_offset) & 1;
                let pauli_x_bit = (pauli_x_bits[word_idx] >> bit_offset) & 1;
                if pauli_z_bit == 1 && pauli_x_bit == 1 {
                    y_gate(
                        &self.qubit_slices_z_bits[q],
                        &self.qubit_slices_x_bits[q],
                        &mut self.sign_bits,
                    );
                } else if pauli_z_bit == 1 && pauli_x_bit == 0 {
                    z_gate(
                        &self.qubit_slices_z_bits[q],
                        &self.qubit_slices_x_bits[q],
                        &mut self.sign_bits,
                    );
                } else if pauli_z_bit == 0 && pauli_x_bit == 1 {
                    x_gate(
                        &self.qubit_slices_z_bits[q],
                        &self.qubit_slices_x_bits[q],
                        &mut self.sign_bits,
                    );
                }
            }
            return;
        }
        // we apply the gadget using a ladder
        // we find the first non-identity
        let (a, pa) = {
            let mut result = None;
            for (input_word_idx, (input_z_word, input_x_word)) in
                pauli_z_bits.iter().zip(pauli_x_bits).enumerate()
            {
                let occupied_bits = input_z_word | input_x_word;
                let input_bit_offset = occupied_bits.trailing_zeros();
                if input_bit_offset < 64 {
                    let q = input_word_idx * 64 + input_bit_offset as usize;
                    let pauli_z_bit = (input_z_word >> input_bit_offset) & 1;
                    let pauli_x_bit = (input_x_word >> input_bit_offset) & 1;
                    let pauli = match (pauli_z_bit, pauli_x_bit) {
                        (0, 1) => Pauli::X,
                        (1, 0) => Pauli::Z,
                        (1, 1) => Pauli::Y,
                        _ => panic!("Shouldn't be"),
                    };
                    result = Some((q, pauli));
                    break;
                }
            }
            result.expect("No non-identity Pauli found in the string")
        };
        let ga = match pa {
            Pauli::X => Pauli::Z,
            Pauli::Z => Pauli::X,
            Pauli::Y => Pauli::Z,
            _ => panic!("Shouldn't be"),
        };

        // Store the TQE gates to apply in reverse later
        let mut tqe_gates = Vec::new();

        // Forward TQE gates
        for b in (a + 1)..self.n_qubits {
            let word_idx = b / 64;
            let bit_offset = b % 64;
            let pauli_z_bit = (pauli_z_bits[word_idx] >> bit_offset) & 1;
            let pauli_x_bit = (pauli_x_bits[word_idx] >> bit_offset) & 1;
            // Skip identity
            let gb = match (pauli_z_bit, pauli_x_bit) {
                (0, 1) => Pauli::X,
                (1, 0) => Pauli::Z,
                (1, 1) => Pauli::Y,
                _ => continue,
            };

            // Store the gate for reverse application
            tqe_gates.push((a, b, ga, gb));

            let (a_slice_z_bits, b_slice_z_bits) = get_two_mut(&mut self.qubit_slices_z_bits, a, b);
            let (a_slice_x_bits, b_slice_x_bits) = get_two_mut(&mut self.qubit_slices_x_bits, a, b);
            apply_enum_tqe(
                a_slice_z_bits,
                a_slice_x_bits,
                b_slice_z_bits,
                b_slice_x_bits,
                &mut self.sign_bits,
                ga,
                gb,
            );
        }

        // apply the 1q clifford gate
        self.postcompose_half_pi_with(a, pa, half_pis == 3, apply_half_pi);
        // Apply TQE gates in reverse order
        for (qa, qb, ga_rev, gb_rev) in tqe_gates.iter().rev() {
            let (qa_slice_z_bits, qb_slice_z_bits) =
                get_two_mut(&mut self.qubit_slices_z_bits, *qa, *qb);
            let (qa_slice_x_bits, qb_slice_x_bits) =
                get_two_mut(&mut self.qubit_slices_x_bits, *qa, *qb);
            apply_enum_tqe(
                qa_slice_z_bits,
                qa_slice_x_bits,
                qb_slice_z_bits,
                qb_slice_x_bits,
                &mut self.sign_bits,
                *ga_rev,
                *gb_rev,
            );
        }
    }

    /// Conjugate a Pauli string by conjugating the z bits and x bits separately
    fn apply_to_pauli_with(
        &self,
        pauli_z_bits: &[u64],
        pauli_x_bits: &[u64],
        string_mul: StringMulFn,
        count_y: CountYFn,
    ) -> (Vec<u64>, Vec<u64>, bool) {
        let mut phase: u64 = 0;
        let mut result_sign_flip = false;
        let mut result_z_bits = vec![0u64; pauli_z_bits.len()];
        let mut result_x_bits = vec![0u64; pauli_x_bits.len()];
        'outer: for (input_word_idx, input_x_word) in pauli_x_bits.iter().enumerate() {
            let mut remaining_bits = *input_x_word;
            for _ in 0..input_x_word.count_ones() {
                let input_bit_offset = remaining_bits.trailing_zeros() as usize;
                let q = input_word_idx * 64 + input_bit_offset;
                if q == self.n_qubits {
                    break 'outer;
                }
                let (x_img_z_bits, x_img_x_bits, x_img_sign_bit) = self.packed_image(2 * q + 1);
                result_sign_flip ^= x_img_sign_bit;
                phase += string_mul(
                    &mut result_z_bits,
                    &mut result_x_bits,
                    &x_img_z_bits,
                    &x_img_x_bits,
                ) as u64;
                remaining_bits &= remaining_bits - 1;
            }
        }
        'outer: for (input_word_idx, input_z_word) in pauli_z_bits.iter().enumerate() {
            let mut remaining_bits = *input_z_word;
            for _ in 0..input_z_word.count_ones() {
                let input_bit_offset = remaining_bits.trailing_zeros() as usize;
                let q = input_word_idx * 64 + input_bit_offset;
                if q == self.n_qubits {
                    break 'outer;
                }
                let (z_img_z_bits, z_img_x_bits, z_img_sign_bit) = self.packed_image(2 * q);
                result_sign_flip ^= z_img_sign_bit;
                phase += string_mul(
                    &mut result_z_bits,
                    &mut result_x_bits,
                    &z_img_z_bits,
                    &z_img_x_bits,
                ) as u64;
                remaining_bits &= remaining_bits - 1;
            }
        }
        // count the number of ys, each y contribute a i phase, hence shift the total phase by 1
        let n_ys = count_y(pauli_z_bits, pauli_x_bits);
        (
            result_z_bits,
            result_x_bits,
            result_sign_flip ^ ((phase + n_ys) % 4 == 2),
        )
    }

    /// Multi-threaded version of apply_to_pauli_with
    /// the conjugation of z-bits and x-bits, as well as the counting of ys, are done in parallel
    fn apply_to_pauli_mt_with(
        &self,
        pauli_z_bits: &[u64],
        pauli_x_bits: &[u64],
        string_mul: StringMulFn,
        count_y: CountYFn,
    ) -> (Vec<u64>, Vec<u64>, bool) {
        let mut x_img_phase: u64 = 0;
        let mut x_img_sign_flip = false;
        let mut z_img_phase: u64 = 0;
        let mut z_img_sign_flip = false;
        let mut n_ys: u64 = 0;
        let mut x_img_product_z_bits = vec![0u64; pauli_z_bits.len()];
        let mut x_img_product_x_bits = vec![0u64; pauli_x_bits.len()];
        let mut z_img_product_z_bits = vec![0u64; pauli_z_bits.len()];
        let mut z_img_product_x_bits = vec![0u64; pauli_x_bits.len()];
        rayon::scope(|s| {
            s.spawn(|_| {
                'outer: for (input_word_idx, input_x_word) in pauli_x_bits.iter().enumerate() {
                    let mut remaining_bits = *input_x_word;
                    for _ in 0..input_x_word.count_ones() {
                        let input_bit_offset = remaining_bits.trailing_zeros() as usize;
                        let q = input_word_idx * 64 + input_bit_offset;
                        if q == self.n_qubits {
                            break 'outer;
                        }
                        let (x_img_z_bits, x_img_x_bits, x_img_sign_bit) =
                            self.packed_image(2 * q + 1);
                        x_img_sign_flip ^= x_img_sign_bit;
                        x_img_phase += string_mul(
                            &mut x_img_product_z_bits,
                            &mut x_img_product_x_bits,
                            &x_img_z_bits,
                            &x_img_x_bits,
                        ) as u64;
                        remaining_bits &= remaining_bits - 1;
                    }
                }
            });
            s.spawn(|_| {
                'outer: for (input_word_idx, input_z_word) in pauli_z_bits.iter().enumerate() {
                    let mut remaining_bits = *input_z_word;
                    for _ in 0..input_z_word.count_ones() {
                        let input_bit_offset = remaining_bits.trailing_zeros() as usize;
                        let q = input_word_idx * 64 + input_bit_offset;
                        if q == self.n_qubits {
                            break 'outer;
                        }
                        let (z_img_z_bits, z_img_x_bits, z_img_sign_bit) = self.packed_image(2 * q);
                        z_img_sign_flip ^= z_img_sign_bit;
                        z_img_phase += string_mul(
                            &mut z_img_product_z_bits,
                            &mut z_img_product_x_bits,
                            &z_img_z_bits,
                            &z_img_x_bits,
                        ) as u64;
                        remaining_bits &= remaining_bits - 1;
                    }
                }
            });
            s.spawn(|_| {
                n_ys = count_y(pauli_z_bits, pauli_x_bits);
            });
        });
        let phase = string_mul(
            &mut x_img_product_z_bits,
            &mut x_img_product_x_bits,
            &z_img_product_z_bits,
            &z_img_product_x_bits,
        ) as u64;
        // count the number of ys, each y contributes a i phase, hence shift the total phase by 1
        (
            x_img_product_z_bits,
            x_img_product_x_bits,
            x_img_sign_flip
                ^ z_img_sign_flip
                ^ ((x_img_phase + z_img_phase + phase + n_ys) % 4 == 2),
        )
    }

    /// Invert the tableau
    fn invert_with<F, G>(&self, apply_fn: F, apply_mt_fn: G) -> Tableau
    where
        F: Fn(&Tableau, &[u64], &[u64]) -> (Vec<u64>, Vec<u64>, bool) + Copy,
        G: Fn(&Tableau, &[u64], &[u64]) -> (Vec<u64>, Vec<u64>, bool) + Copy,
    {
        let n_images = self.n_qubits * 2;
        let slice_word_len = n_images.div_ceil(64);
        let mut inv_slices_z_bits: Vec<Vec<u64>> = Vec::with_capacity(self.n_qubits);
        let mut inv_slices_x_bits: Vec<Vec<u64>> = Vec::with_capacity(self.n_qubits);
        // TODO: consider parallelising this loop
        for input_q in 0..self.n_qubits {
            let img_word_idx = (2 * input_q) / 64;
            let z_img_bit_offset = (2 * input_q) % 64;
            let x_img_bit_offset = z_img_bit_offset + 1;
            let mut inv_slice_z_bits: Vec<u64> = Vec::with_capacity(slice_word_len);
            let mut inv_slice_x_bits: Vec<u64> = Vec::with_capacity(slice_word_len);
            let mut inv_slice_z_word: u64 = 0;
            let mut inv_slice_x_word: u64 = 0;
            let mut bits_filled = 0;

            for output_q in 0..self.n_qubits {
                let slice_z_word = self.qubit_slices_z_bits[output_q][img_word_idx];
                let slice_x_word = self.qubit_slices_x_bits[output_q][img_word_idx];
                let z_img_z_bit = (slice_z_word >> z_img_bit_offset) & 0b1;
                let x_img_z_bit = (slice_z_word >> x_img_bit_offset) & 0b1;
                let z_img_x_bit = (slice_x_word >> z_img_bit_offset) & 0b1;
                let x_img_x_bit = (slice_x_word >> x_img_bit_offset) & 0b1;

                inv_slice_x_word |= z_img_x_bit << bits_filled;
                inv_slice_x_word |= z_img_z_bit << (bits_filled + 1);

                inv_slice_z_word |= x_img_x_bit << bits_filled;
                inv_slice_z_word |= x_img_z_bit << (bits_filled + 1);
                bits_filled += 2;
                if bits_filled == 64 {
                    inv_slice_z_bits.push(inv_slice_z_word);
                    inv_slice_x_bits.push(inv_slice_x_word);
                    inv_slice_z_word = 0;
                    inv_slice_x_word = 0;
                    bits_filled = 0;
                }
            }
            if bits_filled > 0 {
                inv_slice_z_bits.push(inv_slice_z_word);
                inv_slice_x_bits.push(inv_slice_x_word);
            }
            inv_slices_z_bits.push(inv_slice_z_bits);
            inv_slices_x_bits.push(inv_slice_x_bits);
        }
        let mut tab_inv = Self {
            qubit_slices_z_bits: inv_slices_z_bits,
            qubit_slices_x_bits: inv_slices_x_bits,
            sign_bits: vec![0; self.sign_bits.len()],
            n_qubits: self.n_qubits,
        };

        // Set sign bits.
        for q in 0..self.n_qubits {
            let (z_img_z_bits, z_img_x_bits, _) = tab_inv.packed_image(2 * q);
            let (x_img_z_bits, x_img_x_bits, _) = tab_inv.packed_image(2 * q + 1);
            let (z_img_sign_bit, x_img_sign_bit) = if self.n_qubits >= MT_THRESH {
                let (_, _, z_img_sign_bit) = apply_mt_fn(self, &z_img_z_bits, &z_img_x_bits);
                let (_, _, x_img_sign_bit) = apply_mt_fn(self, &x_img_z_bits, &x_img_x_bits);
                (z_img_sign_bit, x_img_sign_bit)
            } else {
                let (_, _, z_img_sign_bit) = apply_fn(self, &z_img_z_bits, &z_img_x_bits);
                let (_, _, x_img_sign_bit) = apply_fn(self, &x_img_z_bits, &x_img_x_bits);
                (z_img_sign_bit, x_img_sign_bit)
            };
            tab_inv.set_sign_bit(2 * q, z_img_sign_bit);
            tab_inv.set_sign_bit(2 * q + 1, x_img_sign_bit);
        }

        tab_inv
    }

    /// Postcompose another tableau.
    fn compose_with<F, G>(&mut self, other: &Tableau, apply_fn: F, apply_mt_fn: G)
    where
        F: Fn(&Tableau, &[u64], &[u64]) -> (Vec<u64>, Vec<u64>, bool) + Copy,
        G: Fn(&Tableau, &[u64], &[u64]) -> (Vec<u64>, Vec<u64>, bool) + Copy,
    {
        if self.n_qubits != other.n_qubits {
            panic!("Cannot compose tableaux with different numbers of qubits");
        }
        for q in 0..self.n_qubits {
            let z_img_idx = 2 * q;
            let x_img_idx = 2 * q + 1;
            let (mut z_img_z_bits, mut z_img_x_bits, z_img_sign_bit) = self.packed_image(z_img_idx);
            let (mut x_img_z_bits, mut x_img_x_bits, x_img_sign_bit) = self.packed_image(x_img_idx);
            let (z_img_z_bits_new, z_img_x_bits_new, z_img_sign_bit_new) =
                if self.n_qubits >= MT_THRESH {
                    apply_mt_fn(other, &mut z_img_z_bits, &mut z_img_x_bits)
                } else {
                    apply_fn(other, &mut z_img_z_bits, &mut z_img_x_bits)
                };
            let (x_img_z_bits_new, x_img_x_bits_new, x_img_sign_bit_new) =
                if self.n_qubits >= MT_THRESH {
                    apply_mt_fn(other, &mut x_img_z_bits, &mut x_img_x_bits)
                } else {
                    apply_fn(other, &mut x_img_z_bits, &mut x_img_x_bits)
                };

            self.set_packed_image(
                z_img_idx,
                &z_img_z_bits_new,
                &z_img_x_bits_new,
                z_img_sign_bit_new ^ z_img_sign_bit,
            );
            self.set_packed_image(
                x_img_idx,
                &x_img_z_bits_new,
                &x_img_x_bits_new,
                x_img_sign_bit_new ^ x_img_sign_bit,
            );
        }
    }

    /// Creates a tableau from packed qubit slices.
    ///
    /// # Arguments
    ///
    /// * `qubit_slices_z_bits` - Packed Z bits for each qubit slice
    /// * `qubit_slices_x_bits` - Packed X bits for each qubit slice
    /// * `sign_bits` - Sign bits for each column, 0: + and 1: -
    /// * `n_qubits` - Number of qubits the tableau operates on
    ///
    /// # Returns
    ///
    /// A new Tableau instance
    // TODO: need input validation
    pub fn from_packed_qubit_slices(
        qubit_slices_z_bits: Vec<Vec<u64>>,
        qubit_slices_x_bits: Vec<Vec<u64>>,
        sign_bits: Vec<u64>,
        n_qubits: usize,
    ) -> Self {
        Self {
            qubit_slices_z_bits,
            qubit_slices_x_bits,
            sign_bits,
            n_qubits,
        }
    }

    /// Initialise an identity Tableau
    pub fn eye(n_qubits: usize) -> Self {
        let n_images = n_qubits * 2;
        let mut qubit_slices_z_bits = Vec::with_capacity(n_qubits);
        let mut qubit_slices_x_bits = Vec::with_capacity(n_qubits);
        for q in 0..n_qubits {
            let mut paulis = vec![Pauli::I; n_images];
            paulis[2 * q] = Pauli::Z;
            paulis[2 * q + 1] = Pauli::X;
            let (slice_z_bits, slice_x_bits) = paulis_to_u64s(&paulis);
            qubit_slices_z_bits.push(slice_z_bits);
            qubit_slices_x_bits.push(slice_x_bits);
        }
        let n_words = n_images.div_ceil(64);
        Self {
            qubit_slices_z_bits,
            qubit_slices_x_bits,
            sign_bits: vec![0; n_words],
            n_qubits,
        }
    }

    /// Create a random Tableau by applying n_tqe TQE gates and n_basis_change basis change gates to a
    /// identity tableau
    pub fn random(n_qubits: usize, seed: u32, n_basis_change: u32, n_tqe: u32) -> Self {
        let mut tableau = Tableau::eye(n_qubits);
        let mut rng = StdRng::seed_from_u64(seed as u64);

        // Apply random basis change gates
        for _ in 0..n_basis_change {
            let axis = match rng.random_range(0..3) {
                0 => Pauli::X,
                1 => Pauli::Y,
                _ => Pauli::Z,
            };
            let q = rng.random_range(0..n_qubits);
            tableau.postcompose_basis_change(axis, q, false);
        }
        // Apply random TQE gates
        if n_qubits == 1 {
            return tableau;
        }
        // add a ladder of TQE first
        for i in 0..n_qubits - 1 {
            let g0 = match rng.random_range(0..3) {
                0 => Pauli::X,
                1 => Pauli::Y,
                _ => Pauli::Z,
            };
            let g1 = match rng.random_range(0..3) {
                0 => Pauli::X,
                1 => Pauli::Y,
                _ => Pauli::Z,
            };
            tableau.postcompose_tqe(g0, g1, i, i + 1);
        }
        for _ in 0..n_tqe {
            let q0 = rng.random_range(0..n_qubits);
            let mut q1 = rng.random_range(0..n_qubits - 1);
            if q1 >= q0 {
                q1 += 1;
            }
            let g0 = match rng.random_range(0..3) {
                0 => Pauli::X,
                1 => Pauli::Y,
                _ => Pauli::Z,
            };
            let g1 = match rng.random_range(0..3) {
                0 => Pauli::X,
                1 => Pauli::Y,
                _ => Pauli::Z,
            };
            tableau.postcompose_tqe(g0, g1, q0, q1);
        }
        tableau
    }

    /// Get a packed image by index.
    // TODO: consider reusable scratch buffers for images to avoid repeated allocations.
    fn packed_image(&self, img_idx: usize) -> (Vec<u64>, Vec<u64>, bool) {
        let img_word_idx = img_idx / 64;
        let img_bit_offset = img_idx % 64;
        let n_words = self.n_qubits.div_ceil(64);
        let mut img_z_word = 0;
        let mut img_x_word = 0;
        let mut bits_filled = 0;
        let mut img_z_bits = Vec::with_capacity(n_words);
        let mut img_x_bits = Vec::with_capacity(n_words);

        for output_q in 0..self.n_qubits {
            let img_z_bit =
                (self.qubit_slices_z_bits[output_q][img_word_idx] >> img_bit_offset) & 1;
            let img_x_bit =
                (self.qubit_slices_x_bits[output_q][img_word_idx] >> img_bit_offset) & 1;
            img_z_word |= img_z_bit << bits_filled;
            img_x_word |= img_x_bit << bits_filled;
            bits_filled += 1;
            if bits_filled == 64 {
                img_z_bits.push(img_z_word);
                img_x_bits.push(img_x_word);
                bits_filled = 0;
                img_z_word = 0;
                img_x_word = 0;
            }
        }
        if bits_filled > 0 {
            img_z_bits.push(img_z_word);
            img_x_bits.push(img_x_word);
        }
        (
            img_z_bits,
            img_x_bits,
            (self.sign_bits[img_word_idx] >> img_bit_offset & 1 == 1),
        )
    }

    /// Set a packed image.
    fn set_packed_image(
        &mut self,
        img_idx: usize,
        img_z_bits: &[u64],
        img_x_bits: &[u64],
        img_sign_bit: bool,
    ) {
        let img_word_idx = img_idx / 64;
        let img_bit_offset = img_idx % 64;
        for output_q in 0..self.n_qubits {
            let output_word_idx = output_q / 64;
            let output_bit_offset = output_q % 64;
            let img_z_bit = (img_z_bits[output_word_idx] >> output_bit_offset) & 1;
            let img_x_bit = (img_x_bits[output_word_idx] >> output_bit_offset) & 1;
            self.qubit_slices_z_bits[output_q][img_word_idx] &= !(1 << img_bit_offset);
            self.qubit_slices_z_bits[output_q][img_word_idx] |= img_z_bit << img_bit_offset;
            self.qubit_slices_x_bits[output_q][img_word_idx] &= !(1 << img_bit_offset);
            self.qubit_slices_x_bits[output_q][img_word_idx] |= img_x_bit << img_bit_offset;
        }
        if img_sign_bit {
            self.sign_bits[img_word_idx] |= 1 << img_bit_offset;
        } else {
            self.sign_bits[img_word_idx] &= !(1 << img_bit_offset);
        }
    }
    /// flip sign
    fn flip_sign(&mut self, img_idx: usize) {
        let img_word_idx = img_idx / 64;
        let img_bit_offset = img_idx % 64;
        self.sign_bits[img_word_idx] ^= 1 << img_bit_offset;
    }
    /// set sign
    fn set_sign_bit(&mut self, img_idx: usize, img_sign_bit: bool) {
        let img_word_idx = img_idx / 64;
        let img_bit_offset = img_idx % 64;
        if img_sign_bit {
            self.sign_bits[img_word_idx] |= 1 << img_bit_offset;
        } else {
            self.sign_bits[img_word_idx] &= !(1 << img_bit_offset);
        }
    }

    /// Postcompose a TQE gate.
    /// The gate is specified by Pauli operators
    /// on two qubits.
    ///
    /// # Arguments
    ///
    /// * `g0` - Pauli operator for the first qubit
    /// * `g1` - Pauli operator for the second qubit
    /// * `q0` - Index of the first qubit
    /// * `q1` - Index of the second qubit
    ///
    /// # Panics
    ///
    /// Panics if `q0 == q1`, or if `g0` or `g1` is `Pauli::I`.
    pub fn postcompose_tqe(&mut self, g0: Pauli, g1: Pauli, q0: usize, q1: usize) {
        check_distinct_qubits(q0, q1);
        self.postcompose_tqe_with(
            g0,
            g1,
            q0,
            q1,
            slice_xx_gate,
            slice_xy_gate,
            slice_xz_gate,
            slice_yy_gate,
            slice_yz_gate,
            slice_zz_gate,
        );
    }

    /// Precompose a TQE gate.
    /// The gate is specified by Pauli operators
    /// on two qubits.
    ///
    /// # Arguments
    ///
    /// * `g0` - Pauli operator for the first qubit
    /// * `g1` - Pauli operator for the second qubit
    /// * `q0` - Index of the first qubit
    /// * `q1` - Index of the second qubit
    ///
    /// # Panics
    ///
    /// Panics if `q0 == q1`, or if `g0` or `g1` is `Pauli::I`.
    pub fn precompose_tqe(&mut self, g0: Pauli, g1: Pauli, q0: usize, q1: usize) {
        check_distinct_qubits(q0, q1);
        self.precompose_tqe_with(g0, g1, q0, q1, string_mul);
    }

    /// Postcompose a single-qubit basis change gate.
    /// The basis change is specified by a Pauli operator.
    ///
    /// # Arguments
    ///
    /// * `axis` - The Pauli axis for the basis change (X, Y, or Z)
    /// * `q` - Index of the qubit to apply the basis change to
    /// * `dagger` - Whether to apply the inverse (daggered) version of the gate
    ///
    /// # Panics
    ///
    /// Panics if `axis` is `Pauli::I`.
    pub fn postcompose_basis_change(&mut self, axis: Pauli, q: usize, dagger: bool) {
        if dagger {
            self.postcompose_basis_change_with(
                axis,
                q,
                slice_vdg_gate,
                slice_h_gate,
                slice_sdg_gate,
            );
        } else {
            self.postcompose_basis_change_with(axis, q, slice_v_gate, slice_h_gate, slice_s_gate);
        }
    }

    /// Precompose a single-qubit basis change gate.
    /// The basis change is specified by a Pauli operator.
    /// This is equivalent to absorbing H, S, or V gate on the input.
    /// # Arguments
    ///
    /// * `axis` - The Pauli axis for the basis change (X, Y, or Z)
    /// * `q` - Index of the qubit to apply the basis change to
    /// * `dagger` - Whether to apply the inverse (daggered) version of the gate
    ///
    /// # Panics
    ///
    /// Panics if `axis` is `Pauli::I`.
    pub fn precompose_basis_change(&mut self, axis: Pauli, q: usize, dagger: bool) {
        self.precompose_basis_change_with(axis, q, dagger, string_mul);
    }

    /// Precompose a single-qubit Pauli gate.
    ///
    /// # Arguments
    ///
    /// * `p` - The Pauli axis for the Pauli (X, Y, or Z)
    /// * `q` - Index of the qubit to apply the Pauli to
    ///
    /// # Panics
    ///
    /// Panics if `p` is `Pauli::I`.
    pub fn precompose_pauli(&mut self, p: Pauli, q: usize) {
        let z_img_idx = 2 * q;
        let x_img_idx = 2 * q + 1;
        match p {
            Pauli::X => {
                self.flip_sign(z_img_idx);
            }
            Pauli::Y => {
                self.flip_sign(z_img_idx);
                self.flip_sign(x_img_idx);
            }
            Pauli::Z => {
                self.flip_sign(x_img_idx);
            }
            _ => panic!("Unexpected Pauli!"),
        }
    }

    /// Precompose a SWAP gate.
    ///
    /// # Arguments
    ///
    /// * `q0` - Index of the first qubit
    /// * `q1` - Index of the second qubit
    ///
    pub fn precompose_swap(&mut self, q0: usize, q1: usize) {
        let (z0_img_z_bits, z0_img_x_bits, z0_img_sign_bit) = self.packed_image(2 * q0);
        let (z1_img_z_bits, z1_img_x_bits, z1_img_sign_bit) = self.packed_image(2 * q1);
        let (x0_img_z_bits, x0_img_x_bits, x0_img_sign_bit) = self.packed_image(2 * q0 + 1);
        let (x1_img_z_bits, x1_img_x_bits, x1_img_sign_bit) = self.packed_image(2 * q1 + 1);
        self.set_packed_image(2 * q0, &z1_img_z_bits, &z1_img_x_bits, z1_img_sign_bit);
        self.set_packed_image(2 * q1, &z0_img_z_bits, &z0_img_x_bits, z0_img_sign_bit);
        self.set_packed_image(2 * q0 + 1, &x1_img_z_bits, &x1_img_x_bits, x1_img_sign_bit);
        self.set_packed_image(2 * q1 + 1, &x0_img_z_bits, &x0_img_x_bits, x0_img_sign_bit);
    }

    /// Postcompose a SWAP gate.
    ///
    /// # Arguments
    ///
    /// * `q0` - Index of the first qubit
    /// * `q1` - Index of the second qubit
    ///
    pub fn postcompose_swap(&mut self, q0: usize, q1: usize) {
        self.qubit_slices_z_bits.swap(q0, q1);
        self.qubit_slices_x_bits.swap(q0, q1);
    }

    /// Postcompose a half pi rotation gate.
    ///
    /// # Arguments
    ///
    /// * `q` - Index of the qubit
    /// * `axis` - The Pauli axis for the rotation (X, Y, or Z)
    /// * `neg` - Whether the rotation is negative
    ///
    pub fn postcompose_half_pi(&mut self, q: usize, axis: Pauli, neg: bool) {
        self.postcompose_half_pi_with(q, axis, neg, apply_half_pi_gate_slice);
    }

    /// Postcompose a multi-qubit Clifford angle Pauli rotation.
    /// of the tableau.
    ///
    ///
    /// # Arguments
    ///
    /// * `pauli_z_bits` - Z bits of the Pauli string
    /// * `pauli_x_bits` - X bits of the Pauli string
    /// * `half_pis` - number of half pi turns, [0,1,2,3]
    ///
    /// # Panics
    ///
    /// Panics if `pauli_z_bits`/`pauli_x_bits` encode the all-identity Pauli string.
    pub fn postcompose_pauli_gadget(
        &mut self,
        pauli_z_bits: &[u64],
        pauli_x_bits: &[u64],
        half_pis: u8,
    ) {
        self.postcompose_pauli_gadget_with(
            pauli_z_bits,
            pauli_x_bits,
            half_pis,
            apply_enum_tqe_slice,
            apply_half_pi_gate_slice,
            slice_x_gate,
            slice_y_gate,
            slice_z_gate,
        );
    }

    /// Conjugate a Pauli string.
    ///
    /// Applies the tableau transformation to conjugate the input Pauli string,
    /// computing both the resulting Pauli operators and any sign changes.
    ///
    /// # Arguments
    ///
    /// * `pauli_z_bits` - Packed Z bits of the Pauli string (modified in-place)
    /// * `pauli_x_bits` - Packed X bits of the Pauli string (modified in-place)
    ///
    /// # Returns
    ///
    /// The conjugated Z bits, X bits, and a boolean indicating whether the transformation results in a sign flip
    ///
    pub fn apply_to_pauli(
        &self,
        pauli_z_bits: &[u64],
        pauli_x_bits: &[u64],
    ) -> (Vec<u64>, Vec<u64>, bool) {
        self.apply_to_pauli_with(pauli_z_bits, pauli_x_bits, string_mul, count_y)
    }

    /// Multi-threaded version of apply_to_pauli
    ///
    /// Note: For small tableaus it's worth benchmarking against the non-MT `apply_to_pauli`.
    pub fn apply_to_pauli_mt(
        &self,
        pauli_z_bits: &[u64],
        pauli_x_bits: &[u64],
    ) -> (Vec<u64>, Vec<u64>, bool) {
        self.apply_to_pauli_mt_with(pauli_z_bits, pauli_x_bits, string_mul, count_y)
    }

    /// Invert the tableau
    /// https://algassert.com/post/2002
    pub fn invert(&self) -> Tableau {
        self.invert_with(Tableau::apply_to_pauli, Tableau::apply_to_pauli_mt)
    }

    /// Postcompose another tableau.
    /// self   other
    /// >[]> + >[]>
    pub fn compose(&mut self, other: &Tableau) {
        self.compose_with(other, Tableau::apply_to_pauli, Tableau::apply_to_pauli_mt);
    }

    /// Postcompose a single-qubit Pauli gate
    ///
    /// # Panics
    ///
    /// Panics if `pauli` is `Pauli::I`.
    pub fn postcompose_pauli(&mut self, pauli: Pauli, q: usize) {
        self.postcompose_pauli_with(pauli, q, slice_x_gate, slice_y_gate, slice_z_gate);
    }

    /// Return image of X on the ith qubit. The boolean is true when the image has a -1 phase.
    pub fn x_image(&self, input_qubit: usize) -> (Vec<Pauli>, bool) {
        let (x_img_z_bits, x_img_x_bits, x_img_sign_bit) = self.packed_image(2 * input_qubit + 1);
        (
            u64s_to_paulis(&x_img_z_bits, &x_img_x_bits, self.n_qubits),
            x_img_sign_bit,
        )
    }

    /// Return image of Z on the ith qubit. The boolean is true when the image has a -1 phase.
    pub fn z_image(&self, input_qubit: usize) -> (Vec<Pauli>, bool) {
        let (z_img_z_bits, z_img_x_bits, z_img_sign_bit) = self.packed_image(2 * input_qubit);
        (
            u64s_to_paulis(&z_img_z_bits, &z_img_x_bits, self.n_qubits),
            z_img_sign_bit,
        )
    }
}

/// Trait for SIMD operations on Tableau
#[cfg(feature = "simd")]
pub trait SimdTableau {
    /// Postcompose a TQE gate using SIMD operations.
    ///
    /// # Panics
    ///
    /// Panics if `q0 == q1`, or if `g0` or `g1` is `Pauli::I`.
    fn postcompose_tqe_simd<const N: usize>(&mut self, g0: Pauli, g1: Pauli, q0: usize, q1: usize)
    where
        LaneCount<N>: SupportedLaneCount;
    /// Precompose a TQE gate using SIMD operations.
    ///
    /// # Panics
    ///
    /// Panics if `q0 == q1`, or if `g0` or `g1` is `Pauli::I`.
    fn precompose_tqe_simd<const N: usize>(&mut self, g0: Pauli, g1: Pauli, q0: usize, q1: usize)
    where
        LaneCount<N>: SupportedLaneCount;
    /// Postcompose a single-qubit basis change gate using SIMD operations.
    ///
    /// # Panics
    ///
    /// Panics if `axis` is `Pauli::I`.
    fn postcompose_basis_change_simd<const N: usize>(
        &mut self,
        axis: Pauli,
        q: usize,
        dagger: bool,
    ) where
        LaneCount<N>: SupportedLaneCount;
    /// Precompose a single-qubit basis change gate using SIMD operations.
    ///
    /// # Panics
    ///
    /// Panics if `axis` is `Pauli::I`.
    fn precompose_basis_change_simd<const N: usize>(&mut self, axis: Pauli, q: usize, dagger: bool)
    where
        LaneCount<N>: SupportedLaneCount;
    /// Postcompose a half pi rotation gate using SIMD operations.
    fn postcompose_half_pi_simd<const N: usize>(&mut self, q: usize, axis: Pauli, neg: bool)
    where
        LaneCount<N>: SupportedLaneCount;
    /// Postcompose a multi-qubit Clifford angle Pauli rotation using SIMD operations.
    ///
    /// # Panics
    ///
    /// Panics if `pauli_z_bits`/`pauli_x_bits` encode the all-identity Pauli string.
    fn postcompose_pauli_gadget_simd<const N: usize>(
        &mut self,
        pauli_z_bits: &[u64],
        pauli_x_bits: &[u64],
        half_pis: u8,
    ) where
        LaneCount<N>: SupportedLaneCount;
    /// Conjugate a Pauli string using SIMD operations.
    fn apply_to_pauli_simd<const N: usize>(
        &self,
        pauli_z_bits: &[u64],
        pauli_x_bits: &[u64],
    ) -> (Vec<u64>, Vec<u64>, bool)
    where
        LaneCount<N>: SupportedLaneCount;
    /// Multi-threaded version of apply_to_pauli_simd.
    ///
    /// Note: For small tableaus it's worth benchmarking against the non-MT
    /// `apply_to_pauli_simd`.
    fn apply_to_pauli_mt_simd<const N: usize>(
        &self,
        pauli_z_bits: &[u64],
        pauli_x_bits: &[u64],
    ) -> (Vec<u64>, Vec<u64>, bool)
    where
        LaneCount<N>: SupportedLaneCount;
    /// Invert the tableau using SIMD operations.
    fn invert_simd<const N: usize>(&self) -> Tableau
    where
        LaneCount<N>: SupportedLaneCount;
    /// Postcompose another tableau using SIMD operations.
    fn compose_simd<const N: usize>(&mut self, other: &Tableau)
    where
        LaneCount<N>: SupportedLaneCount;
    /// Postcompose a single-qubit Pauli gate using SIMD operations.
    ///
    /// # Panics
    ///
    /// Panics if `pauli` is `Pauli::I`.
    fn postcompose_pauli_simd<const N: usize>(&mut self, pauli: Pauli, q: usize)
    where
        LaneCount<N>: SupportedLaneCount;
}

#[cfg(feature = "simd")]
impl SimdTableau for Tableau {
    fn postcompose_tqe_simd<const N: usize>(&mut self, g0: Pauli, g1: Pauli, q0: usize, q1: usize)
    where
        LaneCount<N>: SupportedLaneCount,
    {
        check_distinct_qubits(q0, q1);
        self.postcompose_tqe_with(
            g0,
            g1,
            q0,
            q1,
            simd_xx_gate::<N>,
            simd_xy_gate::<N>,
            simd_xz_gate::<N>,
            simd_yy_gate::<N>,
            simd_yz_gate::<N>,
            simd_zz_gate::<N>,
        );
    }

    fn precompose_tqe_simd<const N: usize>(&mut self, g0: Pauli, g1: Pauli, q0: usize, q1: usize)
    where
        LaneCount<N>: SupportedLaneCount,
    {
        check_distinct_qubits(q0, q1);
        self.precompose_tqe_with(g0, g1, q0, q1, simd_string_mul::<N>);
    }

    fn postcompose_basis_change_simd<const N: usize>(&mut self, axis: Pauli, q: usize, dagger: bool)
    where
        LaneCount<N>: SupportedLaneCount,
    {
        if dagger {
            self.postcompose_basis_change_with(
                axis,
                q,
                simd_vdg_gate::<N>,
                simd_h_gate::<N>,
                simd_sdg_gate::<N>,
            );
        } else {
            self.postcompose_basis_change_with(
                axis,
                q,
                simd_v_gate::<N>,
                simd_h_gate::<N>,
                simd_s_gate::<N>,
            );
        }
    }

    fn precompose_basis_change_simd<const N: usize>(&mut self, axis: Pauli, q: usize, dagger: bool)
    where
        LaneCount<N>: SupportedLaneCount,
    {
        self.precompose_basis_change_with(axis, q, dagger, simd_string_mul::<N>);
    }

    fn postcompose_half_pi_simd<const N: usize>(&mut self, q: usize, axis: Pauli, neg: bool)
    where
        LaneCount<N>: SupportedLaneCount,
    {
        self.postcompose_half_pi_with(q, axis, neg, apply_half_pi_gate_simd::<N>);
    }

    fn postcompose_pauli_gadget_simd<const N: usize>(
        &mut self,
        pauli_z_bits: &[u64],
        pauli_x_bits: &[u64],
        half_pis: u8,
    ) where
        LaneCount<N>: SupportedLaneCount,
    {
        self.postcompose_pauli_gadget_with(
            pauli_z_bits,
            pauli_x_bits,
            half_pis,
            apply_enum_tqe_simd::<N>,
            apply_half_pi_gate_simd::<N>,
            simd_x_gate::<N>,
            simd_y_gate::<N>,
            simd_z_gate::<N>,
        );
    }

    fn apply_to_pauli_simd<const N: usize>(
        &self,
        pauli_z_bits: &[u64],
        pauli_x_bits: &[u64],
    ) -> (Vec<u64>, Vec<u64>, bool)
    where
        LaneCount<N>: SupportedLaneCount,
    {
        self.apply_to_pauli_with(
            pauli_z_bits,
            pauli_x_bits,
            simd_string_mul::<N>,
            simd_count_y::<N>,
        )
    }

    fn apply_to_pauli_mt_simd<const N: usize>(
        &self,
        pauli_z_bits: &[u64],
        pauli_x_bits: &[u64],
    ) -> (Vec<u64>, Vec<u64>, bool)
    where
        LaneCount<N>: SupportedLaneCount,
    {
        self.apply_to_pauli_mt_with(
            pauli_z_bits,
            pauli_x_bits,
            simd_string_mul::<N>,
            simd_count_y::<N>,
        )
    }

    fn invert_simd<const N: usize>(&self) -> Tableau
    where
        LaneCount<N>: SupportedLaneCount,
    {
        self.invert_with(
            SimdTableau::apply_to_pauli_simd::<N>,
            SimdTableau::apply_to_pauli_mt_simd::<N>,
        )
    }

    fn compose_simd<const N: usize>(&mut self, other: &Tableau)
    where
        LaneCount<N>: SupportedLaneCount,
    {
        self.compose_with(
            other,
            SimdTableau::apply_to_pauli_simd::<N>,
            SimdTableau::apply_to_pauli_mt_simd::<N>,
        );
    }

    fn postcompose_pauli_simd<const N: usize>(&mut self, pauli: Pauli, q: usize)
    where
        LaneCount<N>: SupportedLaneCount,
    {
        self.postcompose_pauli_with(
            pauli,
            q,
            simd_x_gate::<N>,
            simd_y_gate::<N>,
            simd_z_gate::<N>,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(feature = "simd")]
    use crate::SimdTableau;
    #[cfg(feature = "simd")]
    use pg_bitpacked::apply_enum_tqe_simd;
    use pg_bitpacked::apply_enum_tqe_slice;
    use rand::Rng;

    fn random_tqes(
        n_qubits: usize,
        k: usize,
        rng: &mut StdRng,
    ) -> Vec<(Pauli, Pauli, usize, usize)> {
        (0..k)
            .map(|_| {
                let q0 = rng.random_range(0..n_qubits);
                let mut q1 = rng.random_range(0..n_qubits - 1);
                if q1 >= q0 {
                    q1 += 1;
                }
                let g0 = match rng.random_range(0..3) {
                    0 => Pauli::X,
                    1 => Pauli::Y,
                    _ => Pauli::Z,
                };
                let g1 = match rng.random_range(0..3) {
                    0 => Pauli::X,
                    1 => Pauli::Y,
                    _ => Pauli::Z,
                };
                (g0, g1, q0, q1)
            })
            .collect()
    }
    fn random_basis_changes(n_qubits: usize, k: usize, rng: &mut StdRng) -> Vec<(Pauli, usize)> {
        (0..k)
            .map(|_| {
                let q = rng.random_range(0..n_qubits);
                let axis = match rng.random_range(0..3) {
                    0 => Pauli::X,
                    1 => Pauli::Y,
                    _ => Pauli::Z,
                };
                (axis, q)
            })
            .collect()
    }

    /// Shared body for `test_tableau_invert_roundtrip[_simd]`: inversion should round-trip and
    /// composing a tableau with its inverse in either order should produce the identity.
    fn tableau_invert_roundtrip_body(
        invert: impl Fn(&Tableau) -> Tableau,
        compose: impl Fn(&mut Tableau, &Tableau),
    ) {
        for n_qubits in [5, 70] {
            for seed in 0..10 {
                let tab = Tableau::random(n_qubits, seed, 100, 100);
                let tab_inv = invert(&tab);
                assert_eq!(invert(&tab_inv), tab);

                let mut tab_then_inv = tab.clone();
                compose(&mut tab_then_inv, &tab_inv);
                assert_eq!(tab_then_inv, Tableau::eye(n_qubits));

                let mut inv_then_tab = tab_inv;
                compose(&mut inv_then_tab, &tab);
                assert_eq!(inv_then_tab, Tableau::eye(n_qubits));
            }
        }
    }

    #[test]
    fn test_tableau_invert_roundtrip() {
        tableau_invert_roundtrip_body(Tableau::invert, Tableau::compose);
    }

    #[cfg(feature = "simd")]
    #[test]
    fn test_tableau_invert_roundtrip_simd() {
        tableau_invert_roundtrip_body(Tableau::invert_simd::<64>, Tableau::compose_simd::<64>);
    }

    /// Shared body for `test_tqe[_simd]_precompose_postcompose_matches`: precomposing a
    /// sequence of TQE gates is equivalent to postcomposing the same sequence in reverse order.
    fn tqe_precompose_postcompose_body(
        precompose: impl Fn(&mut Tableau, Pauli, Pauli, usize, usize),
        postcompose: impl Fn(&mut Tableau, Pauli, Pauli, usize, usize),
    ) {
        let mut rng = StdRng::seed_from_u64(0);
        for (n_qubits, n_tqes) in [(5, 100), (100, 1000)] {
            let mut tab = Tableau::eye(n_qubits);
            let mut tab2 = Tableau::eye(n_qubits);
            let gates = random_tqes(n_qubits, n_tqes, &mut rng);
            for &(g0, g1, q0, q1) in gates.iter() {
                precompose(&mut tab, g0, g1, q0, q1);
            }
            for &(g0, g1, q0, q1) in gates.iter().rev() {
                postcompose(&mut tab2, g0, g1, q0, q1);
            }
            assert_eq!(tab, tab2);
        }
    }

    #[test]
    fn test_tqe_precompose_postcompose_matches() {
        tqe_precompose_postcompose_body(Tableau::precompose_tqe, Tableau::postcompose_tqe);
    }

    #[cfg(feature = "simd")]
    #[test]
    fn test_tqe_simd_precompose_postcompose_matches() {
        tqe_precompose_postcompose_body(
            Tableau::precompose_tqe_simd::<64>,
            Tableau::postcompose_tqe_simd::<64>,
        );
    }

    /// Shared body for `test_basis_change[_simd]_precompose_postcompose_matches`: precomposing
    /// a sequence of basis change gates is equivalent to postcomposing the same sequence in
    /// reverse order.
    fn basis_change_precompose_postcompose_body(
        precompose: impl Fn(&mut Tableau, Pauli, usize, bool),
        postcompose: impl Fn(&mut Tableau, Pauli, usize, bool),
    ) {
        let mut rng = StdRng::seed_from_u64(0);
        for (n_qubits, n_gates) in [(5, 100), (100, 1000)] {
            let mut tab = Tableau::eye(n_qubits);
            let mut tab2 = Tableau::eye(n_qubits);
            let gates = random_basis_changes(n_qubits, n_gates, &mut rng);
            for &(axis, q) in gates.iter() {
                precompose(&mut tab, axis, q, false);
            }
            for &(axis, q) in gates.iter().rev() {
                postcompose(&mut tab2, axis, q, false);
            }
            assert_eq!(tab, tab2);
        }
    }

    #[test]
    fn test_basis_change_precompose_postcompose_matches() {
        basis_change_precompose_postcompose_body(
            Tableau::precompose_basis_change,
            Tableau::postcompose_basis_change,
        );
    }

    #[cfg(feature = "simd")]
    #[test]
    fn test_basis_change_simd_precompose_postcompose_matches() {
        basis_change_precompose_postcompose_body(
            Tableau::precompose_basis_change_simd::<64>,
            Tableau::postcompose_basis_change_simd::<64>,
        );
    }

    /// Shared body for `test_compose[_simd]`.
    fn compose_body(
        precompose_basis_change: impl Fn(&mut Tableau, Pauli, usize, bool),
        precompose_tqe: impl Fn(&mut Tableau, Pauli, Pauli, usize, usize),
        compose: impl Fn(&mut Tableau, &Tableau),
    ) {
        let mut rng = StdRng::seed_from_u64(0);

        for (n_qubits, n_tqes, n_basis_change) in [(5, 100, 100), (100, 1000, 1000)] {
            let mut sqc_gates = random_basis_changes(n_qubits, n_basis_change, &mut rng);
            let mut tqes = random_tqes(n_qubits, n_tqes, &mut rng);
            let sqc_gates2 = sqc_gates.split_off(50);
            let tqes2 = tqes.split_off(50);
            let mut tab1 = Tableau::eye(n_qubits);
            let mut tab2 = Tableau::eye(n_qubits);
            let mut tab3 = Tableau::eye(n_qubits);
            for &(axis, q) in sqc_gates.iter() {
                precompose_basis_change(&mut tab1, axis, q, false);
                precompose_basis_change(&mut tab3, axis, q, false);
            }
            for &(g0, g1, q0, q1) in tqes.iter() {
                precompose_tqe(&mut tab1, g0, g1, q0, q1);
                precompose_tqe(&mut tab3, g0, g1, q0, q1);
            }
            for &(axis, q) in sqc_gates2.iter() {
                precompose_basis_change(&mut tab2, axis, q, false);
                precompose_basis_change(&mut tab3, axis, q, false);
            }
            for &(g0, g1, q0, q1) in tqes2.iter() {
                precompose_tqe(&mut tab2, g0, g1, q0, q1);
                precompose_tqe(&mut tab3, g0, g1, q0, q1);
            }
            compose(&mut tab2, &tab1);
            assert_eq!(tab2, tab3);
        }
    }

    #[test]
    fn test_compose() {
        compose_body(
            Tableau::precompose_basis_change,
            Tableau::precompose_tqe,
            Tableau::compose,
        );
    }

    #[cfg(feature = "simd")]
    #[test]
    fn test_compose_simd() {
        compose_body(
            Tableau::precompose_basis_change_simd::<64>,
            Tableau::precompose_tqe_simd::<64>,
            Tableau::compose_simd::<64>,
        );
    }

    #[test]
    #[should_panic(expected = "Cannot compose tableaux with different numbers of qubits")]
    fn test_compose_rejects_larger_right_tableau() {
        Tableau::eye(2).compose(&Tableau::eye(3));
    }

    #[test]
    #[should_panic(expected = "Cannot compose tableaux with different numbers of qubits")]
    fn test_compose_rejects_smaller_right_tableau() {
        Tableau::eye(3).compose(&Tableau::eye(2));
    }

    /// Shared body for `test_gadget[_simd]`.
    ///
    /// We construct random gadgets by applying random TQEs
    /// to a Pauli string of weight 1 (with letter P), and an angle theta.
    /// Let the resulting string be Q'.
    /// We call this circuit composed of TQEs unitary U.
    /// U_dagger;P(theta);U = Q'(theta)
    ///
    /// We then verify this equation holds.
    fn gadget_body(
        apply_enum_tqe: impl Fn(
            &mut [u64],
            &mut [u64],
            &mut [u64],
            &mut [u64],
            &mut [u64],
            Pauli,
            Pauli,
        ),
        postcompose_pauli_gadget: impl Fn(&mut Tableau, &[u64], &[u64], u8),
        postcompose_tqe: impl Fn(&mut Tableau, Pauli, Pauli, usize, usize),
        postcompose_half_pi: impl Fn(&mut Tableau, usize, Pauli, bool),
        postcompose_pauli: impl Fn(&mut Tableau, Pauli, usize),
    ) {
        const N_TRIALS: usize = 100;
        let mut rng = StdRng::seed_from_u64(5);
        for (n_qubits, n_tqes) in [(4, 4), (100, 1000)] {
            for _ in 0..N_TRIALS {
                let support = rng.random_range(0..n_qubits);
                let theta = rng.random_range(0..4);
                let p = match rng.random_range(0..3) {
                    0 => Pauli::X,
                    1 => Pauli::Y,
                    _ => Pauli::Z,
                };
                // construct the string
                // Each qubit slice has one u64, and only one is nonzero.
                let mut qubit_slices_z_bits = vec![vec![0u64]; n_qubits];
                let mut qubit_slices_x_bits = vec![vec![0u64]; n_qubits];
                let mut sign_bits = vec![0u64];
                match p {
                    Pauli::X => qubit_slices_x_bits[support][0] = 1,
                    Pauli::Y => {
                        qubit_slices_z_bits[support][0] = 1;
                        qubit_slices_x_bits[support][0] = 1;
                    }
                    Pauli::Z => qubit_slices_z_bits[support][0] = 1,
                    _ => panic!(),
                }

                let tqes = random_tqes(n_qubits, n_tqes, &mut rng);
                for &(g0, g1, q0, q1) in tqes.iter() {
                    let (q0_slice_z_bits, q1_slice_z_bits) =
                        get_two_mut(&mut qubit_slices_z_bits, q0, q1);
                    let (q0_slice_x_bits, q1_slice_x_bits) =
                        get_two_mut(&mut qubit_slices_x_bits, q0, q1);
                    apply_enum_tqe(
                        q0_slice_z_bits,
                        q0_slice_x_bits,
                        q1_slice_z_bits,
                        q1_slice_x_bits,
                        &mut sign_bits,
                        g0,
                        g1,
                    );
                }
                // Pack P' into an image.
                let n_words = n_qubits.div_ceil(64);
                let mut img_z_word = 0;
                let mut img_x_word = 0;
                let mut bits_filled = 0;
                let mut img_z_bits = Vec::with_capacity(n_words);
                let mut img_x_bits = Vec::with_capacity(n_words);

                for output_q in 0..n_qubits {
                    let slice_z_bit = qubit_slices_z_bits[output_q][0];
                    let slice_x_bit = qubit_slices_x_bits[output_q][0];
                    img_z_word |= slice_z_bit << bits_filled;
                    img_x_word |= slice_x_bit << bits_filled;
                    bits_filled += 1;
                    if bits_filled == 64 {
                        img_z_bits.push(img_z_word);
                        img_x_bits.push(img_x_word);
                        bits_filled = 0;
                        img_z_word = 0;
                        img_x_word = 0;
                    }
                }
                if bits_filled > 0 {
                    img_z_bits.push(img_z_word);
                    img_x_bits.push(img_x_word);
                }

                let mut tab1 = Tableau::random(n_qubits, 0, 1000, 1000);
                let mut tab2 = tab1.clone();
                assert_eq!(tab1, tab2);

                let half_pis = if sign_bits[0] == 0 {
                    theta
                } else {
                    match theta {
                        0 => 0,
                        1 => 3,
                        2 => 2,
                        3 => 1,
                        _ => panic!(),
                    }
                };
                postcompose_pauli_gadget(&mut tab1, &img_z_bits, &img_x_bits, half_pis);
                for &(g0, g1, q0, q1) in tqes.iter().rev() {
                    postcompose_tqe(&mut tab2, g0, g1, q0, q1);
                }
                match theta {
                    0 => {}
                    1 => {
                        postcompose_half_pi(&mut tab2, support, p, false);
                    }
                    2 => {
                        postcompose_pauli(&mut tab2, p, support);
                    }
                    3 => {
                        postcompose_half_pi(&mut tab2, support, p, true);
                    }
                    _ => panic!(),
                }
                for &(g0, g1, q0, q1) in tqes.iter() {
                    postcompose_tqe(&mut tab2, g0, g1, q0, q1);
                }
                assert_eq!(tab1, tab2);
            }
        }
    }

    #[test]
    fn test_gadget() {
        gadget_body(
            apply_enum_tqe_slice,
            Tableau::postcompose_pauli_gadget,
            Tableau::postcompose_tqe,
            Tableau::postcompose_half_pi,
            Tableau::postcompose_pauli,
        );
    }

    #[cfg(feature = "simd")]
    #[test]
    fn test_gadget_simd() {
        gadget_body(
            apply_enum_tqe_simd::<64>,
            Tableau::postcompose_pauli_gadget_simd::<64>,
            Tableau::postcompose_tqe_simd::<64>,
            Tableau::postcompose_half_pi_simd::<64>,
            Tableau::postcompose_pauli_simd::<64>,
        );
    }
}
