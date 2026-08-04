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
/// * `zb_str0` - Mutable Z-bits of the first Pauli string (modified in-place)
/// * `xb_str0` - Mutable X-bits of the first Pauli string (modified in-place)
/// * `zb_str1` - Z-bits of the second Pauli string
/// * `xb_str1` - X-bits of the second Pauli string
///
/// # Returns
///
/// The phase factor as the position in [1, i, -1, -i]
///
fn string_mul(zb_str0: &mut [u64], xb_str0: &mut [u64], zb_str1: &[u64], xb_str1: &[u64]) -> u8 {
    let len = zb_str0.len();
    let mut n_anti_commute_sites: u64 = 0;
    let mut n_anti_cyclic_sites: u64 = 0;
    for idx in 0..len {
        let z0 = zb_str0[idx];
        let x0 = xb_str0[idx];
        let z1 = zb_str1[idx];
        let x1 = xb_str1[idx];
        zb_str0[idx] ^= z1;
        xb_str0[idx] ^= x1;
        let z0x1 = z0 & x1;
        let x0z1 = z1 & x0;
        let anti_commute = z0x1 ^ x0z1;
        let anti_cyclic = (x0z1 & !z0 & !x1) | (z0x1 & (x0 ^ z1));
        n_anti_commute_sites += anti_commute.count_ones() as u64;
        n_anti_cyclic_sites += anti_cyclic.count_ones() as u64;
    }
    ((n_anti_commute_sites as i64 - 2 * n_anti_cyclic_sites as i64).rem_euclid(4)) as u8
}

/// Given a Pauli string, count the number of Ys
fn count_y(zb_str: &[u64], xb_str: &[u64]) -> u64 {
    let len = zb_str.len();
    let mut n_ys = 0;
    for idx in 0..len {
        n_ys += (zb_str[idx] & xb_str[idx]).count_ones() as u64;
    }
    n_ys
}

/// Multiplies two bit-packed Pauli strings using SIMD operations, computing the phase.
#[cfg(feature = "simd")]
fn simd_string_mul<const N: usize>(
    zb_str0: &mut [u64],
    xb_str0: &mut [u64],
    zb_str1: &[u64],
    xb_str1: &[u64],
) -> u8
where
    LaneCount<N>: SupportedLaneCount,
{
    let len = zb_str0.len();
    let chunks = len / N;
    let mut n_anti_commute_sites: u64 = 0;
    let mut n_anti_cyclic_sites: u64 = 0;
    for i in 0..chunks {
        let z_chunk0 = &mut zb_str0[i * N..(i + 1) * N];
        let x_chunk0 = &mut xb_str0[i * N..(i + 1) * N];
        let z_chunk1 = &zb_str1[i * N..(i + 1) * N];
        let x_chunk1 = &xb_str1[i * N..(i + 1) * N];
        let z0 = Simd::<u64, N>::from_slice(z_chunk0);
        let x0 = Simd::<u64, N>::from_slice(x_chunk0);
        let z1 = Simd::<u64, N>::from_slice(z_chunk1);
        let x1 = Simd::<u64, N>::from_slice(x_chunk1);
        let z_res = z0 ^ z1;
        let x_res = x0 ^ x1;
        z_res.copy_to_slice(z_chunk0);
        x_res.copy_to_slice(x_chunk0);
        let z0x1 = z0 & x1;
        let x0z1 = z1 & x0;
        let anti_commute = z0x1 ^ x0z1;
        let anti_cyclic = (x0z1 & !z0 & !x1) | (z0x1 & (x0 ^ z1));
        n_anti_commute_sites += anti_commute.count_ones().reduce_sum();
        n_anti_cyclic_sites += anti_cyclic.count_ones().reduce_sum();
    }
    // TODO: Use SIMD for the remaining elements.
    let start = chunks * N;
    for idx in start..len {
        let z0 = zb_str0[idx];
        let x0 = xb_str0[idx];
        let z1 = zb_str1[idx];
        let x1 = xb_str1[idx];
        zb_str0[idx] ^= z1;
        xb_str0[idx] ^= x1;
        let z0x1 = z0 & x1;
        let x0z1 = z1 & x0;
        let anti_commute = z0x1 ^ x0z1;
        let anti_cyclic = (x0z1 & !z0 & !x1) | (z0x1 & (x0 ^ z1));
        n_anti_commute_sites += anti_commute.count_ones() as u64;
        n_anti_cyclic_sites += anti_cyclic.count_ones() as u64;
    }
    ((n_anti_commute_sites as i64 - 2 * n_anti_cyclic_sites as i64).rem_euclid(4)) as u8
}

/// Given a Pauli string, count the number of Ys using SIMD
#[cfg(feature = "simd")]
fn simd_count_y<const N: usize>(zb_str: &[u64], xb_str: &[u64]) -> u64
where
    LaneCount<N>: SupportedLaneCount,
{
    let len = zb_str.len();
    let chunks = len / N;

    let mut n_ys = 0;
    for i in 0..chunks {
        let zb_chunk = &zb_str[i * N..(i + 1) * N];
        let xb_chunk = &xb_str[i * N..(i + 1) * N];
        let zb_chunk_simd = Simd::<u64, N>::from_slice(zb_chunk);
        let xb_chunk_simd = Simd::<u64, N>::from_slice(xb_chunk);
        n_ys += (zb_chunk_simd & xb_chunk_simd).count_ones().reduce_sum();
    }
    // TODO: Use SIMD for the remaining elements.
    let start = chunks * N;
    for idx in start..len {
        let zb_chunk = zb_str[idx];
        let xb_chunk = xb_str[idx];
        n_ys += (zb_chunk & xb_chunk).count_ones() as u64;
    }
    n_ys
}

/// A qubit(row)-major implementation of a unitary tableau
/// # Example
/// |     | Z0 | X0 | Z1 | X1 |
/// | -------- | - | - | - | - |
/// | \(Q_0\)  | Z | X | Y | I |
/// | \(Q_1\)  | X | X | Z | X |
/// | Sign |+|+|+|-|
///
/// The X0 column says that an X operator at qubit 0 on the input will
/// be transformed into XX over the output.
///
/// Our tableau implementation is row major, and the Z bits and X bits of the Pauli letters
/// are stored separately. The tableau above is therefore stored, before bit packing, as:
/// zb_rows = \[
/// \[1,0,1,0\],
/// \[0,0,1,0\]
/// \]
/// xb_rows = \[
/// \[0,1,1,0\],
/// \[1,1,0,1\]
/// \]
/// signs = \[0, 0, 0, 1\]
/// n_qubits = 2
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Tableau {
    zb_rows: Vec<Vec<u64>>,
    xb_rows: Vec<Vec<u64>>,
    signs: Vec<u64>,
    n_qubits: usize,
}

impl fmt::Display for Tableau {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        for i in 0..self.n_qubits {
            let paulis = u64s_to_paulis(&self.zb_rows[i], &self.xb_rows[i], self.n_qubits * 2);
            for p in paulis {
                write!(f, "{p} ")?;
            }
            writeln!(f)?;
        }
        let total_sign_bits = 2 * self.n_qubits;
        let mut bits_printed = 0;
        for s in &self.signs {
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
    /// Get the Z-bits of the tableau rows.
    pub fn get_zb_rows(&self) -> &Vec<Vec<u64>> {
        &self.zb_rows
    }
    /// Get the X-bits of the tableau rows.
    pub fn get_xb_rows(&self) -> &Vec<Vec<u64>> {
        &self.xb_rows
    }
    /// Get the sign bits of the tableau rows. 1 represents a negative sign, and 0 represents a positive sign.
    pub fn get_signs(&self) -> &Vec<u64> {
        &self.signs
    }

    /// Split the tableau into mutable references for two qubits.
    ///
    /// # Panics
    ///
    /// Panics if `q0 == q1`.
    pub fn split_mut(
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
        let (zb0, zb1) = get_two_mut(&mut self.zb_rows, q0, q1);
        let (xb0, xb1) = get_two_mut(&mut self.xb_rows, q0, q1);
        (zb0, zb1, xb0, xb1, &mut self.signs)
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
        let (zb_row0, zb_row1) = get_two_mut(&mut self.zb_rows, q0, q1);
        let (xb_row0, xb_row1) = get_two_mut(&mut self.xb_rows, q0, q1);
        match (g0, g1) {
            (Pauli::X, Pauli::X) => xx_gate(zb_row0, xb_row0, zb_row1, xb_row1, &mut self.signs),
            (Pauli::X, Pauli::Y) => xy_gate(zb_row0, xb_row0, zb_row1, xb_row1, &mut self.signs),
            (Pauli::X, Pauli::Z) => xz_gate(zb_row0, xb_row0, zb_row1, xb_row1, &mut self.signs),
            (Pauli::Y, Pauli::X) => xy_gate(zb_row1, xb_row1, zb_row0, xb_row0, &mut self.signs),
            (Pauli::Y, Pauli::Y) => yy_gate(zb_row0, xb_row0, zb_row1, xb_row1, &mut self.signs),
            (Pauli::Y, Pauli::Z) => yz_gate(zb_row0, xb_row0, zb_row1, xb_row1, &mut self.signs),
            (Pauli::Z, Pauli::X) => xz_gate(zb_row1, xb_row1, zb_row0, xb_row0, &mut self.signs),
            (Pauli::Z, Pauli::Y) => yz_gate(zb_row1, xb_row1, zb_row0, xb_row0, &mut self.signs),
            (Pauli::Z, Pauli::Z) => zz_gate(zb_row0, xb_row0, zb_row1, xb_row1, &mut self.signs),
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
        let (gs, gl, qs, ql) = {
            if g0 <= g1 {
                (g0, g1, q0, q1)
            } else {
                (g1, g0, q1, q0)
            }
        };
        let z0_col_index = 2 * qs;
        let x0_col_index = 2 * qs + 1;
        let z1_col_index = 2 * ql;
        let x1_col_index = 2 * ql + 1;
        let (mut zb_z0_col, mut xb_z0_col, s_z0_col) = self.get_col(z0_col_index);
        let (mut zb_x0_col, mut xb_x0_col, s_x0_col) = self.get_col(x0_col_index);
        let (mut zb_z1_col, mut xb_z1_col, s_z1_col) = self.get_col(z1_col_index);
        let (mut zb_x1_col, mut xb_x1_col, s_x1_col) = self.get_col(x1_col_index);

        match (gs, gl) {
            (Pauli::X, Pauli::X) => {
                // z0 <- z0*x1
                let phase = string_mul(&mut zb_z0_col, &mut xb_z0_col, &zb_x1_col, &xb_x1_col);
                self.set_col(
                    z0_col_index,
                    &zb_z0_col,
                    &xb_z0_col,
                    s_z0_col ^ s_x1_col ^ (phase == 2),
                );
                // z1 <- z1*x0
                let phase = string_mul(&mut zb_z1_col, &mut xb_z1_col, &zb_x0_col, &xb_x0_col);
                self.set_col(
                    z1_col_index,
                    &zb_z1_col,
                    &xb_z1_col,
                    s_z1_col ^ s_x0_col ^ (phase == 2),
                );
            }
            (Pauli::X, Pauli::Y) => {
                // z0 <- z0*y1 = iz0*x1*z1
                let mut phase = string_mul(&mut zb_z0_col, &mut xb_z0_col, &zb_x1_col, &xb_x1_col);
                phase += string_mul(&mut zb_z0_col, &mut xb_z0_col, &zb_z1_col, &xb_z1_col);
                phase = (phase + 1) % 4;
                self.set_col(
                    z0_col_index,
                    &zb_z0_col,
                    &xb_z0_col,
                    s_z0_col ^ s_z1_col ^ s_x1_col ^ (phase == 2),
                );
                // z1 <- z1*x0
                let phase = string_mul(&mut zb_z1_col, &mut xb_z1_col, &zb_x0_col, &xb_x0_col);
                self.set_col(
                    z1_col_index,
                    &zb_z1_col,
                    &xb_z1_col,
                    s_z1_col ^ s_x0_col ^ (phase == 2),
                );
                // x1 <- x1*x0
                let phase = string_mul(&mut zb_x1_col, &mut xb_x1_col, &zb_x0_col, &xb_x0_col);
                self.set_col(
                    x1_col_index,
                    &zb_x1_col,
                    &xb_x1_col,
                    s_x1_col ^ s_x0_col ^ (phase == 2),
                );
            }
            (Pauli::X, Pauli::Z) => {
                // z0 <- z0*z1
                let phase = string_mul(&mut zb_z0_col, &mut xb_z0_col, &zb_z1_col, &xb_z1_col);
                self.set_col(
                    z0_col_index,
                    &zb_z0_col,
                    &xb_z0_col,
                    s_z0_col ^ s_z1_col ^ (phase == 2),
                );
                // x1 <- x1*x0
                let phase = string_mul(&mut zb_x1_col, &mut xb_x1_col, &zb_x0_col, &xb_x0_col);
                self.set_col(
                    x1_col_index,
                    &zb_x1_col,
                    &xb_x1_col,
                    s_x0_col ^ s_x1_col ^ (phase == 2),
                );
            }
            (Pauli::Y, Pauli::Y) => {
                // convert sign to phase for readability
                let z0_phase = (s_z0_col as u8) << 1;
                let x0_phase = (s_x0_col as u8) << 1;
                let z1_phase = (s_z1_col as u8) << 1;
                let x1_phase = (s_x1_col as u8) << 1;
                // y1 = ix1*z1
                let mut zb_y1_col = zb_x1_col;
                let mut xb_y1_col = xb_x1_col;
                let y1_phase = string_mul(&mut zb_y1_col, &mut xb_y1_col, &zb_z1_col, &xb_z1_col)
                    + 1
                    + z1_phase
                    + x1_phase;
                // z0 <- z0*y1
                let z0_phase_new =
                    (string_mul(&mut zb_z0_col, &mut xb_z0_col, &zb_y1_col, &xb_y1_col)
                        + z0_phase
                        + y1_phase)
                        % 4;
                self.set_col(z0_col_index, &zb_z0_col, &xb_z0_col, z0_phase_new == 2);
                // x0 <- x0*y1
                let x0_phase_new =
                    (string_mul(&mut zb_x0_col, &mut xb_x0_col, &zb_y1_col, &xb_y1_col)
                        + x0_phase
                        + y1_phase)
                        % 4;
                self.set_col(x0_col_index, &zb_x0_col, &xb_x0_col, x0_phase_new == 2);
                // y0 = ix0*z0 = i * x0*y1 * z0*y1
                let mut zb_y0_col = zb_x0_col;
                let mut xb_y0_col = xb_x0_col;
                let y0_phase = string_mul(&mut zb_y0_col, &mut xb_y0_col, &zb_z0_col, &xb_z0_col)
                    + z0_phase_new
                    + x0_phase_new
                    + 1;
                // z1 <- z1*y0
                let z1_phase_new =
                    (string_mul(&mut zb_z1_col, &mut xb_z1_col, &zb_y0_col, &xb_y0_col)
                        + z1_phase
                        + y0_phase)
                        % 4;
                self.set_col(z1_col_index, &zb_z1_col, &xb_z1_col, z1_phase_new == 2);
                // x1 <- x1*y0 = (-iy1*z1)*y0 = -iy1* (z1*y0)
                let mut zb_x1_col = zb_y1_col;
                let mut xb_x1_col = xb_y1_col;
                let x1_new_phase =
                    (string_mul(&mut zb_x1_col, &mut xb_x1_col, &zb_z1_col, &xb_z1_col)
                        + y1_phase
                        + z1_phase_new
                        + 3)
                        % 4;
                self.set_col(x1_col_index, &zb_x1_col, &xb_x1_col, x1_new_phase == 2);
            }
            (Pauli::Y, Pauli::Z) => {
                // x1 <- x1*y0 = i x1*x0*z0
                let mut phase = string_mul(&mut zb_x1_col, &mut xb_x1_col, &zb_x0_col, &xb_x0_col);
                phase += string_mul(&mut zb_x1_col, &mut xb_x1_col, &zb_z0_col, &xb_z0_col);
                phase = (phase + 1) % 4;
                self.set_col(
                    x1_col_index,
                    &zb_x1_col,
                    &xb_x1_col,
                    s_x1_col ^ s_z0_col ^ s_x0_col ^ (phase == 2),
                );
                // z0 <- z0*z1
                let phase = string_mul(&mut zb_z0_col, &mut xb_z0_col, &zb_z1_col, &xb_z1_col);
                self.set_col(
                    z0_col_index,
                    &zb_z0_col,
                    &xb_z0_col,
                    s_z0_col ^ s_z1_col ^ (phase == 2),
                );
                // x0 <- x0*z1
                let phase = string_mul(&mut zb_x0_col, &mut xb_x0_col, &zb_z1_col, &xb_z1_col);
                self.set_col(
                    x0_col_index,
                    &zb_x0_col,
                    &xb_x0_col,
                    s_x0_col ^ s_z1_col ^ (phase == 2),
                );
            }
            (Pauli::Z, Pauli::Z) => {
                // x0 <- x0*z1
                let phase = string_mul(&mut zb_x0_col, &mut xb_x0_col, &zb_z1_col, &xb_z1_col);
                self.set_col(
                    x0_col_index,
                    &zb_x0_col,
                    &xb_x0_col,
                    s_x0_col ^ s_z1_col ^ (phase == 2),
                );
                // x1 <- x1*z0
                let phase = string_mul(&mut zb_x1_col, &mut xb_x1_col, &zb_z0_col, &xb_z0_col);
                self.set_col(
                    x1_col_index,
                    &zb_x1_col,
                    &xb_x1_col,
                    s_x1_col ^ s_z0_col ^ (phase == 2),
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
            Pauli::X => v_gate(&mut self.zb_rows[q], &mut self.xb_rows[q], &mut self.signs),
            Pauli::Y => h_gate(&mut self.zb_rows[q], &mut self.xb_rows[q], &mut self.signs),
            Pauli::Z => s_gate(&mut self.zb_rows[q], &mut self.xb_rows[q], &mut self.signs),
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
        let z_col_index = 2 * q;
        let x_col_index = 2 * q + 1;
        let (mut zb_z_col, mut xb_z_col, s_z_col) = self.get_col(z_col_index);
        let (mut zb_x_col, mut xb_x_col, s_x_col) = self.get_col(x_col_index);
        match axis {
            Pauli::X => {
                // v gate
                // x <- x
                // z <- -y = iz*x
                let mut phase = string_mul(&mut zb_z_col, &mut xb_z_col, &zb_x_col, &xb_x_col);
                // 1 is i
                phase = (phase + 1) % 4;
                self.set_col(
                    z_col_index,
                    &zb_z_col,
                    &xb_z_col,
                    s_z_col ^ s_x_col ^ (phase == 2) ^ dagger,
                );
            }
            Pauli::Y => {
                // h gate
                self.set_col(z_col_index, &zb_x_col, &xb_x_col, s_x_col);
                self.set_col(x_col_index, &zb_z_col, &xb_z_col, s_z_col);
            }
            Pauli::Z => {
                // s gate
                // x <- y = ix*z
                // z <- z
                let mut phase = string_mul(&mut zb_x_col, &mut xb_x_col, &zb_z_col, &xb_z_col);
                // 1 is i
                phase = (phase + 1) % 4;
                self.set_col(
                    x_col_index,
                    &zb_x_col,
                    &xb_x_col,
                    s_z_col ^ s_x_col ^ (phase == 2) ^ dagger,
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
            &mut self.zb_rows[q],
            &mut self.xb_rows[q],
            &mut self.signs,
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
            Pauli::X => x_gate(&self.zb_rows[q], &self.xb_rows[q], &mut self.signs),
            Pauli::Y => y_gate(&self.zb_rows[q], &self.xb_rows[q], &mut self.signs),
            Pauli::Z => z_gate(&self.zb_rows[q], &self.xb_rows[q], &mut self.signs),
            _ => panic!("Unexpected basis gate!"),
        }
    }

    /// Post compose a Pauli gadget.
    fn postcompose_pauli_gadget_with(
        &mut self,
        zb_pauli: &[u64],
        xb_pauli: &[u64],
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
            for i in 0..self.n_qubits {
                let chunk = i / 64;
                let bit = i % 64;
                let zb = (zb_pauli[chunk] >> bit) & 1;
                let xb = (xb_pauli[chunk] >> bit) & 1;
                if zb == 1 && xb == 1 {
                    y_gate(&self.zb_rows[i], &self.xb_rows[i], &mut self.signs);
                } else if zb == 1 && xb == 0 {
                    z_gate(&self.zb_rows[i], &self.xb_rows[i], &mut self.signs);
                } else if zb == 0 && xb == 1 {
                    x_gate(&self.zb_rows[i], &self.xb_rows[i], &mut self.signs);
                }
            }
            return;
        }
        // we apply the gadget using a ladder
        // we find the first non-identity
        let (a, pa) = {
            let mut result = None;
            for (n, (zb_chunk, xb_chunk)) in zb_pauli.iter().zip(xb_pauli).enumerate() {
                let zorx = zb_chunk | xb_chunk;
                let bit = zorx.trailing_zeros();
                if bit < 64 {
                    let index = n * 64 + bit as usize;
                    let zb = (zb_chunk >> bit) & 1;
                    let xb = (xb_chunk >> bit) & 1;
                    let pauli = match (zb, xb) {
                        (0, 1) => Pauli::X,
                        (1, 0) => Pauli::Z,
                        (1, 1) => Pauli::Y,
                        _ => panic!("Shouldn't be"),
                    };
                    result = Some((index, pauli));
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
            let chunk = b / 64;
            let bit = b % 64;
            let zb = (zb_pauli[chunk] >> bit) & 1;
            let xb = (xb_pauli[chunk] >> bit) & 1;
            // Skip identity
            let gb = match (zb, xb) {
                (0, 1) => Pauli::X,
                (1, 0) => Pauli::Z,
                (1, 1) => Pauli::Y,
                _ => continue,
            };

            // Store the gate for reverse application
            tqe_gates.push((a, b, ga, gb));

            let (zb_rowa, zb_rowb) = get_two_mut(&mut self.zb_rows, a, b);
            let (xb_rowa, xb_rowb) = get_two_mut(&mut self.xb_rows, a, b);
            apply_enum_tqe(zb_rowa, xb_rowa, zb_rowb, xb_rowb, &mut self.signs, ga, gb);
        }

        // apply the 1q clifford gate
        self.postcompose_half_pi_with(a, pa, half_pis == 3, apply_half_pi);
        // Apply TQE gates in reverse order
        for (qa, qb, ga_rev, gb_rev) in tqe_gates.iter().rev() {
            let (zb_rowa, zb_rowb) = get_two_mut(&mut self.zb_rows, *qa, *qb);
            let (xb_rowa, xb_rowb) = get_two_mut(&mut self.xb_rows, *qa, *qb);
            apply_enum_tqe(
                zb_rowa,
                xb_rowa,
                zb_rowb,
                xb_rowb,
                &mut self.signs,
                *ga_rev,
                *gb_rev,
            );
        }
    }

    /// Conjugate a Pauli string by conjugating the z bits and x bits separately
    fn apply_to_pauli_with(
        &self,
        zb_pauli: &[u64],
        xb_pauli: &[u64],
        string_mul: StringMulFn,
        count_y: CountYFn,
    ) -> (Vec<u64>, Vec<u64>, bool) {
        let mut phase: u64 = 0;
        let mut sign_flip = false;
        let mut base_zb = vec![0u64; zb_pauli.len()];
        let mut base_xb = vec![0u64; xb_pauli.len()];
        'outer: for (chunk_index, xb_chunk) in xb_pauli.iter().enumerate() {
            let mut current_chunk = *xb_chunk;
            for _ in 0..xb_chunk.count_ones() {
                let i = current_chunk.trailing_zeros() as usize;
                let q_index = chunk_index * 64 + i;
                if q_index == self.n_qubits {
                    break 'outer;
                }
                let (zb_x_col, xb_x_col, s) = self.get_col(2 * q_index + 1);
                sign_flip ^= s;
                phase += string_mul(&mut base_zb, &mut base_xb, &zb_x_col, &xb_x_col) as u64;
                current_chunk &= current_chunk - 1; // Remove the lowest set bit
            }
        }
        'outer: for (chunk_index, zb_chunk) in zb_pauli.iter().enumerate() {
            let mut current_chunk = *zb_chunk;
            for _ in 0..zb_chunk.count_ones() {
                let i = current_chunk.trailing_zeros() as usize;
                let q_index = chunk_index * 64 + i;
                if q_index == self.n_qubits {
                    break 'outer;
                }
                let (zb_z_col, xb_z_col, s) = self.get_col(2 * q_index);
                sign_flip ^= s;
                phase += string_mul(&mut base_zb, &mut base_xb, &zb_z_col, &xb_z_col) as u64;
                current_chunk &= current_chunk - 1; // Remove the lowest set bit
            }
        }
        // count the number of ys, each y contribute a i phase, hence shift the total phase by 1
        let n_ys = count_y(zb_pauli, xb_pauli);
        (base_zb, base_xb, sign_flip ^ ((phase + n_ys) % 4 == 2))
    }

    /// Multi-threaded version of apply_to_pauli_with
    /// the conjugation of z-bits and x-bits, as well as the counting of ys, are done in parallel
    fn apply_to_pauli_mt_with(
        &self,
        zb_pauli: &[u64],
        xb_pauli: &[u64],
        string_mul: StringMulFn,
        count_y: CountYFn,
    ) -> (Vec<u64>, Vec<u64>, bool) {
        let mut phase0: u64 = 0;
        let mut sign_flip0 = false;
        let mut phase1: u64 = 0;
        let mut sign_flip1 = false;
        let mut n_ys: u64 = 0;
        let mut base_zb0 = vec![0u64; zb_pauli.len()];
        let mut base_xb0 = vec![0u64; xb_pauli.len()];
        let mut base_zb1 = vec![0u64; zb_pauli.len()];
        let mut base_xb1 = vec![0u64; xb_pauli.len()];
        rayon::scope(|s| {
            s.spawn(|_| {
                'outer: for (chunk_index, xb_chunk) in xb_pauli.iter().enumerate() {
                    let mut current_chunk = *xb_chunk;
                    for _ in 0..xb_chunk.count_ones() {
                        let i = current_chunk.trailing_zeros() as usize;
                        let q_index = chunk_index * 64 + i;
                        if q_index == self.n_qubits {
                            break 'outer;
                        }
                        let (zb_x_col, xb_x_col, s) = self.get_col(2 * q_index + 1);
                        sign_flip0 ^= s;
                        phase0 +=
                            string_mul(&mut base_zb0, &mut base_xb0, &zb_x_col, &xb_x_col) as u64;
                        current_chunk &= current_chunk - 1; // Remove the lowest set bit
                    }
                }
            });
            s.spawn(|_| {
                'outer: for (chunk_index, zb_chunk) in zb_pauli.iter().enumerate() {
                    let mut current_chunk = *zb_chunk;
                    for _ in 0..zb_chunk.count_ones() {
                        let i = current_chunk.trailing_zeros() as usize;
                        let q_index = chunk_index * 64 + i;
                        if q_index == self.n_qubits {
                            break 'outer;
                        }
                        let (zb_z_col, xb_z_col, s) = self.get_col(2 * q_index);
                        sign_flip1 ^= s;
                        phase1 +=
                            string_mul(&mut base_zb1, &mut base_xb1, &zb_z_col, &xb_z_col) as u64;
                        current_chunk &= current_chunk - 1; // Remove the lowest set bit
                    }
                }
            });
            s.spawn(|_| {
                n_ys = count_y(zb_pauli, xb_pauli);
            });
        });
        let phase = string_mul(&mut base_zb0, &mut base_xb0, &base_zb1, &base_xb1) as u64;
        // count the number of ys, each y contributes a i phase, hence shift the total phase by 1
        (
            base_zb0,
            base_xb0,
            sign_flip0 ^ sign_flip1 ^ ((phase0 + phase1 + phase + n_ys) % 4 == 2),
        )
    }

    /// Invert the tableau
    fn invert_with<F, G>(&self, apply_fn: F, apply_mt_fn: G) -> Tableau
    where
        F: Fn(&Tableau, &[u64], &[u64]) -> (Vec<u64>, Vec<u64>, bool) + Copy,
        G: Fn(&Tableau, &[u64], &[u64]) -> (Vec<u64>, Vec<u64>, bool) + Copy,
    {
        let n_cols = self.n_qubits * 2;
        let n_row_u64s = n_cols.div_ceil(64);
        let mut zb_rows_inv: Vec<Vec<u64>> = Vec::with_capacity(self.n_qubits);
        let mut xb_rows_inv: Vec<Vec<u64>> = Vec::with_capacity(self.n_qubits);
        // TODO: consider parallelising this loop
        for col_index in 0..self.n_qubits {
            let col_chunk_index = (2 * col_index) / 64;
            let z_col_bit_offset = (2 * col_index) % 64;
            let x_col_bit_offset = z_col_bit_offset + 1;
            let mut zb_row_inv: Vec<u64> = Vec::with_capacity(n_row_u64s);
            let mut xb_row_inv: Vec<u64> = Vec::with_capacity(n_row_u64s);
            let mut zb_chunk_inv: u64 = 0;
            let mut xb_chunk_inv: u64 = 0;
            let mut row_bits_filled = 0;

            for row_index in 0..self.n_qubits {
                let zb_col_chunk = self.zb_rows[row_index][col_chunk_index];
                let xb_col_chunk = self.xb_rows[row_index][col_chunk_index];
                let zb_z_col = (zb_col_chunk >> z_col_bit_offset) & 0b1;
                let zb_x_col = (zb_col_chunk >> x_col_bit_offset) & 0b1;
                let xb_z_col = (xb_col_chunk >> z_col_bit_offset) & 0b1;
                let xb_x_col = (xb_col_chunk >> x_col_bit_offset) & 0b1;

                xb_chunk_inv |= xb_z_col << row_bits_filled;
                xb_chunk_inv |= zb_z_col << (row_bits_filled + 1);

                zb_chunk_inv |= xb_x_col << row_bits_filled;
                zb_chunk_inv |= zb_x_col << (row_bits_filled + 1);
                row_bits_filled += 2;
                if row_bits_filled == 64 {
                    zb_row_inv.push(zb_chunk_inv);
                    xb_row_inv.push(xb_chunk_inv);
                    zb_chunk_inv = 0;
                    xb_chunk_inv = 0;
                    row_bits_filled = 0;
                }
            }
            if row_bits_filled > 0 {
                zb_row_inv.push(zb_chunk_inv);
                xb_row_inv.push(xb_chunk_inv);
            }
            zb_rows_inv.push(zb_row_inv);
            xb_rows_inv.push(xb_row_inv);
        }
        let mut tab_inv = Self {
            zb_rows: zb_rows_inv,
            xb_rows: xb_rows_inv,
            signs: vec![0; self.signs.len()],
            n_qubits: self.n_qubits,
        };

        // set signs
        for i in 0..self.n_qubits {
            let (zb_z_col, xb_z_col, _) = tab_inv.get_col(2 * i);
            let (zb_x_col, xb_x_col, _) = tab_inv.get_col(2 * i + 1);
            let (z_col_sign_bit, x_col_sign_bit) = if self.n_qubits >= MT_THRESH {
                let (_, _, zsb) = apply_mt_fn(self, &zb_z_col, &xb_z_col);
                let (_, _, xsb) = apply_mt_fn(self, &zb_x_col, &xb_x_col);
                (zsb, xsb)
            } else {
                let (_, _, zsb) = apply_fn(self, &zb_z_col, &xb_z_col);
                let (_, _, xsb) = apply_fn(self, &zb_x_col, &xb_x_col);
                (zsb, xsb)
            };
            tab_inv.set_sign(2 * i, z_col_sign_bit);
            tab_inv.set_sign(2 * i + 1, x_col_sign_bit);
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
            let z_col_index = 2 * q;
            let x_col_index = 2 * q + 1;
            let (mut zb_z_col, mut xb_z_col, s_z_col) = self.get_col(z_col_index);
            let (mut zb_x_col, mut xb_x_col, s_x_col) = self.get_col(x_col_index);
            let (new_zb_z_col, new_xb_z_col, z_col_sign_bit) = if self.n_qubits >= MT_THRESH {
                apply_mt_fn(other, &mut zb_z_col, &mut xb_z_col)
            } else {
                apply_fn(other, &mut zb_z_col, &mut xb_z_col)
            };
            let (new_zb_x_col, new_xb_x_col, x_col_sign_bit) = if self.n_qubits >= MT_THRESH {
                apply_mt_fn(other, &mut zb_x_col, &mut xb_x_col)
            } else {
                apply_fn(other, &mut zb_x_col, &mut xb_x_col)
            };

            self.set_col(
                z_col_index,
                &new_zb_z_col,
                &new_xb_z_col,
                z_col_sign_bit ^ s_z_col,
            );
            self.set_col(
                x_col_index,
                &new_zb_x_col,
                &new_xb_x_col,
                x_col_sign_bit ^ s_x_col,
            );
        }
    }

    /// Creates a new Tableau with the given components.
    ///
    /// # Arguments
    ///
    /// * `zb_rows` - Z-bit rows
    /// * `xb_rows` - X-bit rows
    /// * `signs` - Sign bits for each column, 0: + and 1: -
    /// * `n_qubits` - Number of qubits the tableau operates on
    ///
    /// # Returns
    ///
    /// A new Tableau instance
    // TODO: need input validation
    pub fn new(
        zb_rows: Vec<Vec<u64>>,
        xb_rows: Vec<Vec<u64>>,
        signs: Vec<u64>,
        n_qubits: usize,
    ) -> Self {
        Self {
            zb_rows,
            xb_rows,
            signs,
            n_qubits,
        }
    }

    /// Initialise an identity Tableau
    pub fn eye(n_qubits: usize) -> Self {
        let n_cols = n_qubits * 2;
        let mut zb_rows = Vec::with_capacity(n_qubits);
        let mut xb_rows = Vec::with_capacity(n_qubits);
        for i in 0..n_qubits {
            let mut paulis = vec![Pauli::I; n_cols];
            paulis[2 * i] = Pauli::Z;
            paulis[2 * i + 1] = Pauli::X;
            let (row_zb, row_xb) = paulis_to_u64s(&paulis);
            zb_rows.push(row_zb);
            xb_rows.push(row_xb);
        }
        let n_u64s = n_cols.div_ceil(64);
        Self {
            zb_rows,
            xb_rows,
            signs: vec![0; n_u64s],
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

    /// get a column by index
    // TODO:  consider reusable scratch buffers for columns to avoid repeated allocations
    fn get_col(&self, col_index: usize) -> (Vec<u64>, Vec<u64>, bool) {
        let chunk_index = col_index / 64;
        let bit_index = col_index % 64;
        let n_u64s = self.n_qubits.div_ceil(64);
        let mut zb_chunk = 0;
        let mut xb_chunk = 0;
        let mut bits_filled = 0;
        let mut zb_chunks = Vec::with_capacity(n_u64s);
        let mut xb_chunks = Vec::with_capacity(n_u64s);

        for row_index in 0..self.n_qubits {
            let zb = (self.zb_rows[row_index][chunk_index] >> bit_index) & 1;
            let xb = (self.xb_rows[row_index][chunk_index] >> bit_index) & 1;
            zb_chunk |= zb << bits_filled;
            xb_chunk |= xb << bits_filled;
            bits_filled += 1;
            if bits_filled == 64 {
                zb_chunks.push(zb_chunk);
                xb_chunks.push(xb_chunk);
                bits_filled = 0;
                zb_chunk = 0;
                xb_chunk = 0;
            }
        }
        if bits_filled > 0 {
            zb_chunks.push(zb_chunk);
            xb_chunks.push(xb_chunk);
        }
        (
            zb_chunks,
            xb_chunks,
            (self.signs[chunk_index] >> bit_index & 1 == 1),
        )
    }

    /// set a column
    fn set_col(&mut self, col_index: usize, zb_col: &[u64], xb_col: &[u64], sign_bit: bool) {
        let chunk_index = col_index / 64;
        let bit_index = col_index % 64;
        for row_index in 0..self.n_qubits {
            let col_chunk_index = row_index / 64;
            let col_bit_index = row_index % 64;
            let zb_bit = (zb_col[col_chunk_index] >> col_bit_index) & 1;
            let xb_bit = (xb_col[col_chunk_index] >> col_bit_index) & 1;
            self.zb_rows[row_index][chunk_index] &= !(1 << bit_index);
            self.zb_rows[row_index][chunk_index] |= zb_bit << bit_index;
            self.xb_rows[row_index][chunk_index] &= !(1 << bit_index);
            self.xb_rows[row_index][chunk_index] |= xb_bit << bit_index;
        }
        if sign_bit {
            self.signs[chunk_index] |= 1 << bit_index;
        } else {
            self.signs[chunk_index] &= !(1 << bit_index);
        }
    }
    /// flip sign
    fn flip_sign(&mut self, col_index: usize) {
        let chunk_index = col_index / 64;
        let bit_index = col_index % 64;
        self.signs[chunk_index] ^= 1 << bit_index;
    }
    /// set sign
    fn set_sign(&mut self, col_index: usize, sign_bit: bool) {
        let chunk_index = col_index / 64;
        let bit_index = col_index % 64;
        if sign_bit {
            self.signs[chunk_index] |= 1 << bit_index;
        } else {
            self.signs[chunk_index] &= !(1 << bit_index);
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
        let z_col_index = 2 * q;
        let x_col_index = 2 * q + 1;
        match p {
            Pauli::X => {
                self.flip_sign(z_col_index);
            }
            Pauli::Y => {
                self.flip_sign(z_col_index);
                self.flip_sign(x_col_index);
            }
            Pauli::Z => {
                self.flip_sign(x_col_index);
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
        let (zb_zcol0, xb_zcol0, s_col0) = self.get_col(2 * q0);
        let (zb_zcol1, xb_zcol1, s_col1) = self.get_col(2 * q1);
        let (zb_xcol0, xb_xcol0, s_xcol0) = self.get_col(2 * q0 + 1);
        let (zb_xcol1, xb_xcol1, s_xcol1) = self.get_col(2 * q1 + 1);
        self.set_col(2 * q0, &zb_zcol1, &xb_zcol1, s_col1);
        self.set_col(2 * q1, &zb_zcol0, &xb_zcol0, s_col0);
        self.set_col(2 * q0 + 1, &zb_xcol1, &xb_xcol1, s_xcol1);
        self.set_col(2 * q1 + 1, &zb_xcol0, &xb_xcol0, s_xcol0);
    }

    /// Postcompose a SWAP gate.
    ///
    /// # Arguments
    ///
    /// * `q0` - Index of the first qubit
    /// * `q1` - Index of the second qubit
    ///
    pub fn postcompose_swap(&mut self, q0: usize, q1: usize) {
        self.zb_rows.swap(q0, q1);
        self.xb_rows.swap(q0, q1);
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
    /// * `zb_pauli` - Z bits of the Pauli string
    /// * `xb_pauli` - X bits of the Pauli string
    /// * `half_pis` - number of half pi turns, [0,1,2,3]
    ///
    /// # Panics
    ///
    /// Panics if `zb_pauli`/`xb_pauli` encode the all-identity Pauli string.
    pub fn postcompose_pauli_gadget(&mut self, zb_pauli: &[u64], xb_pauli: &[u64], half_pis: u8) {
        self.postcompose_pauli_gadget_with(
            zb_pauli,
            xb_pauli,
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
    /// * `zb_pauli` - Packed Z bits of the Pauli string (modified in-place)
    /// * `xb_pauli` - Packed X bits of the Pauli string (modified in-place)
    ///
    /// # Returns
    ///
    /// The conjugated Z bits, X bits, and a boolean indicating whether the transformation results in a sign flip
    ///
    pub fn apply_to_pauli(&self, zb_pauli: &[u64], xb_pauli: &[u64]) -> (Vec<u64>, Vec<u64>, bool) {
        self.apply_to_pauli_with(zb_pauli, xb_pauli, string_mul, count_y)
    }

    /// Multi-threaded version of apply_to_pauli
    ///
    /// Note: For small tableaus it's worth benchmarking against the non-MT `apply_to_pauli`.
    pub fn apply_to_pauli_mt(
        &self,
        zb_pauli: &[u64],
        xb_pauli: &[u64],
    ) -> (Vec<u64>, Vec<u64>, bool) {
        self.apply_to_pauli_mt_with(zb_pauli, xb_pauli, string_mul, count_y)
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

    /// Return image of X on the ith qubit. Negative sign indicates a phase of +1.
    pub fn get_x_col_paulis(&self, i: usize) -> (Vec<Pauli>, bool) {
        let res = self.get_col(2 * i + 1);
        (u64s_to_paulis(&res.0, &res.1, self.n_qubits), res.2)
    }

    /// Return image of Z on the ith qubit. Negative sign indicates a phase of +1.
    pub fn get_z_col_paulis(&self, i: usize) -> (Vec<Pauli>, bool) {
        let res = self.get_col(2 * i);
        (u64s_to_paulis(&res.0, &res.1, self.n_qubits), res.2)
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
    /// Panics if `zb_pauli`/`xb_pauli` encode the all-identity Pauli string.
    fn postcompose_pauli_gadget_simd<const N: usize>(
        &mut self,
        zb_pauli: &[u64],
        xb_pauli: &[u64],
        half_pis: u8,
    ) where
        LaneCount<N>: SupportedLaneCount;
    /// Conjugate a Pauli string using SIMD operations.
    fn apply_to_pauli_simd<const N: usize>(
        &self,
        zb_pauli: &[u64],
        xb_pauli: &[u64],
    ) -> (Vec<u64>, Vec<u64>, bool)
    where
        LaneCount<N>: SupportedLaneCount;
    /// Multi-threaded version of apply_to_pauli_simd.
    ///
    /// Note: For small tableaus it's worth benchmarking against the non-MT
    /// `apply_to_pauli_simd`.
    fn apply_to_pauli_mt_simd<const N: usize>(
        &self,
        zb_pauli: &[u64],
        xb_pauli: &[u64],
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
        zb_pauli: &[u64],
        xb_pauli: &[u64],
        half_pis: u8,
    ) where
        LaneCount<N>: SupportedLaneCount,
    {
        self.postcompose_pauli_gadget_with(
            zb_pauli,
            xb_pauli,
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
        zb_pauli: &[u64],
        xb_pauli: &[u64],
    ) -> (Vec<u64>, Vec<u64>, bool)
    where
        LaneCount<N>: SupportedLaneCount,
    {
        self.apply_to_pauli_with(zb_pauli, xb_pauli, simd_string_mul::<N>, simd_count_y::<N>)
    }

    fn apply_to_pauli_mt_simd<const N: usize>(
        &self,
        zb_pauli: &[u64],
        xb_pauli: &[u64],
    ) -> (Vec<u64>, Vec<u64>, bool)
    where
        LaneCount<N>: SupportedLaneCount,
    {
        self.apply_to_pauli_mt_with(zb_pauli, xb_pauli, simd_string_mul::<N>, simd_count_y::<N>)
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
                // each row has one u64, only one is non zero
                let mut zb_rows = vec![vec![0u64]; n_qubits];
                let mut xb_rows = vec![vec![0u64]; n_qubits];
                let mut signs = vec![0u64];
                match p {
                    Pauli::X => xb_rows[support][0] = 1,
                    Pauli::Y => {
                        zb_rows[support][0] = 1;
                        xb_rows[support][0] = 1;
                    }
                    Pauli::Z => zb_rows[support][0] = 1,
                    _ => panic!(),
                }

                let tqes = random_tqes(n_qubits, n_tqes, &mut rng);
                for &(g0, g1, q0, q1) in tqes.iter() {
                    let (zb0, zb1) = get_two_mut(&mut zb_rows, q0, q1);
                    let (xb0, xb1) = get_two_mut(&mut xb_rows, q0, q1);
                    apply_enum_tqe(zb0, xb0, zb1, xb1, &mut signs, g0, g1);
                }
                // turn P' into a column vector
                let n_u64s = n_qubits.div_ceil(64);
                let mut zb_chunk = 0;
                let mut xb_chunk = 0;
                let mut bits_filled = 0;
                let mut zb_col = Vec::with_capacity(n_u64s);
                let mut xb_col = Vec::with_capacity(n_u64s);

                for row_index in 0..n_qubits {
                    let zb = zb_rows[row_index][0];
                    let xb = xb_rows[row_index][0];
                    zb_chunk |= zb << bits_filled;
                    xb_chunk |= xb << bits_filled;
                    bits_filled += 1;
                    if bits_filled == 64 {
                        zb_col.push(zb_chunk);
                        xb_col.push(xb_chunk);
                        bits_filled = 0;
                        zb_chunk = 0;
                        xb_chunk = 0;
                    }
                }
                if bits_filled > 0 {
                    zb_col.push(zb_chunk);
                    xb_col.push(xb_chunk);
                }

                let mut tab1 = Tableau::random(n_qubits, 0, 1000, 1000);
                let mut tab2 = tab1.clone();
                assert_eq!(tab1, tab2);

                let half_pis = if signs[0] == 0 {
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
                postcompose_pauli_gadget(&mut tab1, &zb_col, &xb_col, half_pis);
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
