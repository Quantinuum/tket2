use super::Pauli;
use crate::kernels::*;
use crate::u8_encodings::{XX_U8, XY_U8, XZ_U8, YX_U8, YY_U8, YZ_U8, ZX_U8, ZY_U8, ZZ_U8};
use std::simd::{LaneCount, Simd, SupportedLaneCount};

// ------------ 1Q Clifford gates that mutate the Pauli operators --------------

macro_rules! simd_sq_remainder {
    ($gate_fn:ident, $base:expr, $remainder:expr, $zs:expr, $xs:expr, $signs:expr; $($lane:literal),*) => {
        match $remainder {
            $(
                $lane => {
                    let start = $base;
                    let end = $base + $lane - 1;
                    let z_chunk = &mut $zs[start..=end];
                    let x_chunk = &mut $xs[start..=end];
                    let s_chunk = &mut $signs[start..=end];
                    let zs_simd = Simd::<u64, $lane>::from_slice(z_chunk);
                    let xs_simd = Simd::<u64, $lane>::from_slice(x_chunk);
                    let s_simd = Simd::<u64, $lane>::from_slice(s_chunk);
                    let (new_z, new_x, sign_flip) = $gate_fn(zs_simd, xs_simd);
                    new_z.copy_to_slice(z_chunk);
                    new_x.copy_to_slice(x_chunk);
                    (sign_flip ^ s_simd).copy_to_slice(s_chunk);
                }
            )*
            _ => {},
        }
    };
}

macro_rules! make_simd_sq_gate {
    ($simd_fn:ident, $gate_fn:ident) => {
        #[doc = concat!(
            "SIMD slice version of [`",
            stringify!($gate_fn),
            "`], applied in-place.\n\n",
            "Processes `N`-lane SIMD chunks over the full slice. \n\n",
            "See [`",
            stringify!($gate_fn),
            "`] for the Pauli transformation and sign-flip logic."
        )]

        pub fn $simd_fn<const N: usize>(zs: &mut [u64], xs: &mut [u64], signs: &mut [u64])
        where
            LaneCount<N>: SupportedLaneCount,
        {
            let len = zs.len();
            let chunks = len / N;
            for i in 0..chunks {
                let start = i * N;
                let end = (i + 1) * N - 1;
                let z_chunk = &mut zs[start..=end];
                let x_chunk = &mut xs[start..=end];
                let s_chunk = &mut signs[start..=end];
                let zs_simd = Simd::<u64, N>::from_slice(z_chunk);
                let xs_simd = Simd::<u64, N>::from_slice(x_chunk);
                let s_simd = Simd::<u64, N>::from_slice(s_chunk);
                let (new_z, new_x, sign_flip) = $gate_fn(zs_simd, xs_simd);
                new_z.copy_to_slice(z_chunk);
                new_x.copy_to_slice(x_chunk);
                (sign_flip ^ s_simd).copy_to_slice(s_chunk);
            }
            let base = chunks * N;
            let remainder = len - base;
            simd_sq_remainder!($gate_fn, base, remainder, zs, xs, signs;
                1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
                21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38,
                39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56,
                57, 58, 59, 60, 61, 62, 63
            );
        }
    };
}

// --- 1Q Clifford gates that only mutate the sign bits (i.e. Pauli gates) -----

macro_rules! simd_sq_remainder_sign_only {
    ($gate_fn:ident, $base:expr, $remainder:expr, $zs:expr, $xs:expr, $signs:expr; $($lane:literal),*) => {
        match $remainder {
            $(
                $lane => {
                    let start = $base;
                    let end = $base + $lane - 1;
                    let z_chunk = & $zs[start..=end];
                    let x_chunk = & $xs[start..=end];
                    let s_chunk = &mut $signs[start..=end];
                    let zs_simd = Simd::<u64, $lane>::from_slice(z_chunk);
                    let xs_simd = Simd::<u64, $lane>::from_slice(x_chunk);
                    let s_simd = Simd::<u64, $lane>::from_slice(s_chunk);
                    let sign_flip = $gate_fn(zs_simd, xs_simd);
                    (sign_flip ^ s_simd).copy_to_slice(s_chunk);
                }
            )*
            _ => {},
        }
    };
}
macro_rules! make_simd_sq_gate_sign_only {
    ($simd_fn:ident, $gate_fn:ident) => {
        #[doc = concat!(
            "SIMD slice version of [`",
            stringify!($gate_fn),
            "`], applied in-place.\n\n",
            "Processes `N`-lane SIMD chunks over the full slice. \n\n",
            "See [`",
            stringify!($gate_fn),
            "`] for the sign-flip logic."
        )]

        pub fn $simd_fn<const N: usize>(zs: & [u64], xs: & [u64], signs: &mut [u64])
        where
            LaneCount<N>: SupportedLaneCount,
        {
            let len = zs.len();
            let chunks = len / N;
            for i in 0..chunks {
                let start = i * N;
                let end = (i + 1) * N - 1;
                let z_chunk = & zs[start..=end];
                let x_chunk = & xs[start..=end];
                let s_chunk = &mut signs[start..=end];
                let zs_simd = Simd::<u64, N>::from_slice(z_chunk);
                let xs_simd = Simd::<u64, N>::from_slice(x_chunk);
                let s_simd = Simd::<u64, N>::from_slice(s_chunk);
                let sign_flip = $gate_fn(zs_simd, xs_simd);
                (sign_flip ^ s_simd).copy_to_slice(s_chunk);
            }
            let base = chunks * N;
            let remainder = len - base;
            simd_sq_remainder_sign_only!($gate_fn, base, remainder, zs, xs, signs;
                1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
                21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38,
                39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56,
                57, 58, 59, 60, 61, 62, 63
            );
        }
    };
}

make_simd_sq_gate!(simd_h_gate, h_gate);
make_simd_sq_gate!(simd_s_gate, s_gate);
make_simd_sq_gate!(simd_sdg_gate, sdg_gate);
make_simd_sq_gate!(simd_v_gate, v_gate);
make_simd_sq_gate!(simd_vdg_gate, vdg_gate);
make_simd_sq_gate!(simd_ry90_gate, ry90_gate);
make_simd_sq_gate!(simd_ry270_gate, ry270_gate);
make_simd_sq_gate!(simd_rz90_gate, rz90_gate);
make_simd_sq_gate!(simd_rz270_gate, rz270_gate);
make_simd_sq_gate!(simd_rx90_gate, rx90_gate);
make_simd_sq_gate!(simd_rx270_gate, rx270_gate);
make_simd_sq_gate_sign_only!(simd_x_gate, x_gate);
make_simd_sq_gate_sign_only!(simd_y_gate, y_gate);
make_simd_sq_gate_sign_only!(simd_z_gate, z_gate);

#[inline(always)]
fn get_tq_simd_chunks<'a, const N: usize>(
    start: usize,
    end: usize,
    zs0: &'a mut [u64],
    xs0: &'a mut [u64],
    zs1: &'a mut [u64],
    xs1: &'a mut [u64],
    signs: &'a mut [u64],
) -> (
    &'a mut [u64],
    &'a mut [u64],
    &'a mut [u64],
    &'a mut [u64],
    &'a mut [u64],
    Simd<u64, N>,
    Simd<u64, N>,
    Simd<u64, N>,
    Simd<u64, N>,
    Simd<u64, N>,
)
where
    LaneCount<N>: SupportedLaneCount,
{
    let z0_chunk = &mut zs0[start..=end];
    let x0_chunk = &mut xs0[start..=end];
    let z1_chunk = &mut zs1[start..=end];
    let x1_chunk = &mut xs1[start..=end];
    let s_chunk = &mut signs[start..=end];
    let z0_simd = Simd::<u64, N>::from_slice(z0_chunk);
    let x0_simd = Simd::<u64, N>::from_slice(x0_chunk);
    let z1_simd = Simd::<u64, N>::from_slice(z1_chunk);
    let x1_simd = Simd::<u64, N>::from_slice(x1_chunk);
    let s_simd = Simd::<u64, N>::from_slice(s_chunk);
    (
        z0_chunk, x0_chunk, z1_chunk, x1_chunk, s_chunk, z0_simd, x0_simd, z1_simd, x1_simd, s_simd,
    )
}

// ---------------------------- 2Q Clifford gates ------------------------------

macro_rules! simd_tq_remainder {
    ($gate_fn:ident, $base:expr, $remainder:expr, $zs0:expr, $xs0:expr, $zs1:expr, $xs1:expr, $signs:expr; $($lane:literal),*) => {
        match $remainder {
            $(
                $lane => {
                    let (z0_chunk, x0_chunk, z1_chunk, x1_chunk, s_chunk,
                         z0_simd, x0_simd, z1_simd, x1_simd, s_simd) =
                        get_tq_simd_chunks::<$lane>($base, $base + $lane - 1,
                            $zs0, $xs0, $zs1, $xs1, $signs);
                    let (new_z0, new_x0, new_z1, new_x1, sign_flip) =
                        $gate_fn(z0_simd, x0_simd, z1_simd, x1_simd);
                    new_z0.copy_to_slice(z0_chunk);
                    new_x0.copy_to_slice(x0_chunk);
                    new_z1.copy_to_slice(z1_chunk);
                    new_x1.copy_to_slice(x1_chunk);
                    (sign_flip ^ s_simd).copy_to_slice(s_chunk);
                }
            )*
            _ => {},
        }
    };
}

macro_rules! make_simd_tq_gate {
    ($simd_fn:ident, $gate_fn:ident) => {
        #[doc = concat!(
            "SIMD slice version of [`", stringify!($gate_fn), "`], applied in-place.\n\n",
            "See [`", stringify!($gate_fn), "`] for the Pauli transformation and sign-flip logic."
        )]

        pub fn $simd_fn<const N: usize>(
            zs0: &mut [u64], xs0: &mut [u64],
            zs1: &mut [u64], xs1: &mut [u64],
            signs: &mut [u64],
        ) where LaneCount<N>: SupportedLaneCount {
            let len = zs0.len();
            let chunks = len / N;
            for i in 0..chunks {
                let (z0_chunk, x0_chunk, z1_chunk, x1_chunk, s_chunk,
                     z0_simd, x0_simd, z1_simd, x1_simd, s_simd) =
                    get_tq_simd_chunks::<N>(i * N, (i + 1) * N - 1, zs0, xs0, zs1, xs1, signs);
                let (new_z0, new_x0, new_z1, new_x1, sign_flip) =
                    $gate_fn(z0_simd, x0_simd, z1_simd, x1_simd);
                new_z0.copy_to_slice(z0_chunk);
                new_x0.copy_to_slice(x0_chunk);
                new_z1.copy_to_slice(z1_chunk);
                new_x1.copy_to_slice(x1_chunk);
                (sign_flip ^ s_simd).copy_to_slice(s_chunk);
            }
            let base = chunks * N;
            let remainder = len - base;
            simd_tq_remainder!($gate_fn, base, remainder, zs0, xs0, zs1, xs1, signs;
                1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
                21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38,
                39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56,
                57, 58, 59, 60, 61, 62, 63
            );
        }
    };
}

make_simd_tq_gate!(simd_xx_gate, xx_gate);
make_simd_tq_gate!(simd_xy_gate, xy_gate);
make_simd_tq_gate!(simd_xz_gate, xz_gate);
make_simd_tq_gate!(simd_yy_gate, yy_gate);
make_simd_tq_gate!(simd_yz_gate, yz_gate);
make_simd_tq_gate!(simd_zz_gate, zz_gate);

/// Branchless SIMD TQE gate where the gate type is specified using bit mask arguments.
///
/// # Arguments
///
/// - `zs0` (`&mut [u64]`) - z bits on the first qubit.
/// - `xs0` (`&mut [u64]`) - x bits on the first qubit.
/// - `zs1` (`&mut [u64]`) - z bits on the second qubit.
/// - `xs1` (`&mut [u64]`) - x bits on the second qubit.
/// - `signs` (`&mut [u64]`) - sign bits.
/// - `gz0` (`u64`) - the z component of the gate on the first qubit.
/// - `gx0` (`u64`) - the x component of the gate on the first qubit.
/// - `gz1` (`u64`) - the z component of the gate on the second qubit.
/// - `gx1` (`u64`) - the x component of the gate on the second qubit.
///
/// # Example
///
/// XY gate: `gz0=0u64, gx0=u64::MAX, gz1=u64::MAX, gx1=u64::MAX`
///
/// Compared to the specialised functions (e.g., `simd_xx_gate`), this function
/// avoids branching prediction penalties in some loop scenarios.
pub fn apply_branchless_tqe_simd<const N: usize>(
    zs0: &mut [u64],
    xs0: &mut [u64],
    zs1: &mut [u64],
    xs1: &mut [u64],
    signs: &mut [u64],
    gz0: u64,
    gx0: u64,
    gz1: u64,
    gx1: u64,
) where
    LaneCount<N>: SupportedLaneCount,
{
    let len = zs0.len();
    let chunks = len / N;
    let gz0_simd = Simd::<u64, N>::splat(gz0);
    let gx0_simd = Simd::<u64, N>::splat(gx0);
    let gz1_simd = Simd::<u64, N>::splat(gz1);
    let gx1_simd = Simd::<u64, N>::splat(gx1);

    let not_gx0_simd = !gx0_simd;
    let not_gx1_simd = !gx1_simd;

    let not_gx0 = !gx0;
    let not_gx1 = !gx1;

    for i in 0..chunks {
        let (
            z0_chunk,
            x0_chunk,
            z1_chunk,
            x1_chunk,
            s_chunk,
            pz0_simd,
            px0_simd,
            pz1_simd,
            px1_simd,
            s_simd,
        ) = get_tq_simd_chunks::<N>(i * N, (i + 1) * N - 1, zs0, xs0, zs1, xs1, signs);
        let gz0_px0 = gz0_simd & px0_simd;
        let gx0_pz0 = gx0_simd & pz0_simd;
        let gz1_px1 = gz1_simd & px1_simd;
        let gx1_pz1 = gx1_simd & pz1_simd;

        let a0 = gz0_px0 ^ gx0_pz0;
        let a1 = gz1_px1 ^ gx1_pz1;
        let new_z0_simd = (gz0_simd & a1) ^ pz0_simd;
        let new_x0_simd = (gx0_simd & a1) ^ px0_simd;
        let new_z1_simd = (gz1_simd & a0) ^ pz1_simd;
        let new_x1_simd = (gx1_simd & a0) ^ px1_simd;

        // flip sign if a0&a1&(cyclic0 ^ cyclic1)
        let new_s_simd = s_simd
            ^ (a0
                & a1
                & (((gx0_pz0 & (gz0_simd ^ px0_simd)) | (gz0_px0 & not_gx0_simd & !pz0_simd))
                    ^ ((gx1_pz1 & (gz1_simd ^ px1_simd)) | (gz1_px1 & not_gx1_simd & !pz1_simd))));

        new_z0_simd.copy_to_slice(z0_chunk);
        new_x0_simd.copy_to_slice(x0_chunk);
        new_z1_simd.copy_to_slice(z1_chunk);
        new_x1_simd.copy_to_slice(x1_chunk);
        new_s_simd.copy_to_slice(s_chunk);
    }
    let start = chunks * N;
    for idx in start..len {
        let pz0 = zs0[idx];
        let px0 = xs0[idx];
        let pz1 = zs1[idx];
        let px1 = xs1[idx];
        let gz0_px0 = gz0 & px0;
        let gx0_pz0 = gx0 & pz0;
        let gz1_px1 = gz1 & px1;
        let gx1_pz1 = gx1 & pz1;
        let a0 = gz0_px0 ^ gx0_pz0;
        let a1 = gz1_px1 ^ gx1_pz1;
        zs0[idx] = (gz0 & a1) ^ pz0;
        xs0[idx] = (gx0 & a1) ^ px0;
        zs1[idx] = (gz1 & a0) ^ pz1;
        xs1[idx] = (gx1 & a0) ^ px1;
        signs[idx] ^= a0
            & a1
            & (((gx0_pz0 & (gz0 ^ px0)) | (gz0_px0 & not_gx0 & !pz0))
                ^ ((gx1_pz1 & (gz1 ^ px1)) | (gz1_px1 & not_gx1 & !pz1)));
    }
}

/// Apply TQE gate where the gate type is encoded in two Pauli enums.
/// Operations are performed using SIMD
///
/// # Arguments
///
/// - `zs0` (`&mut [u64]`) -  z bits on the first qubit.
/// - `xs0` (`&mut [u64]`) -  x bits on the first qubit.
/// - `zs1` (`&mut [u64]`) -  z bits on the second qubit.
/// - `xs1` (`&mut [u64]`) -  x bits on the second qubit.
/// - `signs` (`&mut [u64]`) -  sign bits for the qubits.
/// - `g0` (`Pauli`) -  Pauli on the first qubit.
/// - `g1` (`Pauli`) -  Pauli on the second qubit.
///
pub fn apply_enum_tqe_simd<const N: usize>(
    zs0: &mut [u64],
    xs0: &mut [u64],
    zs1: &mut [u64],
    xs1: &mut [u64],
    signs: &mut [u64],
    g0: Pauli,
    g1: Pauli,
) where
    LaneCount<N>: SupportedLaneCount,
{
    match (g0, g1) {
        (Pauli::X, Pauli::X) => {
            simd_xx_gate::<N>(zs0, xs0, zs1, xs1, signs);
        }
        (Pauli::X, Pauli::Y) => {
            simd_xy_gate::<N>(zs0, xs0, zs1, xs1, signs);
        }
        (Pauli::X, Pauli::Z) => {
            simd_xz_gate::<N>(zs0, xs0, zs1, xs1, signs);
        }
        (Pauli::Y, Pauli::X) => {
            simd_xy_gate::<N>(zs1, xs1, zs0, xs0, signs);
        }
        (Pauli::Y, Pauli::Y) => {
            simd_yy_gate::<N>(zs0, xs0, zs1, xs1, signs);
        }
        (Pauli::Y, Pauli::Z) => {
            simd_yz_gate::<N>(zs0, xs0, zs1, xs1, signs);
        }
        (Pauli::Z, Pauli::X) => {
            simd_xz_gate::<N>(zs1, xs1, zs0, xs0, signs);
        }
        (Pauli::Z, Pauli::Y) => {
            simd_yz_gate::<N>(zs1, xs1, zs0, xs0, signs);
        }
        (Pauli::Z, Pauli::Z) => {
            simd_zz_gate::<N>(zs0, xs0, zs1, xs1, signs);
        }
        _ => panic!("Invalid TQE gate"),
    }
}

/// Apply TQE gate where the gate type is encoded in a u8.
/// Operations are performed using SIMD
///
/// The encoding is as follows:
/// - 0: XX
/// - 1: XY
/// - 2: XZ
/// - 3: YX
/// - 4: YY
/// - 5: YZ
/// - 6: ZX
/// - 7: ZY
/// - 8: ZZ
///
/// # Arguments
///
/// - `zs0` (`&mut [u64]`) -  z bits on the first qubit.
/// - `xs0` (`&mut [u64]`) -  x bits on the first qubit.
/// - `zs1` (`&mut [u64]`) -  z bits on the second qubit.
/// - `xs1` (`&mut [u64]`) -  x bits on the second qubit.
/// - `signs` (`&mut [u64]`) -  sign bits for the qubits.
/// - `gate` (`u8`) -  the encoded gate type.
///
pub fn apply_u8_tqe_simd<const N: usize>(
    zs0: &mut [u64],
    xs0: &mut [u64],
    zs1: &mut [u64],
    xs1: &mut [u64],
    signs: &mut [u64],
    gate: u8,
) where
    LaneCount<N>: SupportedLaneCount,
{
    match gate {
        XX_U8 => {
            simd_xx_gate::<N>(zs0, xs0, zs1, xs1, signs);
        }
        XY_U8 => {
            simd_xy_gate::<N>(zs0, xs0, zs1, xs1, signs);
        }
        XZ_U8 => {
            simd_xz_gate::<N>(zs0, xs0, zs1, xs1, signs);
        }
        YX_U8 => {
            simd_xy_gate::<N>(zs1, xs1, zs0, xs0, signs);
        }
        YY_U8 => {
            simd_yy_gate::<N>(zs0, xs0, zs1, xs1, signs);
        }
        YZ_U8 => {
            simd_yz_gate::<N>(zs0, xs0, zs1, xs1, signs);
        }
        ZX_U8 => {
            simd_xz_gate::<N>(zs1, xs1, zs0, xs0, signs);
        }
        ZY_U8 => {
            simd_yz_gate::<N>(zs1, xs1, zs0, xs0, signs);
        }
        ZZ_U8 => {
            simd_zz_gate::<N>(zs0, xs0, zs1, xs1, signs);
        }
        _ => panic!("Invalid TQE gate"),
    }
}

/// Apply half pi Pauli rotation gate
/// Operations are performed using SIMD
///
/// # Arguments
///
/// - `zs` (`&mut [u64]`) - z bits.
/// - `xs` (`&mut [u64]`) - x bits.
/// - `signs` (`&mut [u64]`) - sign bits.
/// - `axis` (`Pauli`) - rotation axis.
/// - `neg` (`bool`) - whether the angle is negative.
///
pub fn apply_half_pi_gate_simd<const N: usize>(
    zs: &mut [u64],
    xs: &mut [u64],
    signs: &mut [u64],
    axis: Pauli,
    neg: bool,
) where
    LaneCount<N>: SupportedLaneCount,
{
    match (axis, neg) {
        (Pauli::X, false) => simd_rx90_gate(zs, xs, signs),
        (Pauli::X, true) => simd_rx270_gate(zs, xs, signs),
        (Pauli::Y, false) => simd_ry90_gate(zs, xs, signs),
        (Pauli::Y, true) => simd_ry270_gate(zs, xs, signs),
        (Pauli::Z, false) => simd_rz90_gate(zs, xs, signs),
        (Pauli::Z, true) => simd_rz270_gate(zs, xs, signs),
        _ => panic!("Invalid axis"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernels::tests::*;
    use crate::paulis_to_u64s;
    use crate::slice::tests::*;
    use crate::test_utils::*;
    use rstest::rstest;

    fn simd_sq_gate_fn(name: &str) -> SliceSqGateFn {
        match name {
            "H" => simd_h_gate::<64>,
            "S" => simd_s_gate::<64>,
            "SDG" => simd_sdg_gate::<64>,
            "V" => simd_v_gate::<64>,
            "VDG" => simd_vdg_gate::<64>,
            "RX90" => simd_rx90_gate::<64>,
            "RX270" => simd_rx270_gate::<64>,
            "RY90" => simd_ry90_gate::<64>,
            "RY270" => simd_ry270_gate::<64>,
            "RZ90" => simd_rz90_gate::<64>,
            "RZ270" => simd_rz270_gate::<64>,
            _ => panic!("Invalid gate name"),
        }
    }

    fn simd_pauli_gate_fn(name: &str) -> SlicePauliGateFn {
        match name {
            "X" => simd_x_gate::<64>,
            "Y" => simd_y_gate::<64>,
            "Z" => simd_z_gate::<64>,
            _ => panic!("Invalid gate name"),
        }
    }

    fn simd_tq_gate_fn(name: &str) -> SliceTqeGateFn {
        match name {
            "XX" => simd_xx_gate::<64>,
            "XY" => simd_xy_gate::<64>,
            "XZ" => simd_xz_gate::<64>,
            "YY" => simd_yy_gate::<64>,
            "YZ" => simd_yz_gate::<64>,
            "ZZ" => simd_zz_gate::<64>,
            _ => panic!("Invalid gate name"),
        }
    }

    #[rstest]
    #[case(300, 0)]
    #[case(5, 1)]
    fn test_simd_pauli_gates(#[case] n_paulis: usize, #[case] seed: u64) {
        let paulis = random_paulis(n_paulis, seed);
        let (zs, xs) = paulis_to_u64s(&paulis);
        for gate in PAULI_GATES.iter() {
            let gate_func = simd_pauli_gate_fn(gate);
            let mut signs: Vec<u64> = vec![0; zs.len()];
            gate_func(&zs, &xs, &mut signs);
            // compute the expected result using the slice function
            let slice_gate_func = slice_pauli_gate_fn(gate);
            let mut expected_signs: Vec<u64> = vec![0; zs.len()];
            slice_gate_func(&zs, &xs, &mut expected_signs);
            assert_eq!(signs, expected_signs);
        }
    }

    #[rstest]
    #[case(300, 0)]
    #[case(5, 1)]
    fn test_simd_sq_gates(#[case] n_paulis: usize, #[case] seed: u64) {
        let paulis = random_paulis(n_paulis, seed);
        let (zs, xs) = paulis_to_u64s(&paulis);
        for gate in SQ_GATES.iter() {
            let gate_func = simd_sq_gate_fn(gate);
            let mut zs_clone = zs.clone();
            let mut xs_clone = xs.clone();
            let mut signs: Vec<u64> = vec![0; zs.len()];
            gate_func(&mut zs_clone, &mut xs_clone, &mut signs);
            // compute the expected result using the slice function
            let slice_gate_func = slice_sq_gate_fn(gate);
            let mut expected_zs = zs.clone();
            let mut expected_xs = xs.clone();
            let mut expected_signs: Vec<u64> = vec![0; zs.len()];
            slice_gate_func(&mut expected_zs, &mut expected_xs, &mut expected_signs);
            assert_eq!(zs_clone, expected_zs);
            assert_eq!(xs_clone, expected_xs);
            assert_eq!(signs, expected_signs);
        }
    }

    #[rstest]
    #[case(300, 0, 1)]
    #[case(5, 1, 2)]
    fn test_simd_tq_gates(#[case] n_paulis: usize, #[case] seed0: u64, #[case] seed1: u64) {
        let paulis0 = random_paulis(n_paulis, seed0);
        let (zs0, xs0) = paulis_to_u64s(&paulis0);
        let paulis1 = random_paulis(n_paulis, seed1);
        let (zs1, xs1) = paulis_to_u64s(&paulis1);
        for gate in TQ_GATES.iter() {
            let gate_func = simd_tq_gate_fn(gate);
            let mut zs0_clone = zs0.clone();
            let mut xs0_clone = xs0.clone();
            let mut zs1_clone = zs1.clone();
            let mut xs1_clone = xs1.clone();
            let mut signs: Vec<u64> = vec![0; zs0.len()];
            gate_func(
                &mut zs0_clone,
                &mut xs0_clone,
                &mut zs1_clone,
                &mut xs1_clone,
                &mut signs,
            );
            // compute the expected result using the slice function
            let slice_gate_func = slice_tq_gate_fn(gate);
            let mut expected_zs0 = zs0.clone();
            let mut expected_xs0 = xs0.clone();
            let mut expected_zs1 = zs1.clone();
            let mut expected_xs1 = xs1.clone();
            let mut expected_signs: Vec<u64> = vec![0; zs0.len()];
            slice_gate_func(
                &mut expected_zs0,
                &mut expected_xs0,
                &mut expected_zs1,
                &mut expected_xs1,
                &mut expected_signs,
            );
            assert_eq!(zs0_clone, expected_zs0);
            assert_eq!(xs0_clone, expected_xs0);
            assert_eq!(zs1_clone, expected_zs1);
            assert_eq!(xs1_clone, expected_xs1);
            assert_eq!(signs, expected_signs);
        }
    }

    #[rstest]
    #[case(5, 0)]
    #[case(64, 1)]
    #[case(130, 2)]
    fn test_tqe_wrapper_dispatch_simd(#[case] len: usize, #[case] seed: u64) {
        for (g0, g1, gate, reference_func, swapped) in tq_wrapper_cases() {
            let base = (
                random_u64s(len, seed),
                random_u64s(len, seed + 1),
                random_u64s(len, seed + 2),
                random_u64s(len, seed + 3),
                random_u64s(len, seed + 4),
            );
            let (
                mut expected_zs0,
                mut expected_xs0,
                mut expected_zs1,
                mut expected_xs1,
                mut expected_signs,
            ) = base.clone();
            let (mut enum_zs0, mut enum_xs0, mut enum_zs1, mut enum_xs1, mut enum_signs) =
                base.clone();
            let (mut u8_zs0, mut u8_xs0, mut u8_zs1, mut u8_xs1, mut u8_signs) = base;

            apply_tq_reference(
                reference_func,
                swapped,
                &mut expected_zs0,
                &mut expected_xs0,
                &mut expected_zs1,
                &mut expected_xs1,
                &mut expected_signs,
            );
            apply_enum_tqe_simd::<64>(
                &mut enum_zs0,
                &mut enum_xs0,
                &mut enum_zs1,
                &mut enum_xs1,
                &mut enum_signs,
                g0,
                g1,
            );
            apply_u8_tqe_simd::<64>(
                &mut u8_zs0,
                &mut u8_xs0,
                &mut u8_zs1,
                &mut u8_xs1,
                &mut u8_signs,
                gate,
            );

            let expected = (
                expected_zs0.as_slice(),
                expected_xs0.as_slice(),
                expected_zs1.as_slice(),
                expected_xs1.as_slice(),
                expected_signs.as_slice(),
            );
            assert_eq!(
                (
                    enum_zs0.as_slice(),
                    enum_xs0.as_slice(),
                    enum_zs1.as_slice(),
                    enum_xs1.as_slice(),
                    enum_signs.as_slice(),
                ),
                expected
            );
            assert_eq!(
                (
                    u8_zs0.as_slice(),
                    u8_xs0.as_slice(),
                    u8_zs1.as_slice(),
                    u8_xs1.as_slice(),
                    u8_signs.as_slice(),
                ),
                expected
            );
        }
    }

    #[rstest]
    #[case(5, 0)]
    #[case(64, 1)]
    #[case(130, 2)]
    fn test_half_pi_wrapper_dispatch_simd(#[case] len: usize, #[case] seed: u64) {
        for (axis, neg, reference_func) in half_pi_wrapper_cases() {
            let base = (
                random_u64s(len, seed),
                random_u64s(len, seed + 1),
                random_u64s(len, seed + 2),
            );
            let (mut expected_zs, mut expected_xs, mut expected_signs) = base.clone();
            let (mut wrapper_zs, mut wrapper_xs, mut wrapper_signs) = base;

            reference_func(&mut expected_zs, &mut expected_xs, &mut expected_signs);
            apply_half_pi_gate_simd::<64>(
                &mut wrapper_zs,
                &mut wrapper_xs,
                &mut wrapper_signs,
                axis,
                neg,
            );

            assert_eq!(wrapper_zs, expected_zs);
            assert_eq!(wrapper_xs, expected_xs);
            assert_eq!(wrapper_signs, expected_signs);
        }
    }
}
