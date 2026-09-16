use super::Pauli;
use crate::kernels::*;
use crate::u8_encodings::{XX_U8, XY_U8, XZ_U8, YX_U8, YY_U8, YZ_U8, ZX_U8, ZY_U8, ZZ_U8};

// ------------ 1Q Clifford gates that mutate the Pauli operators --------------

// skip fmt due to a bug: https://github.com/rust-lang/rustfmt/issues/5489
#[rustfmt::skip]
macro_rules! make_slice_sq_gate {
    ($slice_fn:ident, $gate_fn:ident) => {
        #[doc = concat!(
            "Scalar slice version of [`",
            stringify!($gate_fn),
            "`], applied in-place.\n\n",
            "See [`",
            stringify!($gate_fn),
            "`] for the Pauli transformation and sign-flip logic."
        )]
        pub fn $slice_fn(zs: &mut [u64], xs: &mut [u64], signs: &mut [u64]) {
            for i in 0..zs.len() {
                let (new_z, new_x, sign_flip) = $gate_fn(zs[i], xs[i]);
                zs[i] = new_z;
                xs[i] = new_x;
                signs[i] ^= sign_flip;
            }
        }
    };
}

// --- 1Q Clifford gates that only mutate the sign bits (i.e. Pauli gates) -----

#[rustfmt::skip]
macro_rules! make_slice_sq_gate_sign_only {
    ($slice_fn:ident, $gate_fn:ident) => {
        #[doc = concat!(
            "Scalar slice version of [`",
            stringify!($gate_fn),
            "`], applied in-place.\n\n",
            "See [`",
            stringify!($gate_fn),
            "`] for the sign-flip logic."
        )]
        pub fn $slice_fn(zs: &[u64], xs: &[u64], signs: &mut [u64]) {
            for i in 0..zs.len() {
                signs[i] ^= $gate_fn(zs[i], xs[i]);
            }
        }
    };
}

// ---------------------------- 2Q Clifford gates ------------------------------

#[rustfmt::skip]
macro_rules! make_slice_tq_gate {
    ($slice_fn:ident, $gate_fn:ident) => {
        #[doc = concat!(
            "Scalar slice version of [`",
            stringify!($gate_fn),
            "`], applied in-place.\n\n",
            "See [`",
            stringify!($gate_fn),
            "`] for the Pauli transformation and sign-flip logic."
        )]
        pub fn $slice_fn(
            zs0: &mut [u64],
            xs0: &mut [u64],
            zs1: &mut [u64],
            xs1: &mut [u64],
            signs: &mut [u64],
        ) {
            for i in 0..zs0.len() {
                let (new_z0, new_x0, new_z1, new_x1, sign_flip) =
                    $gate_fn(zs0[i], xs0[i], zs1[i], xs1[i]);
                zs0[i] = new_z0;
                xs0[i] = new_x0;
                zs1[i] = new_z1;
                xs1[i] = new_x1;
                signs[i] ^= sign_flip;
            }
        }
    };
}

make_slice_sq_gate!(slice_h_gate, h_gate);
make_slice_sq_gate!(slice_s_gate, s_gate);
make_slice_sq_gate!(slice_sdg_gate, sdg_gate);
make_slice_sq_gate!(slice_v_gate, v_gate);
make_slice_sq_gate!(slice_vdg_gate, vdg_gate);
make_slice_sq_gate!(slice_ry90_gate, ry90_gate);
make_slice_sq_gate!(slice_ry270_gate, ry270_gate);
make_slice_sq_gate!(slice_rz90_gate, rz90_gate);
make_slice_sq_gate!(slice_rz270_gate, rz270_gate);
make_slice_sq_gate!(slice_rx90_gate, rx90_gate);
make_slice_sq_gate!(slice_rx270_gate, rx270_gate);

make_slice_sq_gate_sign_only!(slice_x_gate, x_gate);
make_slice_sq_gate_sign_only!(slice_y_gate, y_gate);
make_slice_sq_gate_sign_only!(slice_z_gate, z_gate);

make_slice_tq_gate!(slice_xx_gate, xx_gate);
make_slice_tq_gate!(slice_xy_gate, xy_gate);
make_slice_tq_gate!(slice_xz_gate, xz_gate);
make_slice_tq_gate!(slice_yy_gate, yy_gate);
make_slice_tq_gate!(slice_yz_gate, yz_gate);
make_slice_tq_gate!(slice_zz_gate, zz_gate);

/// Apply TQE gate where the gate type is encoded in two Pauli enums.
///
/// # Arguments
///
/// - `zs0` (`&mut [u64]`) - z bits on the first qubit.
/// - `xs0` (`&mut [u64]`) - x bits on the first qubit.
/// - `zs1` (`&mut [u64]`) - z bits on the second qubit.
/// - `xs1` (`&mut [u64]`) - x bits on the second qubit.
/// - `signs` (`&mut [u64]`) - sign bits for the qubits.
/// - `g0` (`Pauli`) -  Pauli on the first qubit.
/// - `g1` (`Pauli`) -  Pauli on the second qubit.
///
pub fn apply_enum_tqe_slice(
    zs0: &mut [u64],
    xs0: &mut [u64],
    zs1: &mut [u64],
    xs1: &mut [u64],
    signs: &mut [u64],
    g0: Pauli,
    g1: Pauli,
) {
    match (g0, g1) {
        (Pauli::X, Pauli::X) => slice_xx_gate(zs0, xs0, zs1, xs1, signs),
        (Pauli::X, Pauli::Y) => slice_xy_gate(zs0, xs0, zs1, xs1, signs),
        (Pauli::X, Pauli::Z) => slice_xz_gate(zs0, xs0, zs1, xs1, signs),
        (Pauli::Y, Pauli::X) => slice_xy_gate(zs1, xs1, zs0, xs0, signs),
        (Pauli::Y, Pauli::Y) => slice_yy_gate(zs0, xs0, zs1, xs1, signs),
        (Pauli::Y, Pauli::Z) => slice_yz_gate(zs0, xs0, zs1, xs1, signs),
        (Pauli::Z, Pauli::X) => slice_xz_gate(zs1, xs1, zs0, xs0, signs),
        (Pauli::Z, Pauli::Y) => slice_yz_gate(zs1, xs1, zs0, xs0, signs),
        (Pauli::Z, Pauli::Z) => slice_zz_gate(zs0, xs0, zs1, xs1, signs),
        _ => panic!("Invalid TQE gate"),
    }
}

/// Apply half pi Pauli rotation gate
///
/// # Arguments
///
/// - `zs` (`&mut [u64]`) - z bits.
/// - `xs` (`&mut [u64]`) - x bits.
/// - `signs` (`&mut [u64]`) - sign bits.
/// - `axis` (`Pauli`) - rotation axis.
/// - `neg` (`bool`) - whether the angle is negative. You should use this to apply the inverse of the gate.
///
pub fn apply_half_pi_gate_slice(
    zs: &mut [u64],
    xs: &mut [u64],
    signs: &mut [u64],
    axis: Pauli,
    neg: bool,
) {
    match (axis, neg) {
        (Pauli::X, false) => slice_rx90_gate(zs, xs, signs),
        (Pauli::X, true) => slice_rx270_gate(zs, xs, signs),
        (Pauli::Y, false) => slice_ry90_gate(zs, xs, signs),
        (Pauli::Y, true) => slice_ry270_gate(zs, xs, signs),
        (Pauli::Z, false) => slice_rz90_gate(zs, xs, signs),
        (Pauli::Z, true) => slice_rz270_gate(zs, xs, signs),
        _ => panic!("Invalid axis"),
    }
}

/// Apply TQE gate where the gate type is encoded in a u8.
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
pub fn apply_u8_tqe_slice(
    zs0: &mut [u64],
    xs0: &mut [u64],
    zs1: &mut [u64],
    xs1: &mut [u64],
    signs: &mut [u64],
    gate: u8,
) {
    match gate {
        XX_U8 => slice_xx_gate(zs0, xs0, zs1, xs1, signs),
        XY_U8 => slice_xy_gate(zs0, xs0, zs1, xs1, signs),
        XZ_U8 => slice_xz_gate(zs0, xs0, zs1, xs1, signs),
        YX_U8 => slice_xy_gate(zs1, xs1, zs0, xs0, signs),
        YY_U8 => slice_yy_gate(zs0, xs0, zs1, xs1, signs),
        YZ_U8 => slice_yz_gate(zs0, xs0, zs1, xs1, signs),
        ZX_U8 => slice_xz_gate(zs1, xs1, zs0, xs0, signs),
        ZY_U8 => slice_yz_gate(zs1, xs1, zs0, xs0, signs),
        ZZ_U8 => slice_zz_gate(zs0, xs0, zs1, xs1, signs),
        _ => panic!("Invalid TQE gate"),
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::kernels::tests::*;
    use crate::test_utils::*;
    use crate::{paulis_to_u64s, u64s_to_paulis, u64s_to_sign_flips};
    use rstest::rstest;

    pub(crate) type SliceSqGateFn = fn(&mut [u64], &mut [u64], &mut [u64]);
    pub(crate) type SlicePauliGateFn = fn(&[u64], &[u64], &mut [u64]);
    pub(crate) type SliceTqeGateFn = fn(&mut [u64], &mut [u64], &mut [u64], &mut [u64], &mut [u64]);

    pub(crate) fn slice_sq_gate_fn(name: &str) -> SliceSqGateFn {
        match name {
            "H" => slice_h_gate,
            "S" => slice_s_gate,
            "SDG" => slice_sdg_gate,
            "V" => slice_v_gate,
            "VDG" => slice_vdg_gate,
            "RX90" => slice_rx90_gate,
            "RX270" => slice_rx270_gate,
            "RY90" => slice_ry90_gate,
            "RY270" => slice_ry270_gate,
            "RZ90" => slice_rz90_gate,
            "RZ270" => slice_rz270_gate,
            _ => panic!("Invalid gate name"),
        }
    }

    pub(crate) fn slice_pauli_gate_fn(name: &str) -> SlicePauliGateFn {
        match name {
            "X" => slice_x_gate,
            "Y" => slice_y_gate,
            "Z" => slice_z_gate,
            _ => panic!("Invalid gate name"),
        }
    }

    pub(crate) fn slice_tq_gate_fn(name: &str) -> SliceTqeGateFn {
        match name {
            "XX" => slice_xx_gate,
            "XY" => slice_xy_gate,
            "XZ" => slice_xz_gate,
            "YY" => slice_yy_gate,
            "YZ" => slice_yz_gate,
            "ZZ" => slice_zz_gate,
            _ => panic!("Invalid gate name"),
        }
    }

    // ------------------------ Test conjugation for [u64] --------------------------
    #[rstest]
    #[case(300, 0)]
    #[case(5, 1)]
    fn test_slice_sq_gates(#[case] n_paulis: usize, #[case] seed: u64) {
        let paulis = random_paulis(n_paulis, seed);
        let (zs, xs) = paulis_to_u64s(&paulis);

        for gate in SQ_GATES.iter() {
            let gate_func = slice_sq_gate_fn(gate);
            let mut zs_clone = zs.clone();
            let mut xs_clone = xs.clone();
            let mut signs: Vec<u64> = vec![0; zs.len()];
            gate_func(&mut zs_clone, &mut xs_clone, &mut signs);
            let new_paulis = u64s_to_paulis(&zs_clone, &xs_clone, paulis.len());
            let new_signs = u64s_to_sign_flips(&signs, paulis.len());
            // compute the expected result using the boolean function
            let bool_gate_fn = sq_gate_fn(gate);
            let mut expected_paulis = Vec::with_capacity(paulis.len());
            let mut expected_signs: Vec<bool> = Vec::with_capacity(paulis.len());
            for p in paulis.iter() {
                let (z, x) = pauli_to_bits(p);
                let (new_z, new_x, sign_flip) = bool_gate_fn(z, x);
                let p_prime = bits_to_pauli(new_z, new_x);
                expected_paulis.push(p_prime);
                expected_signs.push(sign_flip);
            }
            assert_eq!(new_paulis, expected_paulis);
            assert_eq!(new_signs, expected_signs);
        }
    }

    #[rstest]
    #[case(300, 0)]
    #[case(5, 1)]
    fn test_slice_pauli_gates(#[case] n_paulis: usize, #[case] seed: u64) {
        let paulis = random_paulis(n_paulis, seed);
        let (zs, xs) = paulis_to_u64s(&paulis);

        for gate in PAULI_GATES.iter() {
            let gate_func = slice_pauli_gate_fn(gate);
            let mut signs: Vec<u64> = vec![0; zs.len()];
            gate_func(&zs, &xs, &mut signs);
            let new_signs = u64s_to_sign_flips(&signs, paulis.len());
            // compute the expected result using the boolean function
            let bool_gate_fn = pauli_gate_fn(gate);
            let mut expected_signs: Vec<bool> = Vec::with_capacity(paulis.len());
            for p in paulis.iter() {
                let (z, x) = pauli_to_bits(p);
                let sign_flip = bool_gate_fn(z, x);
                expected_signs.push(sign_flip);
            }
            assert_eq!(new_signs, expected_signs);
        }
    }

    #[rstest]
    #[case(300, 0, 1)]
    #[case(5, 1, 2)]
    fn test_slice_tq_gates(#[case] n_paulis: usize, #[case] seed0: u64, #[case] seed1: u64) {
        let paulis0 = random_paulis(n_paulis, seed0);
        let (zs0, xs0) = paulis_to_u64s(&paulis0);
        let paulis1 = random_paulis(n_paulis, seed1);
        let (zs1, xs1) = paulis_to_u64s(&paulis1);

        for gate in TQ_GATES.iter() {
            let gate_func = slice_tq_gate_fn(gate);
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
            let new_paulis0 = u64s_to_paulis(&zs0_clone, &xs0_clone, paulis0.len());
            let new_paulis1 = u64s_to_paulis(&zs1_clone, &xs1_clone, paulis1.len());
            let new_signs = u64s_to_sign_flips(&signs, paulis0.len());
            // compute the expected result using the boolean function
            let bool_gate_fn = tq_gate_fn(gate);
            let mut expected_signs: Vec<bool> = Vec::with_capacity(paulis0.len());
            let mut expected_paulis0: Vec<Pauli> = Vec::with_capacity(paulis0.len());
            let mut expected_paulis1: Vec<Pauli> = Vec::with_capacity(paulis1.len());
            for (p0, p1) in paulis0.iter().zip(paulis1.iter()) {
                let (z0, x0) = pauli_to_bits(p0);
                let (z1, x1) = pauli_to_bits(p1);
                let (new_z0, new_x0, new_z1, new_x1, sign_flip) = bool_gate_fn(z0, x0, z1, x1);
                expected_paulis0.push(bits_to_pauli(new_z0, new_x0));
                expected_paulis1.push(bits_to_pauli(new_z1, new_x1));
                expected_signs.push(sign_flip);
            }
            assert_eq!(new_paulis0, expected_paulis0);
            assert_eq!(new_paulis1, expected_paulis1);
            assert_eq!(new_signs, expected_signs);
        }
    }

    // ---------------------------- Test dispatching -------------------------------

    pub(crate) fn tq_wrapper_cases() -> [(Pauli, Pauli, u8, SliceTqeGateFn, bool); 9] {
        [
            (Pauli::X, Pauli::X, XX_U8, slice_xx_gate, false),
            (Pauli::X, Pauli::Y, XY_U8, slice_xy_gate, false),
            (Pauli::X, Pauli::Z, XZ_U8, slice_xz_gate, false),
            (Pauli::Y, Pauli::X, YX_U8, slice_xy_gate, true),
            (Pauli::Y, Pauli::Y, YY_U8, slice_yy_gate, false),
            (Pauli::Y, Pauli::Z, YZ_U8, slice_yz_gate, false),
            (Pauli::Z, Pauli::X, ZX_U8, slice_xz_gate, true),
            (Pauli::Z, Pauli::Y, ZY_U8, slice_yz_gate, true),
            (Pauli::Z, Pauli::Z, ZZ_U8, slice_zz_gate, false),
        ]
    }

    pub(crate) fn half_pi_wrapper_cases() -> [(Pauli, bool, SliceSqGateFn); 6] {
        [
            (Pauli::X, false, slice_rx90_gate),
            (Pauli::X, true, slice_rx270_gate),
            (Pauli::Y, false, slice_ry90_gate),
            (Pauli::Y, true, slice_ry270_gate),
            (Pauli::Z, false, slice_rz90_gate),
            (Pauli::Z, true, slice_rz270_gate),
        ]
    }

    pub(crate) fn apply_tq_reference(
        gate_func: SliceTqeGateFn,
        swapped: bool,
        zs0: &mut [u64],
        xs0: &mut [u64],
        zs1: &mut [u64],
        xs1: &mut [u64],
        signs: &mut [u64],
    ) {
        if swapped {
            gate_func(zs1, xs1, zs0, xs0, signs);
        } else {
            gate_func(zs0, xs0, zs1, xs1, signs);
        }
    }

    #[rstest]
    #[case(5, 0)]
    #[case(64, 1)]
    #[case(130, 2)]
    fn test_tqe_wrapper_dispatch_slice(#[case] len: usize, #[case] seed: u64) {
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
            apply_enum_tqe_slice(
                &mut enum_zs0,
                &mut enum_xs0,
                &mut enum_zs1,
                &mut enum_xs1,
                &mut enum_signs,
                g0,
                g1,
            );
            apply_u8_tqe_slice(
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
    fn test_half_pi_wrapper_dispatch_slice(#[case] len: usize, #[case] seed: u64) {
        for (axis, neg, reference_func) in half_pi_wrapper_cases() {
            let base = (
                random_u64s(len, seed),
                random_u64s(len, seed + 1),
                random_u64s(len, seed + 2),
            );
            let (mut expected_zs, mut expected_xs, mut expected_signs) = base.clone();
            let (mut wrapper_zs, mut wrapper_xs, mut wrapper_signs) = base;

            reference_func(&mut expected_zs, &mut expected_xs, &mut expected_signs);
            apply_half_pi_gate_slice(
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
