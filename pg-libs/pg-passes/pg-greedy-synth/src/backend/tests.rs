#[cfg(feature = "simd")]
use super::SimdBackend;
use super::{CostKernel, ScalarBackend};
use crate::tqe::TQEType;
use pg_bitpacked::{
    XX_U8, XY_U8, XZ_U8, YX_U8, YY_U8, YZ_U8, ZX_U8, ZY_U8, ZZ_U8, apply_u8_tqe_slice,
    paulis_to_u64s, u8_tqe_to_u64, u64s_to_paulis,
};
use pg_core::Pauli;
use rand::{Rng, SeedableRng, rngs::StdRng};
use rstest::rstest;

const N_COLUMNS: usize = 65 * 64;
const SEED: u64 = 0x5EED;

fn random_pauli(rng: &mut StdRng) -> Pauli {
    match rng.random_range(0..4) {
        0 => Pauli::I,
        1 => Pauli::X,
        2 => Pauli::Y,
        3 => Pauli::Z,
        _ => unreachable!(),
    }
}

fn anticommutes(left: Pauli, right: Pauli) -> bool {
    left != Pauli::I && right != Pauli::I && left != right
}

fn random_pauli_slice() -> Vec<Vec<Pauli>> {
    let mut rng = StdRng::seed_from_u64(SEED);
    (0..2)
        .map(|_| (0..N_COLUMNS).map(|_| random_pauli(&mut rng)).collect())
        .collect()
}

fn transformed_slice(slice: &[Vec<Pauli>], gate: TQEType) -> Vec<Vec<Pauli>> {
    let mut transformed = slice.to_vec();
    let (mut zb_q0, mut xb_q0) = paulis_to_u64s(&slice[0]);
    let (mut zb_q1, mut xb_q1) = paulis_to_u64s(&slice[1]);
    let mut sign_bits = vec![0; zb_q0.len()];
    apply_u8_tqe_slice(
        &mut zb_q0,
        &mut xb_q0,
        &mut zb_q1,
        &mut xb_q1,
        &mut sign_bits,
        gate,
    );
    transformed[0] = u64s_to_paulis(&zb_q0, &xb_q0, N_COLUMNS);
    transformed[1] = u64s_to_paulis(&zb_q1, &xb_q1, N_COLUMNS);
    transformed
}

fn set_bit(mask: &mut [u64], index: usize) {
    mask[index / 64] |= 1 << (index % 64);
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum PairType {
    Identity,
    Anticommute,
    Commute,
}

fn pair_type(left: Pauli, right: Pauli) -> PairType {
    if left == Pauli::I && right == Pauli::I {
        PairType::Identity
    } else if anticommutes(left, right) {
        PairType::Anticommute
    } else {
        PairType::Commute
    }
}

fn pair_cost_difference(original: &[Vec<Pauli>], transformed: &[Vec<Pauli>], index: usize) -> i64 {
    use PairType::{Anticommute as A, Commute as C, Identity as I};

    let old = (
        pair_type(original[0][index], original[0][index + 1]),
        pair_type(original[1][index], original[1][index + 1]),
    );
    let new = (
        pair_type(transformed[0][index], transformed[0][index + 1]),
        pair_type(transformed[1][index], transformed[1][index + 1]),
    );

    match (old, new) {
        ((C, I), (C, C))
        | ((I, C), (C, C))
        | ((A, I), (A, C))
        | ((I, A), (C, A))
        | ((C, C), (A, A)) => 1,
        ((C, C), (C, I))
        | ((C, C), (I, C))
        | ((A, C), (A, I))
        | ((C, A), (I, A))
        | ((A, A), (C, C)) => -1,
        _ => 0,
    }
}

fn assert_single_cost_masks(backend: &impl CostKernel, gate: TQEType) {
    let original = random_pauli_slice();
    let transformed = transformed_slice(&original, gate);
    let (zb_q0, xb_q0) = paulis_to_u64s(&original[0]);
    let (zb_q1, xb_q1) = paulis_to_u64s(&original[1]);
    let mut expected_increases = vec![0; zb_q0.len()];
    let mut expected_decreases = vec![0; zb_q0.len()];

    for (index, ((old_q0, old_q1), (new_q0, new_q1))) in original[0]
        .iter()
        .zip(&original[1])
        .zip(transformed[0].iter().zip(&transformed[1]))
        .enumerate()
    {
        let old_weight = i8::from(*old_q0 != Pauli::I) + i8::from(*old_q1 != Pauli::I);
        let new_weight = i8::from(*new_q0 != Pauli::I) + i8::from(*new_q1 != Pauli::I);
        match old_weight - new_weight {
            -1 => set_bit(&mut expected_increases, index),
            0 => {}
            1 => set_bit(&mut expected_decreases, index),
            _ => unreachable!("a TQE changes support by at most one"),
        }
    }

    let (gate_z_q0, gate_x_q0, gate_z_q1, gate_x_q1) = u8_tqe_to_u64(gate);
    let mask = vec![u64::MAX; zb_q0.len()];
    let mut increases = vec![0; zb_q0.len()];
    let mut decreases = vec![0; zb_q0.len()];
    backend.single_cost_masks(
        &zb_q0,
        &xb_q0,
        &zb_q1,
        &xb_q1,
        gate_z_q0,
        gate_x_q0,
        gate_z_q1,
        gate_x_q1,
        &mask,
        &mut increases,
        &mut decreases,
    );
    assert_eq!(increases, expected_increases);
    assert_eq!(decreases, expected_decreases);
}

fn assert_pair_cost_masks(backend: &impl CostKernel, gate: TQEType) {
    let original = random_pauli_slice();
    let transformed = transformed_slice(&original, gate);
    let (zb_q0, xb_q0) = paulis_to_u64s(&original[0]);
    let (zb_q1, xb_q1) = paulis_to_u64s(&original[1]);
    let mut expected_increases = vec![0; zb_q0.len()];
    let mut expected_decreases = vec![0; zb_q0.len()];

    for index in (0..N_COLUMNS).step_by(2) {
        match pair_cost_difference(&original, &transformed, index) {
            -1 => set_bit(&mut expected_decreases, index),
            0 => {}
            1 => set_bit(&mut expected_increases, index),
            _ => unreachable!(),
        }
    }

    let (gate_z_q0, gate_x_q0, gate_z_q1, gate_x_q1) = u8_tqe_to_u64(gate);
    let mask = vec![u64::MAX; zb_q0.len()];
    let mut increases = vec![0; zb_q0.len()];
    let mut decreases = vec![0; zb_q0.len()];
    backend.pair_cost_masks(
        &zb_q0,
        &xb_q0,
        &zb_q1,
        &xb_q1,
        gate_z_q0,
        gate_x_q0,
        gate_z_q1,
        gate_x_q1,
        &mask,
        &mut increases,
        &mut decreases,
    );
    assert_eq!(increases, expected_increases);
    assert_eq!(decreases, expected_decreases);
}

fn assert_unweighted_pair_cost(backend: &impl CostKernel, gate: TQEType) {
    let original = random_pauli_slice();
    let transformed = transformed_slice(&original, gate);
    let expected = (0..N_COLUMNS)
        .step_by(2)
        .map(|index| pair_cost_difference(&original, &transformed, index))
        .sum::<i64>();
    let (zb_q0, xb_q0) = paulis_to_u64s(&original[0]);
    let (zb_q1, xb_q1) = paulis_to_u64s(&original[1]);
    let (gate_z_q0, gate_x_q0, gate_z_q1, gate_x_q1) = u8_tqe_to_u64(gate);

    assert_eq!(
        backend.unweighted_pair_cost(
            &zb_q0, &xb_q0, &zb_q1, &xb_q1, gate_z_q0, gate_x_q0, gate_z_q1, gate_x_q1,
        ),
        expected
    );
}

#[rstest]
fn scalar_single_cost_masks(
    #[values(XX_U8, XY_U8, XZ_U8, YX_U8, YY_U8, YZ_U8, ZX_U8, ZY_U8, ZZ_U8)] gate: TQEType,
) {
    assert_single_cost_masks(&ScalarBackend, gate);
}

#[rstest]
fn scalar_pair_cost_masks(
    #[values(XX_U8, XY_U8, XZ_U8, YX_U8, YY_U8, YZ_U8, ZX_U8, ZY_U8, ZZ_U8)] gate: TQEType,
) {
    assert_pair_cost_masks(&ScalarBackend, gate);
}

#[rstest]
fn scalar_unweighted_pair_cost(
    #[values(XX_U8, XY_U8, XZ_U8, YX_U8, YY_U8, YZ_U8, ZX_U8, ZY_U8, ZZ_U8)] gate: TQEType,
) {
    assert_unweighted_pair_cost(&ScalarBackend, gate);
}

#[cfg(feature = "simd")]
#[rstest]
fn simd_single_cost_masks(
    #[values(XX_U8, XY_U8, XZ_U8, YX_U8, YY_U8, YZ_U8, ZX_U8, ZY_U8, ZZ_U8)] gate: TQEType,
) {
    assert_single_cost_masks(&SimdBackend, gate);
}

#[cfg(feature = "simd")]
#[rstest]
fn simd_pair_cost_masks(
    #[values(XX_U8, XY_U8, XZ_U8, YX_U8, YY_U8, YZ_U8, ZX_U8, ZY_U8, ZZ_U8)] gate: TQEType,
) {
    assert_pair_cost_masks(&SimdBackend, gate);
}

#[cfg(feature = "simd")]
#[rstest]
fn simd_unweighted_pair_cost(
    #[values(XX_U8, XY_U8, XZ_U8, YX_U8, YY_U8, YZ_U8, ZX_U8, ZY_U8, ZZ_U8)] gate: TQEType,
) {
    assert_unweighted_pair_cost(&SimdBackend, gate);
}
