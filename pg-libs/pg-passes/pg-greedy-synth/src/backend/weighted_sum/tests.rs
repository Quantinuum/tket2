use super::*;
use crate::backend::ScalarBackend;
use pg_core::{Op, Pauli, RotationData};
use rstest::rstest;

fn rotation(pauli: Pauli) -> Op {
    Op::Rotation {
        data: RotationData::new(vec![pauli], 0.25),
    }
}

fn assert_weighted_delta(
    strategy: &mut impl WeightedSumStrategy,
    slice: &PackedPGSlice,
    increases: &[u64],
    decreases: &[u64],
    expected: f64,
) {
    strategy.prepare(slice);

    let tolerance = 1e-12 * (1.0 + expected.abs());
    let actual = strategy.weighted_delta(increases, decreases);
    assert!((actual - expected).abs() <= tolerance);
}

fn assert_weighted_sum_cases(mut strategy: impl WeightedSumStrategy) {
    let backend = ScalarBackend;
    let mut slice = PackedPGSlice::new(1);
    for _ in 0..65 {
        slice.push_op(&rotation(Pauli::X), &backend);
    }
    slice.start_new_set();
    slice.push_op(&rotation(Pauli::Z), &backend);

    // Bits 0 and 63 belong to the first set. Bit 65 belongs to the second set
    // and therefore has weight ALPHA. Bit 1 is subtracted once.
    assert_weighted_delta(
        &mut strategy,
        &slice,
        &[1 | (1 << 63), 1 << 1],
        &[1 << 1, 0],
        1.0 + super::super::ALPHA,
    );

    // A shorter mask stops within the first word.
    assert_weighted_delta(&mut strategy, &slice, &[0b101], &[0b010], 1.0);

    // After the first set is retired, its successor is rebased and becomes the
    // first visible set with weight one.
    assert!(slice.progress_set());
    assert_weighted_delta(&mut strategy, &slice, &[0b10], &[0], 1.0);
}

#[rstest]
#[case::expanded_sparse(ExpandedSparseWeightedSum::default())]
#[case::grouped(GroupedWeightedSum::default())]
fn weighted_sum_strategies_agree(#[case] strategy: impl WeightedSumStrategy) {
    assert_weighted_sum_cases(strategy);
}

#[cfg(feature = "simd")]
#[test]
fn simd_weighted_sum_strategy_agrees() {
    assert_weighted_sum_cases(ExpandedSimdWeightedSum::default());
}
