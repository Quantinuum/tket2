use super::*;
use crate::backend::ScalarBackend;
use pg_core::{Op, Pauli, ResetData, RotationData};
use rand::{SeedableRng, rngs::StdRng};
use std::collections::HashSet;

fn mixed_frontier() -> Frontier {
    let backend = ScalarBackend;
    let mut slice = PackedPGSlice::new(4);
    slice.push_op(
        &Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::Z, Pauli::I, Pauli::I], 0.25),
        },
        &backend,
    );
    slice.push_op(
        &Op::Reset {
            data: ResetData::new(
                vec![Pauli::I, Pauli::I, Pauli::X, Pauli::X],
                vec![Pauli::I, Pauli::I, Pauli::Z, Pauli::I],
                false,
                false,
            ),
        },
        &backend,
    );

    let mut frontier = Frontier::new(4);
    frontier.rebuild(&slice);
    frontier
}

#[test]
fn test_sampling() {
    let frontier = mixed_frontier();
    let candidates = frontier.sample_gates(usize::MAX, &mut StdRng::seed_from_u64(7));

    // Four TQEs reduce the rotation and one reduces the reset pair.
    assert_eq!(candidates.len(), 5);
    assert_eq!(
        candidates.iter().copied().collect::<HashSet<_>>().len(),
        candidates.len()
    );

    for candidate in &candidates {
        let mut reduced = mixed_frontier();
        let before = reduced.ops[candidate.frontier_index.0].cost();
        reduced.apply_tqe(candidate.tqe);
        let after = reduced.ops[candidate.frontier_index.0].cost();
        assert_eq!(after + 1, before);
    }

    let selected = candidates[0];
    let other = candidates
        .iter()
        .find(|candidate| candidate.frontier_index != selected.frontier_index)
        .unwrap();
    let mut reduced = mixed_frontier();
    assert!(reduced.is_least_cost(selected.frontier_index));
    reduced.apply_tqe(selected.tqe);
    assert!(!reduced.is_least_cost(selected.frontier_index));
    assert!(reduced.is_least_cost(other.frontier_index));
}
