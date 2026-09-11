use super::*;
use crate::{backend::ScalarBackend, packed_pg_slice::PackedPGSlice};
use pg_core::{Op, Pauli, RotationData};

fn sample_frontier() -> Frontier {
    let mut slice = PackedPGSlice::new(2);
    let backend = ScalarBackend;
    slice.push_op(
        &Op::Rotation {
            data: RotationData::new(vec![Pauli::Z, Pauli::X], 0.2),
        },
        &backend,
    );
    let mut frontier = Frontier::new(2);
    frontier.rebuild(&slice);
    frontier
}

fn sample_frontier_3q() -> Frontier {
    let mut slice = PackedPGSlice::new(3);
    let backend = ScalarBackend;
    slice.push_op(
        &Op::Rotation {
            data: RotationData::new(vec![Pauli::Z, Pauli::X, Pauli::X], 0.2),
        },
        &backend,
    );
    let mut frontier = Frontier::new(3);
    frontier.rebuild(&slice);
    frontier
}

#[test]
fn test_insert() {
    let frontier = sample_frontier();
    let mut rng = rand::rng();
    let gates = frontier.sample_gates(50, &mut rng);
    let pool = GatePool::new(4);
    pool.insert(gates[0], -1.0, None);
    pool.insert(gates[0], -1.0, None);
    pool.insert(gates[1], -1.0, None);
    assert_eq!(pool.candidates.len(), 2);
}

#[test]
#[should_panic(expected = "no valid candidates found")]
fn test_invalidation() {
    let frontier = sample_frontier();
    let mut rng = rand::rng();
    let gates = frontier.sample_gates(50, &mut rng);
    let mut pool = GatePool::new(4);
    pool.insert(gates[0], -1.0, None);
    pool.insert(gates[1], -1.0, None);
    pool.invalidate(gates[0].tqe);
    select_candidate(&pool, &frontier, &[0, 0]);
}

#[test]
fn test_ranking() {
    let frontier = sample_frontier();
    let mut rng = rand::rng();
    let gates = frontier.sample_gates(50, &mut rng);
    let pool = GatePool::new(4);
    pool.insert(gates[0], -2.0, None);
    pool.insert(gates[1], -1.0, None);
    let gate = select_candidate(&pool, &frontier, &[0, 0]);
    assert_eq!(gate.gate, gates[0]);
}

#[test]
fn test_depth() {
    let frontier = sample_frontier_3q();
    let mut rng = rand::rng();
    let gates = frontier.sample_gates(50, &mut rng);
    let pool = GatePool::new(4);
    for (q0, q1) in [(0, 1), (0, 2), (1, 2)] {
        let gate = gates
            .iter()
            .find(|gate| gate.tqe.q0 == q0 && gate.tqe.q1 == q1)
            .unwrap();
        pool.insert(*gate, -1.0, None);
    }
    let candidate = select_candidate(&pool, &frontier, &[1, 0, 0]);
    // The selected candidate must act on the last two qubits.
    assert!(candidate.gate.tqe.q0 != 0 && candidate.gate.tqe.q1 != 0);
}
