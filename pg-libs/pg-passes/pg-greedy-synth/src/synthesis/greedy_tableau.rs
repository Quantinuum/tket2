use super::ParallelMode;
use super::common::{progress_bar, tqe_op, update_depth};
use crate::backend::TQECostBackend;
use crate::frontier::{Frontier, FrontierReductionGate, ReducedTableauPair};
use crate::packed_pg_slice::PackedBackend;
use crate::reducer::{GatePool, select_candidate};
use pg_bitpacked::{X_U8, Y_U8, Z_U8};
use pg_core::{GateData, GateType, Op, PauliGraph};
use pg_qm_tableau::Tableau;
use rand::{SeedableRng, rngs::StdRng};
use rayon::prelude::*;

pub(crate) fn synthesise_tableau<P, C>(
    mut tableau: Tableau,
    packed_backend: &P,
    cost_backend: &C,
    pool_size: usize,
    top_up_size: usize,
    qubit_depth: &mut [u64],
    seed: u64,
    parallel_mode: ParallelMode,
    enable_progress: bool,
) -> PauliGraph
where
    P: PackedBackend,
    C: TQECostBackend + Sync,
{
    let n_qubits = tableau.get_n_qubits();
    assert!(n_qubits > 0);
    assert_eq!(qubit_depth.len(), n_qubits);
    let mut output = PauliGraph::new(n_qubits);
    let mut frontier = Frontier::from_tableau(&tableau);
    let mut pool = GatePool::new(n_qubits);
    let mut rng = StdRng::seed_from_u64(seed);
    let mut reduced_pairs = Vec::with_capacity(n_qubits);
    let progress = progress_bar(enable_progress, n_qubits);

    loop {
        let completed = frontier.pop_tableau_pairs();
        let completed_any = !completed.is_empty();
        if completed_any {
            if let Some(progress) = &progress {
                progress.inc(completed.len() as u64);
            }
            reduced_pairs.extend(completed);
        }
        if frontier.is_empty() {
            break;
        }

        let sample_count = if completed_any {
            pool_size
        } else {
            // Add more candidates if this iteration did not finish reducing a tableau pair.
            top_up_size
        };
        sample_tableau_candidates(
            &tableau,
            cost_backend,
            &frontier,
            &pool,
            sample_count,
            &mut rng,
            parallel_mode,
        );
        let selected = select_candidate(&pool, &frontier, qubit_depth);
        let tqe = selected.gate.tqe;
        update_depth(qubit_depth, tqe);
        output.add_op(tqe_op(tqe));
        packed_backend.apply_tqe_to_tableau(&mut tableau, tqe);
        frontier.apply_tqe(tqe);
        pool.invalidate(tqe);
    }

    if let Some(progress) = progress {
        progress.finish();
    }
    append_tableau_corrections(&mut output, &reduced_pairs, n_qubits);
    output
}

fn sample_tableau_candidates<C: TQECostBackend + Sync>(
    tableau: &Tableau,
    cost_backend: &C,
    frontier: &Frontier,
    pool: &GatePool,
    count: usize,
    rng: &mut StdRng,
    parallel_mode: ParallelMode,
) {
    let gates = frontier.sample_gates(count, rng);
    let insert = |gate: &FrontierReductionGate| {
        let cost = cost_backend.tqe_tableau_cost(tableau, gate.tqe);
        pool.insert(*gate, cost, None);
    };
    if parallel_mode.should_use_parallel(tableau.get_n_qubits() * 2, gates.len()) {
        gates.par_iter().for_each(insert);
    } else {
        gates.iter().for_each(insert);
    }
}

fn append_tableau_corrections(
    output: &mut PauliGraph,
    reduced_pairs: &[ReducedTableauPair],
    n_qubits: usize,
) {
    // Each reduced pair is supported on one qubit. First rotate its
    // local anticommuting Pauli basis to the canonical Z/X basis, then correct
    // sign bits. The final permutation is restored with SWAP gates.
    assert_eq!(reduced_pairs.len(), n_qubits);
    let mut qubit_to_pair = vec![0; n_qubits];
    let mut pair_to_qubit = vec![0; n_qubits];
    for pair in reduced_pairs {
        let qubit = pair.qubit;
        qubit_to_pair[qubit] = pair.pair_index;
        pair_to_qubit[pair.pair_index] = qubit;
        let mut left_sign_bit = pair.left_sign_bit;
        let mut right_sign_bit = pair.right_sign_bit;
        match (pair.left, pair.right) {
            (X_U8, Y_U8) => {
                output.add_op(single_gate(GateType::H, qubit));
                output.add_op(single_gate(GateType::Sdg, qubit));
                right_sign_bit = !right_sign_bit;
            }
            (X_U8, Z_U8) => output.add_op(single_gate(GateType::H, qubit)),
            (Y_U8, X_U8) => {
                output.add_op(single_gate(GateType::Vdg, qubit));
                left_sign_bit = !left_sign_bit;
            }
            (Y_U8, Z_U8) => {
                output.add_op(single_gate(GateType::H, qubit));
                output.add_op(single_gate(GateType::Vdg, qubit));
            }
            (Z_U8, X_U8) => {}
            (Z_U8, Y_U8) => output.add_op(single_gate(GateType::Sdg, qubit)),
            _ => panic!("reduced tableau pair is not an anticommuting pair on one qubit"),
        }
        let correction = match (left_sign_bit, right_sign_bit) {
            (true, true) => Some(GateType::Y),
            (true, false) => Some(GateType::X),
            (false, true) => Some(GateType::Z),
            (false, false) => None,
        };
        if let Some(gate_type) = correction {
            output.add_op(single_gate(gate_type, qubit));
        }
    }

    for qubit in 0..n_qubits {
        let pair_index = qubit_to_pair[qubit];
        if qubit != pair_index {
            let other = pair_to_qubit[qubit];
            qubit_to_pair.swap(qubit, other);
            pair_to_qubit.swap(pair_index, qubit);
            output.add_op(Op::Gate {
                data: GateData::new(GateType::SWAP, vec![qubit, other]),
            });
        }
    }
}

fn single_gate(gate_type: GateType, qubit: usize) -> Op {
    Op::Gate {
        data: GateData::new(gate_type, vec![qubit]),
    }
}
