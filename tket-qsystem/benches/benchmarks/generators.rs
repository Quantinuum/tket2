//! Helpers for building HUGRs that exercise the QSystem lowering pass.

use hugr::CircuitUnit;
use hugr::Hugr;
use hugr::Wire;
use hugr::builder::{Dataflow, DataflowHugr, FunctionBuilder};
use hugr::extension::prelude::qb_t;
use hugr::types::Signature;
use tket::TketOp;
use tket::extension::rotation::rotation_type;

/// Build a module-rooted HUGR containing many repeated `tket.quantum` ops.
///
/// Each layer applies, across all qubits:
/// - `H` on every qubit (multi-op / function-template lowering),
/// - `CX` between neighbouring qubit pairs (multi-op),
/// - `Rx` on every qubit, reusing a single copyable rotation input (multi-op),
/// - `Reset` on every qubit (direct-map lowering).
///
/// This mixes the function-template path (the one that previously re-registered
/// a replacement for every occurrence) with the direct-map path, so the total
/// op count grows linearly with `layers` while the set of distinct ops stays
/// tiny.
pub fn make_h_cx_rx_reset_layers(num_qubits: usize, layers: usize) -> Hugr {
    let qb_row = vec![qb_t(); num_qubits];
    let mut inputs = qb_row.clone();
    inputs.push(rotation_type());

    let mut h = FunctionBuilder::new("main", Signature::new(inputs, qb_row.clone())).unwrap();

    let mut wires: Vec<Wire> = h.input_wires().collect();
    let angle = wires.pop().expect("rotation input wire");

    let mut circ = h.as_circuit(wires);
    for _ in 0..layers {
        for q in 0..num_qubits {
            circ.append(TketOp::H, [q]).unwrap();
        }
        for i in 0..num_qubits / 2 {
            circ.append(TketOp::CX, [2 * i, 2 * i + 1]).unwrap();
        }
        for q in 0..num_qubits {
            circ.append_and_consume(
                TketOp::Rx,
                [CircuitUnit::Linear(q), CircuitUnit::Wire(angle)],
            )
            .unwrap();
        }
        for q in 0..num_qubits {
            circ.append(TketOp::Reset, [q]).unwrap();
        }
    }
    let qbs = circ.finish();
    h.finish_hugr_with_outputs(qbs).unwrap()
}
