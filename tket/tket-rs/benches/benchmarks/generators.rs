use hugr::Hugr;
use hugr::builder::{BuildError, CircuitBuilder, DFGBuilder, Dataflow, DataflowHugr};
use hugr::extension::prelude::qb_t;
use hugr::types::Signature;
use tket::TketOp;

/// Helper function for building circuits.
///
/// TODO: Extracted from tket::ops::test. Should we expose that instead?
pub fn build_simple_circuit(
    num_qubits: usize,
    f: impl FnOnce(&mut CircuitBuilder<DFGBuilder<Hugr>>) -> Result<(), BuildError>,
) -> Result<Hugr, BuildError> {
    let qb_row = vec![qb_t(); num_qubits];
    let mut h = DFGBuilder::new(Signature::new(qb_row.clone(), qb_row))?;

    let qbs = h.input_wires();

    let mut circ = h.as_circuit(qbs);

    f(&mut circ)?;

    let qbs = circ.finish();
    h.finish_hugr_with_outputs(qbs)
}

/// Create a circuit with layers of CNOTs.
///
/// - In each odd layer, we apply CNOTs between qubits `2i` and `2i+1` for each possible `i`.
/// - In each even layer, we apply CNOTs between qubits `2i+1` and `2i+2` for each possible `i`.
///
/// For example, for 4 qubits and 5 layers, we get the following circuit:
/// ```text
/// --*-----*-----*--
///   |     |     |
/// --x--*--x--*--x--
///      |     |
/// --*--x--*--x--*--
///   |     |     |
/// --x-----x-----x--
/// ```
pub fn make_cnot_layers(num_qubits: usize, layers: usize) -> Hugr {
    build_simple_circuit(num_qubits, |circ| {
        for layer in 0..layers {
            let start = layer % 2;
            let cnot_count = (num_qubits - start) / 2;
            for i in 0..cnot_count {
                let q = i * 2 + start;
                circ.append(TketOp::CX, [q, q + 1])?;
            }
        }
        Ok(())
    })
    .unwrap()
}
