//! Test running tket1 passes on hugr circuit.

use tket1_passes::{Tket1Circuit, Tket1Pass};

use hugr::builder::{BuildError, Dataflow, DataflowHugr, FunctionBuilder};
use hugr::extension::prelude::qb_t;
use hugr::types::Signature;
use hugr::{HugrView, Node};
use rayon::iter::ParallelIterator;
use rstest::{fixture, rstest};
use tket::extension::{TKET_EXTENSION_ID, TKET1_EXTENSION_ID};
use tket::serialize::pytket::{EncodeOptions, EncodedCircuit};
use tket::{Circuit, TketOp};

/// JSON encoding of the clifford simp pytket pass.
const CLIFFORD_SIMP_STR: &str = r#"{"StandardPass": {"allow_swaps": true, "name": "CliffordSimp", "target_2qb_gate": "CX"}, "pass_class": "StandardPass"}"#;

/// A flat quantum circuit inside a function.
///
/// This should optimize to the identity.
#[fixture]
fn circ_flat_quantum() -> Circuit {
    fn build() -> Result<Circuit, BuildError> {
        let input_t = vec![qb_t(), qb_t()];
        let output_t = vec![qb_t(), qb_t()];
        let mut h =
            FunctionBuilder::new("preset_qubits", Signature::new(input_t, output_t)).unwrap();

        let mut circ = h.as_circuit(h.input_wires());

        circ.append(TketOp::X, [0])?;
        circ.append(TketOp::CX, [0, 1])?;
        circ.append(TketOp::X, [0])?;
        circ.append(TketOp::CX, [1, 0])?;
        circ.append(TketOp::X, [0])?;
        circ.append(TketOp::X, [1])?;
        circ.append(TketOp::CX, [0, 1])?;

        let wires = circ.finish();
        // Implicit swap
        let wires = [wires[1], wires[0]];

        let hugr = h.finish_hugr_with_outputs(wires).unwrap();

        Ok(hugr.into())
    }
    build().unwrap()
}

#[rstest]
#[case(circ_flat_quantum(), 0)]
fn test_clifford_simp(#[case] circ: Circuit, #[case] num_remaining_gates: usize) {
    let mut encoded =
        EncodedCircuit::new(&circ, EncodeOptions::new().with_subcircuits(true)).unwrap();

    encoded
        .par_iter_mut()
        .for_each(|(_region, serial_circuit)| {
            let mut circuit_ptr = Tket1Circuit::from_serial_circuit(serial_circuit).unwrap();
            Tket1Pass::run_from_json(CLIFFORD_SIMP_STR, &mut circuit_ptr).unwrap();
            *serial_circuit = circuit_ptr.to_serial_circuit().unwrap();
        });

    let mut new_circ = circ.clone();
    let updated_regions = encoded
        .reassemble_inplace(new_circ.hugr_mut(), None)
        .unwrap();

    let quantum_ops: usize = updated_regions
        .iter()
        .map(|region| count_quantum_gates(&new_circ, *region))
        .sum();
    assert_eq!(quantum_ops, num_remaining_gates);
}

/// Helper method to count the number of quantum operations in a hugr region.
fn count_quantum_gates(circuit: &Circuit, region: Node) -> usize {
    circuit
        .hugr()
        .children(region)
        .filter(|child| {
            let op = circuit.hugr().get_optype(*child);
            op.as_extension_op()
                .is_some_and(|e| [TKET_EXTENSION_ID, TKET1_EXTENSION_ID].contains(e.extension_id()))
        })
        .count()
}
