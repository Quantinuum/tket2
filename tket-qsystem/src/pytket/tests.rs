//! General tests.

use std::collections::{HashMap, HashSet};

use hugr::builder::{Dataflow, DataflowHugr, FunctionBuilder};
use hugr::extension::prelude::{bool_t, qb_t};

use hugr::HugrView;
use hugr::types::Signature;
use itertools::Itertools;
use rstest::{fixture, rstest};
use tket::TketOp;
use tket::extension::TKET1_EXTENSION_ID;
use tket_json_rs::circuit_json::{self, SerialCircuit};
use tket_json_rs::register;

use tket::circuit::Circuit;
use tket::serialize::pytket::TKETDecode;
use tket::serialize::pytket::{DecodeOptions, EncodeOptions};

use crate::extension::futures::FutureOpBuilder;
use crate::extension::qsystem::QSystemOp;
use crate::pytket::{qsystem_decoder_config, qsystem_encoder_config};

const NATIVE_GATES_JSON: &str = r#"{
        "phase": "0",
        "bits": [],
        "qubits": [["q", [0]], ["q", [1]]],
        "commands": [
            {"args": [["q", [0]], ["q", [1]]], "op": {"type": "ZZMax"}},
            {"args": [["q", [0]], ["q", [1]]], "op": {"params": ["((pi) / (2)) / (pi)"], "type": "ZZPhase"}},
            {"args":[["q",[0]]],"op":{"params":["(pi) / (3)", "beta"],"type":"PhasedX"}}
        ],
        "implicit_permutation": [[["q", [0]], ["q", [0]]], [["q", [1]], ["q", [1]]]]
    }"#;

/// Check some properties of the serial circuit.
fn validate_serial_circ(circ: &SerialCircuit) {
    // Check that all commands have valid arguments.
    for command in &circ.commands {
        for arg in &command.args {
            assert!(
                circ.qubits.contains(&register::Qubit::from(arg.clone()))
                    || circ.bits.contains(&register::Bit::from(arg.clone())),
                "Circuit command {command:?} has an invalid argument '{arg:?}'"
            );
        }
    }

    // Check that the implicit permutation is valid.
    let perm: HashMap<register::ElementId, register::ElementId> = circ
        .implicit_permutation
        .iter()
        .map(|p| (p.0.clone().id, p.1.clone().id))
        .collect();
    for (key, value) in &perm {
        let valid_qubits = circ.qubits.contains(&register::Qubit::from(key.clone()))
            && circ.qubits.contains(&register::Qubit::from(value.clone()));
        assert!(
            valid_qubits,
            "Circuit has an invalid permutation '{key:?} -> {value:?}'"
        );
    }
    assert_eq!(
        perm.len(),
        circ.implicit_permutation.len(),
        "Circuit has duplicate permutations",
    );
    assert_eq!(
        HashSet::<&register::ElementId>::from_iter(perm.values()).len(),
        perm.len(),
        "Circuit has duplicate values in permutations"
    );
}

fn compare_serial_circs(a: &SerialCircuit, b: &SerialCircuit) {
    assert_eq!(a.name, b.name);
    assert_eq!(a.phase, b.phase);
    assert_eq!(&a.qubits, &b.qubits);
    assert_eq!(a.commands.len(), b.commands.len());

    let bits_a: HashSet<_> = a.bits.iter().collect();
    let bits_b: HashSet<_> = b.bits.iter().collect();
    assert_eq!(bits_a, bits_b);

    // We ignore the commands order here, as two encodings may swap
    // non-dependant operations.
    //
    // The correct thing here would be to run a deterministic toposort and
    // compare the commands in that order. This is just a quick check that
    // everything is present, ignoring wire dependencies.
    //
    // Another problem is that `Command`s cannot be compared directly;
    // - `command.op.signature`, and `n_qb` are optional and sometimes
    //      unset in pytket-generated circs.
    // - qubit arguments names may differ if they have been allocated inside the circuit,
    //      as they depend on the traversal argument. Same with classical params.
    // Here we define an ad-hoc subset that can be compared.
    //
    // TODO: Do a proper comparison independent of the toposort ordering, and
    // track register reordering.
    #[derive(PartialEq, Eq, Hash, Debug)]
    struct CommandInfo {
        op_type: tket_json_rs::OpType,
        params: Vec<String>,
        n_args: usize,
    }

    impl From<&tket_json_rs::circuit_json::Command> for CommandInfo {
        fn from(command: &tket_json_rs::circuit_json::Command) -> Self {
            let mut info = CommandInfo {
                op_type: command.op.op_type,
                params: command.op.params.clone().unwrap_or_default(),
                n_args: command.args.len(),
            };

            // Special case for qsystem ops, where ZZMax does not exist.
            if command.op.op_type == tket_json_rs::OpType::ZZMax {
                info.op_type = tket_json_rs::OpType::ZZPhase;
                info.params = vec!["0.5".to_string()];
            }

            info
        }
    }

    let a_command_count: HashMap<CommandInfo, usize> = a.commands.iter().map_into().counts();
    let b_command_count: HashMap<CommandInfo, usize> = b.commands.iter().map_into().counts();

    for (a, &count_a) in &a_command_count {
        let count_b = b_command_count.get(a).copied().unwrap_or_default();
        assert_eq!(
            count_a, count_b,
            "command {a:?} appears {count_a} times in rhs and {count_b} times in lhs.\ncounts for a: {a_command_count:#?}\ncounts for b: {b_command_count:#?}"
        );
    }
    assert_eq!(a_command_count.len(), b_command_count.len());
}

/// A simple circuit with some qsystem operations.
#[fixture]
fn circ_qsystem_native_gates() -> Circuit {
    let input_t = vec![qb_t()];
    let output_t = vec![bool_t(), bool_t()];
    let mut h =
        FunctionBuilder::new("qsystem_native_gates", Signature::new(input_t, output_t)).unwrap();

    let [qb0] = h.input_wires_arr();
    let [qb1] = h.add_dataflow_op(TketOp::QAlloc, []).unwrap().outputs_arr();

    let [future_bit_0] = h
        .add_dataflow_op(QSystemOp::LazyMeasure, [qb0])
        .unwrap()
        .outputs_arr();
    let [future_bit_1] = h
        .add_dataflow_op(QSystemOp::LazyMeasure, [qb1])
        .unwrap()
        .outputs_arr();

    let [bit_0] = h.add_read(future_bit_0, bool_t()).unwrap();
    let [bit_1] = h.add_read(future_bit_1, bool_t()).unwrap();

    let hugr = h.finish_hugr_with_outputs([bit_0, bit_1]).unwrap();

    hugr.into()
}

/// Check that all circuit ops have been translated to a native gate.
///
/// Panics if there are tk1 ops in the circuit.
fn check_no_tk1_ops(circ: &Circuit) {
    for node in circ.hugr().entry_descendants() {
        let Some(op) = circ.hugr().get_optype(node).as_extension_op() else {
            continue;
        };
        if op.extension_id() == &TKET1_EXTENSION_ID {
            let payload = match op.args().first() {
                Some(t) => t.to_string(),
                None => "no payload".to_string(),
            };
            panic!(
                "{} found in circuit with payload '{payload}'",
                op.qualified_id()
            );
        }
    }
}

#[rstest]
#[case::native_gates(NATIVE_GATES_JSON, 3, 2, false)]
fn json_roundtrip(
    #[case] circ_s: &str,
    #[case] num_commands: usize,
    #[case] num_qubits: usize,
    #[case] has_tk1_ops: bool,
) {
    let ser: circuit_json::SerialCircuit = serde_json::from_str(circ_s).unwrap();
    assert_eq!(ser.commands.len(), num_commands);

    let circ: Circuit = ser
        .decode(DecodeOptions::new().with_config(qsystem_decoder_config()))
        .unwrap();
    assert_eq!(circ.qubit_count(), num_qubits);

    if !has_tk1_ops {
        check_no_tk1_ops(&circ);
    }

    let reser: SerialCircuit = SerialCircuit::encode(
        &circ,
        EncodeOptions::new().with_config(qsystem_encoder_config()),
    )
    .unwrap();
    validate_serial_circ(&reser);
    compare_serial_circs(&ser, &reser);
}

/// Test the serialisation roundtrip from a tket circuit.
///
/// Note: this is not a pure roundtrip as the encoder may add internal qubits/bits to the circuit.
#[rstest]
#[case::native_gates(circ_qsystem_native_gates(), Signature::new_endo(vec![qb_t(), qb_t(), bool_t(), bool_t()]))]
fn circuit_roundtrip(#[case] circ: Circuit, #[case] decoded_sig: Signature) {
    use tket::serialize::pytket::EncodeOptions;

    let ser: SerialCircuit = SerialCircuit::encode(
        &circ,
        EncodeOptions::new().with_config(qsystem_encoder_config()),
    )
    .unwrap();
    let deser: Circuit = ser
        .decode(DecodeOptions::new().with_config(qsystem_decoder_config()))
        .unwrap();

    let deser_sig = deser.circuit_signature();
    assert_eq!(
        &decoded_sig.input, &deser_sig.input,
        "Input signature mismatch\n  Expected: {}\n  Actual:   {}",
        &decoded_sig, &deser_sig
    );
    assert_eq!(
        &decoded_sig.output, &deser_sig.output,
        "Output signature mismatch\n  Expected: {}\n  Actual:   {}",
        &decoded_sig, &deser_sig
    );

    let reser = SerialCircuit::encode(
        &deser,
        EncodeOptions::new().with_config(qsystem_encoder_config()),
    )
    .unwrap();
    validate_serial_circ(&reser);
    compare_serial_circs(&ser, &reser);
}
