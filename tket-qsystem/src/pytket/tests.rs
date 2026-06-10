//! General tests.
use std::collections::{HashMap, HashSet};
use std::sync::Arc;

use hugr::builder::{Dataflow, DataflowHugr, FunctionBuilder};
use hugr::extension::prelude::{bool_t, qb_t};

use hugr::ops::OpParent;
use hugr::types::Signature;
use hugr::{Hugr, HugrView};
use itertools::Itertools;
use rstest::{fixture, rstest};
use tket::TketOp;
use tket::extension::TKET1_EXTENSION_ID;
use tket::extension::measurement::MeasurementOp;
use tket::serialize::pytket::EncodedCircuit;
use tket::serialize::pytket::TKETDecode;
use tket::serialize::pytket::{DecodeOptions, EncodeOptions};
use tket_json_rs::circuit_json::{self, SerialCircuit};
use tket_json_rs::register;

use crate::extension::futures::FutureOpBuilder;
use crate::extension::qsystem::REGISTRY;
use crate::extension::qsystem::{QSystemPlatform, helios::HeliosOp};
use crate::extension::result::ResultOp;
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
    for permutation in &circ.implicit_permutation {
        let key = &permutation.0.id;
        let value = &permutation.1.id;
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

    // Allow additional bit ids after a roundtrip, as the encoder may freely
    // allocate new IDs instead of reusing old ones.
    let bits_a: HashSet<_> = a.bits.iter().collect();
    let bits_b: HashSet<_> = b.bits.iter().collect();
    assert!(
        bits_b.is_superset(&bits_a),
        "Some bit IDs in original circuit are missing the roundtrip. Original: [{}], Roundtrip: [{}]",
        bits_a.iter().join(", "),
        bits_b.iter().join(", "),
    );

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

    // Treat the commands as a multiset; iteration order is irrelevant here.
    #[expect(
        clippy::iter_over_hash_type,
        reason = "commands are compared as a multiset"
    )]
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
fn circ_qsystem_native_gates() -> Hugr {
    let input_t = vec![qb_t()];
    let output_t = vec![bool_t(), bool_t()];
    let mut h =
        FunctionBuilder::new("qsystem_native_gates", Signature::new(input_t, output_t)).unwrap();

    let [qb0] = h.input_wires_arr();
    let [qb1] = h.add_dataflow_op(TketOp::QAlloc, []).unwrap().outputs_arr();

    let [bit_0] = h
        .add_dataflow_op(HeliosOp::LazyMeasure, [qb0])
        .unwrap()
        .outputs_arr();
    let [bit_1] = h
        .add_dataflow_op(HeliosOp::LazyMeasure, [qb1])
        .unwrap()
        .outputs_arr();
    let [bit_0] = h.add_read(bit_0, bool_t()).unwrap();
    let [bit_1] = h.add_read(bit_1, bool_t()).unwrap();

    h.finish_hugr_with_outputs([bit_0, bit_1]).unwrap()
}

/// A circuit where the output is only reachable via an order edge.
///
/// The pytket pass used to drop the order edge here.
/// <https://github.com/Quantinuum/tket2/issues/1410>
#[fixture]
fn circ_dropped_order_edge() -> Hugr {
    let input_t = vec![];
    let output_t = vec![];
    let mut h =
        FunctionBuilder::new("dropped_order_edge", Signature::new(input_t, output_t)).unwrap();

    let [q] = h.add_dataflow_op(TketOp::QAlloc, []).unwrap().outputs_arr();
    let [q] = h.add_dataflow_op(TketOp::H, [q]).unwrap().outputs_arr();
    let [q] = h.add_dataflow_op(TketOp::H, [q]).unwrap().outputs_arr();
    let [b] = h
        .add_dataflow_op(TketOp::MeasureFree, [q])
        .unwrap()
        .outputs_arr();
    let [b] = h
        .add_dataflow_op(MeasurementOp::Read, [b])
        .unwrap()
        .outputs_arr();
    let result = h
        .add_dataflow_op(ResultOp::new_bool("result"), [b])
        .unwrap();

    h.set_order(&result, &h.output());

    h.finish_hugr_with_outputs([]).unwrap()
}

/// Check that all circuit ops have been translated to a native gate.
///
/// Panics if there are tk1 ops in the circuit.
fn check_no_tk1_ops(hugr: &Hugr) {
    for node in hugr.entry_descendants() {
        let Some(op) = hugr.get_optype(node).as_extension_op() else {
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

/// A simple Sol circuit with PhasedXX and Rz.
const SOL_NATIVE_GATES_JSON: &str = r#"{
    "phase": "0",
    "bits": [],
    "qubits": [["q", [0]], ["q", [1]]],
    "commands": [
        {"args": [["q", [0]], ["q", [1]]], "op": {"params": ["0.5", "0.25"], "type": "PhasedXX"}},
        {"args": [["q", [0]]], "op": {"params": ["0.5"], "type": "Rz"}}
    ],
    "implicit_permutation": [[["q", [0]], ["q", [0]]], [["q", [1]], ["q", [1]]]]
}"#;

#[rstest]
#[case::helios_native_gates(QSystemPlatform::Helios, NATIVE_GATES_JSON, 3, 2, false)]
#[case::sol_native_gates(QSystemPlatform::Sol, SOL_NATIVE_GATES_JSON, 2, 2, false)]
fn json_roundtrip(
    #[case] platform: QSystemPlatform,
    #[case] circ_s: &str,
    #[case] num_commands: usize,
    #[case] num_qubits: usize,
    #[case] has_tk1_ops: bool,
) {
    let ser: circuit_json::SerialCircuit = serde_json::from_str(circ_s).unwrap();
    assert_eq!(ser.commands.len(), num_commands);

    let hugr: Hugr = ser
        .decode(DecodeOptions::new().with_config(qsystem_decoder_config(platform)))
        .unwrap();
    assert_eq!(tket::Circuit::new(&hugr).qubit_count(), num_qubits);

    if !has_tk1_ops {
        check_no_tk1_ops(&hugr);
    }

    let reser: SerialCircuit = SerialCircuit::encode(
        &hugr,
        EncodeOptions::new().with_config(qsystem_encoder_config(platform)),
    )
    .unwrap();
    validate_serial_circ(&reser);
    compare_serial_circs(&ser, &reser);
}

/// We currently cannot encode any measurement ops in the Helios/Sol extensions, as
/// they all return futures that cannot be translated.
#[rstest]
#[ignore]
#[case::native_gates(circ_qsystem_native_gates())]
fn circuit_standalone_roundtrip(#[case] hugr: Hugr) {
    let circ_signature = hugr
        .entrypoint_optype()
        .inner_function_type()
        .expect("Dataflow entrypoint")
        .into_owned();
    let decode_options = DecodeOptions::new()
        .with_signature(circ_signature.clone())
        .with_config(qsystem_decoder_config(QSystemPlatform::Helios))
        .with_extensions(REGISTRY.clone());
    let encode_options = EncodeOptions::new()
        .with_subcircuits(true)
        .with_config(qsystem_encoder_config(QSystemPlatform::Helios))
        .keep_empty_circuits(true);

    let encoded = EncodedCircuit::new_standalone(&hugr, encode_options.clone())
        .unwrap_or_else(|e| panic!("{e}"));

    assert!(encoded.contains_circuit(hugr.entrypoint()));
    assert_eq!(encoded.len(), 1);

    // Re-encode the EncodedCircuit
    let extracted_from_circ = encoded
        .reassemble(
            hugr.entrypoint(),
            Some("main".to_string()),
            decode_options.clone(),
        )
        .unwrap_or_else(|e| panic!("{e}"));
    extracted_from_circ
        .validate()
        .unwrap_or_else(|e| panic!("{e}"));

    // Extract the head pytket circuit, and re-encode it on its own.
    let ser: &SerialCircuit = &encoded[hugr.entrypoint()];
    let deser: Hugr = ser.decode(decode_options).unwrap_or_else(|e| panic!("{e}"));

    let deser_sig = deser
        .entrypoint_optype()
        .inner_function_type()
        .expect("Dataflow entrypoint")
        .into_owned();
    assert_eq!(
        &circ_signature.input, &deser_sig.input,
        "Input signature mismatch\n  Expected: {}\n  Actual:   {}",
        &circ_signature, &deser_sig
    );
    assert_eq!(
        &circ_signature.output, &deser_sig.output,
        "Output signature mismatch\n  Expected: {}\n  Actual:   {}",
        &circ_signature, &deser_sig
    );

    let reser = SerialCircuit::encode(
        &deser,
        EncodeOptions::new().with_config(qsystem_encoder_config(QSystemPlatform::Helios)),
    )
    .unwrap();
    validate_serial_circ(&reser);
    compare_serial_circs(ser, &reser);
}

#[rstest]
#[case::dropped_order_edge(circ_dropped_order_edge(), 1)]
fn encoded_circuit_roundtrip(#[case] hugr: Hugr, #[case] num_circuits: usize) {
    let circ_signature = hugr
        .entrypoint_optype()
        .inner_function_type()
        .expect("Dataflow entrypoint")
        .into_owned();
    let encode_options = EncodeOptions::new()
        .with_subcircuits(true)
        .with_config(qsystem_encoder_config(QSystemPlatform::Helios));

    let encoded = EncodedCircuit::new(&hugr, encode_options).unwrap_or_else(|e| panic!("{e}"));

    assert!(encoded.contains_circuit(hugr.entrypoint()));
    assert_eq!(encoded.len(), num_circuits);

    let mut deser = hugr.clone();
    encoded
        .reassemble_inplace(
            &mut deser,
            Some(Arc::new(qsystem_decoder_config(QSystemPlatform::Helios))),
        )
        .unwrap_or_else(|e| panic!("{e}"));

    deser.validate().unwrap_or_else(|e| panic!("{e}"));

    let deser_sig = deser
        .entrypoint_optype()
        .inner_function_type()
        .expect("Dataflow entrypoint")
        .into_owned();
    assert_eq!(
        &circ_signature.input, &deser_sig.input,
        "Input signature mismatch\n  Expected: {}\n  Actual:   {}",
        &circ_signature, &deser_sig
    );
    assert_eq!(
        &circ_signature.output, &deser_sig.output,
        "Output signature mismatch\n  Expected: {}\n  Actual:   {}",
        &circ_signature, &deser_sig
    );
}

#[rstest]
/// Regression test for <https://github.com/Quantinuum/tket2/issues/1410>
///
/// The pytket pass used to drop the order edge between the `Result` operation
/// and the output.
fn regression_dropped_order_edge(circ_dropped_order_edge: Hugr) {
    let hugr = circ_dropped_order_edge;

    let encode_options = EncodeOptions::new()
        .with_subcircuits(true)
        .with_config(qsystem_encoder_config(QSystemPlatform::Helios));
    let encoded = EncodedCircuit::new(&hugr, encode_options).unwrap_or_else(|e| panic!("{e}"));
    assert!(encoded.contains_circuit(hugr.entrypoint()));

    let mut deser = hugr.clone();
    encoded
        .reassemble_inplace(
            &mut deser,
            Some(Arc::new(qsystem_decoder_config(QSystemPlatform::Helios))),
        )
        .unwrap_or_else(|e| panic!("{e}"));

    deser.validate().unwrap_or_else(|e| panic!("{e}"));

    // Find the result and output nodes
    let func_node = deser.entrypoint();
    let result_node = deser
        .children(func_node)
        .find(|n| deser.get_optype(*n).cast::<ResultOp>().is_some())
        .unwrap();
    let output_node = deser.get_io(func_node).unwrap()[1];

    // Check that the order edge is still there
    let order_edge_out = deser.get_optype(result_node).other_output_port().unwrap();
    let order_edge_in = deser.get_optype(output_node).other_input_port().unwrap();
    assert_eq!(
        deser
            .linked_inputs(result_node, order_edge_out)
            .collect_vec(),
        vec![(output_node, order_edge_in)]
    );
}
