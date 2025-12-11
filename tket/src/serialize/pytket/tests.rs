//! General tests.

use std::collections::{HashMap, HashSet};
use std::io::BufReader;

use cool_asserts::assert_matches;
use hugr::builder::{
    Container, Dataflow, DataflowHugr, DataflowSubContainer, FunctionBuilder, HugrBuilder,
    ModuleBuilder, SubContainer,
};
use hugr::extension::prelude::{UnwrapBuilder, bool_t, option_type, qb_t};
use hugr::std_extensions::arithmetic::float_types::{ConstF64, float64_type};
use rayon::iter::ParallelIterator;
use std::sync::Arc;

use hugr::HugrView;
use hugr::hugr::hugrmut::HugrMut;
use hugr::ops::handle::FuncID;
use hugr::ops::{OpParent, OpType, Value};
use hugr::std_extensions::arithmetic::float_ops::FloatOps;
use hugr::types::{Signature, SumType};
use itertools::Itertools;
use rstest::{fixture, rstest};
use tket_json_rs::circuit_json::{self, SerialCircuit};
use tket_json_rs::optype;
use tket_json_rs::register;

use super::{METADATA_INPUT_PARAMETERS, METADATA_Q_REGISTERS, TKETDecode};
use crate::TketOp;
use crate::circuit::Circuit;
use crate::extension::TKET1_EXTENSION_ID;
use crate::extension::bool::{BoolOp, bool_type};
use crate::extension::rotation::{ConstRotation, RotationOp, rotation_type};
use crate::extension::sympy::SympyOpDef;
use crate::serialize::pytket::PytketEncodeError;
use crate::serialize::pytket::extension::{CoreDecoder, OpaqueTk1Op, PreludeEmitter};
use crate::serialize::pytket::{
    DecodeInsertionTarget, DecodeOptions, EncodeOptions, EncodedCircuit, PytketDecodeError,
    PytketDecodeErrorInner, PytketDecoderConfig, PytketEncodeOpError, PytketEncoderConfig,
    default_decoder_config, default_encoder_config,
};

const EMPTY_CIRCUIT: &str = r#"{
        "phase": "0",
        "bits": [["c", [0]]],
        "qubits": [["q", [0]], ["q", [1]]],
        "commands": [],
        "implicit_permutation": [[["q", [0]], ["q", [0]]], [["q", [1]], ["q", [1]]]]
    }"#;

const SIMPLE_JSON: &str = r#"{
        "phase": "0",
        "bits": [],
        "qubits": [["q", [0]], ["q", [1]]],
        "commands": [
            {"args": [["q", [0]]], "op": {"type": "H"}},
            {"args": [["q", [0]], ["q", [1]]], "op": {"type": "CX"}}
        ],
        "implicit_permutation": [[["q", [0]], ["q", [0]]], [["q", [1]], ["q", [1]]]]
    }"#;

const MULTI_REGISTER: &str = r#"{
        "phase": "0",
        "bits": [],
        "qubits": [["q", [2]], ["q", [1]], ["my_qubits", [2]]],
        "commands": [
            {"args": [["my_qubits", [2]]], "op": {"type": "H"}},
            {"args": [["q", [2]], ["q", [1]]], "op": {"type": "CX"}}
        ],
        "implicit_permutation": []
    }"#;

const UNKNOWN_OP: &str = r#"{
        "phase": "1/2",
        "bits": [["c", [0]], ["c", [1]]],
        "qubits": [["q", [0]], ["q", [1]], ["q", [2]]],
        "commands": [
            {"args": [["q", [0]], ["q", [1]], ["q", [2]]], "op": {"type": "CSWAP"}},
            {"args": [["q", [1]], ["c", [1]]], "op": {"type": "Measure"}}
        ],
        "created_qubits": [],
        "discarded_qubits": [],
        "implicit_permutation": [[["q", [0]], ["q", [0]]], [["q", [1]], ["q", [1]]], [["q", [2]], ["q", [2]]]]
    }"#;

const SMALL_PARAMETERIZED: &str = r#"{
        "phase": "0.0",
        "bits": [],
        "qubits": [["q", [0]]],
        "commands": [
            {"args":[["q",[0]]],"op":{"params":["(pi) / (2)"],"type":"Rz"}}
        ],
        "created_qubits": [],
        "discarded_qubits": [],
        "implicit_permutation": [[["q", [0]], ["q", [0]]]]
    }"#;

const PARAMETERIZED: &str = r#"{
        "phase": "0.0",
        "bits": [],
        "qubits": [["q", [0]], ["q", [1]]],
        "commands": [
            {"args":[["q",[0]]],"op":{"type":"H"}},
            {"args":[["q",[1]],["q",[0]]],"op":{"type":"CX"}},
            {"args":[["q",[0]]],"op":{"params":["((pi) / (2)) / (pi)"],"type":"Rz"}},
            {"args": [["q", [0]]], "op": {"params": ["(3.141596) / (pi)", "alpha", "((pi) / (4)) / (pi)"], "type": "TK1"}}
        ],
        "created_qubits": [],
        "discarded_qubits": [],
        "implicit_permutation": [[["q", [0]], ["q", [0]]], [["q", [1]], ["q", [1]]]]
    }"#;

const BARRIER: &str = r#"{
        "phase": "0.0",
        "bits": [["c", [0]], ["c", [1]]],
        "qubits": [["q", [0]], ["q", [1]], ["q", [2]]],
        "commands": [
            {"args": [["q", [0]], ["q", [1]]], "op": {"type": "CX"}},
            {"args": [["q", [1]], ["q", [2]]], "op": {"type": "Barrier", "data": "Something invalid"}},
            {"args": [["q", [2]], ["c", [1]]], "op": {"type": "Measure"}}
        ],
        "created_qubits": [],
        "discarded_qubits": [],
        "implicit_permutation": [[["q", [0]], ["q", [0]]], [["q", [1]], ["q", [1]]], [["q", [2]], ["q", [2]]]]
    }"#;

const IMPLICIT_PERMUTATION: &str = r#"{
        "phase": "0.0",
        "bits": [["c", [0]], ["c", [1]]],
        "qubits": [["q", [0]], ["q", [1]], ["q", [2]]],
        "commands": [
            {"args": [["q", [0]], ["q", [1]]], "op": {"type": "CX"}}
        ],
        "created_qubits": [],
        "discarded_qubits": [],
        "implicit_permutation": [[["q", [0]], ["q", [1]]], [["q", [1]], ["q", [2]]], [["q", [2]], ["q", [0]]]]
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
        let valid_bits = circ.bits.contains(&register::Bit::from(key.clone()))
            && circ.bits.contains(&register::Bit::from(value.clone()));
        assert!(
            valid_qubits || valid_bits,
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
            CommandInfo {
                op_type: command.op.op_type,
                params: command.op.params.clone().unwrap_or_default(),
                n_args: command.args.len(),
            }
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

/// A simple circuit with some preset qubit registers
#[fixture]
fn circ_preset_qubits() -> Circuit {
    let input_t = vec![qb_t()];
    let output_t = vec![qb_t(), qb_t()];
    let mut h = FunctionBuilder::new("preset_qubits", Signature::new(input_t, output_t)).unwrap();

    let [qb0] = h.input_wires_arr();
    let [qb1] = h.add_dataflow_op(TketOp::QAlloc, []).unwrap().outputs_arr();

    let [qb0, qb1] = h
        .add_dataflow_op(TketOp::CZ, [qb0, qb1])
        .unwrap()
        .outputs_arr();

    let mut hugr = h.finish_hugr_with_outputs([qb0, qb1]).unwrap();

    // A preset register for the first qubit output
    hugr.set_metadata(
        hugr.entrypoint(),
        METADATA_Q_REGISTERS,
        serde_json::json!([["q", [2]], ["q", [10]], ["q", [8]]]),
    );

    hugr.into()
}

/// A simple circuit with some input parameters
#[fixture]
fn circ_parameterized() -> Circuit {
    let input_t = vec![qb_t(), rotation_type(), rotation_type(), rotation_type()];
    let output_t = vec![qb_t()];
    let mut h = FunctionBuilder::new("parameterized", Signature::new(input_t, output_t)).unwrap();

    let [q, r0, r1, r2] = h.input_wires_arr();

    let [q] = h
        .add_dataflow_op(TketOp::Rx, [q, r0])
        .unwrap()
        .outputs_arr();
    let [q] = h
        .add_dataflow_op(TketOp::Ry, [q, r1])
        .unwrap()
        .outputs_arr();
    let [q] = h
        .add_dataflow_op(TketOp::Rz, [q, r2])
        .unwrap()
        .outputs_arr();

    let mut hugr = h.finish_hugr_with_outputs([q]).unwrap();

    // Preset names for some of the inputs
    hugr.set_metadata(
        hugr.entrypoint(),
        METADATA_INPUT_PARAMETERS,
        serde_json::json!(["alpha", "beta"]),
    );

    hugr.into()
}

/// A circuit with a TK1 opaque operation.
#[fixture]
fn circ_tk1_ops() -> Circuit {
    let input_t = vec![qb_t(), qb_t()];
    let output_t = vec![qb_t(), qb_t()];
    let mut h = FunctionBuilder::new("tk1_ops", Signature::new(input_t, output_t)).unwrap();

    let [q1, q2] = h.input_wires_arr();

    // An unsupported tk1-only operation.
    let mut tk1op = tket_json_rs::circuit_json::Operation::default();
    tk1op.op_type = tket_json_rs::optype::OpType::CH;
    tk1op.n_qb = Some(2);
    let op: OpType = OpaqueTk1Op::new_from_op(&tk1op, 2, 0)
        .as_extension_op()
        .into();
    let [q1, q2] = h.add_dataflow_op(op, [q1, q2]).unwrap().outputs_arr();

    let hugr = h.finish_hugr_with_outputs([q1, q2]).unwrap();
    hugr.into()
}

/// A circuit with a non-flat unsupported subgraph.
///
/// Tries to allocate a qubit, and panics if it fails.
/// This creates an unsupported conditional inside the region.
#[fixture]
fn circ_unsupported_subtree() -> Circuit {
    let input_t = vec![];
    let output_t = vec![qb_t()];
    let mut h =
        FunctionBuilder::new("unsupported_subtree", Signature::new(input_t, output_t)).unwrap();

    let [maybe_q] = h
        .add_dataflow_op(TketOp::TryQAlloc, [])
        .unwrap()
        .outputs_arr();
    let [q] = h.build_unwrap_sum(1, option_type(qb_t()), maybe_q).unwrap();

    let hugr = h.finish_hugr_with_outputs([q]).unwrap();
    hugr.into()
}

/// A circuit with a recursive function call.
#[fixture]
fn circ_recursive() -> Circuit {
    let input_t = vec![qb_t()];
    let output_t = vec![qb_t()];
    let mut h = FunctionBuilder::new("recursive", Signature::new(input_t, output_t)).unwrap();
    let func: FuncID<true> = h.container_node().into();

    let [q] = h.input_wires_arr();
    let [q] = h.call(&func, &[], [q]).unwrap().outputs_arr();
    let hugr = h.finish_hugr_with_outputs([q]).unwrap();

    hugr.into()
}

/// A circuit with global constant definitions.
#[fixture]
fn circ_global_defs() -> Circuit {
    let input_t = vec![qb_t()];
    let output_t = vec![qb_t()];
    let mut h = FunctionBuilder::new(
        "global_param",
        Signature::new(input_t.clone(), output_t.clone()),
    )
    .unwrap();

    let (rot_const, func_decl) = {
        let mut module = h.module_root_builder();
        let rot_const = module.add_constant(Value::from(ConstRotation::new(0.2).unwrap()));
        let func_decl = module
            .declare("func", Signature::new(input_t, output_t).into())
            .unwrap();
        (rot_const, func_decl)
    };

    let [q] = h.input_wires_arr();
    let rot = h.load_const(&rot_const);
    let [q] = h
        .add_dataflow_op(TketOp::Rx, [q, rot])
        .unwrap()
        .outputs_arr();
    let [q] = h.call(&func_decl, &[], [q]).unwrap().outputs_arr();
    let hugr = h.finish_hugr_with_outputs([q]).unwrap();

    hugr.into()
}

/// A circuit with non-local dataflow edges.
#[fixture]
fn circ_non_local() -> Circuit {
    let input_t = vec![qb_t(), rotation_type()];
    let inner_input_t = vec![qb_t()];
    let output_t = vec![qb_t()];
    let mut h =
        FunctionBuilder::new("non_local", Signature::new(input_t, output_t.clone())).unwrap();

    let [q, rot] = h.input_wires_arr();
    let [q] = {
        let mut dfg = h
            .dfg_builder(Signature::new(inner_input_t, output_t), [q])
            .unwrap();
        // Rx with non-local input
        let [q] = dfg.input_wires_arr();
        let [q] = dfg
            .add_dataflow_op(TketOp::Rx, [q, rot])
            .unwrap()
            .outputs_arr();
        dfg.set_outputs([q]).unwrap();
        dfg.finish_sub_container().unwrap()
    }
    .outputs_arr();
    let hugr = h.finish_hugr_with_outputs([q]).unwrap();

    hugr.into()
}

/// A simple circuit with ancillae
#[fixture]
fn circ_measure_ancilla() -> Circuit {
    let input_t = vec![qb_t()];
    let output_t = vec![bool_t(), bool_t()];
    let mut h = FunctionBuilder::new("meas_ancilla", Signature::new(input_t, output_t)).unwrap();

    let [qb] = h.input_wires_arr();
    let [anc] = h.add_dataflow_op(TketOp::QAlloc, []).unwrap().outputs_arr();

    let [qb, meas_qb] = h
        .add_dataflow_op(TketOp::Measure, [qb])
        .unwrap()
        .outputs_arr();
    let [anc, meas_anc] = h
        .add_dataflow_op(TketOp::Measure, [anc])
        .unwrap()
        .outputs_arr();

    let [] = h
        .add_dataflow_op(TketOp::QFree, [qb])
        .unwrap()
        .outputs_arr();
    let [] = h
        .add_dataflow_op(TketOp::QFree, [anc])
        .unwrap()
        .outputs_arr();

    h.finish_hugr_with_outputs([meas_qb, meas_anc])
        .unwrap()
        .into()
}

#[fixture]
fn circ_add_angles_symbolic() -> (Circuit, String) {
    let input_t = vec![qb_t(), rotation_type(), rotation_type()];
    let output_t = vec![qb_t()];
    let mut h =
        FunctionBuilder::new("add_angles_symbolic", Signature::new(input_t, output_t)).unwrap();

    let [qb, f1, f2] = h.input_wires_arr();
    let [f12] = h
        .add_dataflow_op(RotationOp::radd, [f1, f2])
        .unwrap()
        .outputs_arr();
    let [qb] = h
        .add_dataflow_op(TketOp::Rx, [qb, f12])
        .unwrap()
        .outputs_arr();

    let circ = h.finish_hugr_with_outputs([qb]).unwrap().into();
    (circ, "(f0) + (f1)".to_string())
}

#[fixture]
fn circ_add_angles_constants() -> (Circuit, String) {
    let qb_row = vec![qb_t()];
    let mut h = FunctionBuilder::new(
        "add_angles_constants",
        Signature::new(qb_row.clone(), qb_row),
    )
    .unwrap();

    let qb = h.input_wires().next().unwrap();

    let point2 = h.add_load_value(ConstRotation::new(0.2).unwrap());
    let point3 = h.add_load_value(ConstRotation::new(0.3).unwrap());
    let point5 = h
        .add_dataflow_op(RotationOp::radd, [point2, point3])
        .unwrap()
        .out_wire(0);

    let qbs = h
        .add_dataflow_op(TketOp::Rx, [qb, point5])
        .unwrap()
        .outputs();
    let circ = h.finish_hugr_with_outputs(qbs).unwrap().into();
    (circ, "(0.2) + (0.3)".to_string())
}

#[fixture]
/// An Rx operation using some complex ops to compute its angle.
fn circ_complex_angle_computation() -> (Circuit, String) {
    let input_t = vec![qb_t(), rotation_type(), rotation_type()];
    let output_t = vec![qb_t()];
    let mut h = FunctionBuilder::new(
        "complex_angle_computation",
        Signature::new(input_t, output_t),
    )
    .unwrap();

    let [qb, r0, r1] = h.input_wires_arr();

    // Loading rotations and sympy expressions
    let point2 = h.add_load_value(ConstRotation::new(0.2).unwrap());
    let sympy = h
        .add_dataflow_op(SympyOpDef.with_expr("cos(pi)".to_string()), [])
        .unwrap()
        .out_wire(0);
    let added_rot = h
        .add_dataflow_op(RotationOp::radd, [sympy, point2])
        .unwrap()
        .out_wire(0);

    // Float operations and conversions
    let f0 = h
        .add_dataflow_op(RotationOp::to_halfturns, [r0])
        .unwrap()
        .out_wire(0);
    let f1 = h
        .add_dataflow_op(RotationOp::to_halfturns, [r1])
        .unwrap()
        .out_wire(0);
    let fpow = h
        .add_dataflow_op(FloatOps::fpow, [f0, f1])
        .unwrap()
        .out_wire(0);
    let rpow = h
        .add_dataflow_op(RotationOp::from_halfturns_unchecked, [fpow])
        .unwrap()
        .out_wire(0);

    let final_rot = h
        .add_dataflow_op(RotationOp::radd, [rpow, added_rot])
        .unwrap()
        .out_wire(0);

    let qbs = h
        .add_dataflow_op(TketOp::Rx, [qb, final_rot])
        .unwrap()
        .outputs();

    let circ = h.finish_hugr_with_outputs(qbs).unwrap().into();
    (circ, "((f0) ** (f1)) + ((cos(pi)) + (0.2))".to_string())
}

/// A circuit with a nested DFG block.
#[fixture]
fn circ_nested_dfgs() -> Circuit {
    let input_t = vec![qb_t()];
    let output_t = vec![bool_t()];
    let mut h =
        FunctionBuilder::new("nested_dfgs", Signature::new(input_t, output_t.clone())).unwrap();

    let [qb] = h.input_wires_arr();
    let rot = h.add_load_value(ConstRotation::new(0.5).unwrap());

    let inner_dfg = {
        let mut inner_dfg = h
            .dfg_builder(
                Signature::new(vec![qb_t(), rotation_type()], output_t),
                [qb, rot],
            )
            .unwrap();
        let [qb, rot] = inner_dfg.input_wires_arr();

        let [qb] = inner_dfg
            .add_dataflow_op(TketOp::Rx, [qb, rot])
            .unwrap()
            .outputs_arr();
        let [bool] = inner_dfg
            .add_dataflow_op(TketOp::MeasureFree, [qb])
            .unwrap()
            .outputs_arr();
        let [bool] = inner_dfg
            .add_dataflow_op(BoolOp::read, [bool])
            .unwrap()
            .outputs_arr();

        inner_dfg.finish_with_outputs([bool]).unwrap()
    };
    let [bool] = inner_dfg.outputs_arr();

    h.finish_hugr_with_outputs([bool]).unwrap().into()
}

// A circuit with some simple circuit and an unsupported subgraph that does not interact with it.
#[fixture]
fn circ_independent_subgraph() -> Circuit {
    let input_t = vec![qb_t(), qb_t(), option_type(bool_t()).into()];
    let output_t = vec![qb_t(), qb_t(), bool_t()];
    let mut h =
        FunctionBuilder::new("independent_subgraph", Signature::new(input_t, output_t)).unwrap();

    let [q1, q2, maybe_b] = h.input_wires_arr();

    let [q1, q2] = h
        .add_dataflow_op(TketOp::CX, [q1, q2])
        .unwrap()
        .outputs_arr();
    let [maybe_b] = h
        .build_unwrap_sum(1, option_type(bool_t()), maybe_b)
        .unwrap();

    let hugr = h.finish_hugr_with_outputs([q1, q2, maybe_b]).unwrap();
    hugr.into()
}

// A circuit with an unsupported wire from the input to the output.
#[fixture]
fn circ_unsupported_io_wire() -> Circuit {
    let input_t = vec![qb_t(), qb_t(), option_type(qb_t()).into()];
    let output_t = vec![qb_t(), qb_t(), option_type(qb_t()).into()];
    let mut h = FunctionBuilder::new(
        "unsupported_input_to_output",
        Signature::new(input_t, output_t),
    )
    .unwrap();

    let [q1, q2, maybe_q] = h.input_wires_arr();

    let [q1, q2] = h
        .add_dataflow_op(TketOp::CX, [q1, q2])
        .unwrap()
        .outputs_arr();

    let hugr = h.finish_hugr_with_outputs([q1, q2, maybe_q]).unwrap();
    hugr.into()
}

// Nodes with order edges should be marked as unsupported to preserve the connection.
#[fixture]
fn circ_order_edge() -> Circuit {
    let input_t = vec![qb_t(), qb_t()];
    let output_t = vec![qb_t(), qb_t()];
    let mut h = FunctionBuilder::new("order_edge", Signature::new(input_t, output_t)).unwrap();

    let [q1, q2] = h.input_wires_arr();

    let cx1 = h.add_dataflow_op(TketOp::CX, [q1, q2]).unwrap();
    let [q1, q2] = cx1.outputs_arr();

    let cx2 = h.add_dataflow_op(TketOp::CX, [q1, q2]).unwrap();
    let [q1, q2] = cx2.outputs_arr();

    let cx3 = h.add_dataflow_op(TketOp::CX, [q1, q2]).unwrap();
    let [q1, q2] = cx3.outputs_arr();

    h.set_order(&cx1, &cx3);

    let hugr = h.finish_hugr_with_outputs([q1, q2]).unwrap();
    hugr.into()
}

// Bool types get converted automatically between native and tket representations.
#[fixture]
fn circ_bool_conversion() -> Circuit {
    let input_t = vec![bool_t(), bool_type()];
    let output_t = vec![bool_t(), bool_type()];
    let mut h = FunctionBuilder::new("bool_conversion", Signature::new(input_t, output_t)).unwrap();

    let [native_b0, tket_b1] = h.input_wires_arr();

    let [tket_b0] = h
        .add_dataflow_op(BoolOp::make_opaque, [native_b0])
        .unwrap()
        .outputs_arr();
    let [native_b1] = h
        .add_dataflow_op(BoolOp::read, [tket_b1])
        .unwrap()
        .outputs_arr();

    let hugr = h.finish_hugr_with_outputs([native_b1, tket_b0]).unwrap();
    hugr.into()
}

/// A circuit that requires tracking info in `extra_subgraph` or `straight_through_wires`
/// (see `EncodedCircuitInfo`), for a nested circuit in a CircBox.
#[fixture]
fn circ_unsupported_extras_in_circ_box() -> Circuit {
    let input_t = vec![option_type(bool_t()).into(), option_type(qb_t()).into()];
    let output_t = vec![bool_t(), option_type(qb_t()).into()];
    let mut h = FunctionBuilder::new(
        "unsupported_extras_in_circ_box",
        Signature::new(input_t.clone(), output_t.clone()),
    )
    .unwrap();

    let [maybe_b, maybe_q] = h.input_wires_arr();

    let [maybe_b, maybe_q] = {
        let mut nested = h
            .dfg_builder(Signature::new(input_t, output_t), [maybe_b, maybe_q])
            .unwrap();
        let [maybe_b, maybe_q] = nested.input_wires_arr();

        let [maybe_b] = nested
            .build_unwrap_sum(1, option_type(bool_t()), maybe_b)
            .unwrap();

        nested
            .finish_with_outputs([maybe_b, maybe_q])
            .unwrap()
            .outputs_arr()
    };

    let hugr = h.finish_hugr_with_outputs([maybe_b, maybe_q]).unwrap();
    hugr.into()
}

// A circuit with an output parameter wire.
#[fixture]
fn circ_output_parameter_wire() -> Circuit {
    let input_t = vec![];
    let output_t = vec![float64_type(), rotation_type()];
    let mut h =
        FunctionBuilder::new("output_parameter_wire", Signature::new(input_t, output_t)).unwrap();

    let pi = h.add_load_value(ConstF64::new(std::f64::consts::PI));
    let two = h.add_load_value(ConstF64::new(2.0));
    let two_pi = h
        .add_dataflow_op(FloatOps::fmul, [pi, two])
        .unwrap()
        .out_wire(0);
    let two_pi_rotation = h
        .add_dataflow_op(RotationOp::from_halfturns_unchecked, [two_pi])
        .unwrap()
        .out_wire(0);

    let hugr = h
        .finish_hugr_with_outputs([two_pi, two_pi_rotation])
        .unwrap();
    hugr.into()
}

// A circuit with a [float64] wire, which should be treated as unsupported.
#[fixture]
fn circ_complex_param_type() -> Circuit {
    let input_t = vec![];
    let output_t = vec![SumType::new_tuple(vec![float64_type()]).into()];
    let mut h =
        FunctionBuilder::new("complex_param_type", Signature::new(input_t, output_t)).unwrap();

    let float64 = h.add_load_value(ConstF64::new(1.0));
    let float_tuple = h.make_tuple([float64]).unwrap();

    let hugr = h.finish_hugr_with_outputs([float_tuple]).unwrap();
    hugr.into()
}

/// A program with two unsupported subgraphs not associated to any qubit or bit.
/// <https://github.com/CQCL/tket2/issues/1294>
#[fixture]
fn circ_unsupported_subgraph_no_registers() -> Circuit {
    let input_t = vec![qb_t()];
    let output_t = vec![qb_t(), rotation_type()];
    let mut h = FunctionBuilder::new(
        "unsupported_subgraph_no_registers",
        Signature::new(input_t, output_t),
    )
    .unwrap();
    let [q] = h.input_wires_arr();

    // Declare two functions to call.
    let func1 = {
        let call_input_t = vec![];
        let call_output_t = vec![float64_type()];
        h.module_root_builder()
            .declare("func1", Signature::new(call_input_t, call_output_t).into())
            .unwrap()
    };

    let func2 = {
        h.module_root_builder()
            .declare("func2", Signature::new_endo(vec![rotation_type()]).into())
            .unwrap()
    };

    // An unsupported call that'll require an opaque subgraph to encode.
    let call = h.call(&func1, &[], []).unwrap();
    let [f] = call.outputs_arr();

    // An operation that must be marked as unsupported, since it's input cannot be encoded.
    let [rot] = h
        .add_dataflow_op(RotationOp::from_halfturns_unchecked, [f])
        .unwrap()
        .outputs_arr();
    let [q] = h
        .add_dataflow_op(TketOp::Rz, [q, rot])
        .unwrap()
        .outputs_arr();

    // A separate call that will generate a second opaque subgraph.
    let [rot2] = h.call(&func2, &[], [rot]).unwrap().outputs_arr();

    let hugr = h.finish_hugr_with_outputs([q, rot2]).unwrap();
    hugr.into()
}

// A circuit that discards the first qubit input and only outputs the second one.
#[fixture]
fn circ_discard_first_qubit() -> Circuit {
    let input_t = vec![qb_t(), qb_t()];
    let output_t = vec![qb_t()];
    let mut h =
        FunctionBuilder::new("discard_first_qubit", Signature::new(input_t, output_t)).unwrap();

    let [q1, q2] = h.input_wires_arr();

    h.add_dataflow_op(TketOp::MeasureFree, [q1]).unwrap();

    let [q2] = h.add_dataflow_op(TketOp::X, [q2]).unwrap().outputs_arr();

    let hugr = h.finish_hugr_with_outputs([q2]).unwrap();
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
#[case::simple(SIMPLE_JSON, 2, 2, false)]
#[case::multi_register(MULTI_REGISTER, 2, 3, false)]
#[case::unknown_op(UNKNOWN_OP, 2, 3, true)]
#[case::small_parametrized(SMALL_PARAMETERIZED, 1, 1, false)]
#[case::parametrized(PARAMETERIZED, 4, 2, true)] // TK1 op is not supported
#[case::barrier(BARRIER, 3, 3, false)]
#[case::implicit_permutation(IMPLICIT_PERMUTATION, 1, 3, false)]
fn json_roundtrip(
    #[case] circ_s: &str,
    #[case] num_commands: usize,
    #[case] num_qubits: usize,
    #[case] has_tk1_ops: bool,
) {
    let ser: circuit_json::SerialCircuit = serde_json::from_str(circ_s).unwrap();
    assert_eq!(ser.commands.len(), num_commands);

    let circ: Circuit = ser.decode(DecodeOptions::new()).unwrap();
    assert_eq!(circ.qubit_count(), num_qubits);

    if !has_tk1_ops {
        check_no_tk1_ops(&circ);
    }

    let reser: SerialCircuit = SerialCircuit::encode(&circ, EncodeOptions::new()).unwrap();
    validate_serial_circ(&reser);
    compare_serial_circs(&ser, &reser);
}

#[rstest]
#[cfg_attr(miri, ignore)] // Opening files is not supported in (isolated) miri
#[case::barenco_tof_10("../test_files/pytket/barenco_tof_10.json")]
fn json_file_roundtrip(#[case] circ: impl AsRef<std::path::Path>) {
    let reader = BufReader::new(std::fs::File::open(circ).unwrap());
    let ser: circuit_json::SerialCircuit = serde_json::from_reader(reader).unwrap();
    let circ: Circuit = ser.decode(DecodeOptions::new()).unwrap();

    check_no_tk1_ops(&circ);

    let reser: SerialCircuit = SerialCircuit::encode(&circ, EncodeOptions::new()).unwrap();
    validate_serial_circ(&reser);
    compare_serial_circs(&ser, &reser);
}

/// Test parameter to select which decoders/encoders to enable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CircuitRoundtripTestConfig {
    // Use the default decoder/encoder configuration.
    Default,
    // Use only the prelude and core decoders/encoders, with no std ones.
    NoStd,
}

impl CircuitRoundtripTestConfig {
    fn decoder_config(&self) -> PytketDecoderConfig {
        match self {
            CircuitRoundtripTestConfig::Default => default_decoder_config(),
            CircuitRoundtripTestConfig::NoStd => {
                let mut config = PytketDecoderConfig::new();
                config.add_decoder(CoreDecoder);
                config.add_decoder(PreludeEmitter);
                config.add_type_translator(PreludeEmitter);
                config
            }
        }
    }

    fn encoder_config<H: HugrView>(&self) -> PytketEncoderConfig<H> {
        match self {
            CircuitRoundtripTestConfig::Default => default_encoder_config(),
            CircuitRoundtripTestConfig::NoStd => {
                let mut config = PytketEncoderConfig::new();
                config.add_emitter(PreludeEmitter);
                config.add_type_translator(PreludeEmitter);
                config
            }
        }
    }
}

#[rstest]
fn encoded_circuit_attributes(circ_measure_ancilla: Circuit) {
    let circ = circ_measure_ancilla;

    let encode_options = EncodeOptions::new().with_subcircuits(true);

    let encoded = EncodedCircuit::new(&circ, encode_options).unwrap_or_else(|e| panic!("{e}"));

    assert!(encoded.contains_circuit(circ.parent()));
    assert_eq!(encoded.len(), 1);
    assert!(!encoded.is_empty());

    let (_, serial_circ) = encoded.iter().exactly_one().ok().unwrap();
    assert_eq!(serial_circ.commands.len(), 2);

    let par_sum: usize = encoded
        .par_iter()
        .map(|(_, circ)| circ.commands.len())
        .sum();
    assert_eq!(par_sum, 2);
}

/// Test the standalone serialisation roundtrip from a tket circuit.
///
/// This is not a pure roundtrip as the encoder may add internal qubits/bits to
/// the circuit.
///
/// Standalone circuit do not currently support unsupported subgraphs with
/// nested structure or non-local edges.
#[rstest]
#[case::meas_ancilla(circ_measure_ancilla(), CircuitRoundtripTestConfig::Default)]
#[case::preset_qubits(circ_preset_qubits(), CircuitRoundtripTestConfig::Default)]
#[case::preset_parameterized(circ_parameterized(), CircuitRoundtripTestConfig::Default)]
// TODO: Should pass once CircBox encoding of DFGs is re-enabled.
#[should_panic(expected = "Cannot encode subgraphs with nested structure")]
#[case::nested_dfgs(circ_nested_dfgs(), CircuitRoundtripTestConfig::Default)]
#[case::tk1_ops(circ_tk1_ops(), CircuitRoundtripTestConfig::Default)]
#[case::missing_decoders(circ_measure_ancilla(), CircuitRoundtripTestConfig::NoStd)]
fn circuit_standalone_roundtrip(#[case] circ: Circuit, #[case] config: CircuitRoundtripTestConfig) {
    let circ_signature = circ.circuit_signature().into_owned();
    let decode_options = DecodeOptions::new()
        .with_signature(circ_signature.clone())
        .with_config(config.decoder_config());
    let encode_options = EncodeOptions::new()
        .with_subcircuits(true)
        .with_config(config.encoder_config());

    let encoded = EncodedCircuit::new_standalone(&circ, encode_options.clone())
        .unwrap_or_else(|e| panic!("{e}"));

    assert!(encoded.contains_circuit(circ.parent()));
    assert_eq!(encoded.len(), 1);

    // Re-encode the EncodedCircuit
    let extracted_from_circ = encoded
        .reassemble(
            circ.parent(),
            Some("main".to_string()),
            decode_options.clone(),
        )
        .unwrap_or_else(|e| panic!("{e}"));
    extracted_from_circ
        .validate()
        .unwrap_or_else(|e| panic!("{e}"));

    // Extract the head pytket circuit, and re-encode it on its own.
    let ser: &SerialCircuit = &encoded[circ.parent()];
    let deser: Circuit = ser.decode(decode_options).unwrap_or_else(|e| panic!("{e}"));

    deser.hugr().validate().unwrap_or_else(|e| panic!("{e}"));

    let deser_sig = deser.circuit_signature();
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

    let reser = SerialCircuit::encode(&deser, encode_options).unwrap();

    validate_serial_circ(&reser);
    compare_serial_circs(ser, &reser);
}

/// Test that more complex unsupported subgraphs (nested structure, non-local edges) are rejected when encoding a standalone circuit.
#[rstest]
#[case::unsupported_subtree(circ_unsupported_subtree())]
#[case::global_defs(circ_global_defs())]
#[case::recursive(circ_recursive())]
fn reject_standalone_complex_subgraphs(#[case] circ: Circuit) {
    let try_encoded = EncodedCircuit::new_standalone(&circ, EncodeOptions::new());
    assert_matches!(
        try_encoded,
        Err(PytketEncodeError::OpEncoding(
            PytketEncodeOpError::UnsupportedStandaloneSubgraph { .. }
        ))
    );
}

/// Test that modifying the hugr before reassembling an EncodedCircuit fails.
#[rstest]
fn fail_on_modified_hugr(circ_tk1_ops: Circuit) {
    let encoded = EncodedCircuit::new(&circ_tk1_ops, EncodeOptions::new().with_subcircuits(true))
        .unwrap_or_else(|e| panic!("{e}"));

    let mut a_new_hugr = ModuleBuilder::new();
    a_new_hugr
        .declare("decl", Signature::new_endo(vec![qb_t()]).into())
        .unwrap();
    let mut a_new_hugr = a_new_hugr.finish_hugr().unwrap();

    let try_reassemble = encoded.reassemble_inplace(&mut a_new_hugr, None);

    assert_matches!(
        try_reassemble,
        Err(PytketDecodeError {
            inner: PytketDecodeErrorInner::IncompatibleTargetRegion { .. },
            ..
        })
    );
}

/// Test the serialisation roundtrip from a tket circuit into an EncodedCircuit and back.
#[rstest]
#[case::meas_ancilla(circ_measure_ancilla(), 1, CircuitRoundtripTestConfig::Default)]
#[case::preset_qubits(circ_preset_qubits(), 1, CircuitRoundtripTestConfig::Default)]
#[case::preset_parameterized(circ_parameterized(), 1, CircuitRoundtripTestConfig::Default)]
#[case::nested_dfgs(circ_nested_dfgs(), 2, CircuitRoundtripTestConfig::Default)]
#[case::flat_opaque(circ_tk1_ops(), 1, CircuitRoundtripTestConfig::Default)]
#[case::unsupported_subtree(circ_unsupported_subtree(), 3, CircuitRoundtripTestConfig::Default)]
#[case::global_defs(circ_global_defs(), 1, CircuitRoundtripTestConfig::Default)]
#[case::recursive(circ_recursive(), 1, CircuitRoundtripTestConfig::Default)]
#[case::independent_subgraph(circ_independent_subgraph(), 3, CircuitRoundtripTestConfig::Default)]
#[case::unsupported_io_wire(circ_unsupported_io_wire(), 1, CircuitRoundtripTestConfig::Default)]
#[case::order_edge(circ_order_edge(), 1, CircuitRoundtripTestConfig::Default)]
#[case::bool_conversion(circ_bool_conversion(), 1, CircuitRoundtripTestConfig::Default)]
#[case::complex_param_type(circ_complex_param_type(), 1, CircuitRoundtripTestConfig::Default)]
// TODO: We need to track [`EncodedCircuitInfo`] for nested CircBoxes too. We
// have temporarily disabled encoding of DFG and function calls as CircBoxes to
// avoid an error here.
#[case::unsupported_extras_in_circ_box(
    circ_unsupported_extras_in_circ_box(),
    4,
    CircuitRoundtripTestConfig::Default
)]
#[case::output_parameter_wire(circ_output_parameter_wire(), 1, CircuitRoundtripTestConfig::Default)]
#[case::non_local(circ_non_local(), 2, CircuitRoundtripTestConfig::Default)]
#[case::unsupported_subgraph_no_registers(
    circ_unsupported_subgraph_no_registers(),
    1,
    CircuitRoundtripTestConfig::Default
)]
#[case::discard_first_qubit(circ_discard_first_qubit(), 1, CircuitRoundtripTestConfig::Default)]

fn encoded_circuit_roundtrip(
    #[case] circ: Circuit,
    #[case] num_circuits: usize,
    #[case] config: CircuitRoundtripTestConfig,
) {
    let circ_signature = circ.circuit_signature().into_owned();
    let encode_options = EncodeOptions::new()
        .with_subcircuits(true)
        .with_config(config.encoder_config());

    let encoded = EncodedCircuit::new(&circ, encode_options).unwrap_or_else(|e| panic!("{e}"));

    assert!(encoded.contains_circuit(circ.parent()));
    assert_eq!(encoded.len(), num_circuits);

    let mut deser = circ.clone();
    encoded
        .reassemble_inplace(deser.hugr_mut(), Some(Arc::new(config.decoder_config())))
        .unwrap_or_else(|e| panic!("{e}"));

    deser.hugr().validate().unwrap_or_else(|e| panic!("{e}"));

    let deser_sig = deser.circuit_signature();
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

/// Test serialisation of circuits with a symbolic expression.
///
/// Note: this is not a proper roundtrip as the symbols f0 and f1 are not
/// converted back to circuit inputs. This would require parsing symbolic
/// expressions.
#[rstest]
#[case::symbolic(circ_add_angles_symbolic())]
#[case::constants(circ_add_angles_constants())]
#[case::complex(circ_complex_angle_computation())]
fn test_add_angle_serialise(#[case] circ_add_angles: (Circuit, String)) {
    let (circ, expected) = circ_add_angles;

    let ser: SerialCircuit = SerialCircuit::encode(&circ, EncodeOptions::new()).unwrap();
    assert_eq!(ser.commands.len(), 1);
    assert_eq!(ser.commands[0].op.op_type, optype::OpType::Rx);
    assert_eq!(ser.commands[0].op.params, Some(vec![expected]));

    let deser: Circuit = ser.decode(DecodeOptions::new()).unwrap();
    let reser = SerialCircuit::encode(&deser, EncodeOptions::new()).unwrap();
    validate_serial_circ(&reser);
    compare_serial_circs(&ser, &reser);
}

/// Test the different options for inplace decoding.
#[rstest]
fn test_inplace_decoding() {
    let serial: circuit_json::SerialCircuit = serde_json::from_str(SIMPLE_JSON).unwrap();

    let mut builder = ModuleBuilder::new();

    let func1 = serial
        .decode_inplace(
            builder.hugr_mut(),
            DecodeInsertionTarget::Function { fn_name: None },
            DecodeOptions::new(),
        )
        .unwrap();
    let circ_signature = builder
        .hugr()
        .get_optype(func1)
        .inner_function_type()
        .unwrap()
        .into_owned();

    let dfg = {
        let mut fn_build = builder
            .define_function("func2", circ_signature.clone())
            .unwrap();
        let fn2_node = fn_build.container_node();
        let [inp, out] = fn_build.io();

        let dfg = serial
            .decode_inplace(
                fn_build.hugr_mut(),
                DecodeInsertionTarget::Region { parent: fn2_node },
                DecodeOptions::new(),
            )
            .unwrap();

        // Wire up the inserted dfg
        for inp_idx in 0..circ_signature.input_count() {
            fn_build.hugr_mut().connect(inp, inp_idx, dfg, inp_idx);
        }
        for out_idx in 0..circ_signature.output_count() {
            fn_build.hugr_mut().connect(dfg, out_idx, out, out_idx);
        }

        dfg
    };

    // Finish up and validate the final HUGR
    let hugr = builder.finish_hugr().unwrap();

    assert!(hugr.get_optype(func1).is_func_defn());
    assert!(hugr.get_optype(dfg).is_dfg());
}

/// Test the lazy qubit/bit decoding behaviour when the registers are not
/// present in the decoded region signature.
///
/// If the registers are never consumed, they won't be initialized at all.
#[rstest]
fn test_qubit_elision() {
    let ser: circuit_json::SerialCircuit = serde_json::from_str(EMPTY_CIRCUIT).unwrap();
    assert_eq!(ser.commands.len(), 0);

    let circ: Circuit = ser
        .decode(DecodeOptions::new().with_signature(Signature::new_endo(vec![])))
        .unwrap();
    assert_eq!(circ.qubit_count(), 0);

    check_no_tk1_ops(&circ);

    // The circuit should have no alloc/frees or const definitions
    assert_eq!(circ.num_operations(), 0);
}
