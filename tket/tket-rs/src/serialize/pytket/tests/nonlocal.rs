//! Regression tests for non-local edges crossing encoded dataflow regions.

use hugr::builder::{Container, Dataflow, DataflowHugr, FunctionBuilder, SubContainer};
use hugr::extension::prelude::{Noop, bool_t};
use hugr::ops::handle::NodeHandle;
use hugr::ops::{OpTrait, Value};
use hugr::std_extensions::arithmetic::float_types::{ConstF64, float64_type};
use hugr::types::{Signature, Type};
use hugr::{Hugr, HugrView, Node, Wire};
use rstest::rstest;

use crate::extension::rotation::{ConstRotation, rotation_type};
use crate::serialize::pytket::{EncodeOptions, EncodedCircuit};

/// Check that region inputs keep both local and non-local consumers.
#[rstest]
#[case::boolean(bool_t())]
#[case::float(float64_type())]
#[case::rotation(rotation_type())]
fn nonlocal_input(#[case] ty: Type) {
    let mut builder = FunctionBuilder::new(
        "nonlocal_input",
        Signature::new(vec![ty.clone()], vec![ty.clone(); 3]),
    )
    .unwrap();
    let [source] = builder.input_wires_arr();
    let (left, left_consumer) = add_nonlocal_consumer(&mut builder, source, &ty);
    let (right, right_consumer) = add_nonlocal_consumer(&mut builder, source, &ty);
    let mut hugr = builder
        .finish_hugr_with_outputs([left, right, source])
        .unwrap();

    for _ in 0..2 {
        validate_roundtrip(&mut hugr);
        let source = assert_nonlocal_fanout(&hugr, [left_consumer, right_consumer], &ty);
        let [input, output] = hugr.get_io(hugr.entrypoint()).unwrap();
        assert_eq!(source, Wire::new(input, 0));
        assert_eq!(
            hugr.single_linked_output(output, 2),
            Some((source.node(), source.source()))
        );
    }
}

/// Check that a boolean constant used only in nested regions keeps its value.
#[test]
fn nonlocal_boolean_constant() {
    let ty = bool_t();
    let mut builder = FunctionBuilder::new(
        "nonlocal_boolean_constant",
        Signature::new(vec![], vec![ty.clone(); 2]),
    )
    .unwrap();
    let source = builder.add_load_const(Value::true_val());
    let (left, left_consumer) = add_nonlocal_consumer(&mut builder, source, &ty);
    let (right, right_consumer) = add_nonlocal_consumer(&mut builder, source, &ty);
    let mut hugr = builder.finish_hugr_with_outputs([left, right]).unwrap();

    for _ in 0..2 {
        validate_roundtrip(&mut hugr);
        let source = assert_nonlocal_fanout(&hugr, [left_consumer, right_consumer], &ty);
        let constant = hugr.single_linked_output(source.node(), 0).unwrap().0;
        assert_eq!(
            hugr.get_optype(constant).as_const().unwrap().value(),
            &Value::true_val()
        );
    }
}

/// Check that parameter constants used only in nested regions remain connected.
#[rstest]
#[case::float(Value::from(ConstF64::new(0.25)))]
#[case::rotation(Value::from(ConstRotation::new(0.25).unwrap()))]
fn nonlocal_parameter_constant(#[case] value: Value) {
    let ty = value.get_type();
    let mut builder = FunctionBuilder::new(
        "nonlocal_parameter_constant",
        Signature::new(vec![], vec![ty.clone(); 2]),
    )
    .unwrap();
    let source = builder.add_load_const(value);
    let (left, left_consumer) = add_nonlocal_consumer(&mut builder, source, &ty);
    let (right, right_consumer) = add_nonlocal_consumer(&mut builder, source, &ty);
    let mut hugr = builder.finish_hugr_with_outputs([left, right]).unwrap();

    for _ in 0..2 {
        validate_roundtrip(&mut hugr);
        assert_nonlocal_fanout(&hugr, [left_consumer, right_consumer], &ty);
    }
}

/// Check that opaque outputs keep both local and non-local consumers.
#[rstest]
#[case::boolean(bool_t())]
#[case::float(float64_type())]
#[case::rotation(rotation_type())]
fn nonlocal_opaque_output(#[case] ty: Type) {
    let mut builder = FunctionBuilder::new(
        "nonlocal_opaque_output",
        Signature::new(vec![ty.clone()], vec![ty.clone(); 3]),
    )
    .unwrap();
    let [input] = builder.input_wires_arr();
    let source = builder
        .add_dataflow_op(Noop::new(ty.clone()), [input])
        .unwrap()
        .out_wire(0);
    let (left, left_consumer) = add_nonlocal_consumer(&mut builder, source, &ty);
    let (right, right_consumer) = add_nonlocal_consumer(&mut builder, source, &ty);
    builder.add_other_wire(source.node(), left.node());
    builder.add_other_wire(source.node(), right.node());
    let mut hugr = builder
        .finish_hugr_with_outputs([left, right, source])
        .unwrap();

    for _ in 0..2 {
        validate_roundtrip(&mut hugr);
        let decoded = assert_nonlocal_fanout(&hugr, [left_consumer, right_consumer], &ty);
        assert_eq!(decoded, source);
        let output = hugr.get_io(hugr.entrypoint()).unwrap()[1];
        assert_eq!(
            hugr.single_linked_output(output, 2),
            Some((source.node(), source.source()))
        );
    }
}

/// Add a nested region whose input bypasses its local input node.
fn add_nonlocal_consumer(
    builder: &mut FunctionBuilder<Hugr>,
    source: Wire,
    ty: &Type,
) -> (Wire, Node) {
    let mut nested = builder
        .dfg_builder(Signature::new(vec![], vec![ty.clone()]), [])
        .unwrap();
    let consumer = nested
        .add_dataflow_op(Noop::new(ty.clone()), [source])
        .unwrap();
    nested.set_outputs([consumer.out_wire(0)]).unwrap();
    let output = nested.finish_sub_container().unwrap().out_wire(0);
    (output, consumer.node())
}

/// Roundtrip all dataflow regions and validate the graph before and after.
fn validate_roundtrip(hugr: &mut Hugr) {
    hugr.validate().unwrap();
    let encoded = EncodedCircuit::new(
        hugr,
        EncodeOptions::new()
            .with_subcircuits(true)
            .keep_empty_circuits(true),
    )
    .unwrap();
    encoded.reassemble_inplace(hugr, None).unwrap();
    hugr.validate().unwrap();
}

/// Check that both nested consumers use the same wire in the outer region.
fn assert_nonlocal_fanout(hugr: &Hugr, consumers: [Node; 2], ty: &Type) -> Wire {
    let source = hugr.single_linked_output(consumers[0], 0).unwrap();
    assert_eq!(hugr.single_linked_output(consumers[1], 0), Some(source));
    assert_eq!(hugr.get_parent(source.0), Some(hugr.entrypoint()));
    assert_eq!(
        hugr.get_optype(source.0).value_output_type(source.1),
        Some(ty.clone())
    );
    Wire::new(source.0, source.1)
}
