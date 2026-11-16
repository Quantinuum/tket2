use std::sync::LazyLock;

use hugr::builder::{DFGBuilder, Dataflow, DataflowHugr};
use hugr::extension::prelude::qb_t;
use hugr::extension::{Version, simple_op::MakeRegisteredOp};
use hugr::std_extensions::logic::LogicOp;
use hugr::types::Signature;

use crate::update_maps::{
    OpMapping, OpReplacementTemplate, TypeMapping, TypeReplacementTemplate, VersionedElement,
};
use tket::{
    TketOp, extension::measurement::MeasurementOp, hugr::extension::prelude::bool_t,
    passes::replace_types::NodeTemplate,
};

/// Mapping to replace the legacy TKET boolean type with the built-in HUGR boolean type.
///
/// Maps `tket.bool@0.2.0::bool` to the type returned by [`bool_t`].
pub static BOOL_TYPE_MAPPING: LazyLock<TypeMapping> = LazyLock::new(|| get_bool_type_map());

/// Mapping to replace legacy operations from the `tket.quantum` extension.
///
/// - `tket.quantum@0.2.1::MeasureFree` becomes
///   `tket.quantum@0.3.0::MeasureFree` followed by `tket.measurement@0.1.0::Read`.
pub static MEAS_OP_MAPPING: LazyLock<OpMapping> = LazyLock::new(|| get_tket_quantum_op_map());

/// Mapping to replace legacy operations from the `tket.bool` extension.
///
/// - `tket.bool@0.2.0` operations `and`, `eq`, `not`, `or`, and `xor` become
///   the corresponding HUGR [`LogicOp`] operations.
/// - `tket.bool@0.2.0::read` and `tket.bool@0.2.0::make_opaque` become
///   passthroughs, connecting their inputs directly to their outputs.
pub static BOOL_OP_MAPPING: LazyLock<OpMapping> = LazyLock::new(get_bool_op_map);

/// Define the operation mapping for the `tket.quantum` extension.
///
/// `tket.quantum@0.2.1::MeasureFree` becomes
/// `tket.quantum@0.3.0::MeasureFree` followed by `tket.measurement@0.1.0::Read`.
pub fn get_tket_quantum_op_map() -> OpMapping {
    let extension_name = "tket.quantum";
    let old_version = Version::new(0, 2, 1);

    vec![(
        VersionedElement::new(
            "MeasureFree".to_string(),
            extension_name.to_string(),
            old_version,
        ),
        OpReplacementTemplate::TemplateInstance(build_measurement_replacement()),
    )]
    .into()
}

/// Builds the replacement for MeasureFree.
///
/// The replacement is a linked HUGR with MeasureFree followed by Read.
fn build_measurement_replacement() -> NodeTemplate {
    let mut builder = DFGBuilder::new(Signature::new([qb_t()], [bool_t()])).unwrap();
    let [qubit] = builder.input_wires_arr();
    let measurement = builder
        .add_dataflow_op(TketOp::MeasureFree, [qubit])
        .unwrap()
        .out_wire(0);
    let result = builder
        .add_dataflow_op(MeasurementOp::Read, [measurement])
        .unwrap()
        .out_wire(0);
    let replacement = builder.finish_hugr_with_outputs([result]).unwrap();

    NodeTemplate::linked_hugr(replacement)
}

/// Define the operation mapping for the `tket.bool` extension.
///
/// Boolean logic operations become the corresponding HUGR [`LogicOp`] operations,
/// while `read` and `make_opaque` become passthroughs.
pub fn get_bool_op_map() -> OpMapping {
    let extension_name = "tket.bool";
    let version = Version::new(0, 2, 0);

    vec![
        (
            VersionedElement::new(
                "and".to_string(),
                extension_name.to_string(),
                version.clone(),
            ),
            OpReplacementTemplate::TemplateInstance(NodeTemplate::SingleOp(
                LogicOp::And.to_extension_op().unwrap().into(),
            )),
        ),
        (
            VersionedElement::new(
                "eq".to_string(),
                extension_name.to_string(),
                version.clone(),
            ),
            OpReplacementTemplate::TemplateInstance(NodeTemplate::SingleOp(
                LogicOp::Eq.to_extension_op().unwrap().into(),
            )),
        ),
        (
            VersionedElement::new(
                "make_opaque".to_string(),
                extension_name.to_string(),
                version.clone(),
            ),
            OpReplacementTemplate::Empty,
        ),
        (
            VersionedElement::new(
                "not".to_string(),
                extension_name.to_string(),
                version.clone(),
            ),
            OpReplacementTemplate::TemplateInstance(NodeTemplate::SingleOp(
                LogicOp::Not.to_extension_op().unwrap().into(),
            )),
        ),
        (
            VersionedElement::new(
                "or".to_string(),
                extension_name.to_string(),
                version.clone(),
            ),
            OpReplacementTemplate::TemplateInstance(NodeTemplate::SingleOp(
                LogicOp::Or.to_extension_op().unwrap().into(),
            )),
        ),
        (
            VersionedElement::new(
                "read".to_string(),
                extension_name.to_string(),
                version.clone(),
            ),
            OpReplacementTemplate::Empty,
        ),
        (
            VersionedElement::new(
                "xor".to_string(),
                extension_name.to_string(),
                version.clone(),
            ),
            OpReplacementTemplate::TemplateInstance(NodeTemplate::SingleOp(
                LogicOp::Xor.to_extension_op().unwrap().into(),
            )),
        ),
    ]
    .into()
}

/// Define the mapping to replace the legacy TKET boolean type with the built-in HUGR boolean type.
///
/// Maps `tket.bool@0.2.0::bool` to the type returned by [`bool_t`].
pub fn get_bool_type_map() -> TypeMapping {
    vec![(
        VersionedElement::new(
            "bool".to_string(),
            "tket.bool".to_string(),
            Version::new(0, 2, 0),
        ),
        TypeReplacementTemplate::Type(bool_t()),
    )]
    .into()
}
