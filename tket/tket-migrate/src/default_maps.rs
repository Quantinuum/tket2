use std::sync::{Arc, LazyLock};

use hugr::Extension;
use hugr::builder::{DFGBuilder, Dataflow, DataflowHugr};
use hugr::extension::Version;
use hugr::extension::prelude::{bool_t, qb_t};
use hugr::ops::{
    Value,
    constant::{CustomConst, ValueName, downcast_equal_consts},
};
use hugr::types::{Signature, Type};

use crate::extension_loading::{get_extension, get_extension_op_by_name};
use crate::update_maps::{
    ConstMapping, OpMapping, OpReplacementTemplate, SimpleTemplate, TypeMapping,
};
use tket::passes::replace_types::NodeTemplate;

/// Mapping to replace the legacy TKET boolean type with the built-in HUGR boolean type.
///
/// Maps `tket.bool@0.2.0::bool` to the type returned by [`bool_t`].
pub(crate) static BOOL_TYPE_MAPPING: LazyLock<TypeMapping> = LazyLock::new(get_bool_type_map);

/// Converts legacy TKET boolean literals to built-in HUGR boolean values.
pub(crate) static BOOL_CONST_MAPPING: LazyLock<ConstMapping> = LazyLock::new(get_bool_const_map);

/// Mapping to replace legacy operations from the `tket.quantum` extension.
///
/// - `tket.quantum@0.2.1::MeasureFree` becomes
///   `tket.quantum@0.3.0::MeasureFree` followed by `tket.measurement@0.1.0::Read`.
pub(crate) static MEAS_OP_MAPPING: LazyLock<OpMapping> = LazyLock::new(get_tket_quantum_op_map);

/// Mapping to replace legacy operations from the `tket.bool` extension.
///
/// - `tket.bool@0.2.0` operations `and`, `eq`, `not`, `or`, and `xor` become
///   the corresponding operations in `logic@0.1.0`.
/// - `tket.bool@0.2.0::read` and `tket.bool@0.2.0::make_opaque` become
///   passthroughs, connecting their inputs directly to their outputs.
pub(crate) static BOOL_OP_MAPPING: LazyLock<OpMapping> = LazyLock::new(get_bool_op_map);

/// Define the operation mapping for the `tket.quantum` extension.
///
/// `tket.quantum@0.2.1::MeasureFree` becomes
/// `tket.quantum@0.3.0::MeasureFree` followed by `tket.measurement@0.1.0::Read`.
fn get_tket_quantum_op_map() -> OpMapping {
    let extension_name = "tket.quantum";
    let old_quantum_version = Version::new(0, 2, 1);
    let new_quantum_version = Version::new(0, 3, 0);

    let old_quantum_extension = get_extension(extension_name, &old_quantum_version).unwrap();
    let new_quantum_extension = get_extension(extension_name, &new_quantum_version).unwrap();

    let mut mapping = OpMapping::new(extension_name.into(), old_quantum_version);
    mapping.insert_all(
        old_quantum_extension
            .operations()
            .map(|(name, _definition)| {
                let replacement = match name.as_str() {
                    "MeasureFree" => build_measurement_replacement(new_quantum_extension),
                    "symbolic_angle" => {
                        OpReplacementTemplate::Parametrized(Arc::new(move |args, _rt| {
                            let angle =
                                new_quantum_extension.instantiate_extension_op(name, args)?;
                            Ok(Some(NodeTemplate::SingleOp(angle.into())))
                        }))
                    }
                    _ => OpReplacementTemplate::Simple(SimpleTemplate::TemplateInstance(
                        NodeTemplate::SingleOp(
                            get_extension_op_by_name(name, new_quantum_extension)
                                .unwrap()
                                .into(),
                        ),
                    )),
                };
                (name.as_str(), replacement)
            })
            .collect(),
    );
    mapping
}

/// Builds the replacement for MeasureFree.
///
/// The replacement is a linked HUGR with MeasureFree followed by Read.
fn build_measurement_replacement(quantum_extension: &Extension) -> OpReplacementTemplate {
    let measure_free = get_extension_op_by_name("MeasureFree", quantum_extension).unwrap();

    let meas_extension = get_extension("tket.measurement", &Version::new(0, 1, 0)).unwrap();
    let read = get_extension_op_by_name("Read", meas_extension).unwrap();

    // here
    let mut builder = DFGBuilder::new(Signature::new([qb_t()], [bool_t()])).unwrap();
    let [qubit] = builder.input_wires_arr();
    let measurement = builder
        // same as get_instantiated_op
        .add_dataflow_op(measure_free, [qubit])
        .unwrap()
        .out_wire(0);
    let result = builder
        .add_dataflow_op(read, [measurement])
        .unwrap()
        .out_wire(0);
    let replacement = builder.finish_hugr_with_outputs([result]).unwrap();

    OpReplacementTemplate::Simple(SimpleTemplate::TemplateInstance(NodeTemplate::linked_hugr(
        replacement,
    )))
}

/// Define the operation mapping for the `tket.bool` extension.
///
/// Boolean logic operations become the corresponding HUGR operations in
/// `logic@0.1.0`, while `read` and `make_opaque` become passthroughs.
fn get_bool_op_map() -> OpMapping {
    let bool_id = "tket.bool";
    let old_version = Version::new(0, 2, 0);
    let mut mapping = OpMapping::new(bool_id.into(), old_version);
    let logic_ext = get_extension("logic", &Version::new(0, 1, 0)).unwrap();
    mapping.insert_all(
        [
            ("and", "And"),
            ("eq", "Eq"),
            ("not", "Not"),
            ("or", "Or"),
            ("xor", "Xor"),
        ]
        .into_iter()
        .map(|(name, replacement)| {
            let op = get_extension_op_by_name(replacement, logic_ext)
                .unwrap()
                .into();
            (
                name,
                OpReplacementTemplate::Simple(SimpleTemplate::TemplateInstance(
                    NodeTemplate::SingleOp(op),
                )),
            )
        })
        .collect(),
    );
    mapping.insert(
        "make_opaque",
        OpReplacementTemplate::Simple(SimpleTemplate::Empty),
    );
    mapping.insert("read", OpReplacementTemplate::Simple(SimpleTemplate::Empty));
    mapping
}

/// Define the mapping to replace the legacy TKET boolean type with the built-in HUGR boolean type.
///
/// Maps `tket.bool@0.2.0::bool` to the type returned by [`bool_t`].
fn get_bool_type_map() -> TypeMapping {
    let mut mapping = TypeMapping::new("tket.bool".into(), Version::new(0, 2, 0));
    mapping.insert("bool", bool_t());
    mapping
}

/// The boolean literal defined by `tket.bool@0.2.0`.
/// See the [original implementation](https://github.com/Quantinuum/tket2/blob/b7cda585fd4163af7109e54a36c97a852b693e78/tket/src/extension/bool.rs#L69-L98).
#[derive(Clone, Debug, Hash, PartialEq, serde::Serialize, serde::Deserialize)]
pub(crate) struct ConstBool(pub(crate) bool);

#[typetag::serde]
impl CustomConst for ConstBool {
    fn name(&self) -> ValueName {
        format!("ConstBool({})", self.0).into()
    }

    fn get_type(&self) -> Type {
        get_extension("tket.bool", &Version::new(0, 2, 0))
            .unwrap()
            .get_type("bool")
            .unwrap()
            .instantiate([])
            .unwrap()
            .into()
    }

    fn equal_consts(&self, other: &dyn CustomConst) -> bool {
        downcast_equal_consts(self, other)
    }
}

// NICOLA: TODO: DOCS
fn get_bool_const_map() -> ConstMapping {
    let mut mapping = ConstMapping::new("tket.bool".into(), Version::new(0, 2, 0));
    mapping.insert("bool", |value, _| {
        let b = value.value().downcast_ref::<ConstBool>().unwrap();
        Ok(Value::from_bool(b.0))
    });
    mapping
}
