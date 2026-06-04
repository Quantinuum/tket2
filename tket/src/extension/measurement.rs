use std::sync::{Arc, Weak};

use hugr::{
    Extension, Wire,
    builder::{BuildError, Dataflow},
    extension::{
        ExtensionBuildError, ExtensionId, OpDef, SignatureFunc, TypeDef, Version,
        prelude::bool_t,
        simple_op::{MakeOpDef, MakeRegisteredOp, try_from_name},
    },
    types::{CustomType, Signature, Type, TypeBound},
};
use lazy_static::lazy_static;
use smol_str::SmolStr;
use strum::{EnumIter, EnumString, IntoStaticStr};

/// The ID of the `tket.measurement` extension.
pub const MEASUREMENT_EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.measurement");
/// The `tket.measurement` extension version.
pub const MEASUREMENT_EXTENSION_VERSION: Version = Version::new(0, 1, 0);

lazy_static! {
    /// The `tket.measurement` extension.
    pub static ref MEASUREMENT_EXTENSION: Arc<Extension> = {
        Extension::new_arc(
            MEASUREMENT_EXTENSION_ID,
            MEASUREMENT_EXTENSION_VERSION,
            |ext, ext_ref| {
                let _ = add_measurement_type_def(ext, ext_ref.clone()).unwrap();
                MeasurementOp::load_all_ops(ext, ext_ref).unwrap();
            },
        )
    };

    /// The name of the `Measurement` type.
    pub static ref MEASUREMENT_TYPE_ID: SmolStr = SmolStr::new_inline("Measurement");
}

fn add_measurement_type_def(
    ext: &mut Extension,
    extension_ref: Weak<Extension>,
) -> Result<&TypeDef, ExtensionBuildError> {
    ext.add_type(
        MEASUREMENT_TYPE_ID.to_owned(),
        vec![],
        "A copyable type representing the result of a measurement operation".into(),
        TypeBound::Copyable.into(),
        &extension_ref,
    )
}
/// Returns a `Measurement` [CustomType].
pub fn measurement_custom_type() -> CustomType {
    MEASUREMENT_EXTENSION
        .get_type(&MEASUREMENT_TYPE_ID)
        .unwrap()
        .instantiate([])
        .unwrap()
}

/// Returns a `Measurement` [Type].
pub fn measurement_type() -> Type {
    measurement_custom_type().into()
}

#[derive(
    Clone,
    Copy,
    Debug,
    serde::Serialize,
    serde::Deserialize,
    Hash,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    EnumIter,
    IntoStaticStr,
    EnumString,
)]
#[non_exhaustive]
/// Simple enum of `tket.measurement` operations.
pub enum MeasurementOp {
    /// Read a measurement, consuming it and returning a Hugr bool.
    Read,
}

impl MakeOpDef for MeasurementOp {
    fn opdef_id(&self) -> hugr::ops::OpName {
        <&'static str>::from(self).into()
    }

    fn init_signature(&self, extension_ref: &Weak<Extension>) -> SignatureFunc {
        let measurement_type = Type::new_extension(CustomType::new(
            MEASUREMENT_TYPE_ID.to_owned(),
            vec![],
            MEASUREMENT_EXTENSION_ID,
            TypeBound::Copyable,
            extension_ref,
        ));
        match self {
            MeasurementOp::Read => Signature::new([measurement_type], [bool_t()]).into(),
        }
    }

    fn from_def(op_def: &OpDef) -> Result<Self, hugr::extension::simple_op::OpLoadError> {
        try_from_name(op_def.name(), op_def.extension_id())
    }

    fn extension(&self) -> ExtensionId {
        MEASUREMENT_EXTENSION_ID
    }

    fn description(&self) -> String {
        match self {
            MeasurementOp::Read => "Consumes a measurement, converting it into a bool.".into(),
        }
    }

    fn extension_ref(&self) -> Weak<Extension> {
        Arc::downgrade(&MEASUREMENT_EXTENSION)
    }
}

impl MakeRegisteredOp for MeasurementOp {
    fn extension_id(&self) -> ExtensionId {
        MEASUREMENT_EXTENSION_ID
    }

    fn extension_ref(&self) -> Arc<Extension> {
        MEASUREMENT_EXTENSION.clone()
    }
}

/// An extension trait for [Dataflow] providing methods to add `tket.measurement`
/// operations.
pub trait MeasurementOpBuilder: Dataflow {
    /// Add a `tket.measurement.Read` op.
    fn add_measurement_read(&mut self, measurement: Wire) -> Result<[Wire; 1], BuildError> {
        Ok(self
            .add_dataflow_op(MeasurementOp::Read, [measurement])?
            .outputs_arr())
    }
}

impl<D: Dataflow> MeasurementOpBuilder for D {}

#[cfg(test)]
mod test {
    use std::sync::Arc;

    use hugr::HugrView;
    use hugr::builder::{DFGBuilder, Dataflow, DataflowHugr};
    use hugr::extension::OpDef;
    use hugr::types::Signature;
    use strum::IntoEnumIterator;

    use super::*;

    fn get_opdef(op: MeasurementOp) -> Option<&'static Arc<OpDef>> {
        MEASUREMENT_EXTENSION.get_op(&op.opdef_id())
    }

    #[test]
    fn create_extension() {
        assert_eq!(MEASUREMENT_EXTENSION.name(), &MEASUREMENT_EXTENSION_ID);

        for op in MeasurementOp::iter() {
            assert_eq!(MeasurementOp::from_def(get_opdef(op).unwrap()), Ok(op));
        }
    }

    #[test]
    fn measurement_ops_validate() {
        let mut builder =
            DFGBuilder::new(Signature::new(vec![measurement_type()], vec![bool_t()])).unwrap();
        let [msmt] = builder.input_wires_arr();
        let [bit] = builder.add_measurement_read(msmt).unwrap();
        let hugr = builder.finish_hugr_with_outputs([bit]).unwrap();
        hugr.validate().unwrap();
    }
}
