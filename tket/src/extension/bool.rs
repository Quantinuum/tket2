//! This module defines a Hugr extension used to represent bools as an opaque type.
//!
//! This type is meant to be lowered to a sum that is either a unit sum (i.e. the
//! standard bool representation in Hugr) or a future in order to enable lazier
//! measurements.
use std::sync::{Arc, Weak};

use hugr::{
    Extension, Wire,
    builder::{BuildError, Dataflow},
    extension::{
        ExtensionBuildError, ExtensionId, SignatureFunc, TypeDef, Version,
        simple_op::{MakeOpDef, MakeRegisteredOp, try_from_name},
    },
    ops::constant::{CustomConst, ValueName},
    types::{CustomType, Signature, Type, TypeBound},
};
use lazy_static::lazy_static;
use smol_str::SmolStr;
use strum::{EnumIter, EnumString, IntoStaticStr};

/// The ID of the `tket.bool` extension.
pub const BOOL_EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.bool");
/// The "tket.bool" extension version
pub const BOOL_EXTENSION_VERSION: Version = Version::new(0, 2, 0);

lazy_static! {
    /// The "tket.bool" extension.
    pub static ref BOOL_EXTENSION: Arc<Extension>  = {
        Extension::new_arc(BOOL_EXTENSION_ID, BOOL_EXTENSION_VERSION, |ext, ext_ref| {
            let _ = add_bool_type_def(ext, ext_ref.clone()).unwrap();
            BoolOp::load_all_ops(ext, ext_ref).unwrap();
        })
    };

    /// The name of the `bool` type.
    pub static ref BOOL_TYPE_NAME: SmolStr = SmolStr::new_inline("bool");
}

fn add_bool_type_def(
    ext: &mut Extension,
    extension_ref: Weak<Extension>,
) -> Result<&TypeDef, ExtensionBuildError> {
    ext.add_type(
        BOOL_TYPE_NAME.to_owned(),
        vec![],
        "An opaque bool type".into(),
        TypeBound::Copyable.into(),
        &extension_ref,
    )
}

/// Returns a `tket.bool` [CustomType].
pub fn bool_custom_type(extension_ref: &Weak<Extension>) -> CustomType {
    CustomType::new(
        BOOL_TYPE_NAME.to_owned(),
        vec![],
        BOOL_EXTENSION_ID,
        TypeBound::Copyable,
        extension_ref,
    )
}

/// Returns a `bool` [Type].
pub fn bool_type() -> Type {
    bool_custom_type(&Arc::downgrade(&BOOL_EXTENSION)).into()
}

#[derive(Debug, Clone, PartialEq, Hash, serde::Serialize, serde::Deserialize)]
/// Structure for holding constant `tket.bool` values.
pub struct ConstBool(bool);

impl ConstBool {
    /// Creates a new [`ConstBool`].
    pub fn new(value: bool) -> Self {
        Self(value)
    }

    /// Returns the value of the constant.
    pub fn value(&self) -> bool {
        self.0
    }
}

#[typetag::serde]
impl CustomConst for ConstBool {
    fn name(&self) -> ValueName {
        format!("ConstBool({})", self.0).into()
    }

    fn equal_consts(&self, other: &dyn CustomConst) -> bool {
        hugr::ops::constant::downcast_equal_consts(self, other)
    }

    fn get_type(&self) -> Type {
        bool_type()
    }
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
#[allow(missing_docs, non_camel_case_types)]
#[non_exhaustive]
/// Simple enum of "tket.bool" operations.
pub enum BoolOp {
    // Gets a Hugr bool_t value from the opaque type.
    read,
    // Converts a Hugr bool_t value into the opaque type.
    make_opaque,
    eq,
    not,
    and,
    or,
    xor,
}

impl MakeOpDef for BoolOp {
    fn opdef_id(&self) -> hugr::ops::OpName {
        <&'static str>::from(self).into()
    }

    fn init_signature(&self, extension_ref: &Weak<Extension>) -> SignatureFunc {
        let bool_type = Type::new_extension(bool_custom_type(extension_ref));
        let sum_type = Type::new_unit_sum(2);
        match self {
            BoolOp::read => Signature::new(bool_type, sum_type).into(),
            BoolOp::make_opaque => Signature::new(sum_type, bool_type).into(),
            BoolOp::not => Signature::new(bool_type.clone(), bool_type.clone()).into(),
            BoolOp::eq | BoolOp::and | BoolOp::or | BoolOp::xor => Signature::new(
                vec![bool_type.clone(), bool_type.clone()],
                bool_type.clone(),
            )
            .into(),
        }
    }

    fn from_def(
        op_def: &hugr::extension::OpDef,
    ) -> Result<Self, hugr::extension::simple_op::OpLoadError> {
        try_from_name(op_def.name(), op_def.extension_id())
    }

    fn extension(&self) -> ExtensionId {
        BOOL_EXTENSION_ID
    }

    fn description(&self) -> String {
        match self {
            BoolOp::read => "Convert a tket.bool into a Hugr bool_t (a unit sum).".into(),
            BoolOp::make_opaque => "Convert a Hugr bool_t (a unit sum) into an tket.bool.".into(),
            BoolOp::eq => "Equality between two tket.bools.".into(),
            BoolOp::not => "Negation of a tket.bool.".into(),
            BoolOp::and => "Logical AND between two tket.bools.".into(),
            BoolOp::or => "Logical OR between two tket.bools.".into(),
            BoolOp::xor => "Logical XOR between two tket.bools.".into(),
        }
    }

    fn extension_ref(&self) -> Weak<Extension> {
        Arc::downgrade(&BOOL_EXTENSION)
    }
}

impl MakeRegisteredOp for BoolOp {
    fn extension_id(&self) -> ExtensionId {
        BOOL_EXTENSION_ID
    }

    fn extension_ref(&self) -> Weak<Extension> {
        Arc::downgrade(&BOOL_EXTENSION)
    }
}
/// An extension trait for [Dataflow] providing methods to add "tket.bool"
/// operations.
pub trait BoolOpBuilder: Dataflow {
    /// Add a "tket.bool.read" op.
    fn add_bool_read(&mut self, bool_input: Wire) -> Result<[Wire; 1], BuildError> {
        Ok(self
            .add_dataflow_op(BoolOp::read, [bool_input])?
            .outputs_arr())
    }

    /// Add a "tket.bool.make_opaque" op.
    fn add_bool_make_opaque(&mut self, sum_input: Wire) -> Result<[Wire; 1], BuildError> {
        Ok(self
            .add_dataflow_op(BoolOp::make_opaque, [sum_input])?
            .outputs_arr())
    }

    /// Add a "tket.bool.Eq" op.
    fn add_eq(&mut self, bool1: Wire, bool2: Wire) -> Result<[Wire; 1], BuildError> {
        Ok(self
            .add_dataflow_op(BoolOp::eq, [bool1, bool2])?
            .outputs_arr())
    }

    /// Add a "tket.bool.Not" op.
    fn add_not(&mut self, bool_input: Wire) -> Result<[Wire; 1], BuildError> {
        Ok(self
            .add_dataflow_op(BoolOp::not, [bool_input])?
            .outputs_arr())
    }

    /// Add a "tket.bool.And" op.
    fn add_and(&mut self, bool1: Wire, bool2: Wire) -> Result<[Wire; 1], BuildError> {
        Ok(self
            .add_dataflow_op(BoolOp::and, [bool1, bool2])?
            .outputs_arr())
    }

    /// Add a "tket.bool.Or" op.
    fn add_or(&mut self, bool1: Wire, bool2: Wire) -> Result<[Wire; 1], BuildError> {
        Ok(self
            .add_dataflow_op(BoolOp::or, [bool1, bool2])?
            .outputs_arr())
    }

    /// Add a "tket.bool.Xor" op.
    fn add_xor(&mut self, bool1: Wire, bool2: Wire) -> Result<[Wire; 1], BuildError> {
        Ok(self
            .add_dataflow_op(BoolOp::xor, [bool1, bool2])?
            .outputs_arr())
    }
}

impl<D: Dataflow> BoolOpBuilder for D {}

#[cfg(test)]
pub(crate) mod test {
    use super::*;
    use hugr::HugrView;
    use hugr::{
        builder::{DFGBuilder, Dataflow, DataflowHugr},
        extension::{OpDef, simple_op::MakeExtensionOp},
    };
    use strum::IntoEnumIterator;

    fn get_opdef(op: BoolOp) -> Option<&'static Arc<OpDef>> {
        BOOL_EXTENSION.get_op(&op.op_id())
    }

    #[test]
    fn create_extension() {
        assert_eq!(BOOL_EXTENSION.name(), &BOOL_EXTENSION_ID);

        for o in BoolOp::iter() {
            assert_eq!(BoolOp::from_def(get_opdef(o).unwrap()), Ok(o));
        }
    }

    #[test]
    fn test_bool_type() {
        let bool_custom_type = BOOL_EXTENSION
            .get_type(&BOOL_TYPE_NAME)
            .unwrap()
            .instantiate([])
            .unwrap();
        let bool_ty = Type::new_extension(bool_custom_type);
        assert_eq!(bool_ty, bool_type());
        let bool_const = ConstBool::new(true);
        assert_eq!(bool_const.get_type(), bool_ty);
        assert!(bool_const.value());
        assert!(bool_const.validate().is_ok());
    }

    #[test]
    fn test_read() {
        let bool_type = bool_type();
        let sum_type = Type::new_unit_sum(2);

        let hugr = {
            let mut builder = DFGBuilder::new(Signature::new(bool_type, sum_type)).unwrap();
            let [input] = builder.input_wires_arr();
            let output = builder.add_bool_read(input).unwrap();
            builder.finish_hugr_with_outputs(output).unwrap()
        };
        hugr.validate().unwrap();
    }

    #[test]
    fn test_make_opaque() {
        let bool_type = bool_type();
        let sum_type = Type::new_unit_sum(2);

        let hugr = {
            let mut builder = DFGBuilder::new(Signature::new(sum_type, bool_type)).unwrap();
            let [input] = builder.input_wires_arr();
            let output = builder.add_bool_make_opaque(input).unwrap();
            builder.finish_hugr_with_outputs(output).unwrap()
        };
        hugr.validate().unwrap();
    }
}
