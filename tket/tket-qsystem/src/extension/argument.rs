//! This module defines a Hugr extension for entrypoint parameter support
use std::sync::{Arc, Weak};

use hugr::{
    Extension, Wire,
    builder::{BuildError, Dataflow},
    extension::{
        ExtensionId, OpDef, SignatureError, SignatureFunc, Version,
        simple_op::{
            HasConcrete, HasDef, MakeExtensionOp, MakeOpDef, MakeRegisteredOp, OpLoadError,
            try_from_name,
        },
    },
    ops::OpName,
    types::{PolyFuncTypeRV, Signature, Term, Type, TypeArg, TypeBound, type_param::TypeParam},
};

use lazy_static::lazy_static;
use serde::{Deserialize, Serialize};
use strum::{EnumIter, EnumString, IntoStaticStr};

/// The ID of the `tket.argument` extension.
pub const EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.argument");
/// The "tket.argument" extension version
pub const EXTENSION_VERSION: Version = Version::new(0, 1, 0);

lazy_static! {
    /// The "tket.argument" extension.
    pub static ref EXTENSION: Arc<Extension> = {
        Extension::new_arc(EXTENSION_ID, EXTENSION_VERSION, |ext, ext_ref| {
            ReadArgOpDef::load_all_ops(ext, ext_ref).unwrap();
        })
    };
}

#[derive(
    Clone,
    Copy,
    Debug,
    Serialize,
    Deserialize,
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
/// Runtime argument reading operation
pub enum ReadArgOpDef {
    /// Read a runtime argument of any supported type, identified by a string tag
    #[strum(serialize = "read_arg")]
    ReadArg,
}

impl MakeOpDef for ReadArgOpDef {
    fn opdef_id(&self) -> OpName {
        <&'static str>::from(self).into()
    }

    fn init_signature(&self, _extension_ref: &Weak<Extension>) -> SignatureFunc {
        // Params: [0: tag (string), 1: output type T (any runtime type, including linear)]
        let t_var = Type::new_var_use(1, TypeBound::Linear);
        PolyFuncTypeRV::new(
            vec![TypeParam::StringKind, TypeBound::Linear.into()],
            Signature::new(vec![], vec![t_var]),
        )
        .into()
    }

    fn from_def(op_def: &OpDef) -> Result<Self, OpLoadError> {
        try_from_name(op_def.name(), op_def.extension_id())
    }

    fn extension(&self) -> ExtensionId {
        EXTENSION_ID
    }

    fn description(&self) -> String {
        "Read a runtime argument of the given type identified by a string tag".to_string()
    }

    fn extension_ref(&self) -> Weak<Extension> {
        Arc::downgrade(&EXTENSION)
    }
}

#[derive(Debug, Clone, PartialEq)]
/// A concrete operation that reads a typed runtime argument identified by a string tag.
pub struct ReadArgOp {
    /// String tag identifying the argument (must match the provider's key).
    pub tag: String,
    /// The HUGR type of the argument to read.
    pub output_type: Type,
}

impl ReadArgOp {
    /// Create a new `ReadArgOp` for the given tag and output type.
    pub fn new(tag: impl Into<String>, output_type: Type) -> Self {
        Self {
            tag: tag.into(),
            output_type,
        }
    }
}

impl MakeExtensionOp for ReadArgOp {
    fn op_id(&self) -> OpName {
        ReadArgOpDef::ReadArg.opdef_id()
    }

    fn from_extension_op(ext_op: &hugr::ops::ExtensionOp) -> Result<Self, OpLoadError>
    where
        Self: Sized,
    {
        let def = ext_op.def();
        let args = ext_op.args();
        let read_op_def = ReadArgOpDef::from_def(def)?;
        read_op_def.instantiate(args)
    }

    fn type_args(&self) -> Vec<TypeArg> {
        vec![
            self.tag.clone().into(),
            Term::from(self.output_type.clone()),
        ]
    }
}

impl MakeRegisteredOp for ReadArgOp {
    fn extension_id(&self) -> ExtensionId {
        EXTENSION_ID
    }

    fn extension_ref(&self) -> Arc<Extension> {
        EXTENSION.clone()
    }
}

impl HasDef for ReadArgOp {
    type Def = ReadArgOpDef;
}

impl HasConcrete for ReadArgOpDef {
    type Concrete = ReadArgOp;

    fn instantiate(&self, type_args: &[TypeArg]) -> Result<Self::Concrete, OpLoadError> {
        match type_args {
            [TypeArg::String(tag), ty_arg] => {
                let output_type =
                    Type::try_from(ty_arg.clone()).map_err(|_| SignatureError::InvalidTypeArgs)?;
                Ok(ReadArgOp {
                    tag: tag.clone(),
                    output_type,
                })
            }
            _ => Err(SignatureError::InvalidTypeArgs.into()),
        }
    }
}

/// A builder trait for adding `ReadArgOp`s to a dataflow.
pub trait ReadArgBuilder: Dataflow {
    /// Emit a `ReadArgOp` returning a value of the given type identified by `tag`.
    fn add_read_arg(
        &mut self,
        tag: impl Into<String>,
        output_type: Type,
    ) -> Result<Wire, BuildError> {
        let op = ReadArgOp::new(tag, output_type);
        let handle = self.add_dataflow_op(op, [])?;
        debug_assert_eq!(handle.outputs().len(), 1);
        Ok(handle.out_wire(0))
    }
}
impl<D: Dataflow> ReadArgBuilder for D {}

#[cfg(test)]
pub(crate) mod test {
    use super::*;
    use hugr::extension::prelude::bool_t;
    use hugr::extension::simple_op::{HasConcrete, MakeRegisteredOp};
    use hugr::std_extensions::arithmetic::float_types::float64_type;
    use hugr::std_extensions::arithmetic::int_types::int_type;
    use hugr::std_extensions::collections::array::array_type;
    use hugr::types::TypeArg;
    use rstest::rstest;

    fn roundtrip(op: ReadArgOp) -> ReadArgOp {
        let type_args = op.type_args();
        ReadArgOpDef::ReadArg.instantiate(&type_args).unwrap()
    }

    #[rstest]
    #[case("my_bool", bool_t())]
    #[case("my_int", int_type(TypeArg::BoundedNat(6)))]
    #[case("my_f64", float64_type())]
    #[case("my_arr_bool", array_type(10, bool_t()))]
    #[case("my_arr_int", array_type(10, int_type(TypeArg::BoundedNat(6))))]
    #[case("my_arr_f64", array_type(10, float64_type()))]
    fn test_roundtrip(#[case] tag: &str, #[case] output_type: Type) {
        let op = ReadArgOp::new(tag, output_type);
        assert_eq!(roundtrip(op.clone()), op);
    }

    #[test]
    fn test_to_extension_op_roundtrip() {
        let op = ReadArgOp::new("my_bool", bool_t());
        let ext_op = op
            .clone()
            .to_extension_op()
            .expect("should build extension op");
        let roundtripped = ReadArgOp::from_extension_op(&ext_op).expect("should decode");
        assert_eq!(roundtripped, op);
    }
}
