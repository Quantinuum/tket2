//! This module defines the Hugr extension used to represent futures.
//!
//! `Future<t>` is a linear type representing a value that will be available in
//! the future.  It can be consumed by `Read`, returning a `t`.  It can be
//! duplicated by `Dup`, and discarded with `Free`.
use std::sync::{Arc, Weak};

use hugr::{
    Extension, Wire,
    builder::{BuildError, Dataflow},
    extension::{
        ExtensionBuildError, ExtensionId, OpDef, SignatureError, SignatureFunc, TypeDef, Version,
        simple_op::{
            HasConcrete, HasDef, MakeExtensionOp, MakeOpDef, MakeRegisteredOp, OpLoadError,
            try_from_name,
        },
    },
    ops::{OpType, custom::ExtensionOp},
    types::{CustomType, PolyFuncType, Signature, Type, TypeArg, TypeBound, type_param::TypeParam},
};
use lazy_static::lazy_static;
use serde::{Deserialize, Serialize};
use smol_str::SmolStr;
use strum::{EnumIter, EnumString, IntoStaticStr};

/// The ID of the `tket.futures` extension.
pub const EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.futures");
/// The "tket.futures" extension version
pub const EXTENSION_VERSION: Version = Version::new(0, 2, 0);

lazy_static! {
    /// The "tket.futures" extension.
    pub static ref EXTENSION: Arc<Extension>  = {
        Extension::new_arc(EXTENSION_ID, EXTENSION_VERSION, |ext, ext_ref| {
            let _ = add_future_type_def(ext, ext_ref.clone()).unwrap();

            FutureOpDef::load_all_ops( ext, ext_ref).unwrap();
        })
    };

    /// The name of the `Future` type.
    pub static ref FUTURE_TYPE_NAME: SmolStr = SmolStr::new_inline("Future");
}

fn add_future_type_def(
    ext: &mut Extension,
    extension_ref: Weak<Extension>,
) -> Result<&TypeDef, ExtensionBuildError> {
    ext.add_type(
        FUTURE_TYPE_NAME.to_owned(),
        vec![TypeBound::Linear.into()],
        "A value that is computed asynchronously".into(),
        TypeBound::Linear.into(),
        &extension_ref,
    )
}

/// Returns a `Future<t>` [CustomType].
pub fn future_custom_type(t: Type, extension_ref: &Weak<Extension>) -> CustomType {
    CustomType::new(
        FUTURE_TYPE_NAME.to_owned(),
        vec![t.into()],
        EXTENSION_ID,
        TypeBound::Linear,
        extension_ref,
    )
}

/// Returns a `Future<t>` [Type].
pub fn future_type(t: Type) -> Type {
    future_custom_type(t, &Arc::downgrade(&EXTENSION)).into()
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
#[allow(missing_docs)]
#[non_exhaustive]
/// Simple enum of "tket.futures" operations.
pub enum FutureOpDef {
    Read,
    Dup,
    Free,
}

impl MakeOpDef for FutureOpDef {
    fn opdef_id(&self) -> hugr::ops::OpName {
        <&'static str>::from(self).into()
    }

    fn init_signature(&self, extension_ref: &Weak<Extension>) -> SignatureFunc {
        let t_param = TypeParam::from(TypeBound::Linear);
        let t_type = Type::new_var_use(0, TypeBound::Linear);
        let future_type = Type::new_extension(future_custom_type(t_type.clone(), extension_ref));
        match self {
            FutureOpDef::Read => {
                PolyFuncType::new([t_param], Signature::new(future_type, t_type)).into()
            }
            FutureOpDef::Dup => PolyFuncType::new(
                [t_param],
                Signature::new(future_type.clone(), vec![future_type.clone(), future_type]),
            )
            .into(),
            FutureOpDef::Free => {
                PolyFuncType::new([t_param], Signature::new(future_type.clone(), vec![])).into()
            }
        }
    }

    fn extension(&self) -> ExtensionId {
        EXTENSION_ID
    }

    fn from_def(op_def: &OpDef) -> Result<Self, hugr::extension::simple_op::OpLoadError> {
        try_from_name(op_def.name(), op_def.extension_id())
    }

    fn description(&self) -> String {
        match self {
            FutureOpDef::Read => "Read a value from a Future, consuming it".into(),
            FutureOpDef::Dup => {
                "Duplicate a Future. The original Future is consumed and two Futures are returned"
                    .into()
            }
            FutureOpDef::Free => "Consume a future without reading it.".into(),
        }
    }

    fn extension_ref(&self) -> Weak<Extension> {
        Arc::downgrade(&EXTENSION)
    }
}

impl HasConcrete for FutureOpDef {
    type Concrete = FutureOp;

    fn instantiate(&self, type_args: &[TypeArg]) -> Result<Self::Concrete, OpLoadError> {
        match type_args {
            [TypeArg::Runtime(ty)] => Ok(FutureOp {
                op: *self,
                typ: ty.clone(),
            }),
            _ => Err(SignatureError::InvalidTypeArgs.into()),
        }
    }
}

impl From<&FutureOp> for &'static str {
    fn from(value: &FutureOp) -> Self {
        value.op.into()
    }
}

/// Concrete "tket.futures" operations with type set.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FutureOp {
    /// The `FutureOpDef` that defines this operation.
    pub op: FutureOpDef,
    /// The inner type of the `Future` this op acts on.
    pub typ: Type,
}

impl MakeExtensionOp for FutureOp {
    fn op_id(&self) -> hugr::ops::OpName {
        self.op.opdef_id()
    }

    fn from_extension_op(ext_op: &ExtensionOp) -> Result<Self, OpLoadError>
    where
        Self: Sized,
    {
        FutureOpDef::from_def(ext_op.def())?.instantiate(ext_op.args())
    }

    fn type_args(&self) -> Vec<hugr::types::TypeArg> {
        vec![self.typ.clone().into()]
    }
}

impl MakeRegisteredOp for FutureOp {
    fn extension_id(&self) -> ExtensionId {
        EXTENSION_ID
    }

    fn extension_ref(&self) -> Weak<Extension> {
        Arc::downgrade(&EXTENSION)
    }
}

impl HasDef for FutureOp {
    type Def = FutureOpDef;
}

impl TryFrom<&OpType> for FutureOpDef {
    type Error = OpLoadError;

    fn try_from(value: &OpType) -> Result<Self, Self::Error> {
        Self::from_op(
            value
                .as_extension_op()
                .ok_or(OpLoadError::NotMember(value.to_string()))?,
        )
    }
}

/// An extension trait for [Dataflow] providing methods to add "tket.futures"
/// operations.
pub trait FutureOpBuilder: Dataflow {
    /// Add a "tket.futures.Read" op.
    fn add_read(&mut self, lifted: Wire, typ: Type) -> Result<[Wire; 1], BuildError> {
        Ok(self
            .add_dataflow_op(
                FutureOp {
                    op: FutureOpDef::Read,
                    typ,
                },
                [lifted],
            )?
            .outputs_arr())
    }

    /// Add a "tket.futures.Dup" op.
    fn add_dup(&mut self, lifted: Wire, typ: Type) -> Result<[Wire; 2], BuildError> {
        Ok(self
            .add_dataflow_op(
                FutureOp {
                    op: FutureOpDef::Dup,
                    typ,
                },
                [lifted],
            )?
            .outputs_arr())
    }

    /// Add a "tket.futures.Free" op.
    fn add_free(&mut self, lifted: Wire, typ: Type) -> Result<(), BuildError> {
        let op = self.add_dataflow_op(
            FutureOp {
                op: FutureOpDef::Free,
                typ,
            },
            [lifted],
        )?;
        assert!(op.outputs().len() == 0);
        Ok(())
    }
}

impl<D: Dataflow> FutureOpBuilder for D {}

#[cfg(test)]
pub(crate) mod test {

    use hugr::HugrView;
    use hugr::builder::{Dataflow, DataflowHugr, FunctionBuilder};
    use std::sync::Arc;
    use strum::IntoEnumIterator;

    use super::*;

    fn get_opdef(op: FutureOpDef) -> Option<&'static Arc<OpDef>> {
        EXTENSION.get_op(&op.opdef_id())
    }

    #[test]
    fn create_extension() {
        assert_eq!(EXTENSION.name(), &EXTENSION_ID);

        for o in FutureOpDef::iter() {
            assert_eq!(FutureOpDef::from_def(get_opdef(o).unwrap()), Ok(o));
        }
    }

    #[test]
    fn future_op_from_def() {
        let typ = Type::UNIT;

        assert_eq!(
            FutureOp {
                op: FutureOpDef::Free,
                typ: typ.clone()
            },
            FutureOpDef::Free.instantiate(&[typ.into()]).unwrap()
        )
    }

    #[test]
    fn circuit() {
        let t_param = TypeParam::from(TypeBound::Linear);
        let t = Type::new_var_use(0, TypeBound::Linear);
        let future_type = future_type(t.clone());

        let hugr = {
            let mut func_builder = FunctionBuilder::new(
                "circuit",
                PolyFuncType::new(vec![t_param], Signature::new(future_type, t.clone())),
            )
            .unwrap();
            let [future_w] = func_builder.input_wires_arr();
            let [future_w, lazy_dup_w] = func_builder.add_dup(future_w, t.clone()).unwrap();
            func_builder.add_free(future_w, t.clone()).unwrap();
            let [t_w] = func_builder.add_read(lazy_dup_w, t).unwrap();
            func_builder.finish_hugr_with_outputs([t_w]).unwrap()
        };
        hugr.validate().unwrap();
    }
}
