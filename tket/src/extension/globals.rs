#![allow(missing_docs)]

use std::sync::{Arc, Weak};

use hugr::{
    extension::{
        prelude::option_type,
        simple_op::{
            try_from_name, HasConcrete, MakeExtensionOp, MakeOpDef, MakeRegisteredOp, OpLoadError,
        },
        ExtensionId, SignatureError, SignatureFunc, Version,
    },
    ops::{ExtensionOp, OpName},
    types::{
        type_param::{TermTypeError, TypeParam},
        PolyFuncType, Signature, Type, TypeArg, TypeBound,
    },
    Extension,
};

/// The ID of the `tket.bool` extension.
pub const EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.globals");
/// The "tket.bool" extension version
pub const EXTENSION_VERSION: Version = Version::new(0, 1, 0);

lazy_static::lazy_static! {
    /// The "tket.bool" extension.
    pub static ref EXTENSION: Arc<Extension>  = {
        Extension::new_arc(EXTENSION_ID, EXTENSION_VERSION, |ext, ext_ref| {
            GlobalsOpDef::load_all_ops(ext, ext_ref).unwrap();
        })
    };

    pub static ref NAME_PARAM: TypeParam = TypeParam::StringType;
    pub static ref TYPE_PARAM: TypeParam = TypeParam::RuntimeType(TypeBound::Linear);
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
    strum::EnumIter,
    strum::IntoStaticStr,
    strum::EnumString,
)]
#[allow(missing_docs, non_camel_case_types)]
#[non_exhaustive]
pub enum GlobalsOpDef {
    swap,
}

impl MakeOpDef for GlobalsOpDef {
    fn opdef_id(&self) -> OpName {
        <&'static str>::from(self).into()
    }

    fn init_signature(&self, _extension_ref: &Weak<Extension>) -> SignatureFunc {
        match self {
            Self::swap => PolyFuncType::new(
                [NAME_PARAM.to_owned(), TYPE_PARAM.to_owned()],
                Signature::new_endo(Type::from(option_type(Type::new_var_use(
                    1,
                    TypeBound::Linear,
                )))),
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
        EXTENSION_ID
    }

    fn description(&self) -> String {
        match self {
            Self::swap => {
                "Swap the contents of the named global variable with the argument.".to_string()
            }
        }
    }

    fn extension_ref(&self) -> Weak<Extension> {
        Arc::downgrade(&EXTENSION)
    }
}

pub enum GlobalsOp {
    Swap { name: String, ty: Type },
}

impl MakeExtensionOp for GlobalsOp {
    fn op_id(&self) -> OpName {
        GlobalsOpDef::swap.opdef_id()
    }

    fn from_extension_op(ext_op: &ExtensionOp) -> Result<Self, OpLoadError>
    where
        Self: Sized,
    {
        GlobalsOpDef::from_def(ext_op.def())?.instantiate(ext_op.args())
    }

    fn type_args(&self) -> Vec<TypeArg> {
        match self {
            Self::Swap { name, ty } => {
                vec![TypeArg::String(name.clone()), TypeArg::Runtime(ty.clone())]
            }
        }
    }
}

impl HasConcrete for GlobalsOpDef {
    type Concrete = GlobalsOp;

    fn instantiate(&self, type_args: &[TypeArg]) -> Result<Self::Concrete, OpLoadError> {
        let [name_arg, ty_arg] = type_args else {
            Err(SignatureError::from(TermTypeError::WrongNumberArgs(
                type_args.len(),
                2,
            )))?
        };

        let Some(name) = name_arg.as_string() else {
            Err(SignatureError::from(TermTypeError::TypeMismatch {
                term: name_arg.clone().into(),
                type_: NAME_PARAM.to_owned().into(),
            }))?
        };

        let Some(ty) = ty_arg.as_runtime() else {
            Err(SignatureError::from(TermTypeError::TypeMismatch {
                term: ty_arg.clone().into(),
                type_: TYPE_PARAM.to_owned().into(),
            }))?
        };

        Ok(GlobalsOp::Swap { name, ty })
    }
}

impl MakeRegisteredOp for GlobalsOp {
    fn extension_id(&self) -> ExtensionId {
        EXTENSION_ID
    }

    fn extension_ref(&self) -> Arc<Extension> {
        EXTENSION.clone()
    }
}
