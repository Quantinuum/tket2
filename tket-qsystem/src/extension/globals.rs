#![allow(missing_docs)]

use std::sync::{Arc, Weak};

use hugr::{
    Extension,
    extension::{
        ExtensionId, SignatureError, SignatureFunc, Version,
        prelude::option_type,
        simple_op::{
            HasConcrete, MakeExtensionOp, MakeOpDef, MakeRegisteredOp, OpLoadError, try_from_name,
        },
    },
    ops::{ExtensionOp, OpName},
    types::{
        PolyFuncType, Signature, Type, TypeArg, TypeBound,
        type_param::{TermTypeError, TypeParam},
    },
};
use hugr_core::types::{FuncValueType, PolyFuncTypeRV, TypeRV, TypeRowRV};

/// The ID of the `tket.globals` extension.
pub const EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.globals");
/// The "tket.globals" extension version
pub const EXTENSION_VERSION: Version = Version::new(0, 1, 0);

lazy_static::lazy_static! {
    /// The "tket.globals" extension.
    pub static ref EXTENSION: Arc<Extension>  = {
        Extension::new_arc(EXTENSION_ID, EXTENSION_VERSION, |ext, ext_ref| {
            GlobalsOpDef::load_all_ops(ext, ext_ref).unwrap();
        })
    };

    pub static ref NAME_PARAM: TypeParam = TypeParam::StringType;
    pub static ref TYPE_PARAM: TypeParam = TypeParam::RuntimeType(TypeBound::Linear);

    /// The [TypeParam] of various types and ops specifying the input signature of a function.
    pub static ref INPUTS_PARAM: TypeParam =
    TypeParam::ListType(Box::new(TypeBound::Linear.into()));
    /// The [TypeParam] of various types and ops specifying the output signature of a function.
    pub static ref OUTPUTS_PARAM: TypeParam = TypeParam::ListType(Box::new(TypeBound::Linear.into()));
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
#[allow(non_camel_case_types)]
#[non_exhaustive]
pub enum GlobalsOpDef {
    /// Swap the contents of the named global variable with the argument.
    swap,
    /// Apply a function to the contents of the named global variable.
    with,
    /// Map a function over the contents of the named global variable.
    map,
}

impl MakeOpDef for GlobalsOpDef {
    fn opdef_id(&self) -> OpName {
        <&'static str>::from(self).into()
    }

    fn init_signature(&self, _extension_ref: &Weak<Extension>) -> SignatureFunc {
        match self {
            Self::swap => PolyFuncType::new(
                [NAME_PARAM.to_owned(), TYPE_PARAM.to_owned()],
                Signature::new_endo([Type::from(option_type([Type::new_var_use(
                    1,
                    TypeBound::Linear,
                )]))]),
            )
            .into(),
            Self::with => {
                let global_ty = TypeRV::new_var_use(1, TypeBound::Linear);
                let input_row = TypeRV::new_row_var_use(2, TypeBound::Linear);
                let output_row = TypeRV::new_row_var_use(3, TypeBound::Linear);

                let func_ty = TypeRV::new_function(FuncValueType::new(
                    [input_row.clone()],
                    [output_row.clone()],
                ));
                PolyFuncTypeRV::new(
                    [
                        NAME_PARAM.to_owned(),
                        TYPE_PARAM.to_owned(),
                        INPUTS_PARAM.to_owned(),
                        OUTPUTS_PARAM.to_owned(),
                    ],
                    FuncValueType::new(
                        [global_ty.clone(), func_ty, input_row],
                        [global_ty.clone(), output_row],
                    ),
                )
                .into()
            }
            Self::map => {
                let global_ty = TypeRV::new_var_use(1, TypeBound::Linear);
                let input_row = TypeRV::new_row_var_use(2, TypeBound::Linear);
                let output_row = TypeRV::new_row_var_use(3, TypeBound::Linear);
                let func_ty = TypeRV::new_function(FuncValueType::new(
                    [global_ty.clone(), input_row.clone().into()],
                    [global_ty.clone(), output_row.clone().into()],
                ));
                PolyFuncTypeRV::new(
                    [
                        NAME_PARAM.to_owned(),
                        TYPE_PARAM.to_owned(),
                        INPUTS_PARAM.to_owned(),
                        OUTPUTS_PARAM.to_owned(),
                    ],
                    FuncValueType::new([func_ty, input_row], [output_row]),
                )
                .into()
            }
        }
    }

    fn from_def(op_def: &hugr::extension::OpDef) -> Result<Self, OpLoadError> {
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
            Self::with => "".to_string(),
            Self::map => "".to_string(),
        }
    }

    fn extension_ref(&self) -> Weak<Extension> {
        Arc::downgrade(&EXTENSION)
    }
}

pub enum GlobalsOp {
    Swap {
        name: String,
        ty: Type,
    },
    With {
        name: String,
        ty_arg: TypeArg,
        inputs: TypeRowRV,
        outputs: TypeRowRV,
    },
    Map {
        name: String,
        ty_arg: TypeArg,
        inputs: TypeRowRV,
        outputs: TypeRowRV,
    },
}

impl MakeExtensionOp for GlobalsOp {
    fn op_id(&self) -> OpName {
        match self {
            Self::Swap { .. } => GlobalsOpDef::swap.opdef_id(),
            Self::With { .. } => GlobalsOpDef::with.opdef_id(),
            Self::Map { .. } => GlobalsOpDef::with.opdef_id(),
        }
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
            Self::With {
                name,
                ty_arg,
                inputs,
                outputs,
            } => {
                vec![
                    TypeArg::String(name.clone()),
                    ty_arg.clone(),
                    inputs.clone().into(),
                    outputs.clone().into(),
                ]
            }
            Self::Map {
                name,
                ty_arg,
                inputs,
                outputs,
            } => {
                vec![
                    TypeArg::String(name.clone()),
                    ty_arg.clone(),
                    inputs.clone().into(),
                    outputs.clone().into(),
                ]
            }
        }
    }
}

impl HasConcrete for GlobalsOpDef {
    type Concrete = GlobalsOp;

    fn instantiate(&self, type_args: &[TypeArg]) -> Result<Self::Concrete, OpLoadError> {
        let expected_num_args = match self {
            Self::swap => 2,
            Self::with => 4,
            Self::map => 4,
        };

        let [name_arg, ty_arg] = &type_args[..2] else {
            Err(SignatureError::from(TermTypeError::WrongNumberArgs(
                type_args.len(),
                expected_num_args,
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

        match self {
            Self::swap => Ok(GlobalsOp::Swap { name, ty }),
            Self::with => {
                let Ok(inputs) = TypeRowRV::try_from(type_args[2].clone()) else {
                    Err(SignatureError::from(TermTypeError::TypeMismatch {
                        term: Box::new(type_args[2].clone()),
                        type_: Box::new(INPUTS_PARAM.to_owned()),
                    }))?
                };
                let Ok(outputs) = TypeRowRV::try_from(type_args[3].clone()) else {
                    Err(SignatureError::from(TermTypeError::TypeMismatch {
                        term: Box::new(type_args[3].clone()),
                        type_: Box::new(OUTPUTS_PARAM.to_owned()),
                    }))?
                };

                Ok(GlobalsOp::With {
                    name,
                    ty_arg: ty_arg.clone(),
                    inputs,
                    outputs,
                })
            }
            Self::map => {
                let Ok(inputs) = TypeRowRV::try_from(type_args[2].clone()) else {
                    Err(SignatureError::from(TermTypeError::TypeMismatch {
                        term: Box::new(type_args[2].clone()),
                        type_: Box::new(INPUTS_PARAM.to_owned()),
                    }))?
                };
                let Ok(outputs) = TypeRowRV::try_from(type_args[3].clone()) else {
                    Err(SignatureError::from(TermTypeError::TypeMismatch {
                        term: Box::new(type_args[3].clone()),
                        type_: Box::new(OUTPUTS_PARAM.to_owned()),
                    }))?
                };

                Ok(GlobalsOp::Map {
                    name,
                    ty_arg: ty_arg.clone(),
                    inputs,
                    outputs,
                })
            }
        }
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
