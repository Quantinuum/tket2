//! This module defines the `tket.globals` extension.

use std::sync::{Arc, Weak};

use hugr::{
    Extension,
    extension::{
        ExtensionId, SignatureError, SignatureFunc, Version,
        simple_op::{
            HasConcrete, MakeExtensionOp, MakeOpDef, MakeRegisteredOp, OpLoadError, try_from_name,
        },
    },
    ops::{ExtensionOp, OpName},
    types::{
        TypeArg, TypeBound,
        type_param::{TermKindError, TypeParam},
    },
};
use hugr_core::types::{FuncValueType, PolyFuncTypeRV, Type, TypeRowRV};

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

    /// The [TypeParam] specifying the name of a global variable.
    pub static ref NAME_PARAM: TypeParam = TypeParam::StringKind;
    /// The [TypeParam] specifying the runtime type of a global variable.
    pub static ref TYPE_PARAM: TypeParam = TypeParam::TypeKind(TypeBound::Linear);

    /// The [TypeParam] of various types and ops specifying the input signature of a function.
    pub static ref INPUTS_PARAM: TypeParam = TypeParam::new_list_kind(TypeBound::Linear);
    /// The [TypeParam] of various types and ops specifying the explicit output signature of a function.
    pub static ref OUTPUTS_PARAM: TypeParam = TypeParam::new_list_kind(TypeBound::Linear);
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
#[expect(non_camel_case_types)]
#[non_exhaustive]
/// Op definitions exposed by the `tket.globals` extension.
pub enum GlobalsOpDef {
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
            Self::with => {
                let global_ty = Type::new_var_use(1, TypeBound::Linear);
                let input_row = TypeRowRV::new_var_use(2, TypeBound::Linear);
                let output_row = TypeRowRV::new_var_use(3, TypeBound::Linear);
                let func_ty = TypeRowRV::from([Type::new_function(FuncValueType::new(
                    input_row.clone(),
                    output_row.clone(),
                ))]);
                PolyFuncTypeRV::new(
                    [
                        NAME_PARAM.to_owned(),
                        TYPE_PARAM.to_owned(),
                        INPUTS_PARAM.to_owned(),
                        OUTPUTS_PARAM.to_owned(),
                    ],
                    FuncValueType::new(
                        TypeRowRV::from([global_ty.clone()])
                            .concat(func_ty)
                            .concat(input_row),
                        TypeRowRV::from([global_ty]).concat(output_row),
                    ),
                )
                .into()
            }
            Self::map => {
                let global_ty = Type::new_var_use(1, TypeBound::Linear);
                let input_row = TypeRowRV::new_var_use(2, TypeBound::Linear);
                let output_row = TypeRowRV::new_var_use(3, TypeBound::Linear);
                let func_ty = TypeRowRV::from([Type::new_function(FuncValueType::new(
                    TypeRowRV::from([global_ty.clone()]).concat(input_row.clone()),
                    TypeRowRV::from([global_ty.clone()]).concat(output_row.clone()),
                ))]);
                PolyFuncTypeRV::new(
                    [
                        NAME_PARAM.to_owned(),
                        TYPE_PARAM.to_owned(),
                        INPUTS_PARAM.to_owned(),
                        OUTPUTS_PARAM.to_owned(),
                    ],
                    FuncValueType::new(func_ty.concat(input_row), output_row),
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
            Self::with => {
                "Run a function with the set contents of the global variable.".to_string()
            }
            Self::map => {
                "Map a function over the contents of the named global variable.".to_string()
            }
        }
    }

    fn extension_ref(&self) -> Weak<Extension> {
        Arc::downgrade(&EXTENSION)
    }
}

#[derive(Debug)]
/// Concrete instantiations of operations in the `tket.globals` extension.
pub enum GlobalsOp {
    /// Run a function with the contents of a named global variable.
    With {
        /// Global variable identifier.
        name: String,
        /// Runtime type argument of the global variable.
        ty_arg: TypeArg,
        /// Input row of the called function.
        inputs: TypeRowRV,
        /// Explicit output row of the called function.
        outputs: TypeRowRV,
    },
    /// Map a function over the contents of a named global variable.
    Map {
        /// Global variable identifier.
        name: String,
        /// Runtime type argument of the global variable.
        ty_arg: TypeArg,
        /// Input row of the mapped function.
        inputs: TypeRowRV,
        /// Explicit output row of the mapped function.
        outputs: TypeRowRV,
    },
}

impl MakeExtensionOp for GlobalsOp {
    fn op_id(&self) -> OpName {
        match self {
            Self::With { .. } => GlobalsOpDef::with.opdef_id(),
            Self::Map { .. } => GlobalsOpDef::map.opdef_id(),
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
        let expected_num_args = 4;

        let [name_arg, ty_arg, inputs_arg, outputs_arg] = type_args else {
            Err(SignatureError::from(TermKindError::WrongNumberArgs(
                type_args.len(),
                expected_num_args,
            )))?
        };

        let Some(name) = name_arg.as_string() else {
            Err(SignatureError::from(TermKindError::KindMismatch {
                term: name_arg.clone().into(),
                kind: NAME_PARAM.to_owned().into(),
            }))?
        };

        let Ok(inputs) = TypeRowRV::try_from(inputs_arg.clone()) else {
            Err(SignatureError::from(TermKindError::KindMismatch {
                term: Box::new(inputs_arg.clone()),
                kind: Box::new(INPUTS_PARAM.to_owned()),
            }))?
        };
        let Ok(outputs) = TypeRowRV::try_from(outputs_arg.clone()) else {
            Err(SignatureError::from(TermKindError::KindMismatch {
                term: Box::new(outputs_arg.clone()),
                kind: Box::new(OUTPUTS_PARAM.to_owned()),
            }))?
        };

        match self {
            Self::with => Ok(GlobalsOp::With {
                name,
                ty_arg: ty_arg.clone(),
                inputs,
                outputs,
            }),
            Self::map => Ok(GlobalsOp::Map {
                name,
                ty_arg: ty_arg.clone(),
                inputs,
                outputs,
            }),
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

#[cfg(test)]
mod test {
    use hugr::{
        HugrView,
        builder::{Dataflow, DataflowSubContainer, HugrBuilder, ModuleBuilder},
        extension::{prelude::qb_t, simple_op::MakeExtensionOp},
        types::Signature,
    };
    use strum::IntoEnumIterator;

    use super::*;

    #[test]
    fn create_extension() {
        assert_eq!(EXTENSION.name(), &EXTENSION_ID);

        for o in GlobalsOpDef::iter() {
            assert_eq!(
                GlobalsOpDef::from_def(EXTENSION.get_op(&o.op_id()).unwrap()),
                Ok(o)
            );
        }
    }

    #[rstest::rstest]
    #[case::wrong_num_args(
        &[],
        OpLoadError::InvalidArgs(SignatureError::TypeArgMismatch(
            TermKindError::WrongNumberArgs(0, 4)
        ))
    )]
    #[case::name_type_mismatch(
        &[
            TypeArg::BoundedNat(0),
            qb_t().into(),
            TypeRowRV::new().into(),
            TypeRowRV::new().into(),
        ],
        OpLoadError::InvalidArgs(SignatureError::TypeArgMismatch(TermKindError::KindMismatch {
            term: Box::new(TypeArg::BoundedNat(0)),
            kind: Box::new(TypeParam::StringKind),
        }))
    )]
    #[case::inputs_type_mismatch(
        &[
            TypeArg::String("g".to_string()),
            qb_t().into(),
            TypeArg::BoundedNat(1),
            TypeRowRV::new().into(),
        ],
        OpLoadError::InvalidArgs(SignatureError::TypeArgMismatch(TermKindError::KindMismatch {
            term: Box::new(TypeArg::BoundedNat(1)),
            kind: Box::new(TypeParam::ListKind(Box::new(TypeBound::Linear.into()))),
        }))
    )]
    #[case::outputs_type_mismatch(
        &[
            TypeArg::String("g".to_string()),
            qb_t().into(),
            TypeRowRV::new().into(),
            TypeArg::BoundedNat(2),
        ],
        OpLoadError::InvalidArgs(SignatureError::TypeArgMismatch(TermKindError::KindMismatch {
            term: Box::new(TypeArg::BoundedNat(2)),
            kind: Box::new(TypeParam::ListKind(Box::new(TypeBound::Linear.into()))),
        }))
    )]
    fn test_globals_op_instantiate_errors(
        #[case] type_args: &[TypeArg],
        #[case] error: OpLoadError,
    ) {
        assert_eq!(
            GlobalsOpDef::with.instantiate(type_args).unwrap_err(),
            error,
        );
    }

    #[test]
    fn test_with_map_op_builder() {
        let mut module_builder = ModuleBuilder::new();

        // Function to be called by `map` op
        let map_func = {
            let map_func_builder = module_builder
                .define_function("map_func", Signature::new(vec![qb_t()], vec![qb_t()]))
                .unwrap();
            let [global_state] = map_func_builder.input_wires_arr();
            map_func_builder
                .finish_with_outputs([global_state])
                .unwrap()
        };

        // Function to be called by `with` op
        let mut with_func_builder = module_builder
            .define_function("with_func", Signature::new(vec![], vec![]))
            .unwrap();
        let loaded_map_func = with_func_builder.load_func(map_func.handle(), &[]).unwrap();
        let map_op = GlobalsOp::Map {
            name: "my_global".to_string(),
            ty_arg: qb_t().into(),
            inputs: TypeRowRV::new(),
            outputs: TypeRowRV::new(),
        };
        with_func_builder
            .add_dataflow_op(map_op, [loaded_map_func])
            .unwrap();
        let with_func = with_func_builder.finish_with_outputs([]).unwrap();

        // Function under test
        let mut func_builder = module_builder
            .define_function(
                "with_op_builder",
                Signature::new(vec![qb_t()], vec![qb_t()]),
            )
            .unwrap();
        let [global_in] = func_builder.input_wires_arr();
        let loaded_func = func_builder.load_func(with_func.handle(), &[]).unwrap();
        let with_op = GlobalsOp::With {
            name: "my_global".to_string(),
            ty_arg: qb_t().into(),
            inputs: TypeRowRV::new(),
            outputs: TypeRowRV::new(),
        };
        let [global_out] = func_builder
            .add_dataflow_op(with_op, [global_in, loaded_func])
            .unwrap()
            .outputs_arr();
        func_builder.finish_with_outputs([global_out]).unwrap();

        let hugr = module_builder.finish_hugr().unwrap();
        hugr.validate().unwrap();
    }
}
