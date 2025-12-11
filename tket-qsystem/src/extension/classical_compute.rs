//! This module lays out a framework for interacting with classical compute
//! devices in a program - see the [compute/wasm.rs] or [compute/gpu.rs] for
//! details.
use hugr::{
    extension::{Extension, ExtensionBuildError, ExtensionId, TypeDefBound},
    types::{CustomType, Type, TypeBound, TypeRow, TypeRowRV, type_param::TypeParam},
};
use lazy_static::lazy_static;
use smol_str::SmolStr;
use std::marker::PhantomData;
use std::sync::Weak;

use super::utils::row_to_arg;

lazy_static! {
    /// The name of the `module` type.
    pub static ref MODULE_TYPE_NAME: SmolStr = SmolStr::new_inline("module");
    /// The name of the `context` type.
    pub static ref CONTEXT_TYPE_NAME: SmolStr = SmolStr::new_inline("context");
    /// The name of the `func` type.
    pub static ref FUNC_TYPE_NAME: SmolStr = SmolStr::new_inline("func");

    /// The name of the `result` type.
    pub static ref RESULT_TYPE_NAME: SmolStr = SmolStr::new_inline("result");

    /// The [TypeParam] of `lookup_by_id` specifying the id of the function.
    pub static ref ID_PARAM: TypeParam = TypeParam::max_nat_type();
    /// The [TypeParam] of `lookup_by_name` specifying the name of the function.
    pub static ref NAME_PARAM: TypeParam = TypeParam::StringType;
    /// The [TypeParam] of various types and ops specifying the input signature of a function.
    pub static ref INPUTS_PARAM: TypeParam =
        TypeParam::ListType(Box::new(TypeBound::Linear.into()));
    /// The [TypeParam] of various types and ops specifying the output signature of a function.
    pub static ref OUTPUTS_PARAM: TypeParam = TypeParam::ListType(Box::new(TypeBound::Linear.into()));
}

pub(crate) fn add_compute_type_defs(
    extension: &mut Extension,
    extension_ref: &Weak<Extension>,
) -> Result<(), ExtensionBuildError> {
    extension.add_type(
        MODULE_TYPE_NAME.to_owned(),
        vec![],
        format!("{} module", extension.name()),
        TypeDefBound::copyable(),
        extension_ref,
    )?;
    extension.add_type(
        CONTEXT_TYPE_NAME.to_owned(),
        vec![],
        format!("{} context", extension.name()),
        TypeDefBound::any(),
        extension_ref,
    )?;
    extension.add_type(
        FUNC_TYPE_NAME.to_owned(),
        vec![INPUTS_PARAM.to_owned(), OUTPUTS_PARAM.to_owned()],
        format!("{} func", extension.name()),
        TypeDefBound::copyable(),
        extension_ref,
    )?;
    extension.add_type(
        RESULT_TYPE_NAME.to_owned(),
        vec![OUTPUTS_PARAM.to_owned()],
        format!("{} result", extension.name()),
        TypeDefBound::any(),
        extension_ref,
    )?;
    Ok(())
}

#[derive(Clone, Debug, Eq, PartialEq, Hash)]
/// An enum of types defined by compute extensions.
///
/// Instances will provide `impl From<ComputeType<T>>` for [CustomType] and [Type], and `impl
/// TryFrom<CustomType>` and `impl TryFrom<CustomType>` for [`ComputeType<T>`].
pub enum ComputeType<T> {
    /// `module`
    Module,
    /// `context`
    Context,
    /// `func`
    Func {
        /// The input signature of the function. Note that row variables are
        /// allowed.
        inputs: TypeRowRV,
        /// The output signature of the function. Note that row variables are
        /// allowed.
        outputs: TypeRowRV,
    },
    /// `result`
    Result {
        /// The output signature of the function. Note that row variables are
        /// allowed.
        outputs: TypeRowRV,
    },
    /// Non-constructible variant.
    _Unreachable(std::convert::Infallible, PhantomData<T>),
}

impl<T> ComputeType<T> {
    /// Construct a new `func` type.
    pub fn new_func(inputs: impl Into<TypeRowRV>, outputs: impl Into<TypeRowRV>) -> Self {
        Self::Func {
            inputs: inputs.into(),
            outputs: outputs.into(),
        }
    }

    pub(crate) fn get_type(
        &self,
        extension_id: ExtensionId,
        extension_ref: &Weak<Extension>,
    ) -> Type {
        self.custom_type(extension_id, extension_ref).into()
    }

    pub(crate) fn func_custom_type(
        inputs: impl Into<TypeRowRV>,
        outputs: impl Into<TypeRowRV>,
        extension_id: ExtensionId,
        extension_ref: &Weak<Extension>,
    ) -> CustomType {
        CustomType::new(
            FUNC_TYPE_NAME.to_owned(),
            [row_to_arg(inputs), row_to_arg(outputs)],
            extension_id,
            TypeBound::Copyable,
            extension_ref,
        )
    }

    pub(crate) fn result_custom_type(
        outputs: impl Into<TypeRowRV>,
        extension_id: ExtensionId,
        extension_ref: &Weak<Extension>,
    ) -> CustomType {
        CustomType::new(
            RESULT_TYPE_NAME.to_owned(),
            [row_to_arg(outputs)],
            extension_id,
            TypeBound::Linear,
            extension_ref,
        )
    }

    pub(crate) fn custom_type(
        &self,
        extension_id: ExtensionId,
        extension_ref: &Weak<Extension>,
    ) -> CustomType {
        match self {
            Self::Module => CustomType::new(
                MODULE_TYPE_NAME.to_owned(),
                [],
                extension_id,
                TypeBound::Copyable,
                extension_ref,
            ),
            Self::Context => CustomType::new(
                CONTEXT_TYPE_NAME.to_owned(),
                [],
                extension_id,
                TypeBound::Linear,
                extension_ref,
            ),
            Self::Func { inputs, outputs } => {
                Self::func_custom_type(inputs.clone(), outputs.clone(), extension_id, extension_ref)
            }
            Self::Result { outputs } => {
                Self::result_custom_type(outputs.clone(), extension_id, extension_ref)
            }
            Self::_Unreachable(x, _) => match *x {},
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
/// Concrete instantiation(i.e. with type args applied) of op_defs defined in compute extensions
pub enum ComputeOp<T> {
    /// A `get_context` op.
    GetContext,
    /// A `dispose_context` op.
    DisposeContext,
    /// A `lookup_by_id` op.
    LookupById {
        /// The name of the function to be looked up.
        id: u64,
        /// The input signature of the function to be looked up.
        /// Note that row variables are allowed here.
        inputs: TypeRowRV,
        /// The output signature of the function to be looked up.
        /// Note that row variables are allowed here.
        outputs: TypeRowRV,
    },
    /// A `lookup_by_name` op.
    LookupByName {
        /// The name of the function to be looked up.
        name: String,
        /// The input signature of the function to be looked up.
        /// Note that row variables are allowed here.
        inputs: TypeRowRV,
        /// The output signature of the function to be looked up.
        /// Note that row variables are allowed here.
        outputs: TypeRowRV,
    },
    /// A `call` op.
    Call {
        /// The input signature of the function to be called
        /// Note that row variables are not allowed here.
        inputs: TypeRow,
        /// The output signature of the function to be called
        /// Note that row variables are not allowed here.
        outputs: TypeRow,
    },
    /// A `read_result` op.
    ReadResult {
        /// The output signature of the function to be called
        /// Note that row variables are not allowed here.
        outputs: TypeRow,
    },
    /// Non-constructible variant.
    _Unreachable(std::convert::Infallible, PhantomData<T>),
}

macro_rules! compute_opdef {
    ($ext_id:expr, $ext:ty, $opdef:ident) => {
        use serde::{Deserialize, Serialize};
        use strum::{EnumIter, EnumString, IntoStaticStr};

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
        #[expect(non_camel_case_types)]
        #[non_exhaustive]
        /// Simple enum of ops defined by the this extension.
        pub enum $opdef {
            /// Retrieve the context for the given id.
            get_context,
            /// Dispose of the context type.
            dispose_context,
            /// Lookup a ComputeType::Func with the given signature.
            lookup_by_id,
            /// Lookup a ComputeType::Func with the given name and signature.
            lookup_by_name,
            /// Call a ComputeType::Func.
            call,
            /// Read a future result type from a `call`.
            read_result,
        }

        impl From<ComputeType<$ext>> for CustomType {
            fn from(value: ComputeType<$ext>) -> Self {
                value.custom_type($ext_id, &EXTENSION_REF)
            }
        }

        impl From<ComputeType<$ext>> for Type {
            fn from(value: ComputeType<$ext>) -> Self {
                value.get_type($ext_id, &EXTENSION_REF)
            }
        }

        impl TryFrom<Type> for ComputeType<$ext> {
            type Error = ();

            fn try_from(value: Type) -> Result<Self, Self::Error> {
                let TypeEnum::Extension(custom_type) = value.as_type_enum() else {
                    Err(())?
                };

                custom_type.to_owned().try_into().map_err(|_| ())
            }
        }

        impl MakeOpDef for $opdef {
            fn opdef_id(&self) -> hugr::ops::OpName {
                <&'static str>::from(self).into()
            }

            fn init_signature(&self, extension_ref: &Weak<Extension>) -> SignatureFunc {
                let context_type =
                    ComputeType::<$ext>::Context.get_type(self.extension(), extension_ref);
                let module_type =
                    ComputeType::<$ext>::Module.get_type(self.extension(), extension_ref);
                match self {
                    // [usize] -> [Context]
                    Self::get_context => Signature::new(
                        usize_t(),
                        Type::from(ComputeOp::<$ext>::get_context_return_type(
                            self.extension(),
                            extension_ref,
                        )),
                    )
                    .into(),
                    // [Context] -> []
                    Self::dispose_context => Signature::new(context_type, type_row![]).into(),
                    // <id: usize, inputs: TypeRow, outputs: TypeRow> [Module] -> [ComputeType::Func { inputs, outputs }]
                    Self::lookup_by_id => {
                        let inputs = TypeRV::new_row_var_use(1, TypeBound::Copyable);
                        let outputs = TypeRV::new_row_var_use(2, TypeBound::Copyable);

                        let func_type = ComputeType::<$ext>::func_custom_type(
                            inputs,
                            outputs,
                            self.extension(),
                            extension_ref,
                        )
                        .into();
                        PolyFuncTypeRV::new(
                            [
                                ID_PARAM.to_owned(),
                                INPUTS_PARAM.to_owned(),
                                OUTPUTS_PARAM.to_owned(),
                            ],
                            Signature::new(vec![module_type], vec![func_type]),
                        )
                        .into()
                    }
                    // <name: String, inputs: TypeRow, outputs: TypeRow> [Module] -> [ComputeType::Func { inputs, outputs }]
                    Self::lookup_by_name => {
                        let inputs = TypeRV::new_row_var_use(1, TypeBound::Copyable);
                        let outputs = TypeRV::new_row_var_use(2, TypeBound::Copyable);

                        let func_type = ComputeType::<$ext>::func_custom_type(
                            inputs,
                            outputs,
                            self.extension(),
                            extension_ref,
                        )
                        .into();
                        PolyFuncTypeRV::new(
                            [
                                NAME_PARAM.to_owned(),
                                INPUTS_PARAM.to_owned(),
                                OUTPUTS_PARAM.to_owned(),
                            ],
                            Signature::new(vec![module_type], vec![func_type]),
                        )
                        .into()
                    }
                    // <inputs: TypeRow, outputs: TypeRow> [Context, ComputeType::Func { inputs, outputs }, inputs] -> [Context, future<tuple<outputs>>>]
                    Self::call => {
                        let context_type: TypeRV = context_type.into();
                        let inputs = TypeRV::new_row_var_use(0, TypeBound::Copyable);
                        let outputs = TypeRV::new_row_var_use(1, TypeBound::Copyable);
                        let func_type = Type::new_extension(ComputeType::<$ext>::func_custom_type(
                            inputs.clone(),
                            outputs.clone(),
                            self.extension(),
                            extension_ref,
                        ));
                        let result_type =
                            TypeRV::new_extension(ComputeType::<$ext>::result_custom_type(
                                outputs,
                                self.extension(),
                                extension_ref,
                            ));

                        PolyFuncTypeRV::new(
                            [INPUTS_PARAM.to_owned(), OUTPUTS_PARAM.to_owned()],
                            FuncValueType::new(
                                vec![context_type.clone(), func_type.into(), inputs],
                                vec![result_type],
                            ),
                        )
                        .into()
                    }
                    Self::read_result => {
                        let context_type: TypeRV = context_type.into();
                        let outputs = TypeRV::new_row_var_use(0, TypeBound::Copyable);
                        let result_type =
                            TypeRV::new_extension(ComputeType::<$ext>::result_custom_type(
                                outputs.clone(),
                                self.extension(),
                                extension_ref,
                            ));
                        PolyFuncTypeRV::new(
                            [OUTPUTS_PARAM.to_owned()],
                            FuncValueType::new(vec![result_type], vec![context_type, outputs]),
                        )
                        .into()
                    }
                }
            }

            fn extension_ref(&self) -> Weak<Extension> {
                EXTENSION_REF.to_owned()
            }

            fn from_def(op_def: &hugr::extension::OpDef) -> Result<Self, OpLoadError>
            where
                Self: Sized,
            {
                try_from_name(op_def.name().as_ref(), op_def.extension_id())
            }

            fn extension(&self) -> ExtensionId {
                EXTENSION_ID
            }
        }

        impl HasConcrete for $opdef {
            type Concrete = ComputeOp<$ext>;

            fn instantiate(&self, type_args: &[TypeArg]) -> Result<Self::Concrete, OpLoadError> {
                match self {
                    Self::get_context => {
                        let [] = type_args else {
                            Err(SignatureError::from(TermTypeError::WrongNumberArgs(
                                type_args.len(),
                                0,
                            )))?
                        };
                        Ok(Self::Concrete::GetContext)
                    }
                    Self::dispose_context => {
                        let [] = type_args else {
                            Err(SignatureError::from(TermTypeError::WrongNumberArgs(
                                type_args.len(),
                                0,
                            )))?
                        };
                        Ok(Self::Concrete::DisposeContext)
                    }
                    // <usize,in_row,out_row> [] -> []
                    Self::lookup_by_id => {
                        let Some([id_arg, inputs_arg, outputs_arg]): Option<[_; 3]> =
                            type_args.to_vec().try_into().ok()
                        else {
                            Err(SignatureError::from(TermTypeError::WrongNumberArgs(
                                type_args.len(),
                                3,
                            )))?
                        };

                        let Some(id) = id_arg.as_nat() else {
                            Err(SignatureError::from(TermTypeError::TypeMismatch {
                                term: Box::new(id_arg),
                                type_: Box::new(ID_PARAM.to_owned()),
                            }))?
                        };

                        let Ok(inputs) = TypeRowRV::try_from(inputs_arg.clone()) else {
                            Err(SignatureError::from(TermTypeError::TypeMismatch {
                                term: Box::new(inputs_arg),
                                type_: Box::new(INPUTS_PARAM.to_owned()),
                            }))?
                        };

                        let Ok(outputs) = TypeRowRV::try_from(outputs_arg.clone()) else {
                            Err(SignatureError::from(TermTypeError::TypeMismatch {
                                term: Box::new(outputs_arg),
                                type_: Box::new(OUTPUTS_PARAM.to_owned()),
                            }))?
                        };
                        Ok(Self::Concrete::LookupById {
                            id,
                            inputs,
                            outputs,
                        })
                    }
                    // <String,in_row,out_row> [] -> []
                    Self::lookup_by_name => {
                        let Some([name_arg, inputs_arg, outputs_arg]): Option<[_; 3]> =
                            type_args.to_vec().try_into().ok()
                        else {
                            Err(SignatureError::from(TermTypeError::WrongNumberArgs(
                                type_args.len(),
                                3,
                            )))?
                        };

                        let Some(name) = name_arg.as_string() else {
                            Err(SignatureError::from(TermTypeError::TypeMismatch {
                                term: Box::new(name_arg),
                                type_: Box::new(NAME_PARAM.to_owned()),
                            }))?
                        };

                        let Ok(inputs) = TypeRowRV::try_from(inputs_arg.clone()) else {
                            Err(SignatureError::from(TermTypeError::TypeMismatch {
                                term: Box::new(inputs_arg),
                                type_: Box::new(INPUTS_PARAM.to_owned()),
                            }))?
                        };

                        let Ok(outputs) = TypeRowRV::try_from(outputs_arg.clone()) else {
                            Err(SignatureError::from(TermTypeError::TypeMismatch {
                                term: Box::new(outputs_arg),
                                type_: Box::new(OUTPUTS_PARAM.to_owned()),
                            }))?
                        };
                        Ok(Self::Concrete::LookupByName {
                            name,
                            inputs,
                            outputs,
                        })
                    }
                    Self::call => {
                        let Some([inputs_arg, outputs_arg]): Option<[_; 2]> =
                            type_args.to_vec().try_into().ok()
                        else {
                            Err(SignatureError::from(TermTypeError::WrongNumberArgs(
                                type_args.len(),
                                2,
                            )))?
                        };

                        let Ok(inputs) = TypeRowRV::try_from(inputs_arg.clone()) else {
                            Err(SignatureError::from(TermTypeError::TypeMismatch {
                                term: Box::new(inputs_arg),
                                type_: Box::new(INPUTS_PARAM.to_owned()),
                            }))?
                        };

                        let Ok(outputs) = TypeRowRV::try_from(outputs_arg.clone()) else {
                            Err(SignatureError::from(TermTypeError::TypeMismatch {
                                term: Box::new(outputs_arg),
                                type_: Box::new(OUTPUTS_PARAM.to_owned()),
                            }))?
                        };

                        Ok(Self::Concrete::Call {
                            inputs: inputs.try_into()?,
                            outputs: outputs.try_into()?,
                        })
                    }
                    Self::read_result => {
                        let Some([outputs_arg]): Option<[_; 1]> =
                            type_args.to_vec().try_into().ok()
                        else {
                            Err(SignatureError::from(TermTypeError::WrongNumberArgs(
                                type_args.len(),
                                1,
                            )))?
                        };

                        let Ok(outputs) = TypeRowRV::try_from(outputs_arg.clone()) else {
                            Err(SignatureError::from(TermTypeError::TypeMismatch {
                                term: Box::new(outputs_arg),
                                type_: Box::new(OUTPUTS_PARAM.to_owned()),
                            }))?
                        };
                        Ok(Self::Concrete::ReadResult {
                            outputs: outputs.try_into()?,
                        })
                    }
                }
            }
        }
        impl TryFrom<&OpType> for $opdef {
            type Error = OpLoadError;

            fn try_from(value: &OpType) -> Result<Self, Self::Error> {
                Self::from_op(
                    value
                        .as_extension_op()
                        .ok_or(OpLoadError::NotMember(value.to_string()))?,
                )
            }
        }

        impl ComputeOp<$ext> {
            fn compute_op_def(&self) -> $opdef {
                match self {
                    Self::GetContext => <$opdef>::get_context,
                    Self::DisposeContext => <$opdef>::dispose_context,
                    Self::LookupById { .. } => <$opdef>::lookup_by_id,
                    Self::LookupByName { .. } => <$opdef>::lookup_by_name,
                    Self::Call { .. } => <$opdef>::call,
                    Self::ReadResult { .. } => <$opdef>::read_result,
                    Self::_Unreachable(x, _) => match *x {},
                }
            }

            fn get_context_return_type(
                extension_id: ExtensionId,
                extension_ref: &Weak<Extension>,
            ) -> SumType {
                option_type(ComputeType::<$ext>::Context.get_type(extension_id, extension_ref))
            }
        }

        impl HasDef for ComputeOp<$ext> {
            type Def = $opdef;
        }

        impl MakeExtensionOp for ComputeOp<$ext> {
            fn op_id(&self) -> OpName {
                self.compute_op_def().opdef_id()
            }

            fn from_extension_op(ext_op: &ExtensionOp) -> Result<Self, OpLoadError>
            where
                Self: Sized,
            {
                <$opdef>::from_op(ext_op)?.instantiate(ext_op.args())
            }

            fn type_args(&self) -> Vec<TypeArg> {
                match self {
                    Self::GetContext => vec![],
                    Self::DisposeContext => vec![],
                    Self::LookupById {
                        id,
                        inputs,
                        outputs,
                    } => {
                        let inputs = TypeArg::from(inputs.clone());
                        let outputs = TypeArg::from(outputs.clone());
                        vec![TypeArg::BoundedNat(*id), inputs, outputs]
                    }
                    Self::LookupByName {
                        name,
                        inputs,
                        outputs,
                    } => {
                        let inputs = TypeArg::from(inputs.clone());
                        let outputs = TypeArg::from(outputs.clone());
                        vec![name.clone().into(), inputs, outputs]
                    }
                    Self::Call { inputs, outputs } => {
                        let inputs = TypeArg::from(inputs.clone());
                        let outputs = TypeArg::from(outputs.clone());
                        vec![inputs, outputs]
                    }
                    Self::ReadResult { outputs } => vec![outputs.clone().into()],
                    Self::_Unreachable(x, _) => match *x {},
                }
            }
        }
    };
}

macro_rules! compute_builder {
    ($ext:ty, $builder_name:ident) => {
        /// An extension trait for [Dataflow] providing methods to add extension
        /// operations and constants.
        pub trait $builder_name: Dataflow {
            /// Add a `get_context` op.
            fn add_get_context(&mut self, id: Wire) -> Result<Wire, BuildError> {
                let op = self.add_dataflow_op(ComputeOp::<$ext>::GetContext, vec![id])?;
                Ok(op.out_wire(0))
            }

            /// Add a `dispose_context` op.
            fn add_dispose_context(&mut self, id: Wire) -> Result<(), BuildError> {
                let _ = self.add_dataflow_op(ComputeOp::<$ext>::DisposeContext, vec![id])?;
                Ok(())
            }

            /// Add a `lookup_by_id` op.
            fn add_lookup_by_id(
                &mut self,
                id: impl Into<u64>,
                inputs: impl Into<TypeRowRV>,
                outputs: impl Into<TypeRowRV>,
                module: Wire,
            ) -> Result<Wire, BuildError> {
                Ok(self
                    .add_dataflow_op(
                        ComputeOp::<$ext>::LookupById {
                            id: id.into(),
                            inputs: inputs.into(),
                            outputs: outputs.into(),
                        },
                        [module],
                    )?
                    .out_wire(0))
            }

            /// Add a `lookup_by_name` op.
            fn add_lookup_by_name(
                &mut self,
                name: impl Into<String>,
                inputs: impl Into<TypeRowRV>,
                outputs: impl Into<TypeRowRV>,
                module: Wire,
            ) -> Result<Wire, BuildError> {
                Ok(self
                    .add_dataflow_op(
                        ComputeOp::<$ext>::LookupByName {
                            name: name.into(),
                            inputs: inputs.into(),
                            outputs: outputs.into(),
                        },
                        [module],
                    )?
                    .out_wire(0))
            }

            /// Add a `call` op.
            ///
            /// We infer the signature from the type of the `func` wire.
            fn add_call(
                &mut self,
                context: Wire,
                func: Wire,
                inputs: impl IntoIterator<Item = Wire>,
            ) -> Result<Wire, BuildError> {
                let func_wire_type = self.get_wire_type(func)?;
                let Some(ComputeType::<$ext>::Func {
                    inputs: in_types,
                    outputs: out_types,
                }) = func_wire_type.clone().try_into().ok()
                else {
                    // TODO Add an Error variant to BuildError for: Input wire has wrong type
                    panic!("func wire is not a func type: {func_wire_type}")
                };
                let (in_types, out_types) =
                    (TypeRow::try_from(in_types)?, TypeRow::try_from(out_types)?);

                Ok(self
                    .add_dataflow_op(
                        ComputeOp::<$ext>::Call {
                            inputs: in_types,
                            outputs: out_types,
                        },
                        [context, func].into_iter().chain(inputs),
                    )?
                    .out_wire(0))
            }

            /// Add a `read_result` op.
            fn add_read_result(&mut self, result: Wire) -> Result<(Wire, Vec<Wire>), BuildError> {
                let result_wire_type = self.get_wire_type(result)?;
                let Some(ComputeType::<$ext>::Result { outputs }) =
                    self.get_wire_type(result)?.clone().try_into().ok()
                else {
                    // TODO Add an Error variant to BuildError for: Input wire has wrong type
                    panic!("result wire is not a result type: {result_wire_type}")
                };
                let outputs = TypeRow::try_from(outputs)?;

                let op =
                    self.add_dataflow_op(ComputeOp::<$ext>::ReadResult { outputs }, [result])?;
                let context = op.out_wire(0);
                let results = op.outputs().skip(1).collect_vec();
                Ok((context, results))
            }
        }

        impl<T: Dataflow> $builder_name for T {}
    };
}

pub mod gpu;
pub mod wasm;
