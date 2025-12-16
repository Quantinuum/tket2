//! This module defines the `tket.gpu` Hugr extension used to model calling
//! into a library loaded on a GPU.
//!
//! 'tket.gpu' provides the following types:
//!  - `tket.gpu.module`: A WebAssembly module.
//!  - `tket.gpu.context`: A WebAssembly context.
//!  - `tket.gpu.func`: A WebAssembly function.
//!  - `tket.gpu.result`: A WebAssembly result.
//!
//!  Each of which can be constructed in rust via [GpuType].
//!
//!  `tket.gpu.context` is a linear type that orders runtime effects. It is
//!  obtained via the `tket.gpu.get_context` operation and destroyed via
//!  `tket.gpu.dispose_context`.
//!
//!  A `tket.gpu.module` is obtained by loading a `ConstGpuModule` constant.
//!  We assume that all modules are available in all contexts.
//!
//!  `tket.gpu.get_context` takes a `prelude.usize`, allowing multiple independent
//!  contexts to exist simultaneously. `get_context` is fallible, returning
//!  `None` if the specified context has already been obtained via an earlier
//!  `get_context` op and not `dispose_context`ed.
//!
//!  `tket.gpu.func` is a type representing a handle to a function in a
//!  `tket.gpu.module`. It carries type args defining its signature, but not
//!  its name.
//!
//!  A `tket.gpu.func` is obtained from either a `tket.gpu.lookup_by_id` or
//!  `tket.gpu.lookup_by_name`op, which takes a compile-time identifier (name or id)
//!  and signature, and a runtime module.
//!  TODO Likely the module should be compile time here, but I think we need
//!  extension-op-static-edges to do this properly.
//!
//!  `tket.gpu.func`s are called via the `tket.gpu.call` op. This op takes:
//!   - a `tket.gpu.context` identifying where the call will execute;
//!   - a `tket.gpu.func` identifying the function to call;
//!   - Input arguments as specified by the type of the `tket.gpu.func`.
//!
//!   It returns a `tket.gpu.result`, which must then be read by using
//!   `tket.gpu.read_result`, which yields a `tket.gpu.context` tupled with
//!   the output values of the `tket.gpu.func`.
//!
//!   We provide [GpuType] to assist in constructing and interpreting [Type]s.
//!
//!   We provide [GpuOp] to assist in constructing and interpreting [ExtensionOp]s.
//!
//!   We provide [GpuOpBuilder] to assist in building [hugr::Hugr]s using the
//!   `tket.gpu` extension.

use std::sync::{Arc, Weak};

use hugr::{
    Extension, Wire,
    builder::{BuildError, Dataflow},
    extension::{
        ExtensionId, ExtensionRegistry, PRELUDE, SignatureError, SignatureFunc, Version,
        prelude::{option_type, usize_t},
        simple_op::{
            HasConcrete, HasDef, MakeExtensionOp, MakeOpDef, MakeRegisteredOp, OpLoadError,
            try_from_name,
        },
    },
    ops::{
        ExtensionOp, OpName, OpType,
        constant::{CustomConst, ValueName, downcast_equal_consts},
    },
    type_row,
    types::{
        CustomType, FuncValueType, PolyFuncTypeRV, Signature, SumType, Type, TypeArg, TypeBound,
        TypeEnum, TypeRV, TypeRow, TypeRowRV, type_param::TermTypeError,
    },
};
use itertools::Itertools as _;
use lazy_static::lazy_static;
use smol_str::format_smolstr;

use super::{ComputeOp, ComputeType, add_compute_type_defs};

pub use super::{
    CONTEXT_TYPE_NAME, FUNC_TYPE_NAME, ID_PARAM, INPUTS_PARAM, MODULE_TYPE_NAME, NAME_PARAM,
    OUTPUTS_PARAM, RESULT_TYPE_NAME,
};

#[derive(Clone, Debug, Eq, PartialEq)]
/// The type used to parameterise Compute operations for the gpu extension.
pub struct GpuExtension;

/// Concrete instantiation(i.e. with type args applied) of a "tket.gpu" operation.
pub type GpuOp = ComputeOp<GpuExtension>;
/// Concrete types defined by the `tket.gpu` extension.
pub type GpuType = ComputeType<GpuExtension>;

/// The "tket.gpu" extension id.
pub const EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.gpu");
/// The "tket.gpu" extension version.
pub const EXTENSION_VERSION: Version = Version::new(0, 1, 1);

lazy_static! {
    /// The `tket.gpu` extension.
    pub static ref EXTENSION: Arc<Extension> =
        Extension::new_arc(EXTENSION_ID, EXTENSION_VERSION, |ext, ext_ref| {
        add_compute_type_defs(ext, ext_ref).unwrap();
        GpuOpDef::load_all_ops(ext, ext_ref, ).unwrap();
    });

    /// A [Weak] reference to the `tket.gpu` op.
    pub static ref EXTENSION_REF: Weak<Extension> = Arc::downgrade(&EXTENSION);

    /// Extension registry including the "tket.gpu" extension and
    /// dependencies.
    pub static ref REGISTRY: ExtensionRegistry = ExtensionRegistry::new([
        EXTENSION.to_owned(),
        PRELUDE.to_owned()
    ]);
}

impl TryFrom<CustomType> for GpuType {
    type Error = SignatureError;
    fn try_from(value: CustomType) -> Result<Self, Self::Error> {
        if value.extension() != &EXTENSION_ID {
            Err(SignatureError::ExtensionMismatch(
                EXTENSION_ID,
                value.extension().to_owned(),
            ))?
        }

        match value.name() {
            n if *n == *MODULE_TYPE_NAME => Ok(GpuType::Module),
            n if *n == *CONTEXT_TYPE_NAME => Ok(GpuType::Context),
            n if *n == *FUNC_TYPE_NAME => {
                let [inputs, outputs] = value.args() else {
                    Err(SignatureError::InvalidTypeArgs)?
                };
                let inputs = TypeRowRV::try_from(inputs.clone())?;
                let outputs = TypeRowRV::try_from(outputs.clone())?;

                Ok(GpuType::Func { inputs, outputs })
            }
            n if *n == *RESULT_TYPE_NAME => {
                let [outputs] = value.args() else {
                    Err(SignatureError::InvalidTypeArgs)?
                };
                let outputs = TypeRowRV::try_from(outputs.clone())?;

                Ok(GpuType::Result { outputs })
            }
            n => Err(SignatureError::NameMismatch(
                format_smolstr!(
                    "{}, {}, {} or {}",
                    MODULE_TYPE_NAME.as_str(),
                    CONTEXT_TYPE_NAME.as_str(),
                    FUNC_TYPE_NAME.as_str(),
                    RESULT_TYPE_NAME.as_str()
                ),
                n.to_owned(),
            )),
        }
    }
}

compute_opdef!(EXTENSION_ID, GpuExtension, GpuOpDef);

impl MakeRegisteredOp for GpuOp {
    fn extension_id(&self) -> ExtensionId {
        EXTENSION_ID
    }

    fn extension_ref(&self) -> Arc<Extension> {
        EXTENSION.clone()
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
/// A Constant identifying a GPU module.
/// Loading this is the only way to obtain a value of `tket.gpu.module` type.
pub struct ConstGpuModule {
    /// The name of the GPU module file to be loaded.
    pub module_filename: String,
    /// The name of an optional config file to be loaded by the GPU program
    pub config_filename: Option<String>,
}

#[typetag::serde]
impl CustomConst for ConstGpuModule {
    fn name(&self) -> ValueName {
        format!(
            "gpu:{}{}",
            self.module_filename,
            self.config_filename
                .clone()
                .map(|a| format!(":{a}"))
                .unwrap_or_default()
        )
        .into()
    }
    fn equal_consts(&self, other: &dyn CustomConst) -> bool {
        downcast_equal_consts(self, other)
    }

    fn get_type(&self) -> Type {
        GpuType::Module.get_type(EXTENSION_ID, &EXTENSION_REF)
    }
}

compute_builder!(GpuExtension, GpuOpBuilder);

/// Trait for adding a const gpu module to a hugr.
pub trait ConstGpuBuilder: GpuOpBuilder {
    /// Add a [ConstGpuModule] and load it.
    fn add_const_module(
        &mut self,
        module_filename: impl Into<String>,
        config_filename: Option<String>,
    ) -> Result<Wire, BuildError> {
        Ok(self.add_load_value(ConstGpuModule {
            module_filename: module_filename.into(),
            config_filename,
        }))
    }
}

impl<T: Dataflow> ConstGpuBuilder for T {}

#[cfg(test)]
mod test {
    use hugr::{
        builder::DFGBuilder,
        extension::prelude::bool_t,
        ops::DataflowOpTrait as _,
        types::{Term, type_param::TypeParam},
    };
    use rstest::rstest;

    use super::*;

    #[rstest]
    #[case(None, "gpu:test_mod")]
    #[case(Some("Lorem".to_string()), "gpu:test_mod:Lorem")]
    fn const_gpu_module(#[case] config_filename: Option<String>, #[case] exp_name: &str) {
        let m1 = ConstGpuModule {
            module_filename: "test_mod".to_string(),
            config_filename: config_filename.clone(),
        };
        let m2 = ConstGpuModule {
            module_filename: "test_mod".to_string(),
            config_filename,
        };
        assert_eq!(m1.name(), exp_name);
        assert!(m1.equal_consts(&m2));
    }

    #[rstest]
    #[case(GpuType::Module)]
    #[case(GpuType::Context)]
    #[case(GpuType::new_func(type_row![], type_row![]))]
    #[case(GpuType::new_func(vec![TypeRV::new_row_var_use(0, TypeBound::Linear)], vec![bool_t()]))]
    fn gpu_type(#[case] gpu_t: GpuType) {
        let hugr_t: Type = gpu_t.clone().into();
        let roundtripped_t = hugr_t.try_into().unwrap();
        assert_eq!(gpu_t, roundtripped_t);
    }

    #[test]
    fn gpu_op_def_instantiate() {
        assert_eq!(
            GpuOpDef::get_context.instantiate(&[]),
            Ok(GpuOp::GetContext)
        );
        assert_eq!(
            GpuOpDef::dispose_context.instantiate(&[]),
            Ok(GpuOp::DisposeContext)
        );
        assert_eq!(
            GpuOpDef::lookup_by_name.instantiate(&[
                "lookup_name".into(),
                TypeArg::new_var_use(0, TypeParam::ListType(Box::new(TypeBound::Linear.into()))),
                vec![].into()
            ]),
            Ok(GpuOp::LookupByName {
                name: "lookup_name".to_string(),
                inputs: vec![TypeRV::new_row_var_use(0, TypeBound::Linear)].into(),
                outputs: TypeRowRV::from(Vec::<TypeRV>::new())
            })
        );
        assert_eq!(
            GpuOpDef::lookup_by_id.instantiate(&[
                TypeArg::BoundedNat(42),
                TypeArg::new_var_use(0, TypeParam::ListType(Box::new(TypeBound::Linear.into()))),
                vec![].into()
            ]),
            Ok(GpuOp::LookupById {
                id: 42,
                inputs: vec![TypeRV::new_row_var_use(0, TypeBound::Linear)].into(),
                outputs: TypeRowRV::from(Vec::<TypeRV>::new())
            })
        );
        assert_eq!(
            GpuOpDef::call.instantiate(&[vec![Type::UNIT.into()].into(), vec![].into()]),
            Ok(GpuOp::Call {
                inputs: vec![Type::UNIT].into(),
                outputs: vec![].into()
            })
        );
    }

    #[rstest]
    #[case::concrete(type_row![], type_row![])]
    #[case::row_vars1(
        vec![
            TypeRV::UNIT,
            TypeRV::try_from(
                TypeArg::new_var_use(
                    0,
                    TypeParam::ListType (Box::new(TypeBound::Copyable.into()))
                )
            ).unwrap()
        ], TypeRowRV::try_from(
            Term::from(vec![TypeArg::from(Type::UNIT), TypeArg::from(usize_t())])
        ).unwrap()
    )]
    fn lookup_signature(
        #[case] inputs: impl Into<TypeRowRV>,
        #[case] outputs: impl Into<TypeRowRV>,
    ) {
        let (inputs, outputs) = (inputs.into(), outputs.into());
        let op = GpuOp::LookupByName {
            name: "test".into(),
            inputs: inputs.clone(),
            outputs: outputs.clone(),
        };
        let module_ty = GpuType::Module.get_type(op.extension_id(), &op.extension_ref());
        let func_ty = Type::new_extension(GpuType::func_custom_type(
            inputs.clone(),
            outputs.clone(),
            op.extension_id(),
            &op.extension_ref(),
        ));
        assert_eq!(
            op.to_extension_op().unwrap().signature(),
            Signature::new(module_ty, func_ty)
        );
    }

    #[rstest]
    #[case(type_row![], type_row![])]
    #[case(vec![Type::UNIT], TypeRow::try_from(Term::from(vec![TypeArg::from(Type::UNIT),TypeArg::from(usize_t())])).unwrap())]
    fn build_all(#[case] in_types: impl Into<TypeRow>, #[case] out_types: impl Into<TypeRow>) {
        use hugr::{
            builder::DataflowHugr as _,
            extension::prelude::{ConstUsize, UnwrapBuilder as _},
        };

        let (in_types, out_types) = (in_types.into(), out_types.into());
        let _hugr = {
            let mut builder =
                DFGBuilder::new(Signature::new(in_types.clone(), out_types.clone())).unwrap();
            let context_id = builder.add_load_value(ConstUsize::new(0));
            let [context] = {
                let mb_c = builder.add_get_context(context_id).unwrap();
                builder
                    .build_unwrap_sum(
                        1,
                        GpuOp::get_context_return_type(EXTENSION_ID, &EXTENSION_REF),
                        mb_c,
                    )
                    .unwrap()
            };
            let module = builder.add_const_module("test_module", None).unwrap();
            let func0 = builder
                .add_lookup_by_id(42_u64, in_types.clone(), out_types.clone(), module)
                .unwrap();

            let func1 = builder
                .add_lookup_by_name("test_func", in_types, out_types.clone(), module)
                .unwrap();

            let result = builder
                .add_call(context, func0, builder.input_wires())
                .unwrap();

            let (context, _results) = builder.add_read_result(result).unwrap();

            let result = builder
                .add_call(context, func1, builder.input_wires())
                .unwrap();

            let (context, results) = builder.add_read_result(result).unwrap();

            builder.add_dispose_context(context).unwrap();

            builder.finish_hugr_with_outputs(results).unwrap()
        };
    }
}
