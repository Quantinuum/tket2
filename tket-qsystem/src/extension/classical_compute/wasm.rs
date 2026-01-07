//! This module defines the `tket.wasm` Hugr extension used to model calling
//! into WebAssembly.
//!
//! 'tket.wasm' provides the following types:
//!  - `tket.wasm.module`: A WebAssembly module.
//!  - `tket.wasm.context`: A WebAssembly context.
//!  - `tket.wasm.func`: A WebAssembly function.
//!  - `tket.wasm.result`: A WebAssembly result.
//!
//!  Each of which can be constructed in rust via [WasmType].
//!
//!  `tket.wasm.context` is a linear type that orders runtime effects. It is
//!  obtained via the `tket.wasm.get_context` operation and destroyed via
//!  `tket.wasm.dispose_context`.
//!
//!  A `tket.wasm.module` is obtained by loading a `ConstWasmModule` constant.
//!  We assume that all modules are available in all contexts.
//!
//!  `tket.wasm.get_context` takes a `prelude.usize`, allowing multiple independent
//!  contexts to exist simultaneously. `get_context` is fallible, returning
//!  `None` if the specified context has already been obtained via an earlier
//!  `get_context` op and not `dispose_context`ed.
//!
//!  `tket.wasm.func` is a type representing a handle to a function in a
//!  `tket.wasm.module`. It carries type args defining its signature, but not
//!  its name.
//!
//!  A `tket.wasm.func` is obtained from either a `tket.wasm.lookup_by_id` or
//!  `tket.wasm.lookup_by_name`op, which takes a compile-time identifier (name or id)
//!  and signature, and a runtime module.
//!  TODO Likely the module should be compile time here, but I think we need
//!  extension-op-static-edges to do this properly.
//!
//!  `tket.wasm.func`s are called via the `tket.wasm.call` op. This op takes:
//!   - a `tket.wasm.context` identifying where the call will execute;
//!   - a `tket.wasm.func` identifying the function to call;
//!   - Input arguments as specified by the type of the `tket.wasm.func`.
//!
//!   It returns a `tket.wasm.result`, which must then be read by using
//!   `tket.wasm.read_result`, which yields a `tket.wasm.context` tupled with
//!   the output values of the `tket.wasm.func`.
//!
//!   We provide [WasmType] to assist in constructing and interpreting [Type]s.
//!
//!   We provide [WasmOp] to assist in constructing and interpreting [ExtensionOp]s.
//!
//!   We provide [WasmOpBuilder] to assist in building [hugr::Hugr]s using the
//!   `tket.wasm` extension.

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
/// The type used to parameterise Compute operations for the wasm extension.
pub struct WasmExtension;

/// Concrete instantiation(i.e. with type args applied) of a "tket.wasm" operation.
pub type WasmOp = ComputeOp<WasmExtension>;
/// Concrete types defined by the `tket.wasm` extension.
pub type WasmType = ComputeType<WasmExtension>;

/// The "tket.wasm" extension id.
pub const EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.wasm");
/// The "tket.wasm" extension version.
pub const EXTENSION_VERSION: Version = Version::new(0, 4, 1);

lazy_static! {
    /// The `tket.wasm` extension.
    pub static ref EXTENSION: Arc<Extension> =
        Extension::new_arc(EXTENSION_ID, EXTENSION_VERSION, |ext, ext_ref| {
        add_compute_type_defs(ext, ext_ref).unwrap();
        WasmOpDef::load_all_ops(ext, ext_ref, ).unwrap();
    });

    /// A [Weak] reference to the `tket.wasm` op.
    pub static ref EXTENSION_REF: Weak<Extension> = Arc::downgrade(&EXTENSION);

    /// Extension registry including the "tket.wasm" extension and
    /// dependencies.
    pub static ref REGISTRY: ExtensionRegistry = ExtensionRegistry::new([
        EXTENSION.to_owned(),
        PRELUDE.to_owned()
    ]);
}

impl TryFrom<CustomType> for WasmType {
    type Error = SignatureError;
    fn try_from(value: CustomType) -> Result<Self, Self::Error> {
        if value.extension() != &EXTENSION_ID {
            Err(SignatureError::ExtensionMismatch(
                EXTENSION_ID,
                value.extension().to_owned(),
            ))?
        }

        match value.name() {
            n if *n == *MODULE_TYPE_NAME => Ok(WasmType::Module),
            n if *n == *CONTEXT_TYPE_NAME => Ok(WasmType::Context),
            n if *n == *FUNC_TYPE_NAME => {
                let [inputs, outputs] = value.args() else {
                    Err(SignatureError::InvalidTypeArgs)?
                };
                let inputs = TypeRowRV::try_from(inputs.clone())?;
                let outputs = TypeRowRV::try_from(outputs.clone())?;

                Ok(WasmType::Func { inputs, outputs })
            }
            n if *n == *RESULT_TYPE_NAME => {
                let [outputs] = value.args() else {
                    Err(SignatureError::InvalidTypeArgs)?
                };
                let outputs = TypeRowRV::try_from(outputs.clone())?;

                Ok(WasmType::Result { outputs })
            }
            n => Err(SignatureError::NameMismatch(
                format_smolstr!(
                    "{}, {}, {} or {}",
                    MODULE_TYPE_NAME.as_str(),
                    CONTEXT_TYPE_NAME.as_str(),
                    FUNC_TYPE_NAME.as_str(),
                    RESULT_TYPE_NAME.as_str(),
                ),
                n.to_owned(),
            )),
        }
    }
}

compute_opdef!(EXTENSION_ID, WasmExtension, WasmOpDef);

impl MakeRegisteredOp for WasmOp {
    fn extension_id(&self) -> ExtensionId {
        EXTENSION_ID
    }

    fn extension_ref(&self) -> Arc<Extension> {
        EXTENSION.clone()
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
/// A Constant identifying a WebAssembly module.
/// Loading this is the only way to obtain a value of `tket.wasm.module` type.
pub struct ConstWasmModule {
    /// The name of the module.
    pub module_filename: String,
}

#[typetag::serde]
impl CustomConst for ConstWasmModule {
    fn name(&self) -> ValueName {
        format!("wasm:{}", self.module_filename).into()
    }
    fn equal_consts(&self, other: &dyn CustomConst) -> bool {
        downcast_equal_consts(self, other)
    }

    fn get_type(&self) -> Type {
        WasmType::Module.get_type(EXTENSION_ID, &EXTENSION_REF)
    }
}

compute_builder!(WasmExtension, WasmOpBuilder);

/// Trait for adding a const wasm module to a hugr.
pub trait ConstWasmBuilder: WasmOpBuilder {
    /// Add a [ConstWasmModule] and load it.
    fn add_const_module(&mut self, module_filename: impl Into<String>) -> Result<Wire, BuildError> {
        Ok(self.add_load_value(ConstWasmModule {
            module_filename: module_filename.into(),
        }))
    }
}

impl<T: Dataflow> ConstWasmBuilder for T {}

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

    #[test]
    fn const_wasm_module() {
        let m1 = ConstWasmModule {
            module_filename: "test_mod".to_string(),
        };
        let m2 = ConstWasmModule {
            module_filename: "test_mod".to_string(),
        };
        assert_eq!(m1.name(), "wasm:test_mod");
        assert!(m1.equal_consts(&m2));
    }

    #[rstest]
    #[case(WasmType::Module)]
    #[case(WasmType::Context)]
    #[case(WasmType::new_func(type_row![], type_row![]))]
    #[case(WasmType::new_func(vec![TypeRV::new_row_var_use(0, TypeBound::Linear)], vec![bool_t()]))]
    fn wasm_type(#[case] wasm_t: WasmType) {
        let hugr_t: Type = wasm_t.clone().into();
        let roundtripped_t = hugr_t.try_into().unwrap();
        assert_eq!(wasm_t, roundtripped_t);
    }

    #[test]
    fn wasm_op_def_instantiate() {
        assert_eq!(
            WasmOpDef::get_context.instantiate(&[]),
            Ok(WasmOp::GetContext)
        );
        assert_eq!(
            WasmOpDef::dispose_context.instantiate(&[]),
            Ok(WasmOp::DisposeContext)
        );
        assert_eq!(
            WasmOpDef::lookup_by_name.instantiate(&[
                "lookup_name".into(),
                TypeArg::new_var_use(0, TypeParam::ListType(Box::new(TypeBound::Linear.into()))),
                vec![].into()
            ]),
            Ok(WasmOp::LookupByName {
                name: "lookup_name".to_string(),
                inputs: vec![TypeRV::new_row_var_use(0, TypeBound::Linear)].into(),
                outputs: TypeRowRV::from(Vec::<TypeRV>::new())
            })
        );
        assert_eq!(
            WasmOpDef::lookup_by_id.instantiate(&[
                TypeArg::BoundedNat(42),
                TypeArg::new_var_use(0, TypeParam::ListType(Box::new(TypeBound::Linear.into()))),
                vec![].into()
            ]),
            Ok(WasmOp::LookupById {
                id: 42,
                inputs: vec![TypeRV::new_row_var_use(0, TypeBound::Linear)].into(),
                outputs: TypeRowRV::from(Vec::<TypeRV>::new())
            })
        );
        assert_eq!(
            WasmOpDef::call.instantiate(&[vec![Type::UNIT.into()].into(), vec![].into()]),
            Ok(WasmOp::Call {
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
        let op = WasmOp::LookupByName {
            name: "test".into(),
            inputs: inputs.clone(),
            outputs: outputs.clone(),
        };
        let extension = Arc::downgrade(&op.extension_ref());
        let module_ty = WasmType::Module.get_type(op.extension_id(), &extension);
        let func_ty = Type::new_extension(WasmType::func_custom_type(
            inputs.clone(),
            outputs.clone(),
            op.extension_id(),
            &extension,
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
                        WasmOp::get_context_return_type(EXTENSION_ID, &EXTENSION_REF),
                        mb_c,
                    )
                    .unwrap()
            };
            let module = builder.add_const_module("test_module").unwrap();
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
