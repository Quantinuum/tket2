//! This module defines the Hugr extension used to represent result reporting operations,
//! with static string tags.
//!
use std::sync::{Arc, Weak};

use hugr::std_extensions::collections;
use hugr::types::Signature;
use hugr::{
    Extension, Wire,
    builder::{BuildError, Dataflow},
    extension::{
        ExtensionId, OpDef, SignatureFunc, Version,
        prelude::bool_t,
        simple_op::{MakeExtensionOp, MakeOpDef, MakeRegisteredOp, OpLoadError, try_from_name},
    },
    ops::OpType,
    std_extensions::arithmetic::{
        float_types::float64_type,
        int_types::{LOG_WIDTH_TYPE_PARAM, int_type},
    },
    type_row,
    types::{PolyFuncType, Type, TypeArg, type_param::TypeParam},
};

use lazy_static::lazy_static;
use serde::{Deserialize, Serialize};
use strum::{EnumIter, EnumString, IntoStaticStr};

/// The "tket.result" extension id.
pub const EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.result");
/// The "tket.result" extension version.
pub const EXTENSION_VERSION: Version = Version::new(0, 2, 0);

lazy_static! {
    /// The "tket.result" extension.
    pub static ref EXTENSION: Arc<Extension> = {
        Extension::new_arc(EXTENSION_ID, EXTENSION_VERSION, |ext, ext_ref| {
            ResultOpDef::load_all_ops(ext, ext_ref).unwrap();
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
/// Result report operations from quantum runtime.
/*
result_int<Tag: StringArg, N: BoundedNat>( int<N> ) // N is bitwidth, e.g. i32, i64
result_uint<Tag: StringArg, N: BoundedNat>( int<N> ) // unsigned
result_bool<Tag: StringArg>( Sum((), ()) )
result_f64<Tag: StringArg>( f64 )

result_arr_int<Tag: StringArg, N: Nat, M: BoundedNat>( Array<N, int<M> > )
result_arr_uint<Tag: StringArg, N: Nat, M: BoundedNat>( Array<N, int<M> > )
result_arr_f64<Tag: StringArg, N: Nat>( Array<N,f64> )
result_arr_bool<Tag: StringArg, N: Nat>( Array<N, Sum((), ()) > )
*/
pub enum ResultOpDef {
    /// Report a boolean result.
    #[strum(serialize = "result_bool")]
    Bool,
    /// Report a signed integer result.
    #[strum(serialize = "result_int")]
    Int,
    /// Report an unsigned integer result.
    #[strum(serialize = "result_uint")]
    UInt,
    /// Report a floating-point result.
    #[strum(serialize = "result_f64")]
    F64,
    /// Report an array of boolean results.
    #[strum(serialize = "result_array_bool")]
    ArrBool,
    /// Report an array of signed integer results.
    #[strum(serialize = "result_array_int")]
    ArrInt,
    /// Report an array of unsigned integer results.
    #[strum(serialize = "result_array_uint")]
    ArrUInt,
    /// Report an array of floating-point results.
    #[strum(serialize = "result_array_f64")]
    ArrF64,
}

impl ResultOpDef {
    /// Type of the argument to the result operation.
    pub fn arg_type(&self) -> Type {
        match self {
            Self::Bool => bool_t(),
            Self::Int | Self::UInt => int_tv(1),
            Self::F64 => float64_type(),
            Self::ArrBool | Self::ArrF64 => {
                let inner_t = self.simple_type_op().arg_type();
                array_arg_type(inner_t)
            }
            Self::ArrInt | Self::ArrUInt => array_arg_type(int_tv(2)),
        }
    }

    fn simple_type_op(&self) -> Self {
        match self {
            Self::ArrBool => Self::Bool,
            Self::ArrInt => Self::Int,
            Self::ArrUInt => Self::UInt,
            Self::ArrF64 => Self::F64,
            _ => *self,
        }
    }

    fn array_type_op(&self) -> Self {
        match self {
            Self::Bool => Self::ArrBool,
            Self::Int => Self::ArrInt,
            Self::UInt => Self::ArrUInt,
            Self::F64 => Self::ArrF64,
            _ => *self,
        }
    }

    /// Type parameters for result operation.
    pub fn type_params(&self) -> Vec<TypeParam> {
        match self {
            Self::Bool | Self::F64 => vec![],
            Self::Int | Self::UInt => vec![LOG_WIDTH_TYPE_PARAM],
            _ => [
                vec![TypeParam::max_nat_type()],
                self.simple_type_op().type_params(),
            ]
            .concat(),
        }
    }

    /// Instantiate the result operation with the given type arguments.
    pub fn instantiate(&self, args: &[TypeArg]) -> Result<ResultOp, OpLoadError> {
        let parsed_args = concrete_result_op_type_args(args)?;

        match (parsed_args, self) {
            ((tag, None, None), Self::Bool | Self::F64) => Ok(ResultOp::_new_basic(tag, *self)),
            ((tag, Some(width), None), Self::Int | Self::UInt) => {
                Ok(ResultOp::_new_int(tag, width as u8, *self))
            }
            ((_, Some(size), _), _) => {
                let inner_args = match args {
                    [t, _] => vec![t.clone()],
                    [t, _, w] => vec![t.clone(), w.clone()],
                    _ => unreachable!(),
                };
                Ok(self
                    .simple_type_op()
                    .instantiate(&inner_args)?
                    .array_op(size))
            }
            _ => Err(hugr::extension::SignatureError::InvalidTypeArgs.into()),
        }
    }

    fn result_signature(&self) -> SignatureFunc {
        PolyFuncType::new(
            [vec![TypeParam::StringType], self.type_params()].concat(),
            Signature::new(self.arg_type(), type_row![]),
        )
        .into()
    }
}

fn array_arg_type(inner_t: Type) -> Type {
    collections::array::array_type_parametric(
        TypeArg::new_var_use(1, TypeParam::max_nat_type()),
        inner_t,
    )
    .unwrap()
}

fn int_tv(int_tv_idx: usize) -> Type {
    int_type(TypeArg::new_var_use(int_tv_idx, LOG_WIDTH_TYPE_PARAM))
}

impl MakeOpDef for ResultOpDef {
    fn opdef_id(&self) -> hugr::ops::OpName {
        <&'static str>::from(self).into()
    }

    fn init_signature(&self, _extension_ref: &std::sync::Weak<Extension>) -> SignatureFunc {
        self.result_signature()
    }

    fn from_def(op_def: &OpDef) -> Result<Self, hugr::extension::simple_op::OpLoadError> {
        try_from_name(op_def.name(), op_def.extension_id())
    }

    fn extension(&self) -> ExtensionId {
        EXTENSION_ID
    }

    fn description(&self) -> String {
        match self {
            Self::Bool => "Report a boolean result.",
            Self::Int => "Report a signed integer result.",
            Self::UInt => "Report an unsigned integer result.",
            Self::F64 => "Report a floating-point result.",
            Self::ArrBool => "Report an array of boolean results.",
            Self::ArrInt => "Report an array of signed integer results.",
            Self::ArrUInt => "Report an array of unsigned integer results.",
            Self::ArrF64 => "Report an array of floating-point results.",
        }
        .to_string()
    }

    fn extension_ref(&self) -> std::sync::Weak<Extension> {
        Arc::downgrade(&EXTENSION)
    }
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize, Hash, PartialEq)]
/// Type arguments to a numeric result operation (not an array).
pub enum SimpleArgs {
    /// No type arguments, simple result type.
    Basic,
    /// Integer type argument, specifying bit width.
    Int(u8),
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize, Hash, PartialEq)]
/// Arguments to a "tket.result" operation.
pub enum ResultArgs {
    /// Simple result type, not an array.
    Simple(SimpleArgs),
    /// Argument for array result type, with inner type arguments and array size.
    Array(SimpleArgs, u64),
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize, Hash, PartialEq)]
/// Concrete instantiation of a "tket.result" operation.
pub struct ResultOp {
    /// Static string tag for the result.
    pub tag: String,
    /// The result operation definition.
    pub result_op: ResultOpDef,
    /// Type arguments for the result operation.
    pub args: ResultArgs,
}

impl ResultOp {
    fn _new_basic(tag: impl Into<String>, result_op: ResultOpDef) -> Self {
        Self {
            tag: tag.into(),
            result_op,
            args: ResultArgs::Simple(SimpleArgs::Basic),
        }
    }

    fn _new_int(tag: impl Into<String>, int_width: u8, int_op: ResultOpDef) -> Self {
        Self {
            tag: tag.into(),
            result_op: int_op,
            args: ResultArgs::Simple(SimpleArgs::Int(int_width)),
        }
    }
    /// Create a new "tket.result" operation for a boolean result.
    pub fn new_bool(tag: impl Into<String>) -> Self {
        Self::_new_basic(tag, ResultOpDef::Bool)
    }

    /// Create a new "tket.result" operation for a floating-point result.
    pub fn new_f64(tag: impl Into<String>) -> Self {
        Self::_new_basic(tag, ResultOpDef::F64)
    }

    /// Convert this "tket.result" operation to an array result operation over the same inner type.
    /// The size of the array is set to the given value.
    /// If this operation is already an array result operation, its size is updated.
    pub fn array_op(mut self, size: u64) -> Self {
        let result_op = self.result_op.array_type_op();
        match &mut self.args {
            ResultArgs::Simple(s_args) => {
                self.args = ResultArgs::Array(s_args.clone(), size);
                self.result_op = result_op;
                self
            }
            ResultArgs::Array(_, s) => {
                *s = size;
                self
            }
        }
    }

    /// Create a new "tket.result" operation for a signed integer result of a given bit width.
    pub fn new_int(tag: impl Into<String>, int_width: u8) -> Self {
        Self::_new_int(tag, int_width, ResultOpDef::Int)
    }

    /// Create a new "tket.result" operation for an unsigned integer result of a given bit width.
    pub fn new_uint(tag: impl Into<String>, int_width: u8) -> Self {
        Self::_new_int(tag, int_width, ResultOpDef::UInt)
    }
}

fn concrete_result_op_type_args(
    args: &[TypeArg],
) -> Result<(String, Option<u64>, Option<u64>), OpLoadError> {
    let err = || hugr::extension::SignatureError::InvalidTypeArgs.into();
    match args {
        [TypeArg::String(arg)] => Ok((arg.to_owned(), None, None)),

        [TypeArg::String(arg), TypeArg::BoundedNat(n)] => Ok((arg.to_owned(), Some(*n), None)),

        [
            TypeArg::String(arg),
            TypeArg::BoundedNat(n),
            TypeArg::BoundedNat(m),
        ] => Ok((arg.to_owned(), Some(*n), Some(*m))),

        _ => Err(err()),
    }
}

impl From<&ResultOp> for &'static str {
    fn from(value: &ResultOp) -> Self {
        value.result_op.into()
    }
}

impl MakeExtensionOp for ResultOp {
    fn op_id(&self) -> hugr::ops::OpName {
        self.result_op.opdef_id()
    }

    fn from_extension_op(
        ext_op: &hugr::ops::custom::ExtensionOp,
    ) -> Result<Self, hugr::extension::simple_op::OpLoadError>
    where
        Self: Sized,
    {
        let def = ext_op.def();
        let args = ext_op.args();
        ResultOpDef::from_def(def)?.instantiate(args)
    }

    fn type_args(&self) -> Vec<TypeArg> {
        let mut type_args = vec![self.tag.clone().into()];

        match self.args {
            ResultArgs::Simple(_) => {}
            ResultArgs::Array(_, size) => {
                type_args.push(TypeArg::BoundedNat(size));
            }
        }

        match self.args {
            ResultArgs::Simple(SimpleArgs::Int(width))
            | ResultArgs::Array(SimpleArgs::Int(width), _) => {
                type_args.push(TypeArg::BoundedNat(width as u64));
            }
            _ => {}
        }

        type_args
    }
}

impl MakeRegisteredOp for ResultOp {
    fn extension_id(&self) -> ExtensionId {
        EXTENSION_ID
    }

    fn extension_ref(&self) -> Arc<Extension> {
        EXTENSION.clone()
    }
}

impl TryFrom<&OpType> for ResultOpDef {
    type Error = OpLoadError;

    fn try_from(value: &OpType) -> Result<Self, Self::Error> {
        let Some(ext) = value.as_extension_op() else {
            Err(OpLoadError::NotMember(value.to_string()))?
        };
        Self::from_extension_op(ext)
    }
}

/// An extension trait for [Dataflow] providing methods to add "tket.result"
/// operations.
pub trait ResultOpBuilder: Dataflow {
    /// Add a "tket.result" op.
    fn add_result(&mut self, result_wire: Wire, op: ResultOp) -> Result<(), BuildError> {
        let handle = self.add_dataflow_op(op, [result_wire])?;

        debug_assert_eq!(handle.outputs().len(), 0);
        Ok(())
    }
}

impl<D: Dataflow> ResultOpBuilder for D {}

#[cfg(test)]
pub(crate) mod test {
    use cool_asserts::assert_matches;
    use hugr::HugrView;
    use hugr::types::Signature;
    use hugr::{
        builder::{Dataflow, DataflowHugr, FunctionBuilder},
        std_extensions::arithmetic::int_types::INT_TYPES,
        std_extensions::collections::array::array_type,
    };
    use std::sync::Arc;
    use strum::IntoEnumIterator;

    use super::*;

    fn get_opdef(op: ResultOpDef) -> Option<&'static Arc<OpDef>> {
        EXTENSION.get_op(&op.op_id())
    }

    #[test]
    fn create_extension() {
        assert_eq!(EXTENSION.name(), &EXTENSION_ID);

        for o in ResultOpDef::iter() {
            assert_eq!(ResultOpDef::from_def(get_opdef(o).unwrap()), Ok(o));
        }
    }

    #[test]
    fn circuit() {
        const ARR_SIZE: u64 = 20;
        let in_row = vec![
            bool_t(),
            float64_type(),
            INT_TYPES[5].clone(),
            INT_TYPES[6].clone(),
        ];
        let in_row = [
            in_row.clone(),
            in_row
                .into_iter()
                .map(|t| array_type(ARR_SIZE, t))
                .collect(),
        ]
        .concat();
        let hugr = {
            let mut func_builder =
                FunctionBuilder::new("circuit", Signature::new(in_row, type_row![])).unwrap();
            let ops = [
                ResultOp::new_bool("b"),
                ResultOp::new_f64("f"),
                ResultOp::new_int("i", 5),
                ResultOp::new_uint("u", 6),
            ];

            for op in &ops {
                let op_t: OpType = op.clone().to_extension_op().unwrap().into();
                let def_op: ResultOpDef = (&op_t).try_into().unwrap();
                assert_eq!(op.result_op, def_op);
                let new_op: ResultOp = op_t.cast().unwrap();
                assert_eq!(&new_op, op);

                let op = op.clone().array_op(ARR_SIZE);
                let op_t: OpType = op.clone().to_extension_op().unwrap().into();
                let def_op: ResultOpDef = (&op_t).try_into().unwrap();

                assert_eq!(op.result_op, def_op);
                let new_op: ResultOp = op_t.cast().unwrap();
                assert_eq!(&new_op, &op);
            }
            let [b, f, i, u, a_b, a_f, a_i, a_u] = func_builder.input_wires_arr();

            for (w, op) in [b, f, i, u].iter().zip(ops.iter()) {
                func_builder.add_result(*w, op.clone()).unwrap();
            }
            for (w, op) in [a_b, a_f, a_i, a_u].iter().zip(ops.iter()) {
                func_builder
                    .add_result(*w, op.clone().array_op(ARR_SIZE))
                    .unwrap();
            }

            func_builder.finish_hugr_with_outputs([]).unwrap()
        };
        assert_matches!(hugr.validate(), Ok(_));
    }
}
