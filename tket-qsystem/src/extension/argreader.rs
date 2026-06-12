//! This module defines a Hugr extension for entrypoint parameter support
use std::sync::{Arc, Weak};

use hugr::{
    Extension, HugrView, Wire,
    builder::{BuildError, Dataflow, DataflowSubContainer, FunctionBuilder},
    extension::{
        ExtensionId, OpDef, SignatureError, SignatureFunc, Version,
        prelude::bool_t,
        simple_op::{
            HasConcrete, HasDef, MakeExtensionOp, MakeOpDef, MakeRegisteredOp, OpLoadError,
            try_from_name,
        },
    },
    hugr::{Hugr, hugrmut::HugrMut},
    ops::handle::NodeHandle,
    ops::{self, OpName, OpType},
    std_extensions::{
        arithmetic::{
            float_types::{self, float64_type},
            int_types::{self, LOG_WIDTH_TYPE_PARAM, int_type},
        },
        collections::borrow_array,
    },
    types::{PolyFuncTypeRV, Signature, SumType, Term, Type, TypeArg, type_param::TypeParam},
};

use anyhow::{Result, bail};
use lazy_static::lazy_static;
use serde::{Deserialize, Serialize};
use strum::{EnumIter, EnumString, IntoStaticStr};

/// The ID of the `tket.argreader` extension.
pub const EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.argreader");
/// The "tket.argreader" extension version
pub const EXTENSION_VERSION: Version = Version::new(0, 1, 0);

lazy_static! {
    /// The "tket.argreader" extension.
    pub static ref EXTENSION: Arc<Extension>  = {
        Extension::new_arc(EXTENSION_ID, EXTENSION_VERSION, |ext, ext_ref| {
            ArgumentReadOpDef::load_all_ops(ext, ext_ref).unwrap();
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
/// Runtime argument reading operations
pub enum ArgumentReadOpDef {
    /// Read a boolean argument
    Bool,
    /// Read a signed integer argument
    Int,
    /// Read an unsigned integer argument
    UInt,
    /// Read a floating point argument
    F64,
    /// Read an array of boolean arguments
    ArrBool,
    /// Read an array of signed integer arguments
    ArrInt,
    /// Read an array of unsigned integer arguments
    ArrUInt,
    /// Read an array of floating point arguments
    ArrF64,
}

fn borrow_array_type(inner_t: Type) -> Type {
    borrow_array::borrow_array_type_parametric(
        TypeArg::new_var_use(1, TypeParam::max_nat_kind()),
        inner_t,
    )
    .unwrap()
}

fn int_tv(int_tv_idx: usize) -> Type {
    int_type(TypeArg::new_var_use(int_tv_idx, LOG_WIDTH_TYPE_PARAM))
}

impl ArgumentReadOpDef {
    /// Type of the return value of this argument
    pub fn output_type(&self) -> Type {
        match self {
            Self::Bool => bool_t(),
            Self::Int | Self::UInt => int_tv(1),
            Self::F64 => float64_type(),
            Self::ArrBool | Self::ArrF64 => {
                let inner_t = self.simple_type_op().output_type();
                borrow_array_type(inner_t)
            }
            Self::ArrInt | Self::ArrUInt => borrow_array_type(int_tv(2)),
        }
    }

    /// If the operation is on an array type, returns the inner element.
    /// Otherwise returns itself.
    pub fn simple_type_op(&self) -> Self {
        match self {
            Self::ArrBool => Self::Bool,
            Self::ArrInt => Self::Int,
            Self::ArrUInt => Self::UInt,
            Self::ArrF64 => Self::F64,
            _ => *self,
        }
    }
    /// If the operation is a scalar, returns the corresponding array type.
    /// Otherwise, it is already an array and returns itself.
    pub fn array_type_op(&self) -> Self {
        match self {
            Self::Bool => Self::ArrBool,
            Self::Int => Self::ArrInt,
            Self::UInt => Self::ArrUInt,
            Self::F64 => Self::ArrF64,
            _ => *self,
        }
    }
    /// Get the description of this operation, for use in documentation and error messages.
    pub fn get_description(&self) -> String {
        match self {
            Self::Bool => "Read a boolean argument",
            Self::Int => "Read an i64 argument",
            Self::UInt => "Read a u64 argument",
            Self::F64 => "Read an f64 argument",
            Self::ArrBool => "Read an array of boolean arguments",
            Self::ArrInt => "Read an array of i64 arguments",
            Self::ArrUInt => "Read an array of u64 arguments",
            Self::ArrF64 => "Read an array of f64 arguments",
        }
        .to_string()
    }
}
impl MakeOpDef for ArgumentReadOpDef {
    fn opdef_id(&self) -> hugr::ops::OpName {
        <&'static str>::from(self).into()
    }

    fn init_signature(&self, _extension_ref: &Weak<Extension>) -> SignatureFunc {
        let params = match self {
            // tag
            Self::Bool | Self::F64 => vec![TypeParam::StringKind],
            Self::Int | Self::UInt => vec![TypeParam::StringKind, LOG_WIDTH_TYPE_PARAM],
            Self::ArrBool | Self::ArrF64 => vec![TypeParam::StringKind, TypeParam::max_nat_kind()],
            Self::ArrInt | Self::ArrUInt => vec![
                TypeParam::StringKind,
                LOG_WIDTH_TYPE_PARAM,
                TypeParam::max_nat_kind(),
            ],
        };
        PolyFuncTypeRV::new(params, Signature::new(vec![], vec![self.output_type()])).into()
    }

    fn from_def(op_def: &OpDef) -> Result<Self, hugr::extension::simple_op::OpLoadError> {
        try_from_name(op_def.name(), op_def.extension_id())
    }

    fn extension(&self) -> ExtensionId {
        EXTENSION_ID
    }

    fn description(&self) -> String {
        self.get_description()
    }

    fn extension_ref(&self) -> Weak<Extension> {
        Arc::downgrade(&EXTENSION)
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, Hash, PartialEq)]
/// The possible scalar arguments for an `ArgumentReadOpDef`.
pub enum SimpleArgs {
    /// The read is of a boolean
    Bool,
    /// The read is of a signed integer, with the given bit width (e.g. 8 for i8, 64 for i64)
    Int(u8),
    /// The read is of an unsigned integer, with the given bit width (e.g. 8 for u8, 64 for u64)
    UInt(u8),
    /// The read is of a 64-bit floating point number
    F64,
}

#[derive(Debug, Clone, PartialEq)]
/// The arguments for an `ArgumentReadOpDef`, which may be either a simple scalar argument or an
/// array argument with a size.
pub enum ReadArgs {
    /// A simple scalar argument
    Simple(SimpleArgs),
    /// An array of a simple scalar argument, with the given length
    Array(SimpleArgs, u64),
}

#[derive(Debug, Clone, PartialEq)]
/// An operation providing a mechanism for requesting runtime arguments
pub struct ArgumentReadOp {
    /// Static string tag for the argument.
    pub tag: String,
    /// The operation definition
    pub read_op: ArgumentReadOpDef,
    /// Type arguments for the operation
    pub args: ReadArgs,
}

impl ArgumentReadOp {
    /// Create a new `ArgumentRead` operation for an unsigned integer return value
    pub fn new_uint(tag: impl Into<String>, int_width: u8) -> Self {
        Self {
            tag: tag.into(),
            read_op: ArgumentReadOpDef::UInt,
            args: ReadArgs::Simple(SimpleArgs::UInt(int_width)),
        }
    }
    /// Create a new `ArgumentRead` operation for a signed integer return value
    pub fn new_int(tag: impl Into<String>, int_width: u8) -> Self {
        Self {
            tag: tag.into(),
            read_op: ArgumentReadOpDef::Int,
            args: ReadArgs::Simple(SimpleArgs::Int(int_width)),
        }
    }
    /// Create a new `ArgumentRead` operation for a boolean return value
    pub fn new_bool(tag: impl Into<String>) -> Self {
        Self {
            tag: tag.into(),
            read_op: ArgumentReadOpDef::Bool,
            args: ReadArgs::Simple(SimpleArgs::Bool),
        }
    }
    /// Create a new `ArgumentRead` operation for a 64-bit floating point return value
    pub fn new_f64(tag: impl Into<String>) -> Self {
        Self {
            tag: tag.into(),
            read_op: ArgumentReadOpDef::F64,
            args: ReadArgs::Simple(SimpleArgs::F64),
        }
    }
    /// Convert this `ArgumentRead` operation to one that reads an array of the same type, with the
    /// given size.
    pub fn array_op(mut self, size: u64) -> Self {
        match &mut self.args {
            ReadArgs::Simple(s_args) => {
                self.args = ReadArgs::Array(s_args.clone(), size);
                self.read_op = self.read_op.array_type_op();
                self
            }
            ReadArgs::Array(_, s) => {
                *s = size;
                self
            }
        }
    }
}

impl MakeExtensionOp for ArgumentReadOp {
    fn op_id(&self) -> OpName {
        self.read_op.opdef_id()
    }

    fn from_extension_op(ext_op: &hugr::ops::ExtensionOp) -> Result<Self, OpLoadError>
    where
        Self: Sized,
    {
        let def = ext_op.def();
        let args = ext_op.args();
        let read_op_def = ArgumentReadOpDef::from_def(def)?;
        read_op_def.instantiate(args)
    }

    fn type_args(&self) -> Vec<TypeArg> {
        let mut type_args = vec![self.tag.clone().into()];
        match self.args {
            ReadArgs::Simple(SimpleArgs::Int(width))
            | ReadArgs::Simple(SimpleArgs::UInt(width)) => {
                type_args.push(TypeArg::BoundedNat(width as u64));
            }
            ReadArgs::Array(SimpleArgs::Int(width), size)
            | ReadArgs::Array(SimpleArgs::UInt(width), size) => {
                type_args.push(TypeArg::BoundedNat(size));
                type_args.push(TypeArg::BoundedNat(width as u64));
            }
            ReadArgs::Simple(SimpleArgs::Bool) => {}
            ReadArgs::Simple(SimpleArgs::F64) => {}
            ReadArgs::Array(SimpleArgs::Bool, size) | ReadArgs::Array(SimpleArgs::F64, size) => {
                type_args.push(TypeArg::BoundedNat(size));
            }
        }
        type_args
    }
}

impl MakeRegisteredOp for ArgumentReadOp {
    fn extension_id(&self) -> ExtensionId {
        EXTENSION_ID
    }

    fn extension_ref(&self) -> Arc<Extension> {
        EXTENSION.clone()
    }
}

impl TryFrom<&OpType> for ArgumentReadOpDef {
    type Error = OpLoadError;
    fn try_from(value: &OpType) -> Result<Self, Self::Error> {
        let Some(ext) = value.as_extension_op() else {
            return Err(OpLoadError::NotMember(value.to_string()));
        };
        Self::from_extension_op(ext)
    }
}

/// A builder trait for adding `ArgumentReadOp`s to a dataflow.
pub trait ArgumentReadOpBuilder: Dataflow {
    /// Add an `ArgumentReadOp` to this dataflow, returning the output wire of the operation.
    fn add_read(&mut self, op: ArgumentReadOp) -> Result<Wire, BuildError> {
        let handle = self.add_dataflow_op(op, [])?;
        debug_assert!(handle.outputs().len() == 1);
        Ok(handle.out_wire(0))
    }
}
impl<D: Dataflow> ArgumentReadOpBuilder for D {}

impl HasDef for ArgumentReadOp {
    type Def = ArgumentReadOpDef;
}

impl HasConcrete for ArgumentReadOpDef {
    type Concrete = ArgumentReadOp;

    fn instantiate(&self, type_args: &[TypeArg]) -> Result<Self::Concrete, OpLoadError> {
        match (self, type_args) {
            (Self::Bool, [TypeArg::String(arg)]) => Ok(ArgumentReadOp::new_bool(arg)),

            (Self::F64, [TypeArg::String(arg)]) => Ok(ArgumentReadOp::new_f64(arg)),

            (Self::Int, [TypeArg::String(arg), TypeArg::BoundedNat(log_width)]) => {
                Ok(ArgumentReadOp::new_int(arg, *log_width as u8))
            }

            (Self::UInt, [TypeArg::String(arg), TypeArg::BoundedNat(log_width)]) => {
                Ok(ArgumentReadOp::new_uint(arg, *log_width as u8))
            }

            (Self::ArrBool, [TypeArg::String(arg), TypeArg::BoundedNat(size)]) => {
                Ok(ArgumentReadOp::new_bool(arg).array_op(*size))
            }

            (Self::ArrF64, [TypeArg::String(arg), TypeArg::BoundedNat(size)]) => {
                Ok(ArgumentReadOp::new_f64(arg).array_op(*size))
            }

            (
                Self::ArrInt,
                [
                    TypeArg::String(arg),
                    TypeArg::BoundedNat(size),
                    TypeArg::BoundedNat(log_width),
                ],
            ) => Ok(ArgumentReadOp::new_int(arg, *log_width as u8).array_op(*size)),

            (
                Self::ArrUInt,
                [
                    TypeArg::String(arg),
                    TypeArg::BoundedNat(size),
                    TypeArg::BoundedNat(log_width),
                ],
            ) => Ok(ArgumentReadOp::new_uint(arg, *log_width as u8).array_op(*size)),

            _ => Err(SignatureError::InvalidTypeArgs.into()),
        }
    }
}

/// Map a HUGR type to an `ArgumentReadOp` that can read an argument of that type at runtime.
///
/// TODO: This function requires a lot of attention. It feels like
/// it must be the wrong way to do it, but the right way isn't clear
/// to me.
pub fn map_type(hugr_type: &Type, idx: usize) -> Result<ArgumentReadOp> {
    match &**hugr_type {
        Term::ExtensionType(custom) => {
            if *custom.extension() == int_types::EXTENSION_ID {
                if custom.name() != "int" {
                    bail!("Can only handle int");
                }
                let [TypeArg::BoundedNat(log_width)] = custom.args() else {
                    bail!("Expected a log width type argument");
                };
                let log_width: u64 = *log_width;
                let log_width = log_width as u8;
                Ok(ArgumentReadOp::new_int(format!("arg_{idx}"), log_width))
            } else if *custom.extension() == float_types::EXTENSION_ID {
                if custom.name() != "float64" {
                    bail!("Can only handle float64");
                }
                Ok(ArgumentReadOp::new_f64(format!("arg_{idx}")))
            } else if *custom.extension() == borrow_array::EXTENSION_ID {
                if custom.name() != "borrow_array" {
                    bail!("Can only handle borrow_array");
                }
                match custom.args() {
                    [TypeArg::BoundedNat(n_elements), element_type] => match element_type {
                        Term::ExtensionType(elem) => {
                            if *elem.extension() == int_types::EXTENSION_ID {
                                if elem.name() != "int" {
                                    bail!("Can only handle int element types in borrow array");
                                }
                                let [TypeArg::BoundedNat(log_width)] = elem.args() else {
                                    bail!(
                                        "Expected a log width type argument for int element type in borrow array"
                                    );
                                };
                                let log_width: u64 = *log_width;
                                let log_width = log_width as u8;
                                Ok(ArgumentReadOp::new_int(format!("arg_{idx}"), log_width)
                                    .array_op(*n_elements))
                            } else if *elem.extension() == float_types::EXTENSION_ID {
                                if elem.name() != "float64" {
                                    bail!("Can only handle float64 element types in borrow array");
                                }
                                Ok(ArgumentReadOp::new_f64(format!("arg_{idx}"))
                                    .array_op(*n_elements))
                            } else {
                                bail!("Unsupported element type in borrow array: {:?}", elem);
                            }
                        }
                        Term::SumType(st) => match st {
                            SumType::Unit { size: 2 } => {
                                Ok(ArgumentReadOp::new_bool(format!("arg_{idx}"))
                                    .array_op(*n_elements))
                            }
                            _ => bail!("Unsupported element type in borrow array: {:?}", st),
                        },
                        _ => bail!(
                            "Unsupported element type in borrow array: {:?}",
                            element_type
                        ),
                    },
                    _ => {
                        bail!(
                            "Expected a borrow_array with a bounded nat and an element type as arguments"
                        );
                    }
                }
            } else {
                bail!("Unsupported extension type: {:?}", custom.extension());
            }
        }
        Term::SumType(st) => match st {
            SumType::Unit { size: 2 } => Ok(ArgumentReadOp::new_bool(format!("arg_{idx}"))),
            _ => bail!("Unsupported sum type: {:?}", st),
        },
        _ => bail!("Unsupported type: {:?}", hugr_type),
    }
}

/// If the incoming HUGR has an entrypoint function with arguments, this function
/// replaces the entrypoint with one that takes no arguments, instead reading them
/// using `ArgReaderOp`s tagged by the respective input argument names. This new
/// entrypoint then invokes the original entrypoint with the read arguments.
///
/// If the incoming HUGR's entrypoint has no arguments, this function does not mutate
/// the HUGR and returns successfully.
pub fn wrap_entrypoint_with_arguments(hugr: &mut Hugr) -> Result<()> {
    let original_entrypoint = hugr.entrypoint();

    let original_sig = {
        let Some(original_func_defn) = hugr.get_optype(original_entrypoint).as_func_defn() else {
            bail!("Entrypoint is not a function");
        };

        original_func_defn.signature().clone()
    };

    // The wrapper takes no ordinary inputs; it reads them via your ArgumentReadOp-like ops.
    // It returns exactly what the original entrypoint returns.
    let wrapper_sig = Signature::new(
        [],                                   // wrapper inputs
        original_sig.body().output().clone(), // wrapper outputs
    );

    // Direct Call has a static/function input after its normal dataflow inputs.
    // Capture this before constructing the builder, because the builder borrows `hugr` mutably.
    let original_func_wire = {
        let original_op = hugr.get_optype(original_entrypoint);
        let Some(static_out) = original_op.static_output_port() else {
            bail!("Original entrypoint has no static function output port");
        };
        Wire::new(original_entrypoint, static_out)
    };

    let wrapper_node = {
        let mut f_build =
            FunctionBuilder::with_hugr(&mut *hugr, "__wrapped_entrypoint", wrapper_sig)?;

        let mut call_inputs = Vec::new();

        for (nth_arg, argument_type) in original_sig.body().input().iter().enumerate() {
            let arg_reader_op = map_type(argument_type, nth_arg)?;
            // need to create a string
            let arg_reader = f_build.add_read(arg_reader_op)?;
            call_inputs.push(arg_reader);
        }
        let call_op = ops::Call::try_new(original_sig.clone(), [])?;
        call_inputs.push(original_func_wire);
        let call = f_build.add_dataflow_op(call_op, call_inputs)?;
        let wrapper_func = f_build.finish_with_outputs(call.outputs())?;
        wrapper_func.node()
    };

    hugr.set_entrypoint(wrapper_node);

    Ok(())
}

#[cfg(test)]
pub(crate) mod test {
    #[test]
    fn test_entrypoint_args() {
        // TODO
    }
}
