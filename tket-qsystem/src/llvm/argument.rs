//! LLVM lowering implementations for the "tket.argument" extension.

use crate::extension::argument::{ReadArgOp, ReadArgOpDef};
use crate::llvm::array_utils::ArrayLowering;
use crate::llvm::prelude::emit_global_string;
use anyhow::{Result, anyhow, bail};
use hugr::extension::prelude::bool_t;
use hugr::llvm::CodegenExtsBuilder;
use hugr::llvm::custom::CodegenExtension;
use hugr::llvm::emit::{EmitFuncContext, EmitOpArgs};
use hugr::llvm::inkwell;
use hugr::std_extensions::arithmetic::float_types::float64_type;
use hugr::std_extensions::arithmetic::int_types::{INT_TYPES, LOG_WIDTH_MAX};
use hugr::std_extensions::collections::array::{
    EXTENSION_ID as ARRAY_EXTENSION_ID, array_type_def,
};
use hugr::types::{Term, Type, TypeArg};
use inkwell::AddressSpace;
use inkwell::builder::Builder;
use inkwell::context::Context;
use inkwell::types::{BasicType, BasicTypeEnum, FloatType, IntType, PointerType, VoidType};
use inkwell::values::{BasicValueEnum, FunctionValue};
use tket::hugr::extension::simple_op::MakeExtensionOp;
use tket::hugr::ops::ExtensionOp;
use tket::hugr::{HugrView, Node};

/// The concrete variant of an argument type, used to select the extern function.
///
/// Produced by [`classify_arg_type`]; lives here because it is purely a codegen concept.
#[derive(Debug, Clone, PartialEq)]
enum ArgKind {
    /// A boolean argument.
    Bool,
    /// A signed 64-bit integer argument (i64 / log-width 6 only).
    I64,
    /// A 64-bit floating-point argument.
    F64,
    /// An array of booleans with a fixed length.
    ArrBool(u64),
    /// An array of signed 64-bit integers with a fixed length.
    ArrI64(u64),
    /// An array of 64-bit floats with a fixed length.
    ArrF64(u64),
}

impl ArgKind {
    /// The LLVM IR call-instruction name for this kind.
    const fn call_name(&self) -> &'static str {
        match self {
            ArgKind::Bool => "read_arg_bool",
            ArgKind::I64 => "read_arg_int",
            ArgKind::F64 => "read_arg_f64",
            ArgKind::ArrBool(_) => "read_arg_bool_array",
            ArgKind::ArrI64(_) => "read_arg_int_array",
            ArgKind::ArrF64(_) => "read_arg_f64_array",
        }
    }

    /// If this is an array kind, return `(length, element scalar)`.
    const fn as_array(&self) -> Option<(u64, Scalar)> {
        match self {
            ArgKind::ArrBool(len) => Some((*len, Scalar::Bool)),
            ArgKind::ArrI64(len) => Some((*len, Scalar::I64)),
            ArgKind::ArrF64(len) => Some((*len, Scalar::F64)),
            _ => None,
        }
    }
}

/// A leaf (non-array) argument type, identifying the corresponding extern.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Scalar {
    Bool,
    I64,
    F64,
}

impl Scalar {
    const fn scalar_kind(self) -> ArgKind {
        match self {
            Scalar::Bool => ArgKind::Bool,
            Scalar::I64 => ArgKind::I64,
            Scalar::F64 => ArgKind::F64,
        }
    }

    const fn array_kind(self, size: u64) -> ArgKind {
        match self {
            Scalar::Bool => ArgKind::ArrBool(size),
            Scalar::I64 => ArgKind::ArrI64(size),
            Scalar::F64 => ArgKind::ArrF64(size),
        }
    }

    /// The LLVM basic type for this scalar.
    fn llvm_type<'c>(self, ctx: &'c Context) -> BasicTypeEnum<'c> {
        match self {
            Scalar::Bool => ctx.bool_type().as_basic_type_enum(),
            Scalar::I64 => ctx.i64_type().as_basic_type_enum(),
            Scalar::F64 => ctx.f64_type().as_basic_type_enum(),
        }
    }
}

/// If `ty` is a HUGR integer type, return its log-width.
///
/// Compares against the canonical [`INT_TYPES`] table rather than just the extension
/// id, so it cannot be confused with another type from the same extension.
fn as_int_log_width(ty: &Type) -> Option<u8> {
    INT_TYPES
        .iter()
        .position(|int_ty| int_ty == ty)
        .map(|w| w as u8)
}

/// If `ty` is a standard `array`, return its `(size, element type)`.
///
/// Verifies both the extension id and the type name against the canonical type def,
/// and recovers the element type via [`Type::try_from`].
fn as_std_array(ty: &Type) -> Option<(u64, Type)> {
    let Term::ExtensionType(custom) = &**ty else {
        return None;
    };
    if *custom.extension() != ARRAY_EXTENSION_ID || custom.name() != array_type_def().name() {
        return None;
    }
    match custom.args() {
        [TypeArg::BoundedNat(size), elem] => {
            Type::try_from(elem.clone()).ok().map(|elem| (*size, elem))
        }
        _ => None,
    }
}

/// Classify a scalar (non-array) argument type.
///
/// Returns `None` if `ty` is not a supported scalar shape, or `Some(Err(..))` if it is
/// recognisably an integer but not the supported i64 width (`argreader_get_i64` is the
/// only integer extern, so narrower widths must be rejected at codegen time).
fn classify_scalar(ty: &Type) -> Option<Result<Scalar>> {
    if *ty == bool_t() {
        Some(Ok(Scalar::Bool))
    } else if *ty == float64_type() {
        Some(Ok(Scalar::F64))
    } else {
        as_int_log_width(ty).map(|log_width| {
            if log_width == LOG_WIDTH_MAX {
                Ok(Scalar::I64)
            } else {
                Err(anyhow!(
                    "Only i64 (log-width {LOG_WIDTH_MAX}) is supported as an integer argument; \
                     got log-width {log_width}"
                ))
            }
        })
    }
}

/// Map the concrete output type of a [`ReadArgOp`] to the extern function variant.
///
/// Errors on unsupported types, including integer types other than i64.
fn classify_arg_type(ty: &Type) -> Result<ArgKind> {
    if let Some(scalar) = classify_scalar(ty) {
        scalar.map(Scalar::scalar_kind)
    } else if let Some((size, elem)) = as_std_array(ty) {
        match classify_scalar(&elem) {
            Some(scalar) => scalar.map(|s| s.array_kind(size)),
            None => bail!("Unsupported array element type for argument reading: {elem}"),
        }
    } else {
        bail!("Unsupported type for argument reading: {ty}");
    }
}

/// Codegen extension for the `tket.argument` extension.
#[derive(Default)]
pub struct ArgumentCodegenExtension<AL: ArrayLowering> {
    array_lowering: AL,
}

impl<AL: ArrayLowering> ArgumentCodegenExtension<AL> {
    /// Creates a new [ArgumentCodegenExtension] with specified array lowering.
    pub const fn new(array_lowering: AL) -> Self {
        Self { array_lowering }
    }
}

impl<AL: ArrayLowering + Clone> CodegenExtension for ArgumentCodegenExtension<AL> {
    fn add_extension<'a, H: HugrView<Node = Node> + 'a>(
        self,
        builder: CodegenExtsBuilder<'a, H>,
    ) -> CodegenExtsBuilder<'a, H>
    where
        Self: 'a,
    {
        builder.simple_extension_op::<ReadArgOpDef>(move |context, args, _op| {
            let op = ReadArgOp::from_extension_op(args.node().as_ref())?;
            ArgumentEmitter(context, self.array_lowering.clone()).emit(args, &op)
        })
    }
}

struct ArgumentEmitter<'c, 'd, 'e, H: HugrView<Node = Node>, AL: ArrayLowering>(
    &'d mut EmitFuncContext<'c, 'e, H>,
    AL,
);

impl<'c, H: HugrView<Node = Node>, AL: ArrayLowering + Clone> ArgumentEmitter<'c, '_, '_, H, AL> {
    fn iw_context(&self) -> &'c Context {
        self.0.typing_session().iw_context()
    }

    fn i64_t(&self) -> IntType<'c> {
        self.iw_context().i64_type()
    }

    fn f64_t(&self) -> FloatType<'c> {
        self.iw_context().f64_type()
    }

    fn bool_t(&self) -> IntType<'c> {
        self.iw_context().bool_type()
    }

    fn ptr_t(&self) -> PointerType<'c> {
        self.iw_context().ptr_type(AddressSpace::default())
    }

    fn void_t(&self) -> VoidType<'c> {
        self.iw_context().void_type()
    }

    fn builder(&self) -> &Builder<'c> {
        self.0.builder()
    }

    /// Declare (or retrieve) the Selene-provided extern for reading an argument of
    /// `kind`, returning its [`FunctionValue`].
    ///
    /// These `argreader_get_*` functions are implemented by the Selene runtime. Their
    /// ABI is:
    /// - scalars: `fn(tag: ptr) -> T`, where `tag` is the NUL-terminated argument name
    ///   and `T` is the scalar (`i1`/`i64`/`f64`).
    /// - arrays: `fn(tag: ptr, out: ptr, len: i64)`, where `out` is a caller-allocated
    ///   buffer the runtime fills (arrays cannot be returned by value across the C ABI)
    ///   and `len` is the element count.
    fn get_argreader_func(&self, kind: &ArgKind) -> Result<FunctionValue<'c>> {
        let (fn_type, func_name) = match kind {
            ArgKind::Bool => (
                self.bool_t().fn_type(&[self.ptr_t().into()], false),
                "argreader_get_bool",
            ),
            ArgKind::I64 => (
                self.i64_t().fn_type(&[self.ptr_t().into()], false),
                "argreader_get_i64",
            ),
            ArgKind::F64 => (
                self.f64_t().fn_type(&[self.ptr_t().into()], false),
                "argreader_get_f64",
            ),
            ArgKind::ArrBool(_) => (
                self.void_t().fn_type(
                    &[
                        self.ptr_t().into(),
                        self.ptr_t().into(),
                        self.i64_t().into(),
                    ],
                    false,
                ),
                "argreader_get_bool_array",
            ),
            ArgKind::ArrI64(_) => (
                self.void_t().fn_type(
                    &[
                        self.ptr_t().into(),
                        self.ptr_t().into(),
                        self.i64_t().into(),
                    ],
                    false,
                ),
                "argreader_get_i64_array",
            ),
            ArgKind::ArrF64(_) => (
                self.void_t().fn_type(
                    &[
                        self.ptr_t().into(),
                        self.ptr_t().into(),
                        self.i64_t().into(),
                    ],
                    false,
                ),
                "argreader_get_f64_array",
            ),
        };
        self.0.get_extern_func(func_name, fn_type)
    }

    fn emit(&mut self, args: EmitOpArgs<'c, '_, ExtensionOp, H>, op: &ReadArgOp) -> Result<()> {
        if op.tag.is_empty() {
            bail!("Empty argument name tag received");
        }
        let kind = classify_arg_type(&op.output_type)?;
        let argread_fn = self.get_argreader_func(&kind)?;
        let tag_ptr = emit_global_string(self.0, &op.tag, "argument_", "")?;
        let call_name = kind.call_name();

        let result = if let Some((len, scalar)) = kind.as_array() {
            self.emit_array_read(
                argread_fn,
                tag_ptr,
                len,
                scalar.llvm_type(self.iw_context()),
                call_name,
            )?
        } else {
            self.builder()
                .build_call(argread_fn, &[tag_ptr.into()], call_name)?
                .try_as_basic_value()
                .unwrap_basic()
        };

        args.outputs.finish(self.builder(), [result])
    }

    fn emit_array_read(
        &mut self,
        argread_fn: FunctionValue<'c>,
        tag_ptr: BasicValueEnum<'c>,
        len: u64,
        elem_ty: BasicTypeEnum<'c>,
        call_name: &str,
    ) -> Result<BasicValueEnum<'c>> {
        let len_u32 =
            u32::try_from(len).map_err(|_| anyhow!("Array length {len} exceeds u32::MAX"))?;
        let len_val = self.i64_t().const_int(len, false);
        let (elems_ptr, array_value) = self.1.alloc_array(self.0, elem_ty, len_u32)?;
        self.builder().build_call(
            argread_fn,
            &[tag_ptr.into(), elems_ptr.into(), len_val.into()],
            call_name,
        )?;
        Ok(array_value)
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::extension::argument::ReadArgOp;
    use crate::llvm::array_utils::DEFAULT_HEAP_ARRAY_LOWERING;
    use hugr::extension::prelude::bool_t;
    use hugr::extension::simple_op::MakeRegisteredOp;
    use hugr::llvm::check_emission;
    use hugr::llvm::test::{TestContext, llvm_ctx, single_op_hugr};
    use hugr::std_extensions::arithmetic::{float_types::float64_type, int_types::int_type};
    use hugr::std_extensions::collections::array::array_type;
    use hugr::types::TypeArg;
    use rstest::rstest;

    use crate::llvm::prelude::QISPreludeCodegen;

    #[rstest]
    #[case::bool(1, ReadArgOp::new("test_bool", bool_t()))]
    #[case::int(2, ReadArgOp::new("test_int", int_type(TypeArg::BoundedNat(6))))]
    #[case::f64(3, ReadArgOp::new("test_f64", float64_type()))]
    #[case::arr_bool(4, ReadArgOp::new("test_arr_bool", array_type(10, bool_t())))]
    #[case::arr_int(
        5,
        ReadArgOp::new("test_arr_int", array_type(10, int_type(TypeArg::BoundedNat(6))))
    )]
    #[case::arr_f64(6, ReadArgOp::new("test_arr_f64", array_type(10, float64_type())))]
    #[should_panic(expected = "Empty argument name tag received")]
    #[case::empty_tag(7, ReadArgOp::new("", bool_t()))]
    #[should_panic(expected = "log-width 6")]
    #[case::narrow_int(8, ReadArgOp::new("test_narrow", int_type(TypeArg::BoundedNat(3))))]
    #[should_panic(expected = "log-width 6")]
    #[case::narrow_int_arr(
        9,
        ReadArgOp::new("test_narrow_arr", array_type(4, int_type(TypeArg::BoundedNat(3))))
    )]
    fn emit_argument_codegen(
        // `_i` seeds the `TestContext` so each case emits to its own snapshot file
        // (`..._1.snap` ... `..._9.snap`); without distinct ids the cases would collide.
        #[case] _i: i32,
        #[with(_i)] mut llvm_ctx: TestContext,
        #[case] op: ReadArgOp,
    ) {
        let pcg = QISPreludeCodegen;
        llvm_ctx.add_extensions(move |ceb| {
            ceb.add_extension(ArgumentCodegenExtension::new(DEFAULT_HEAP_ARRAY_LOWERING))
                .add_extension(DEFAULT_HEAP_ARRAY_LOWERING.codegen_extension())
                .add_prelude_extensions(pcg.clone())
                .add_default_int_extensions()
                .add_float_extensions()
        });
        let ext_op = op.to_extension_op().unwrap().into();
        let mut hugr = single_op_hugr(ext_op);
        check_emission!(hugr, llvm_ctx);
    }

    #[rstest]
    #[case::bool(bool_t(), ArgKind::Bool)]
    #[case::int(int_type(TypeArg::BoundedNat(6)), ArgKind::I64)]
    #[case::f64(float64_type(), ArgKind::F64)]
    #[case::arr_bool(array_type(10, bool_t()), ArgKind::ArrBool(10))]
    #[case::arr_int(array_type(10, int_type(TypeArg::BoundedNat(6))), ArgKind::ArrI64(10))]
    #[case::arr_f64(array_type(10, float64_type()), ArgKind::ArrF64(10))]
    fn test_classify(#[case] ty: Type, #[case] expected: ArgKind) {
        assert_eq!(classify_arg_type(&ty).unwrap(), expected);
    }

    #[rstest]
    #[case(0)]
    #[case(1)]
    #[case(2)]
    #[case(3)]
    #[case(4)]
    #[case(5)]
    fn test_classify_rejects_narrow_int(#[case] log_width: u64) {
        let scalar_err = classify_arg_type(&int_type(TypeArg::BoundedNat(log_width))).unwrap_err();
        assert!(
            scalar_err.to_string().contains("log-width 6"),
            "scalar log_width={log_width}: {scalar_err}"
        );
        let array_err = classify_arg_type(&array_type(4, int_type(TypeArg::BoundedNat(log_width))))
            .unwrap_err();
        assert!(
            array_err.to_string().contains("log-width 6"),
            "array log_width={log_width}: {array_err}"
        );
    }
}
