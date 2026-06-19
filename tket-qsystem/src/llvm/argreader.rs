//! LLVM lowering implementations for the "tket.argreader" extension.

use crate::extension::argreader::{ReadArgOp, ReadArgOpDef};
use crate::llvm::prelude::emit_global_string;
use anyhow::{Result, anyhow, bail};
use hugr::extension::prelude::bool_t;
use hugr::llvm::CodegenExtsBuilder;
use hugr::llvm::custom::CodegenExtension;
use hugr::llvm::emit::{EmitFuncContext, EmitOpArgs};
use hugr::llvm::extension::collections::borrow_array::{
    self as llvm_borrow_array, BorrowArrayCodegen,
};
use hugr::llvm::inkwell;
use hugr::std_extensions::arithmetic::float_types::float64_type;
use hugr::std_extensions::arithmetic::int_types::{INT_TYPES, LOG_WIDTH_MAX};
use hugr::std_extensions::collections::borrow_array::{
    EXTENSION_ID as BORROW_ARRAY_EXTENSION_ID, borrow_array_type_def,
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

/// The concrete variant of an argument type, used to select the selene extern function.
///
/// Produced by [`classify_arg_type`]; lives here because it is purely a codegen concept.
#[derive(Debug, Clone, PartialEq)]
enum ArgKind {
    /// A boolean argument.
    Bool,
    /// A signed 64-bit integer argument (i64 / log-width 6 only).
    Int,
    /// A 64-bit floating-point argument.
    F64,
    /// An array of booleans with a fixed length.
    ArrBool(u64),
    /// An array of signed 64-bit integers with a fixed length.
    ArrInt(u64),
    /// An array of 64-bit floats with a fixed length.
    ArrF64(u64),
}

/// A leaf (non-array) argument type, identifying the corresponding selene extern.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Scalar {
    Bool,
    Int,
    F64,
}

impl Scalar {
    const fn scalar_kind(self) -> ArgKind {
        match self {
            Scalar::Bool => ArgKind::Bool,
            Scalar::Int => ArgKind::Int,
            Scalar::F64 => ArgKind::F64,
        }
    }

    const fn array_kind(self, size: u64) -> ArgKind {
        match self {
            Scalar::Bool => ArgKind::ArrBool(size),
            Scalar::Int => ArgKind::ArrInt(size),
            Scalar::F64 => ArgKind::ArrF64(size),
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

/// If `ty` is a `borrow_array`, return its `(size, element type)`.
///
/// Verifies both the extension id and the type name against the canonical type def,
/// and recovers the element type via [`Type::try_from`].
fn as_borrow_array(ty: &Type) -> Option<(u64, Type)> {
    let Term::ExtensionType(custom) = &**ty else {
        return None;
    };
    if *custom.extension() != BORROW_ARRAY_EXTENSION_ID
        || custom.name() != borrow_array_type_def().name()
    {
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
                Ok(Scalar::Int)
            } else {
                Err(anyhow!(
                    "Only i64 (log-width {LOG_WIDTH_MAX}) is supported as an integer argument; \
                     got log-width {log_width}"
                ))
            }
        })
    }
}

/// Map the concrete output type of a [`ReadArgOp`] to the selene extern function variant.
///
/// Errors on unsupported types, including integer types other than i64.
fn classify_arg_type(ty: &Type) -> Result<ArgKind> {
    if let Some(scalar) = classify_scalar(ty) {
        return scalar.map(Scalar::scalar_kind);
    }
    if let Some((size, elem)) = as_borrow_array(ty) {
        return match classify_scalar(&elem) {
            Some(scalar) => scalar.map(|s| s.array_kind(size)),
            None => bail!("Unsupported borrow_array element type for argument reading: {elem}"),
        };
    }
    bail!("Unsupported type for argument reading: {ty}")
}

/// Codegen extension for the argreader
#[derive(Default)]
pub struct ArgReaderCodegenExtension<BAC: BorrowArrayCodegen> {
    borrow_array_codegen: BAC,
}

impl<BAC: BorrowArrayCodegen> ArgReaderCodegenExtension<BAC> {
    /// Creates a new [ArgReaderCodegenExtension] with specified array lowering.
    pub const fn new(borrow_array_codegen: BAC) -> Self {
        Self {
            borrow_array_codegen,
        }
    }
}

impl<BAC: BorrowArrayCodegen + Clone> CodegenExtension for ArgReaderCodegenExtension<BAC> {
    fn add_extension<'a, H: HugrView<Node = Node> + 'a>(
        self,
        builder: CodegenExtsBuilder<'a, H>,
    ) -> CodegenExtsBuilder<'a, H>
    where
        Self: 'a,
    {
        builder.simple_extension_op::<ReadArgOpDef>(move |context, args, _op| {
            let op = ReadArgOp::from_extension_op(args.node().as_ref())?;
            ArgReaderEmitter(context, self.borrow_array_codegen.clone()).emit(args, &op)
        })
    }
}

struct ArgReaderEmitter<'c, 'd, 'e, H: HugrView<Node = Node>, BAC: BorrowArrayCodegen>(
    &'d mut EmitFuncContext<'c, 'e, H>,
    BAC,
);

impl<'c, H: HugrView<Node = Node>, BAC: BorrowArrayCodegen + Clone>
    ArgReaderEmitter<'c, '_, '_, H, BAC>
{
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

    fn get_argreader_func(&self, kind: &ArgKind) -> Result<FunctionValue<'c>> {
        let (fn_type, func_name) = match kind {
            ArgKind::Bool => (
                self.bool_t().fn_type(&[self.ptr_t().into()], false),
                "argreader_get_bool",
            ),
            ArgKind::Int => (
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
            ArgKind::ArrInt(_) => (
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

        let result = match kind {
            ArgKind::Bool => self
                .builder()
                .build_call(argread_fn, &[tag_ptr.into()], "read_arg_bool")?
                .try_as_basic_value()
                .unwrap_basic(),
            ArgKind::Int => self
                .builder()
                .build_call(argread_fn, &[tag_ptr.into()], "read_arg_int")?
                .try_as_basic_value()
                .unwrap_basic(),
            ArgKind::F64 => self
                .builder()
                .build_call(argread_fn, &[tag_ptr.into()], "read_arg_f64")?
                .try_as_basic_value()
                .unwrap_basic(),
            ArgKind::ArrBool(len) => self.emit_array_read(
                argread_fn,
                tag_ptr,
                len,
                self.bool_t().as_basic_type_enum(),
                "read_arg_bool_array",
            )?,
            ArgKind::ArrInt(len) => self.emit_array_read(
                argread_fn,
                tag_ptr,
                len,
                self.i64_t().as_basic_type_enum(),
                "read_arg_int_array",
            )?,
            ArgKind::ArrF64(len) => self.emit_array_read(
                argread_fn,
                tag_ptr,
                len,
                self.f64_t().as_basic_type_enum(),
                "read_arg_f64_array",
            )?,
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
        if len > u32::MAX as u64 {
            bail!("Array length {} exceeds u32::MAX", len);
        }
        let len_val = self.i64_t().const_int(len, false);
        let (elems_ptr, barray_value) =
            llvm_borrow_array::build_barray_alloc(self.0, &self.1, elem_ty, len, false)?;
        self.builder().build_call(
            argread_fn,
            &[tag_ptr.into(), elems_ptr.into(), len_val.into()],
            call_name,
        )?;
        Ok(barray_value.into())
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::extension::argreader::ReadArgOp;
    use hugr::extension::prelude::bool_t;
    use hugr::extension::simple_op::MakeRegisteredOp;
    use hugr::llvm::check_emission;
    use hugr::llvm::extension::collections::borrow_array::{
        BorrowArrayCodegenExtension, DefaultBorrowArrayCodegen,
    };
    use hugr::llvm::test::{TestContext, llvm_ctx, single_op_hugr};
    use hugr::std_extensions::arithmetic::{float_types::float64_type, int_types::int_type};
    use hugr::std_extensions::collections::borrow_array::borrow_array_type;
    use hugr::types::TypeArg;
    use rstest::rstest;

    use crate::llvm::prelude::QISPreludeCodegen;

    #[rstest]
    #[case::bool(1, ReadArgOp::new("test_bool", bool_t()))]
    #[case::int(2, ReadArgOp::new("test_int", int_type(TypeArg::BoundedNat(6))))]
    #[case::f64(3, ReadArgOp::new("test_f64", float64_type()))]
    #[case::arr_bool(4, ReadArgOp::new("test_arr_bool", borrow_array_type(10, bool_t())))]
    #[case::arr_int(
        5,
        ReadArgOp::new(
            "test_arr_int",
            borrow_array_type(10, int_type(TypeArg::BoundedNat(6)))
        )
    )]
    #[case::arr_f64(
        6,
        ReadArgOp::new("test_arr_f64", borrow_array_type(10, float64_type()))
    )]
    #[should_panic(expected = "Empty argument name tag received")]
    #[case::empty_tag(7, ReadArgOp::new("", bool_t()))]
    #[should_panic(expected = "log-width 6")]
    #[case::narrow_int(8, ReadArgOp::new("test_narrow", int_type(TypeArg::BoundedNat(3))))]
    #[should_panic(expected = "log-width 6")]
    #[case::narrow_int_arr(
        9,
        ReadArgOp::new(
            "test_narrow_arr",
            borrow_array_type(4, int_type(TypeArg::BoundedNat(3)))
        )
    )]
    fn emit_argreader_codegen(
        #[case] _i: i32,
        #[with(_i)] mut llvm_ctx: TestContext,
        #[case] op: ReadArgOp,
    ) {
        let pcg = QISPreludeCodegen;
        let bac = DefaultBorrowArrayCodegen::<QISPreludeCodegen>::default();
        llvm_ctx.add_extensions(move |ceb| {
            ceb.add_extension(ArgReaderCodegenExtension::new(bac.clone()))
                .add_extension(BorrowArrayCodegenExtension::from(bac.clone()))
                .add_prelude_extensions(pcg.clone())
                .add_default_int_extensions()
                .add_float_extensions()
        });
        let ext_op = op.to_extension_op().unwrap().into();
        let mut hugr = single_op_hugr(ext_op);
        check_emission!(hugr, llvm_ctx);
    }

    #[test]
    fn test_classify_bool() {
        assert_eq!(classify_arg_type(&bool_t()).unwrap(), ArgKind::Bool);
    }

    #[test]
    fn test_classify_int_i64() {
        assert_eq!(
            classify_arg_type(&int_type(TypeArg::BoundedNat(6))).unwrap(),
            ArgKind::Int
        );
    }

    #[test]
    fn test_classify_int_rejects_narrow() {
        for log_width in 0u64..=5 {
            let err = classify_arg_type(&int_type(TypeArg::BoundedNat(log_width))).unwrap_err();
            assert!(
                err.to_string().contains("log-width 6"),
                "log_width={log_width}: {err}"
            );
        }
    }

    #[test]
    fn test_classify_f64() {
        assert_eq!(classify_arg_type(&float64_type()).unwrap(), ArgKind::F64);
    }

    #[test]
    fn test_classify_arr_bool() {
        assert_eq!(
            classify_arg_type(&borrow_array_type(10, bool_t())).unwrap(),
            ArgKind::ArrBool(10)
        );
    }

    #[test]
    fn test_classify_arr_int_i64() {
        assert_eq!(
            classify_arg_type(&borrow_array_type(10, int_type(TypeArg::BoundedNat(6)))).unwrap(),
            ArgKind::ArrInt(10)
        );
    }

    #[test]
    fn test_classify_arr_int_rejects_narrow() {
        let err =
            classify_arg_type(&borrow_array_type(4, int_type(TypeArg::BoundedNat(3)))).unwrap_err();
        assert!(err.to_string().contains("log-width 6"), "{err}");
    }

    #[test]
    fn test_classify_arr_f64() {
        assert_eq!(
            classify_arg_type(&borrow_array_type(10, float64_type())).unwrap(),
            ArgKind::ArrF64(10)
        );
    }
}
