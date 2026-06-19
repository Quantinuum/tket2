//! LLVM lowering implementations for the "tket.argreader" extension.

use crate::extension::argreader::{ArgKind, ReadArgOp, ReadArgOpDef, classify_arg_type};
use crate::llvm::prelude::emit_global_string;
use anyhow::{Result, bail};
use hugr::llvm::CodegenExtsBuilder;
use hugr::llvm::custom::CodegenExtension;
use hugr::llvm::emit::{EmitFuncContext, EmitOpArgs};
use hugr::llvm::extension::collections::borrow_array::{self, BorrowArrayCodegen};
use hugr::llvm::inkwell;
use inkwell::AddressSpace;
use inkwell::builder::Builder;
use inkwell::context::Context;
use inkwell::types::{BasicType, BasicTypeEnum, FloatType, IntType, PointerType, VoidType};
use inkwell::values::{BasicValueEnum, FunctionValue};
use tket::hugr::extension::simple_op::MakeExtensionOp;
use tket::hugr::ops::ExtensionOp;
use tket::hugr::{HugrView, Node};

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

    fn emit(
        &mut self,
        args: EmitOpArgs<'c, '_, ExtensionOp, H>,
        op: &ReadArgOp,
    ) -> Result<()> {
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
            borrow_array::build_barray_alloc(self.0, &self.1, elem_ty, len, false)?;
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
    #[case::arr_int(5, ReadArgOp::new("test_arr_int", borrow_array_type(10, int_type(TypeArg::BoundedNat(6)))))]
    #[case::arr_f64(6, ReadArgOp::new("test_arr_f64", borrow_array_type(10, float64_type())))]
    #[should_panic(expected = "Empty argument name tag received")]
    #[case::empty_tag(7, ReadArgOp::new("", bool_t()))]
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
}
