//! LLVM lowering implementations for "tket.results" extension.

use crate::extension::argreader::{ArgumentReadOp, ArgumentReadOpDef, ReadArgs};
use crate::llvm::prelude::emit_global_string;
use anyhow::{Result, anyhow, bail};
use hugr::llvm::CodegenExtsBuilder;
use hugr::llvm::custom::CodegenExtension;
use hugr::llvm::emit::{EmitFuncContext, EmitOpArgs};
use hugr::llvm::extension::collections::borrow_array::{self, BorrowArrayCodegen};
use hugr::llvm::inkwell;
use inkwell::AddressSpace;
use inkwell::builder::Builder;
use inkwell::context::Context;
use inkwell::types::{BasicType, FloatType, IntType, PointerType, VoidType};
use inkwell::values::FunctionValue;
use tket::hugr::extension::simple_op::MakeExtensionOp;
use tket::hugr::ops::ExtensionOp;
use tket::hugr::{HugrView, Node};

/// Codegen extension for results
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
        builder.simple_extension_op::<ArgumentReadOpDef>(move |context, args, _op| {
            let op = ArgumentReadOp::from_extension_op(args.node().as_ref())?;
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

    pub fn get_argreader_func(&self, op: &ArgumentReadOp) -> Result<FunctionValue<'c>> {
        let (fn_type, func_name) = match op.read_op {
            ArgumentReadOpDef::Bool => (
                self.bool_t().fn_type(&[self.ptr_t().into()], false),
                "argreader_get_bool",
            ),
            ArgumentReadOpDef::Int => (
                self.i64_t().fn_type(&[self.ptr_t().into()], false),
                "argreader_get_i64",
            ),
            ArgumentReadOpDef::UInt => (
                self.i64_t().fn_type(&[self.ptr_t().into()], false),
                "argreader_get_u64",
            ),
            ArgumentReadOpDef::F64 => (
                self.f64_t().fn_type(&[self.ptr_t().into()], false),
                "argreader_get_f64",
            ),
            ArgumentReadOpDef::ArrBool => (
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
            ArgumentReadOpDef::ArrInt => (
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
            ArgumentReadOpDef::ArrUInt => (
                self.void_t().fn_type(
                    &[
                        self.ptr_t().into(),
                        self.ptr_t().into(),
                        self.i64_t().into(),
                    ],
                    false,
                ),
                "argreader_get_u64_array",
            ),
            ArgumentReadOpDef::ArrF64 => (
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

    /// Function to help lower the tket result extension.
    fn emit(
        &mut self,
        args: EmitOpArgs<'c, '_, ExtensionOp, H>,
        op: &ArgumentReadOp,
    ) -> Result<()> {
        let argread_fn = self.get_argreader_func(op)?;
        let op = ArgumentReadOp::from_extension_op(args.node().as_ref())?;
        let tag = op.tag;
        if tag.is_empty() {
            bail!("Empty argument name tag received");
        }
        let tag_ptr = emit_global_string(self.0, tag, "argument_", "")?;
        let result = match op.read_op {
            ArgumentReadOpDef::Bool => self
                .builder()
                .build_call(argread_fn, &[tag_ptr.into()], "read_arg_bool")?
                .try_as_basic_value()
                .unwrap_basic(),
            ArgumentReadOpDef::Int => self
                .builder()
                .build_call(argread_fn, &[tag_ptr.into()], "read_arg_int")?
                .try_as_basic_value()
                .unwrap_basic(),
            ArgumentReadOpDef::UInt => self
                .builder()
                .build_call(argread_fn, &[tag_ptr.into()], "read_arg_uint")?
                .try_as_basic_value()
                .unwrap_basic(),
            ArgumentReadOpDef::F64 => self
                .builder()
                .build_call(argread_fn, &[tag_ptr.into()], "read_arg_f64")?
                .try_as_basic_value()
                .unwrap_basic(),
            ArgumentReadOpDef::ArrBool => {
                let ReadArgs::Array(_, len) = op.args else {
                    bail!("Expected array read args for ArrBool");
                };
                let len: u32 = len
                    .try_into()
                    .map_err(|_| anyhow!("Array length exceeds u32::MAX"))?;
                let elem_ty = self.bool_t().as_basic_type_enum();
                let len_val = self.i64_t().const_int(len as u64, false);

                let (elems_ptr, barray_value) =
                    borrow_array::build_barray_alloc(self.0, &self.1, elem_ty, len as u64, false)?;

                self.builder().build_call(
                    argread_fn,
                    &[tag_ptr.into(), elems_ptr.into(), len_val.into()],
                    "read_arg_bool_array",
                )?;

                barray_value.into()
            }
            ArgumentReadOpDef::ArrInt => {
                let ReadArgs::Array(_, len) = op.args else {
                    bail!("Expected array read args for ArrInt");
                };
                let len: u32 = len
                    .try_into()
                    .map_err(|_| anyhow!("Array length exceeds u32::MAX"))?;
                let elem_ty = self.i64_t().as_basic_type_enum();
                let len_val = self.i64_t().const_int(len as u64, false);

                let (elems_ptr, barray_value) =
                    borrow_array::build_barray_alloc(self.0, &self.1, elem_ty, len as u64, false)?;

                self.builder().build_call(
                    argread_fn,
                    &[tag_ptr.into(), elems_ptr.into(), len_val.into()],
                    "read_arg_int_array",
                )?;
                barray_value.into()
            }
            ArgumentReadOpDef::ArrUInt => {
                let ReadArgs::Array(_, len) = op.args else {
                    bail!("Expected array read args for ArrUInt");
                };
                let len: u32 = len
                    .try_into()
                    .map_err(|_| anyhow!("Array length exceeds u32::MAX"))?;
                let elem_ty = self.i64_t().as_basic_type_enum();
                let len_val = self.i64_t().const_int(len as u64, false);

                let (elems_ptr, barray_value) =
                    borrow_array::build_barray_alloc(self.0, &self.1, elem_ty, len as u64, false)?;

                self.builder().build_call(
                    argread_fn,
                    &[tag_ptr.into(), elems_ptr.into(), len_val.into()],
                    "read_arg_uint_array",
                )?;
                barray_value.into()
            }
            ArgumentReadOpDef::ArrF64 => {
                let ReadArgs::Array(_, len) = op.args else {
                    bail!("Expected array read args for ArrF64");
                };
                let len: u32 = len
                    .try_into()
                    .map_err(|_| anyhow!("Array length exceeds u32::MAX"))?;
                let elem_ty = self.f64_t().as_basic_type_enum();
                let len_val = self.i64_t().const_int(len as u64, false);

                let (elems_ptr, barray_value) =
                    borrow_array::build_barray_alloc(self.0, &self.1, elem_ty, len as u64, false)?;

                self.builder().build_call(
                    argread_fn,
                    &[tag_ptr.into(), elems_ptr.into(), len_val.into()],
                    "read_arg_f64_array",
                )?;
                barray_value.into()
            }
        };
        args.outputs.finish(self.builder(), [result])
    }
}

#[cfg(test)]
mod test {
    // TODO
    // Tket2/HUGR devs will be far better equipped to write
    // appropriate tests for this than I.
}
