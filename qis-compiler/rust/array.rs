//! Implementation for heap allocation of arrays using the selene heap.
use anyhow::Result;
use hugr::llvm::emit::EmitFuncContext;
use hugr::llvm::extension::collections::array::ArrayCodegen;
use hugr::llvm::extension::collections::borrow_array::BorrowArrayCodegen;
use hugr::llvm::inkwell;
use hugr::{HugrView, Node};
use inkwell::AddressSpace;
use inkwell::values::{IntValue, PointerValue};
use tket::hugr;
use tket::hugr::llvm::extension::PreludeCodegen;
use tket::hugr::llvm::inkwell::values::BasicValueEnum;
use tket_qsystem::llvm::array_utils::HeapArrayLowering;

#[derive(Clone, Debug, Default)]
/// Codegen extension for array operations using the selene heap.
pub struct SeleneHeapArrayCodegen;

impl ArrayCodegen for SeleneHeapArrayCodegen {
    fn emit_allocate_array<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &mut EmitFuncContext<'c, '_, H>,
        size: IntValue<'c>,
    ) -> Result<PointerValue<'c>> {
        let iw_ctx = ctx.typing_session().iw_context();
        let malloc_sig = iw_ctx
            .i8_type()
            .ptr_type(AddressSpace::default())
            .fn_type(&[iw_ctx.i64_type().into()], false);
        let malloc = ctx.get_extern_func("heap_alloc", malloc_sig)?;
        let res = ctx
            .builder()
            .build_call(malloc, &[size.into()], "")?
            .try_as_basic_value()
            .unwrap_basic();
        Ok(res.into_pointer_value())
    }

    fn emit_free_array<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &mut EmitFuncContext<'c, '_, H>,
        ptr: PointerValue<'c>,
    ) -> Result<()> {
        let iw_ctx = ctx.typing_session().iw_context();
        let ptr_ty = iw_ctx.i8_type().ptr_type(AddressSpace::default());
        let ptr = ctx.builder().build_bit_cast(ptr, ptr_ty, "")?;

        let free_sig = iw_ctx.void_type().fn_type(&[ptr_ty.into()], false);
        let free = ctx.get_extern_func("heap_free", free_sig)?;
        ctx.builder().build_call(free, &[ptr.into()], "")?;
        Ok(())
    }
}

impl SeleneHeapArrayCodegen {
    /// [HeapArrayLowering] using the selene heap.
    pub const LOWERING: HeapArrayLowering<Self> = HeapArrayLowering::new(SeleneHeapArrayCodegen);
}

#[derive(Clone, Debug, Default)]
/// Codegen extension for borrow array operations using the selene heap.
pub struct SeleneHeapBorrowArrayCodegen<PCG>(pub PCG);

impl<PCG: PreludeCodegen> BorrowArrayCodegen for SeleneHeapBorrowArrayCodegen<PCG> {
    fn emit_allocate_array<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &mut EmitFuncContext<'c, '_, H>,
        size: IntValue<'c>,
    ) -> Result<PointerValue<'c>> {
        let iw_ctx = ctx.typing_session().iw_context();
        let malloc_sig = iw_ctx
            .i8_type()
            .ptr_type(AddressSpace::default())
            .fn_type(&[iw_ctx.i64_type().into()], false);
        let malloc = ctx.get_extern_func("heap_alloc", malloc_sig)?;
        let res = ctx
            .builder()
            .build_call(malloc, &[size.into()], "")?
            .try_as_basic_value()
            .unwrap_basic();
        Ok(res.into_pointer_value())
    }

    fn emit_free_array<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &mut EmitFuncContext<'c, '_, H>,
        ptr: PointerValue<'c>,
    ) -> Result<()> {
        let iw_ctx = ctx.typing_session().iw_context();
        let ptr_ty = iw_ctx.i8_type().ptr_type(AddressSpace::default());
        let ptr = ctx.builder().build_bit_cast(ptr, ptr_ty, "")?;

        let free_sig = iw_ctx.void_type().fn_type(&[ptr_ty.into()], false);
        let free = ctx.get_extern_func("heap_free", free_sig)?;
        ctx.builder().build_call(free, &[ptr.into()], "")?;
        Ok(())
    }

    fn emit_panic<H: HugrView<Node = Node>>(
        &self,
        ctx: &mut EmitFuncContext<H>,
        err: BasicValueEnum,
    ) -> Result<()> {
        self.0.emit_panic(ctx, err)
    }
}
