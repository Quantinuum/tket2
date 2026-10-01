//! Pointer-cell lowering for QIS runtimes with synchronized opaque storage.
//!
//! The runtime must export `___ptr_alloc(i64 size, i64 alignment) -> ptr`,
//! `___ptr_get_ptr(ptr) -> ptr`, and `___ptr_free/lock/unlock(ptr) -> void`.
//! Allocation initializes a mutex and returns a non-null opaque handle, or
//! terminates on failure. Its payload must satisfy the requested size/alignment.
//! Free tears down the mutex and allocation after the last handle is released.
//! Lock/unlock provide exclusive access and acquire/release synchronization.
//!
//! HUGR owns the payload layout and reference counting. Map holds the lock
//! throughout its callback; re-entering the same cell is unsupported. Thread
//! returned handles or add order edges when operations require a relative order.
//! These symbols require runtime support; the legacy Selene heap API alone is
//! insufficient. No target silently falls back to unsynchronized accesses.

use anyhow::{Result, anyhow};
use hugr::llvm::emit::EmitFuncContext;
use hugr::llvm::extension::ptr::PtrCodegen;
use hugr::llvm::inkwell::{
    types::{BasicType, StructType},
    values::PointerValue,
};
use hugr::{HugrView, Node};

/// Synchronized pointer cells provided by the QIS runtime.
#[derive(Clone, Debug, Default)]
pub struct QisPtrCodegen;

fn hook_void<H: HugrView<Node = Node>>(
    ctx: &mut EmitFuncContext<H>,
    name: &str,
    ptr: PointerValue,
) -> Result<()> {
    let function = ctx.get_extern_func(
        name,
        ctx.iw_context()
            .void_type()
            .fn_type(&[ctx.llvm_ptr_type().into()], false),
    )?;
    ctx.builder().build_call(function, &[ptr.into()], "")?;
    Ok(())
}

impl PtrCodegen for QisPtrCodegen {
    fn emit_alloc<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &mut EmitFuncContext<'c, '_, H>,
        layout: StructType<'c>,
    ) -> Result<PointerValue<'c>> {
        let i64_t = ctx.iw_context().i64_type();
        let function = ctx.get_extern_func(
            "___ptr_alloc",
            ctx.llvm_ptr_type()
                .fn_type(&[i64_t.into(), i64_t.into()], false),
        )?;
        Ok(ctx
            .builder()
            .build_call(
                function,
                &[
                    layout
                        .size_of()
                        .ok_or_else(|| anyhow!("Unsized pointer cell"))?
                        .into(),
                    layout.get_alignment().into(),
                ],
                "",
            )?
            .try_as_basic_value()
            .unwrap_basic()
            .into_pointer_value())
    }
    fn emit_free<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &mut EmitFuncContext<'c, '_, H>,
        ptr: PointerValue<'c>,
    ) -> Result<()> {
        hook_void(ctx, "___ptr_free", ptr)
    }
    fn emit_get_ptr<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &mut EmitFuncContext<'c, '_, H>,
        ptr: PointerValue<'c>,
        _layout: StructType<'c>,
    ) -> Result<PointerValue<'c>> {
        let function = ctx.get_extern_func(
            "___ptr_get_ptr",
            ctx.llvm_ptr_type()
                .fn_type(&[ctx.llvm_ptr_type().into()], false),
        )?;
        Ok(ctx
            .builder()
            .build_call(function, &[ptr.into()], "")?
            .try_as_basic_value()
            .unwrap_basic()
            .into_pointer_value())
    }
    fn emit_lock<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &mut EmitFuncContext<'c, '_, H>,
        ptr: PointerValue<'c>,
    ) -> Result<()> {
        hook_void(ctx, "___ptr_lock", ptr)
    }
    fn emit_unlock<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &mut EmitFuncContext<'c, '_, H>,
        ptr: PointerValue<'c>,
    ) -> Result<()> {
        hook_void(ctx, "___ptr_unlock", ptr)
    }
}

#[cfg(test)]
mod test;
