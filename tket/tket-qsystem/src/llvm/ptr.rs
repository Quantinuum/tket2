//! Pointer-cell lowering for QIS runtimes with synchronized opaque storage.
//!
//! Proposed runtime ABI (test implementations only; production support is required):
//! `___ptr_create(i64 size, i64 alignment, ptr source) -> ptr`,
//! `___ptr_inc_refcount(ptr handle, i64 delta, ptr destination) -> i1`,
//! `___ptr_get_ptr(ptr) -> ptr`, and `___ptr_lock/unlock(ptr) -> void`.
//!
//! Creation transfers the initialized value into aligned storage with one owner.
//! The runtime owns reference counts, mutexes, storage and final payload recovery;
//! there is no HUGR counter prefix in the payload. Creation must return a live,
//! non-null handle or terminate execution. Dup adjusts ownership by +1
//! with a null destination. Free adjusts by -1 with disjoint typed output storage:
//! true transfers the final payload and retires the cell, while false leaves the
//! output untouched. Negative adjustment with a null destination would discard
//! without a destructor; generated Free always supplies storage to recover its
//! linear value. Final extraction is entered without a caller-held cell lock.
//! No access or unlock follows consumption of a handle.
//!
//! Payload projection requires the lock. Lock/unlock provide exclusive access and
//! acquire/release synchronization. Map holds the lock throughout its callback;
//! same-cell re-entry and callbacks that do not return normally are unsupported.
//! Eq compares opaque identity and returns both handles in input order, without
//! runtime hooks or payload/reference-count access. Handles for the same live
//! cell must have identical addresses and distinct live cells must differ.
//! Thread returned handles or add order edges when relative ordering is required.
//! These symbols are proposals, not currently available production runtime exports.

use anyhow::{Result, anyhow};
use hugr::llvm::emit::EmitFuncContext;
use hugr::llvm::extension::ptr::PtrCodegen;
use hugr::llvm::inkwell::{
    types::{BasicType, BasicTypeEnum},
    values::{BasicValueEnum, IntValue, PointerValue},
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

fn adjust_ownership<'c, H: HugrView<Node = Node>>(
    ctx: &mut EmitFuncContext<'c, '_, H>,
    ptr: PointerValue<'c>,
    delta: i64,
    destination: PointerValue<'c>,
) -> Result<IntValue<'c>> {
    let int = ctx.iw_context().i64_type();
    let pointer = ctx.llvm_ptr_type();
    let function = ctx.get_extern_func(
        "___ptr_inc_refcount",
        ctx.iw_context()
            .bool_type()
            .fn_type(&[pointer.into(), int.into(), pointer.into()], false),
    )?;
    Ok(ctx
        .builder()
        .build_call(
            function,
            &[
                ptr.into(),
                int.const_int(delta as u64, true).into(),
                destination.into(),
            ],
            "",
        )?
        .try_as_basic_value()
        .unwrap_basic()
        .into_int_value())
}

impl PtrCodegen for QisPtrCodegen {
    fn emit_new<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &mut EmitFuncContext<'c, '_, H>,
        initial_value: BasicValueEnum<'c>,
    ) -> Result<PointerValue<'c>> {
        let value_type = initial_value.get_type();
        // A one-field struct has the value's allocation size/alignment, without
        // adding any ownership metadata to the runtime payload.
        let layout = ctx.iw_context().struct_type(&[value_type], false);
        let source_builder = ctx.iw_context().create_builder();
        let entry = ctx.func().get_first_basic_block().unwrap();
        if let Some(first) = entry.get_first_instruction() {
            source_builder.position_before(&first);
        } else {
            source_builder.position_at_end(entry);
        }
        let source = source_builder.build_alloca(value_type, "ptr.initial")?;
        ctx.builder().build_store(source, initial_value)?;
        let i64_t = ctx.iw_context().i64_type();
        let function = ctx.get_extern_func(
            "___ptr_create",
            ctx.llvm_ptr_type().fn_type(
                &[i64_t.into(), i64_t.into(), ctx.llvm_ptr_type().into()],
                false,
            ),
        )?;
        Ok(ctx
            .builder()
            .build_call(
                function,
                &[
                    layout
                        .size_of()
                        .ok_or_else(|| anyhow!("Unsized pointer payload"))?
                        .into(),
                    layout.get_alignment().into(),
                    source.into(),
                ],
                "",
            )?
            .try_as_basic_value()
            .unwrap_basic()
            .into_pointer_value())
    }

    fn emit_dup<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &mut EmitFuncContext<'c, '_, H>,
        ptr: PointerValue<'c>,
        _value_type: BasicTypeEnum<'c>,
    ) -> Result<()> {
        let destination = ctx.llvm_ptr_type().const_null();
        adjust_ownership(ctx, ptr, 1, destination)?;
        Ok(())
    }

    fn emit_free<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &mut EmitFuncContext<'c, '_, H>,
        ptr: PointerValue<'c>,
        _value_type: BasicTypeEnum<'c>,
        destination: PointerValue<'c>,
    ) -> Result<IntValue<'c>> {
        adjust_ownership(ctx, ptr, -1, destination)
    }

    fn emit_get_ptr<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &mut EmitFuncContext<'c, '_, H>,
        ptr: PointerValue<'c>,
        _value_type: BasicTypeEnum<'c>,
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
