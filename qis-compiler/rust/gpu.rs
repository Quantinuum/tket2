//! GPU function API lowering - API version 0.1.0.
//!
//! This module lowers the "tket.gpu" extension to the following external
//! calls:
//!
//! - Validation
//!   ```rust
//!   gpu_validate_api(major: u64, minor: u64, patch: u64) -> bool
//!   ```
//!
//!   Validates that this API is compatible with the GPU library linked
//!   with the final binary.
//!
//!   The present version is 0.1.0.
//!
//!   We opt to allow the linked library to perform validation, as it
//!   may support multiple API versions.
//!
//! - Construction
//!   ```rust
//!   gpu_init(_reserved: u64, gpu_ref_out: *mut u64) -> bool
//!   ```
//!
//!   The reserved argument must be 0 for now.
//!   On success, writes to handle_out a handle to pass to GPU calls.
//!   On failure, returns false.
//!
//! - Destruction
//!   ```rust
//!   gpu_discard(gpu_ref: u64) -> bool
//!   ```
//!
//!   Attempts to clean up the GPU context referenced by gpu_ref.
//!   This can fail - for example, if gpu_ref is invalid, or if
//!   cleanup validation fails. Returns true on success, false on failure.
//!
//! - Error Retrieval
//!   ```rust
//!   gpu_get_error() -> *const i8
//!   ```
//!
//!   Returns a pointer to a null-terminated error string, or nullptr
//!   if no error has been recorded.
//!
//! - Function extraction
//!   ```rust
//!   gpu_get_function_id(name: *const i8, id_out: *mut u64) -> bool
//!   ```
//!
//!   Provides a function id for the given function name through id_out on success,
//!   returning true. If the function name is not found, or gpu_ref is invalid,
//!   returns false.
//!
//! - Function invocation
//!   ```rust
//!   gpu_call(
//!     handle: u64,
//!     function_id: u64,
//!     blob_size: u64,
//!     blob: *const i8,
//!     signature: *const i8
//!   ) -> bool
//!   ```
//!
//!   The GPU api is specified by the user, and arguments are passed
//!   in as a binary blob. To provide validation on the library side,
//!   we provide the blob size and a string representing the function
//!   signature that the user expects. The latter is in the form of
//!   e.g.:
//!   - 'iifb:v' for (i64, u64, f64, bool) -> void
//!   - 'if:i' for (i64, f64) -> i64
//!
//!   where types are encoded as:
//!   - i <=> i64 or u64
//!   - f <=> f64
//!   - b <=> bool
//!
//!  - Result retrieval
//!    ```rust
//!    gpu_get_result(gpu_ref: u64, out_len: u64, out_result: *mut i8) -> bool
//!    ```
//!
//!    Writes the next result in the result queue to out_result,
//!    returning true on success or false on failure. The result
//!    may currently be in the form of i64 or f64, such that in
//!    the current version we always provide out_len = 8, as per
//!    the GpuModule HUGR. This design allows for future expansion
//!    to other result types in future.

use crate::selene_specific;
use anyhow::{Result, bail};
use hugr::extension::prelude::option_type;
use hugr::llvm::{CodegenExtension, CodegenExtsBuilder, inkwell};
use hugr::std_extensions::arithmetic::int_types::int_type;
use hugr::{HugrView, Node};
use inkwell::AddressSpace;
use inkwell::types::{BasicType as _, BasicTypeEnum};
use inkwell::values::{BasicMetadataValueEnum, BasicValue, BasicValueEnum};
use std::cmp::Ordering;
use strum::IntoEnumIterator;
use tket::hugr::extension::ExtensionId;
use tket::hugr::extension::simple_op::{MakeExtensionOp, MakeOpDef};
use tket::hugr::llvm::emit::{EmitFuncContext, EmitOpArgs};
use tket::hugr::ops::ExtensionOp;
use tket::hugr::types::TypeRow;
use tket::hugr::{self};
use tket_qsystem::extension::classical_compute::{self, ComputeOp};
use tket_qsystem::extension::gpu::{self, ConstGpuModule, GpuExtension, GpuOpDef};

const API_MAJOR: u64 = 0;
const API_MINOR: u64 = 1;
const API_PATCH: u64 = 0;

#[derive(Debug, Clone)]
pub struct GpuCodegen;

impl GpuCodegen {
    /// Emit the GetContext operation. This initializes a GPU context
    /// using the external `gpu_init` function, and returns a pair
    /// representing success and the GPU reference itself.
    ///
    /// Although the GpuExtension's GetContext op returns an Option type,
    /// and thus handles failure gracefully, it lowers to a QIS panic, which
    /// is restricted in length and may not be able to convey a generic error
    /// message from the GPU library - which might contain verbose information
    /// pertinent to the failure.
    ///
    /// As such we utilise the verify_gpu_call helper to emit a panic directly
    /// via selene's panic_str function, which can handle c strings.
    fn emit_get_context<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &EmitFuncContext<'c, '_, H>,
        op: EmitOpArgs<'c, '_, ExtensionOp, H>,
    ) -> Result<()> {
        let [index] = &op.inputs[..] else {
            bail!("GetContext operation requires exactly one input");
        };
        // Ensure the API is validated before we make any calls that may
        // be broken in later API versions.
        emit_api_validation(ctx)?;
        let iwc = ctx.iw_context();
        let ts = ctx.typing_session();
        let index = {
            let index = index.into_int_value();
            // always coerce to i64
            let width = index.get_type().get_bit_width();
            match width.cmp(&64) {
                Ordering::Greater => ctx.builder().build_int_truncate(
                    index,
                    iwc.i64_type(),
                    "context_index_trunc",
                )?,
                Ordering::Less => {
                    ctx.builder()
                        .build_int_z_extend(index, iwc.i64_type(), "context_index_zext")?
                }
                Ordering::Equal => index,
            }
        };
        let builder = ctx.builder();

        // allocate space for the gpu_ref that we will return
        let gpu_ref_ptr = builder.build_alloca(iwc.i64_type(), "gpu_ref_ptr")?;

        let get_gpu_init = ctx.get_extern_func(
            "gpu_init",
            iwc.i8_type().fn_type(
                &[
                    iwc.i64_type().into(),
                    iwc.i64_type().ptr_type(AddressSpace::default()).into(),
                ],
                false,
            ),
        )?;

        // `gpu_init` returns false on failure. Otherwise, it provides
        // in gpu_ref_ptr a handle to the GPU context, which is
        // used for subsequent GPU calls.
        let success = builder
            .build_call(
                get_gpu_init,
                &[index.into(), gpu_ref_ptr.into()],
                "gpu_ref_call",
            )?
            .try_as_basic_value()
            .unwrap_basic()
            .into_int_value();
        verify_gpu_call(ctx, success, "gpu_init")?;

        let gpu_ref = builder.build_load(gpu_ref_ptr, "gpu_ref")?;
        let result_t = ts.llvm_sum_type(option_type(int_type(6)))?;
        // Although the result is an option type, we always return true
        // in this lowering: failure is already handled.
        let pair = result_t.build_tag(builder, 1, vec![gpu_ref])?;
        op.outputs
            .finish(ctx.builder(), [pair.as_basic_value_enum()])
    }

    /// Emit the DisposeContext operation. This cleans up the GPU context
    /// using the external `gpu_discard` function.
    ///
    /// On failure, this emits a panic with the error message
    /// from the GPU library.
    fn emit_dispose_context<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &EmitFuncContext<'c, '_, H>,
        op: EmitOpArgs<'c, '_, ExtensionOp, H>,
    ) -> Result<()> {
        let [gpu_ref] = &op.inputs[..] else {
            bail!("DisposeContext operation requires exactly one input");
        };
        let gpu_ref = gpu_ref.into_int_value();
        let iwc = ctx.iw_context();
        // At the point of running discard, we assume the API was already
        // validated - otherwise the context would not have been created.
        // So we assume there is no need to validate again here.

        let gpu_discard = ctx.get_extern_func(
            "gpu_discard",
            iwc.i8_type().fn_type(&[iwc.i64_type().into()], false),
        )?;

        let success = ctx
            .builder()
            .build_call(gpu_discard, &[gpu_ref.into()], "")?
            .try_as_basic_value()
            .unwrap_basic()
            .into_int_value();
        verify_gpu_call(ctx, success, "gpu_discard")?;

        op.outputs.finish(ctx.builder(), [])
    }
    fn emit_lookup_by_name<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &EmitFuncContext<'c, '_, H>,
        op: EmitOpArgs<'c, '_, ExtensionOp, H>,
        name: String,
    ) -> Result<()> {
        let iwc = ctx.iw_context();
        let module = ctx.get_current_module();
        let flag_name = format!("gpu_cache_is_set_function_id_{}", name);
        let stored_id_name = format!("gpu_cache_function_id_{}", name);
        let function_name = format!("gpu_function_id_{}", name);
        let func = match module.get_function(&function_name) {
            None => {
                // We utilise a thread-local global variable to hold
                // the function id once looked up, to avoid repeated
                // lookups.
                let set_flag = module.add_global(iwc.i8_type(), None, &flag_name);
                set_flag.set_thread_local(true);
                set_flag.set_initializer(&iwc.i8_type().const_zero());

                let stored_id = module.add_global(iwc.i64_type(), None, &stored_id_name);
                stored_id.set_thread_local(true);
                stored_id.set_initializer(&iwc.i64_type().const_zero());

                // We wrap the behaviour of looking up the function id into
                // a noinline function to keep the IR clean. If the global
                // ID is all ones, we perform the lookup and store it;
                // otherwise we just return the stored value.
                let current_block = ctx.builder().get_insert_block().unwrap();
                let fn_type = iwc.i64_type().fn_type(&[], false);
                let function = module.add_function(&function_name, fn_type, None);
                let noinline_id =
                    inkwell::attributes::Attribute::get_named_enum_kind_id("noinline");
                let noinline = iwc.create_enum_attribute(noinline_id, 0);
                function.add_attribute(inkwell::attributes::AttributeLoc::Function, noinline);
                let entry = iwc.append_basic_block(function, "entry");
                ctx.builder().position_at_end(entry);

                // Grab the set flag to check if it is initialized.
                let is_set = ctx
                    .builder()
                    .build_load(set_flag.as_pointer_value(), "function_id")?
                    .into_int_value();
                let needs_lookup = ctx.builder().build_int_compare(
                    inkwell::IntPredicate::EQ,
                    is_set,
                    iwc.i8_type().const_zero(),
                    "needs_lookup",
                )?;
                let lookup_block = iwc.append_basic_block(function, "lookup");
                let read_cache_block = iwc.append_basic_block(function, "read_cache");
                ctx.builder().build_conditional_branch(
                    needs_lookup,
                    lookup_block,
                    read_cache_block,
                )?;

                // if it's already set, read from the cache
                ctx.builder().position_at_end(read_cache_block);
                let stored_func_id = ctx
                    .builder()
                    .build_load(stored_id.as_pointer_value(), "function_id")?
                    .into_int_value();
                ctx.builder().build_return(Some(&stored_func_id))?;

                // otherwise, perform the lookup
                ctx.builder().position_at_end(lookup_block);
                // validate before calling if validation hasn't been done yet
                emit_api_validation(ctx)?;
                // call gpu_get_function_id
                let gpu_get_function_id = ctx.get_extern_func(
                    "gpu_get_function_id",
                    iwc.i8_type().fn_type(
                        &[
                            iwc.i8_type().ptr_type(AddressSpace::default()).into(),
                            iwc.i64_type().ptr_type(AddressSpace::default()).into(),
                        ],
                        false,
                    ),
                )?;
                let name_const = ctx
                    .builder()
                    .build_global_string_ptr(&name, "function_name")?;
                let function_id_ptr = ctx
                    .builder()
                    .build_alloca(iwc.i64_type(), "function_id_ptr")?;
                let success = ctx
                    .builder()
                    .build_call(
                        gpu_get_function_id,
                        &[name_const.as_pointer_value().into(), function_id_ptr.into()],
                        "function_id_call",
                    )?
                    .try_as_basic_value()
                    .unwrap_basic()
                    .into_int_value();
                // On failure, we emit a panic
                verify_gpu_call(ctx, success, "gpu_get_function_id")?;
                // Otherwise, store the function id and set the flag
                let func_id = ctx
                    .builder()
                    .build_load(function_id_ptr, "function_id")?
                    .into_int_value();
                ctx.builder()
                    .build_store(stored_id.as_pointer_value(), func_id)?;
                ctx.builder().build_store(
                    set_flag.as_pointer_value(),
                    iwc.i8_type().const_int(1, false),
                )?;
                // return the function id
                ctx.builder().build_return(Some(&func_id))?;

                ctx.builder().position_at_end(current_block);
                function
            }
            Some(f) => f,
        };
        let func_id = ctx
            .builder()
            .build_call(func, &[], "function_id_call")?
            .try_as_basic_value()
            .unwrap_basic()
            .into_int_value();
        op.outputs.finish(ctx.builder(), [func_id.into()])
    }

    fn emit_lookup_by_id<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &EmitFuncContext<'c, '_, H>,
        op: EmitOpArgs<'c, '_, ExtensionOp, H>,
        id: u64,
    ) -> Result<()> {
        // This is a no-op in the current API. We just return
        // the id back.
        op.outputs.finish(
            ctx.builder(),
            [ctx.iw_context().i64_type().const_int(id, false).into()],
        )
    }

    /// Emit the Call operation. This invokes a GPU function using
    /// the external `gpu_call` function.
    ///
    /// Arguments are packed into a binary blob (see the documentation
    /// on `pack_arguments`), and a signature string is generated
    /// to describe the types of the arguments and the expected output
    /// type, which may be used by the GPU library for validation
    /// purposes or for flexibility.
    ///
    /// Upon failure, this emits a panic with the error message
    /// from the GPU library.
    fn emit_gpu_call<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &EmitFuncContext<'c, '_, H>,
        op: EmitOpArgs<'c, '_, ExtensionOp, H>,
        outputs: TypeRow,
    ) -> Result<()> {
        let iwc = ctx.iw_context();

        // Expect at least two inputs: the GPU context and the function id.
        let [gpu_ref, fn_id, fn_args @ ..] = &op.inputs[..] else {
            bail!("GPU call operation requires at least two inputs: context and function");
        };
        let gpu_ref = gpu_ref.into_int_value();
        let func = fn_id.into_int_value();

        let builder = ctx.builder();

        let gpu_call = ctx.get_extern_func(
            "gpu_call",
            iwc.i8_type().fn_type(
                &[
                    gpu_ref.get_type().into(),
                    func.get_type().into(),
                    iwc.i64_type().into(),
                    iwc.i8_type()
                        .ptr_type(inkwell::AddressSpace::default())
                        .into(),
                    iwc.i8_type()
                        .ptr_type(inkwell::AddressSpace::default())
                        .into(),
                ],
                false,
            ),
        )?;

        let mut args: Vec<BasicMetadataValueEnum> = vec![gpu_ref.into(), func.into()];

        let signature = generate_signature(ctx, fn_args, outputs)?;

        // create an i8 array of length blob_size and populate it with
        // the packed arguments
        let (blob, blob_size) = pack_arguments(ctx, fn_args)?;
        let i64_zero = iwc.i64_type().const_zero();
        let blob_ptr = unsafe {
            builder.build_in_bounds_gep(blob, &[i64_zero, i64_zero], "gpu_input_blob_ptr")?
        };

        // pass the blob size, blob pointer, and signature string to gpu_call as
        // arguments
        args.push(iwc.i64_type().const_int(blob_size as u64, false).into());
        args.push(blob_ptr.into());
        args.push(signature.as_pointer_value().into());

        let success = ctx
            .builder()
            .build_call(gpu_call, &args, "")?
            .try_as_basic_value()
            .unwrap_basic()
            .into_int_value();
        // validate the call succeeded (panic otherwise)
        verify_gpu_call(ctx, success, "gpu_call")?;
        op.outputs.finish(ctx.builder(), [gpu_ref.into()])?;
        Ok(())
    }

    fn emit_read_result<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &EmitFuncContext<'c, '_, H>,
        op: EmitOpArgs<'c, '_, ExtensionOp, H>,
        outputs: TypeRow,
    ) -> Result<()> {
        let [gpu_ref] = &op.inputs[..] else {
            bail!("ReadResult operation requires exactly one input");
        };
        let gpu_ref = gpu_ref.into_int_value();

        match &outputs[..] {
            [] => {
                op.outputs
                    .finish(ctx.builder(), [gpu_ref.as_basic_value_enum()])?;
            }
            [single_result] => {
                let iwc = ctx.iw_context();
                let i8_t = iwc.i8_type().as_basic_type_enum();
                let i64_t = iwc.i64_type().as_basic_type_enum();
                let f64_t = iwc.f64_type().as_basic_type_enum();
                let result_ptr_t = iwc.i8_type().ptr_type(AddressSpace::default());

                // Results can currently come in as ints or floats.
                // When this expands we can allocate an appropriate buffer
                // and process as needed; for now, we just handle the two
                // supported types.
                let int_result_ptr = ctx.builder().build_alloca(iwc.i64_type(), "int_result")?;
                let result_ptr =
                    ctx.builder()
                        .build_pointer_cast(int_result_ptr, result_ptr_t, "result_ptr")?;

                let read_func = ctx.get_extern_func(
                    "gpu_get_result",
                    i8_t.fn_type(
                        &[
                            /* gpu_ref    */ iwc.i64_type().into(),
                            /* out_len    */ iwc.i64_type().into(),
                            /* out result */ result_ptr_t.into(),
                        ],
                        false,
                    ),
                )?;

                let call = ctx
                    .builder()
                    .build_call(
                        read_func,
                        &[
                            gpu_ref.into(),
                            iwc.i64_type().const_int(8, false).into(),
                            result_ptr.into(),
                        ],
                        "read_status",
                    )?
                    .try_as_basic_value()
                    .unwrap_basic()
                    .into_int_value();
                // Check status and handle error if needed
                verify_gpu_call(ctx, call, "gpu_get_result_64bits")?;

                match ctx.llvm_type(single_result)? {
                    i if i == i64_t => {
                        // The result type is an integer, so we can just load
                        // it directly.
                        let int_result = ctx
                            .builder()
                            .build_load(int_result_ptr, "int_result")?
                            .into_int_value();
                        op.outputs.finish(
                            ctx.builder(),
                            [
                                gpu_ref.as_basic_value_enum(),
                                int_result.as_basic_value_enum(),
                            ],
                        )?;
                    }
                    f if f == f64_t => {
                        // The result type is a float, so we need to reinterpret
                        // the result pointer from a pointer to int to a pointer
                        // to float, then load it.
                        let float_result_ptr = ctx
                            .builder()
                            .build_bit_cast(
                                int_result_ptr,
                                iwc.f64_type().ptr_type(AddressSpace::default()),
                                "float_result_ptr",
                            )?
                            .into_pointer_value();

                        let float_result = ctx
                            .builder()
                            .build_load(float_result_ptr, "float_result")?
                            .into_float_value();
                        op.outputs.finish(
                            ctx.builder(),
                            [
                                gpu_ref.as_basic_value_enum(),
                                float_result.as_basic_value_enum(),
                            ],
                        )?;
                    }
                    _ => {
                        bail!(
                            "ReadResult operation with a single output expects either i64 or f64, got: {:?}",
                            ctx.llvm_type(single_result)?
                        );
                    }
                }
            }
            other => {
                bail!("ReadResult operation expects either zero or one, got: {other:?}")
            }
        }
        Ok(())
    }

    fn emit_op<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &EmitFuncContext<'c, '_, H>,
        op: EmitOpArgs<'c, '_, ExtensionOp, H>,
    ) -> Result<()> {
        match ComputeOp::<GpuExtension>::from_extension_op(&op.node())? {
            ComputeOp::<GpuExtension>::GetContext => self.emit_get_context(ctx, op),
            ComputeOp::<GpuExtension>::DisposeContext => self.emit_dispose_context(ctx, op),
            ComputeOp::<GpuExtension>::LookupById { id, .. } => self.emit_lookup_by_id(ctx, op, id),
            ComputeOp::<GpuExtension>::LookupByName { name, .. } => {
                self.emit_lookup_by_name(ctx, op, name)
            }
            ComputeOp::<GpuExtension>::Call { outputs, .. } => self.emit_gpu_call(ctx, op, outputs),
            ComputeOp::<GpuExtension>::ReadResult { outputs } => {
                self.emit_read_result(ctx, op, outputs)
            }
            ComputeOp::<GpuExtension>::_Unreachable(x, _) => match x {},
        }
    }

    fn add_gpu_extension<'a, H: HugrView<Node = Node> + 'a, T: MakeOpDef + IntoEnumIterator>(
        self,
        extension_id: ExtensionId,
        builder: CodegenExtsBuilder<'a, H>,
    ) -> CodegenExtsBuilder<'a, H>
    where
        Self: 'a + Sized,
    {
        builder
            .custom_type(
                (
                    extension_id.clone(),
                    classical_compute::CONTEXT_TYPE_NAME.to_owned(),
                ),
                |session, _hugr_type| Ok(session.iw_context().i64_type().into()),
            )
            .custom_type(
                (
                    extension_id.clone(),
                    classical_compute::FUNC_TYPE_NAME.to_owned(),
                ),
                |session, _hugr_type| Ok(session.iw_context().i64_type().into()),
            )
            .custom_type(
                (
                    extension_id.clone(),
                    classical_compute::MODULE_TYPE_NAME.to_owned(),
                ),
                |session, _hugr_type| Ok(session.iw_context().struct_type(&[], false).into()),
            )
            .custom_type(
                (extension_id, classical_compute::RESULT_TYPE_NAME.to_owned()),
                |session, _hugr_type| Ok(session.iw_context().i64_type().into()),
            )
            .simple_extension_op::<T>(move |context, args, _| self.emit_op(context, args))
            .custom_const::<ConstGpuModule>({
                move |ctx, _mod| Ok(ctx.iw_context().const_struct(&[], false).into())
            })
    }
}

impl CodegenExtension for GpuCodegen {
    fn add_extension<'a, H: HugrView<Node = Node> + 'a>(
        self,
        builder: CodegenExtsBuilder<'a, H>,
    ) -> CodegenExtsBuilder<'a, H> {
        self.add_gpu_extension::<'a, H, GpuOpDef>(gpu::EXTENSION_ID, builder)
    }
}

// Helpers

fn emit_api_validation<'c, H: HugrView<Node = Node>>(
    ctx: &EmitFuncContext<'c, '_, H>,
) -> Result<()> {
    let iwc = ctx.iw_context();
    let module = ctx.get_current_module();
    let builder = ctx.builder();
    let func = match module.get_function("run_gpu_validation") {
        Some(f) => f,
        None => {
            let stored = module.add_global(iwc.i8_type(), None, "gpu_validated");
            stored.set_thread_local(true);
            stored.set_initializer(&iwc.i8_type().const_zero());

            let current_block = builder.get_insert_block().unwrap();
            let fn_type = iwc.void_type().fn_type(&[], false);
            let function = module.add_function("run_gpu_validation", fn_type, None);
            let noinline_id = inkwell::attributes::Attribute::get_named_enum_kind_id("noinline");
            let noinline = iwc.create_enum_attribute(noinline_id, 0);
            function.add_attribute(inkwell::attributes::AttributeLoc::Function, noinline);
            let entry = iwc.append_basic_block(function, "entry");
            builder.position_at_end(entry);

            let already_validated = builder.build_int_compare(
                inkwell::IntPredicate::NE,
                builder
                    .build_load(stored.as_pointer_value(), "validated")?
                    .into_int_value(),
                iwc.i8_type().const_zero(),
                "already_validated",
            )?;

            let done_block = iwc.append_basic_block(function, "done");
            let validate_block = iwc.append_basic_block(function, "validate");

            // if we have already validated, return
            builder.build_conditional_branch(already_validated, done_block, validate_block)?;
            builder.position_at_end(done_block);
            builder.build_return(None)?;

            // otherwise, perform validation
            builder.position_at_end(validate_block);

            let major = iwc.i64_type().const_int(API_MAJOR, false);
            let minor = iwc.i64_type().const_int(API_MINOR, false);
            let patch = iwc.i64_type().const_int(API_PATCH, false);
            let validate_func = ctx.get_extern_func(
                "gpu_validate_api",
                iwc.i8_type().fn_type(
                    &[
                        iwc.i64_type().into(),
                        iwc.i64_type().into(),
                        iwc.i64_type().into(),
                    ],
                    false,
                ),
            )?;
            let success = builder
                .build_call(
                    validate_func,
                    &[major.into(), minor.into(), patch.into()],
                    "validate_call",
                )?
                .try_as_basic_value()
                .unwrap_basic()
                .into_int_value();
            verify_gpu_call(ctx, success, "gpu_validate_api")?;
            // Mark as validated
            builder.build_store(stored.as_pointer_value(), iwc.i8_type().const_int(1, false))?;
            builder.build_return(None)?;
            builder.position_at_end(current_block);
            function
        }
    };
    let _ = ctx
        .builder()
        .build_call(func, &[], "run_gpu_validation_call")?
        .try_as_basic_value()
        .unwrap_instruction();
    Ok(())
}

/// A helper method for handling errors from the external
/// library.
///
/// For GPU calls, we have no recovery mechanism built-in, so
/// we emit a panic with the error message from the library.
/// If no error message is available, we use a default message.
///
/// Utilises the selene panic_str function to emit the panic,
/// rather than the size-restricted QIS panic.
fn emit_panic_with_gpu_error<'c, H: HugrView<Node = Node>>(
    ctx: &EmitFuncContext<'c, '_, H>,
) -> Result<()> {
    let module = ctx.get_current_module();
    let iwc = ctx.iw_context();
    let builder = ctx.builder();
    let handle_error = match module.get_function("gpu_error_handler") {
        None => {
            let current_block = builder.get_insert_block().unwrap();
            let fn_type = iwc.void_type().fn_type(&[], false);
            let function = module.add_function("gpu_error_handler", fn_type, None);
            let noinline_id = inkwell::attributes::Attribute::get_named_enum_kind_id("noinline");
            let noinline = iwc.create_enum_attribute(noinline_id, 0);
            function.add_attribute(inkwell::attributes::AttributeLoc::Function, noinline);
            let entry = iwc.append_basic_block(function, "entry");
            builder.position_at_end(entry);
            // Try to get the error message from the GPU library.
            let gpu_get_error = ctx.get_extern_func(
                "gpu_get_error",
                iwc.i8_type()
                    .ptr_type(AddressSpace::default())
                    .fn_type(&[], false),
            )?;

            let error_message = ctx
                .builder()
                .build_call(gpu_get_error, &[], "error_message")?
                .try_as_basic_value()
                .unwrap_basic()
                .into_pointer_value();

            // If it's null, replace with "No error message available"
            let error_message = ctx.builder().build_select(
                ctx.builder().build_int_compare(
                    inkwell::IntPredicate::EQ,
                    error_message,
                    iwc.i8_type().ptr_type(AddressSpace::default()).const_null(),
                    "is_null",
                )?,
                ctx.builder()
                    .build_global_string_ptr("No error message available", "no_gpu_error")?
                    .as_pointer_value(),
                error_message,
                "error_message_nonnull",
            )?;

            // Call panic_str with a generic error code and the message.
            let _ = ctx
                .builder()
                .build_call(
                    selene_specific::panic_str_fn(ctx)?,
                    &[
                        iwc.i32_type().const_int(70002, false).into(),
                        error_message.into(),
                    ],
                    "panic_str_call",
                )?
                .try_as_basic_value()
                .unwrap_instruction();
            builder.build_unreachable()?;
            builder.position_at_end(current_block);
            function
        }
        Some(f) => f,
    };
    let _ = builder
        .build_call(handle_error, &[], "gpu_error_handler_call")?
        .try_as_basic_value()
        .unwrap_basic();
    Ok(())
}

/// A helper to check return codes and handle the branching
/// to `emit_panic_with_gpu_error` upon error.
/// If the return value is true, we continue as normal.
fn verify_gpu_call<'c, H: HugrView<Node = Node>>(
    ctx: &EmitFuncContext<'c, '_, H>,
    success_flag: inkwell::values::IntValue<'c>,
    op_name: &str,
) -> Result<()> {
    let module = ctx.get_current_module();
    let iwc = ctx.iw_context();
    let builder = ctx.builder();
    // Rather than emiting the validation code inline after each gpu call,
    // we define a response validation function and call into it after each
    // gpu call. Keeping it noinline keeps the resulting IR clean, but
    // we can always inline it later if required.
    let handle_error = match module.get_function("validate_gpu_response") {
        None => {
            let current_block = builder.get_insert_block().unwrap();
            let fn_type = iwc.void_type().fn_type(&[iwc.i8_type().into()], false);
            let function = module.add_function("validate_gpu_response", fn_type, None);
            let noinline_id = inkwell::attributes::Attribute::get_named_enum_kind_id("noinline");
            let noinline = iwc.create_enum_attribute(noinline_id, 0);
            function.add_attribute(inkwell::attributes::AttributeLoc::Function, noinline);
            let entry = iwc.append_basic_block(function, "entry");
            builder.position_at_end(entry);
            let ok_block = iwc.append_basic_block(function, "ok");
            let err_block = iwc.append_basic_block(function, "err");
            let success_bool = builder.build_int_compare(
                inkwell::IntPredicate::NE,
                function.get_first_param().unwrap().into_int_value(),
                iwc.i8_type().const_zero(),
                "success",
            )?;
            builder.build_conditional_branch(success_bool, ok_block, err_block)?;
            // on error, we panic
            builder.position_at_end(err_block);
            emit_panic_with_gpu_error(ctx)?;
            builder.build_unreachable()?;
            // on success, return
            builder.position_at_end(ok_block);
            builder.build_return(None)?;
            builder.position_at_end(current_block);
            function
        }
        Some(f) => f,
    };
    let _ = builder
        .build_call(
            handle_error,
            &[success_flag.into()],
            format!("{op_name}_handle_error").as_str(),
        )?
        .try_as_basic_value()
        .unwrap_instruction();
    Ok(())
}

/// A helper to generate a type signature string
/// for the GPU call, given a list of arguments.
///
/// The signature is of the form "arg_types:output_type"
/// where arg_types is a sequence of characters representing
/// the types of each argument, and output_type is
/// a single character representing the output type.
///
///  Types are encoded as:
///   i - i64 or u64
///   f - f64
///   b - bool
fn generate_signature<'c, H: HugrView<Node = Node>>(
    ctx: &EmitFuncContext<'c, '_, H>,
    fn_args: &[BasicValueEnum<'c>],
    outputs: TypeRow,
) -> Result<inkwell::values::GlobalValue<'c>> {
    let iwc = ctx.iw_context();
    let builder = ctx.builder();

    let mut type_str = String::new();
    for &arg in fn_args {
        type_str.push(match arg.get_type() {
            BasicTypeEnum::IntType(i) if i.get_bit_width() == 1 => 'b',
            BasicTypeEnum::IntType(i) if i.get_bit_width() == 64 => 'i',
            BasicTypeEnum::FloatType(_) => 'f',
            _ => {
                bail!(
                    "Unsupported argument type for GPU call: {:?}",
                    arg.get_type()
                );
            }
        });
    }
    type_str.push(':');
    type_str.push(match &outputs[..] {
        [] => 'v',
        [i] if ctx.llvm_type(i)? == iwc.i64_type().as_basic_type_enum() => 'i',
        [f] if ctx.llvm_type(f)? == iwc.f64_type().as_basic_type_enum() => 'f',
        _ => bail!("GPU calls expect either no outputs, or a single output of type i64 or f64"),
    });
    let type_str_const = builder.build_global_string_ptr(&type_str, "arg_types")?;
    Ok(type_str_const)
}

/// A helper to pack the arguments into a binary blob
/// suitable for passing to the GPU call.
///
/// Returns a pointer to the blob and its size in bytes.
///
/// To pack the arguments, we concatenate their binary
/// representation as follows:
/// - Booleans are encoded as 1-byte values (0x00 or 0x01)
/// - Integers are encoded as 8-byte values (u64, matching system endian)
/// - Floats are encoded as 8-byte values (f64, matching system endian)
/// - Each argument is encoded immediately following the previous,
///   without padding, without managing alignment requirements.
///
/// It is recommended that implementations avoid issues that may arise
/// from endian or alignment assumptions by copying blob sections into
/// appropriately aligned local variables rather than iterating through
/// the blob directly.
///
/// The blob is allocated on the stack: the implementation must not
/// attempt to free it.
fn pack_arguments<'c, H: HugrView<Node = Node>>(
    ctx: &EmitFuncContext<'c, '_, H>,
    fn_args: &[BasicValueEnum<'c>],
) -> Result<(inkwell::values::PointerValue<'c>, u32)> {
    //
    let iwc = ctx.iw_context();
    let builder = ctx.builder();

    let mut blob_size = 0;
    for &arg in fn_args {
        blob_size += match arg.get_type() {
            BasicTypeEnum::IntType(i) if i.get_bit_width() == 1 => 1,
            BasicTypeEnum::IntType(i) if i.get_bit_width() == 64 => 8,
            BasicTypeEnum::FloatType(_) => 8,
            _ => {
                bail!(
                    "Unsupported argument type for GPU call: {:?}",
                    arg.get_type()
                );
            }
        };
    }

    let blob_ty = iwc.i8_type().array_type(blob_size);
    let blob = builder.build_alloca(blob_ty, "gpu_input_blob")?;
    let mut offset = 0;
    for &arg in fn_args {
        let dest_ptr = unsafe {
            builder.build_in_bounds_gep(
                blob,
                &[
                    iwc.i64_type().const_zero(),
                    iwc.i64_type().const_int(offset as u64, false),
                ],
                "dest_ptr",
            )?
        };
        match arg.get_type() {
            BasicTypeEnum::IntType(i) if i.get_bit_width() == 1 => {
                // extend to i8
                let arg =
                    builder.build_int_z_extend(arg.into_int_value(), iwc.i8_type(), "arg_i8")?;
                builder.build_store(dest_ptr, arg)?;
                offset += 1;
            }
            BasicTypeEnum::IntType(i) if i.get_bit_width() == 64 => {
                // Store the integer into an aligned i64 temporary,
                // then copy its bytes into the blob in an unaligned manner.
                let tmp = builder.build_alloca(iwc.i64_type(), "arg_i64_tmp")?;
                builder.build_store(tmp, arg)?;
                let src_i8 = builder.build_pointer_cast(
                    tmp,
                    iwc.i8_type()
                        .array_type(8)
                        .ptr_type(AddressSpace::default()),
                    "arg_i8_ptr",
                )?;
                builder.build_memcpy(dest_ptr, 1, src_i8, 1, iwc.i64_type().const_int(8, false))?;
                offset += 8;
            }
            BasicTypeEnum::FloatType(_) => {
                // Store the float into an aligned f64 temporary,
                // then copy its bytes into the blob in an unaligned manner.
                let tmp = builder.build_alloca(iwc.f64_type(), "arg_f64_tmp")?;
                builder.build_store(tmp, arg)?;
                let src_i8 = builder.build_pointer_cast(
                    tmp,
                    iwc.i8_type()
                        .array_type(8)
                        .ptr_type(AddressSpace::default()),
                    "arg_i8_ptr",
                )?;
                builder.build_memcpy(dest_ptr, 1, src_i8, 1, iwc.i64_type().const_int(8, false))?;
                offset += 8;
            }
            // We have already validated types when sizing the blob,
            // so this case is unreachable.
            _ => unreachable!(),
        }
    }
    Ok((blob, blob_size))
}

#[cfg(test)]
mod test {
    use hugr::llvm::check_emission;
    use hugr::llvm::test::{TestContext, llvm_ctx, single_op_hugr};
    use rstest::Context;
    use tket::circuit::TypeRow;
    use tket::hugr::extension::prelude::{bool_t, usize_t};
    use tket::hugr::std_extensions::arithmetic::float_types::float64_type;
    use tket::hugr::std_extensions::arithmetic::int_types::INT_TYPES;
    use tket::hugr::type_row;
    use tket_qsystem::extension::gpu::GpuOp;

    use super::*;

    #[rstest::rstest]
    #[case::get_context(GpuOp::GetContext)]
    #[case::dispose_context(GpuOp::DisposeContext)]
    #[case::lookup_by_id(GpuOp::LookupById {
        id: 42,
        inputs: type_row![].into(),
        outputs: type_row![].into(),
    })]
    #[case::lookup_by_name(GpuOp::LookupByName {
        name: "example_function".into(),
        inputs: type_row![].into(),
        outputs: type_row![].into(),
    })]
    #[case::call_args(GpuOp::Call {
        inputs: TypeRow::from(vec![
            usize_t(),
            INT_TYPES[6].clone(),
            float64_type(),
            bool_t()
        ]),
        outputs: TypeRow::from(vec![]),
    })]
    #[case::call_ret_int(GpuOp::Call {
        inputs: type_row![],
        outputs: TypeRow::from(usize_t()),
    })]
    #[case::call_ret_float(GpuOp::Call {
        inputs: type_row![],
        outputs: TypeRow::from(float64_type()),
    })]
    #[case::read_result_int(GpuOp::ReadResult {
        outputs: TypeRow::from(usize_t()),
    })]
    #[case::read_result_float(GpuOp::ReadResult {
        outputs: TypeRow::from(float64_type()),
    })]
    fn gpu_codegen(#[context] ctx: Context, mut llvm_ctx: TestContext, #[case] op: GpuOp) {
        let _g = {
            let desc = ctx.description.unwrap();
            let mut settings = insta::Settings::clone_current();
            let suffix = settings
                .snapshot_suffix()
                .map_or_else(|| desc.to_string(), |s| format!("{s}_{desc}"));
            settings.set_snapshot_suffix(suffix);
            settings
        }
        .bind_to_scope();

        llvm_ctx.add_extensions(move |cge| {
            cge.add_default_prelude_extensions()
                .add_default_int_extensions()
                .add_float_extensions()
                .add_extension(GpuCodegen)
        });
        let hugr = single_op_hugr(op.into());
        check_emission!(hugr, llvm_ctx);
    }
}
