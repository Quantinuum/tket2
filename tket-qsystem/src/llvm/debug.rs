//! LLVM lowering implementations for "tket.debug" extension.

use crate::llvm::prelude::emit_global_string;
use anyhow::{Result, anyhow, bail};
use hugr::extension::simple_op::MakeExtensionOp;
use hugr::llvm::CodegenExtsBuilder;
use hugr::llvm::custom::CodegenExtension;
use hugr::llvm::emit::{EmitFuncContext, EmitOpArgs};
use hugr::llvm::inkwell::AddressSpace;
use hugr::ops::ExtensionOp;
use hugr::{HugrView, Node};
use tket::extension::debug::{DEBUG_EXTENSION_ID, STATE_RESULT_OP_ID, StateResult};

use super::array_utils::{ArrayLowering, ElemType, struct_1d_arr_alloc, struct_1d_arr_ptr_t};

static TAG_PREFIX: &str = "USER:";

/// Codegen extension for debug functionality.
#[derive(Default)]
pub struct DebugCodegenExtension<AL: ArrayLowering> {
    array_lowering: AL,
}

impl<AL: ArrayLowering> DebugCodegenExtension<AL> {
    /// Creates a new [DebugCodegenExtension] with specified array lowering.
    pub const fn new(array_lowering: AL) -> Self {
        Self { array_lowering }
    }
}

impl<AL: ArrayLowering> CodegenExtension for DebugCodegenExtension<AL> {
    fn add_extension<'a, H: HugrView<Node = Node> + 'a>(
        self,
        builder: CodegenExtsBuilder<'a, H>,
    ) -> CodegenExtsBuilder<'a, H>
    where
        Self: 'a,
    {
        builder.extension_op(DEBUG_EXTENSION_ID, STATE_RESULT_OP_ID, {
            move |context, args| self.emit_state_result(context, args)
        })
    }
}

impl<AL: ArrayLowering> DebugCodegenExtension<AL> {
    /// Lower the `debug` `StateResult` op.
    fn emit_state_result<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &mut EmitFuncContext<'c, '_, H>,
        args: EmitOpArgs<'c, '_, ExtensionOp, H>,
    ) -> Result<()> {
        let builder = ctx.builder();

        // Types (qubits are just i64s).
        let iw_ctx = ctx.iw_context();
        let void_t = iw_ctx.void_type();
        let i8_ptr_t = iw_ctx.i8_type().ptr_type(AddressSpace::default());
        let i64_t = iw_ctx.i64_type();

        // Tag arguments.
        let state_result = StateResult::from_extension_op(args.node().as_ref())?;
        let tag = state_result.tag;
        if tag.is_empty() {
            bail!("Empty state result tag received");
        }
        let tag_ptr = emit_global_string(ctx, tag, "res_", format!("{TAG_PREFIX}STATE:"))?;
        let tag_len = {
            let mut l = builder
                .build_load(tag_ptr.into_pointer_value(), "tag_len")?
                .into_int_value();
            if l.get_type() != i64_t {
                l = builder.build_int_z_extend(l, i64_t, "tag_len")?;
            }
            l
        };

        // Qubit array argument.
        let array_len = state_result.num_qubits;
        let [qubits] = args
            .inputs
            .try_into()
            .map_err(|_| anyhow!(format!("StateResult expects a qubit array argument")))?;
        let qubits_array = self.array_lowering.array_to_ptr(builder, qubits)?;
        let (qubits_ptr, _) = struct_1d_arr_alloc(
            iw_ctx,
            builder,
            array_len.try_into()?,
            &ElemType::Int,
            qubits_array,
        )?;

        // Build the function call.
        let fn_state_result = ctx.get_extern_func(
            "print_state_result",
            void_t.fn_type(
                &[
                    i8_ptr_t.into(),
                    i64_t.into(),
                    struct_1d_arr_ptr_t(iw_ctx, &ElemType::Int).into(),
                ],
                false,
            ),
        )?;

        builder.build_call(
            fn_state_result,
            &[tag_ptr.into(), tag_len.into(), qubits_ptr.into()],
            "print_state_result",
        )?;
        args.outputs.finish(builder, [qubits])
    }
}

#[cfg(test)]
mod test {
    use tket::extension::debug::StateResult;

    use hugr::extension::simple_op::MakeRegisteredOp;
    use hugr::llvm::check_emission;
    use hugr::llvm::test::TestContext;
    use hugr::llvm::test::llvm_ctx;
    use hugr::llvm::test::single_op_hugr;

    use crate::llvm::array_utils::DEFAULT_HEAP_ARRAY_LOWERING;
    use crate::llvm::array_utils::DEFAULT_STACK_ARRAY_LOWERING;
    use crate::llvm::prelude::QISPreludeCodegen;

    use rstest::rstest;

    use super::*;

    #[rstest]
    #[case::state_result(1, StateResult::new("test_state_result".to_string(), 2), &DEFAULT_STACK_ARRAY_LOWERING)]
    #[case::state_result(2, StateResult::new("test_state_result".to_string(), 2), &DEFAULT_HEAP_ARRAY_LOWERING)]
    fn emit_debug_codegen(
        #[case] _i: i32,
        #[with(_i)] mut llvm_ctx: TestContext,
        #[case] op: StateResult,
        #[case] array_lowering: &'static (impl ArrayLowering + Clone),
    ) {
        let pcg = QISPreludeCodegen;
        llvm_ctx.add_extensions(move |ceb| {
            ceb.add_extension(DebugCodegenExtension::new(array_lowering.clone()))
                .add_extension(array_lowering.codegen_extension())
                .add_prelude_extensions(pcg.clone())
                .add_default_int_extensions()
                .add_float_extensions()
        });
        let ext_op = op.to_extension_op().unwrap().into();
        let hugr = single_op_hugr(ext_op);
        check_emission!(hugr, llvm_ctx);
    }
}
