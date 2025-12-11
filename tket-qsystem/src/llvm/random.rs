//! LLVM lowering implementations for the "tket.qsystem.random" extension.

use tket::hugr::{self, llvm::inkwell};

use crate::extension::random::{self, CONTEXT_TYPE_NAME, RandomOp};
use anyhow::{Result, anyhow};
use hugr::llvm::custom::CodegenExtension;
use hugr::llvm::emit::EmitOpArgs;
use hugr::llvm::emit::func::EmitFuncContext;
use inkwell::builder::Builder;
use inkwell::context::Context;
use inkwell::types::{BasicTypeEnum, FloatType, IntType};
use inkwell::values::{BasicValueEnum, FunctionValue};
use tket::hugr::ops::ExtensionOp;
use tket::hugr::{HugrView, Node};

/// Codegen extension for `tket.qsystem.random` extension.
pub struct RandomCodegenExtension;

impl CodegenExtension for RandomCodegenExtension {
    fn add_extension<'a, H: HugrView<Node = Node> + 'a>(
        self,
        builder: tket::hugr::llvm::CodegenExtsBuilder<'a, H>,
    ) -> tket::hugr::llvm::CodegenExtsBuilder<'a, H>
    where
        Self: 'a,
    {
        builder
            .custom_type(
                (random::EXTENSION_ID, CONTEXT_TYPE_NAME.to_owned()),
                |session, _| Ok(rng_context_type(session.iw_context())),
            )
            .simple_extension_op(|context, args, op| RandomEmitter(context).emit(args, op))
    }
}

fn rng_context_type(context: &Context) -> BasicTypeEnum<'_> {
    context.struct_type(&[], false).into()
}

/// Lower the `tket.qsystem.random` extension.
struct RandomEmitter<'c, 'd, 'e, H: HugrView<Node = Node>>(&'d mut EmitFuncContext<'c, 'e, H>);

impl<'c, H: HugrView<Node = Node>> RandomEmitter<'c, '_, '_, H> {
    fn iw_context(&self) -> &'c Context {
        self.0.typing_session().iw_context()
    }

    fn i32_type(&self) -> IntType<'c> {
        self.iw_context().i32_type()
    }

    fn i64_type(&self) -> IntType<'c> {
        self.iw_context().i64_type()
    }

    fn float_type(&self) -> FloatType<'c> {
        self.iw_context().f64_type()
    }

    fn bool_type(&self) -> IntType<'c> {
        self.iw_context().bool_type()
    }

    fn rng_context(&self) -> BasicValueEnum<'c> {
        self.iw_context().const_struct(&[], false).into()
    }

    fn builder(&self) -> &Builder<'c> {
        self.0.builder()
    }

    /// Helper function to `emit` an RNG operation.
    fn emit_op(
        &self,
        args: EmitOpArgs<'c, '_, ExtensionOp, H>,
        name: &str,
        func: Result<FunctionValue<'c>>,
        input_indices: &[usize],
    ) -> Result<()> {
        let inputs: Vec<_> = input_indices.iter().map(|&i| args.inputs[i]).collect();
        let result = self
            .builder()
            .build_call(
                func?,
                &inputs.iter().map(|&v| v.into()).collect::<Vec<_>>(),
                name,
            )?
            .try_as_basic_value()
            .unwrap_left();
        args.outputs
            .finish(self.builder(), [result, self.rng_context()])
    }

    /// Function to help lower the `tket.qsystem.random` extension.
    fn emit(&self, args: EmitOpArgs<'c, '_, ExtensionOp, H>, op: RandomOp) -> Result<()> {
        match op {
            RandomOp::RandomInt => {
                let fn_random_int = self
                    .0
                    .get_extern_func("random_int", self.i32_type().fn_type(&[], false));
                self.emit_op(args, "rint", fn_random_int, &[])
            }
            RandomOp::RandomFloat => {
                let fn_random_float = self
                    .0
                    .get_extern_func("random_float", self.float_type().fn_type(&[], false));
                self.emit_op(args, "rfloat", fn_random_float, &[])
            }
            RandomOp::RandomIntBounded => {
                let fn_random_int_bounded = self.0.get_extern_func(
                    "random_rng",
                    self.i32_type().fn_type(&[self.i32_type().into()], false),
                );
                self.emit_op(args, "rintb", fn_random_int_bounded, &[1])
            }
            RandomOp::RandomAdvance => {
                let fn_random_advance = self.0.get_extern_func(
                    "random_advance",
                    self.iw_context()
                        .void_type()
                        .fn_type(&[self.i64_type().into()], false),
                )?;
                let [ctx, delta] = args
                    .inputs
                    .try_into()
                    .map_err(|_| anyhow!("RandomAdvance expects a context and delta argument"))?;
                self.builder()
                    .build_call(fn_random_advance, &[delta.into()], "radv")?;
                args.outputs.finish(self.builder(), [ctx])
            }
            RandomOp::NewRNGContext => {
                let fn_random_seed = self.0.get_extern_func(
                    "random_seed",
                    self.iw_context()
                        .void_type()
                        .fn_type(&[self.i64_type().into()], false),
                )?;
                let [seed] = args
                    .inputs
                    .try_into()
                    .map_err(|_| anyhow!("NewRNGContext expects a seed argument"))?;
                self.builder()
                    .build_call(fn_random_seed, &[seed.into()], "rseed")?;
                args.outputs.finish(
                    self.builder(),
                    [self.bool_type().const_int(1, false).into()],
                )
            }
            RandomOp::DeleteRNGContext => args.outputs.finish(self.builder(), []),
        }
    }
}

#[cfg(test)]
mod test {
    use crate::extension::random::RandomOp;
    use hugr::extension::simple_op::MakeRegisteredOp;
    use hugr::llvm::check_emission;
    use hugr::llvm::test::TestContext;
    use hugr::llvm::test::llvm_ctx;
    use hugr::llvm::test::single_op_hugr;
    use rstest::rstest;

    use super::*;

    #[rstest]
    #[case::random_int(1, RandomOp::RandomInt)]
    #[case::random_float(2, RandomOp::RandomFloat)]
    #[case::random_int_bounded(3, RandomOp::RandomIntBounded)]
    #[case::random_advance(-1, RandomOp::RandomAdvance)]
    #[case::new_rng_context(4, RandomOp::NewRNGContext)]
    #[case::delete_rng_context(5, RandomOp::DeleteRNGContext)]
    fn emit_random_codegen(
        #[case] _i: i32,
        #[with(_i)] mut llvm_ctx: TestContext,
        #[case] op: RandomOp,
    ) {
        llvm_ctx.add_extensions(|ceb| {
            ceb.add_extension(RandomCodegenExtension)
                .add_default_int_extensions()
                .add_float_extensions()
                .add_logic_extensions()
        });
        let ext_op = op.to_extension_op().unwrap().into();
        let hugr = single_op_hugr(ext_op);
        check_emission!(hugr, llvm_ctx);
    }
}
