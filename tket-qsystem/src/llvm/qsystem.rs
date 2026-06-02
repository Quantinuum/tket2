//! LLVM lowering implementations for "tket.qystem" operations.

use hugr::llvm::extension::PreludeCodegen;
use hugr::llvm::inkwell::types::FunctionType;
use itertools::Itertools as _;
use tket::hugr::{self, llvm::inkwell};

use crate::extension::qsystem::helios::{self as qsystem_helios, HeliosOp};
use crate::extension::qsystem::sol::{self as qsystem_sol, SolOp};
use crate::extension::qsystem::{QSystemPlatform, SharedOp};
use anyhow::{Result, anyhow, bail};
use hugr::extension::prelude::qb_t;
use hugr::llvm::custom::CodegenExtension;
use hugr::llvm::emit::EmitOpArgs;
use hugr::llvm::emit::func::{EmitFuncContext, build_option};
use inkwell::types::BasicType;
use inkwell::values::{BasicValueEnum, FunctionValue, IntValue};
use tket::hugr::llvm::CodegenExtsBuilder;
use tket::hugr::ops::ExtensionOp;
use tket::hugr::{HugrView, Node};

use super::futures::future_type;

/// Codegen extension for quantum operations.
#[derive(derive_more::From)]
pub struct QSystemCodegenExtension<PCG: PreludeCodegen> {
    platform: QSystemPlatform,
    codegen: PCG,
}
impl<PCG: PreludeCodegen> QSystemCodegenExtension<PCG> {
    /// Create a new `QSystemCodegenExtension` for the desired qsystem platform
    pub fn new(platform: QSystemPlatform, codegen: PCG) -> Self {
        Self { platform, codegen }
    }
}

impl<PCG: PreludeCodegen> CodegenExtension for QSystemCodegenExtension<PCG> {
    fn add_extension<'a, H: HugrView<Node = Node> + 'a>(
        self,
        builder: CodegenExtsBuilder<'a, H>,
    ) -> CodegenExtsBuilder<'a, H>
    where
        Self: 'a,
    {
        match self.platform {
            QSystemPlatform::Helios => builder
                .simple_extension_op(move |context, args, op| self.emit_helios(context, args, op))
                .extension_op(
                    qsystem_helios::EXTENSION_ID,
                    qsystem_helios::RUNTIME_BARRIER_NAME,
                    {
                        |context, args| {
                            // Do nothing
                            // TODO don't lower to RuntimeBarrier
                            args.outputs.finish(context.builder(), args.inputs)
                        }
                    },
                ),
            QSystemPlatform::Sol => builder
                .simple_extension_op::<SolOp>(move |context, args, op| {
                    self.emit_sol(context, args, op)
                })
                .extension_op(
                    qsystem_sol::EXTENSION_ID,
                    qsystem_sol::RUNTIME_BARRIER_NAME,
                    |context, args| {
                        // Do nothing
                        // TODO don't lower to RuntimeBarrier
                        args.outputs.finish(context.builder(), args.inputs)
                    },
                ),
        }
    }
}

trait QSystemRuntimeFunction {
    fn name(&self) -> &str;

    fn func_type<'c>(
        &self,
        context: &EmitFuncContext<'c, '_, impl HugrView<Node = Node>>,
        pcg: &impl PreludeCodegen,
    ) -> FunctionType<'c>;

    fn get_func<'c, H: HugrView<Node = Node>>(
        &self,
        context: &EmitFuncContext<'c, '_, H>,
        pcg: &impl PreludeCodegen,
    ) -> Result<FunctionValue<'c>> {
        let func_type = self.func_type(context, pcg);
        context.get_extern_func(self.name(), func_type)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum GenericRuntimeFunction {
    QAlloc,
    QFree,
    LazyMeasureLeaked,
    LazyMeasure,
    Reset,
    Rz,
}

impl QSystemRuntimeFunction for GenericRuntimeFunction {
    fn name(&self) -> &str {
        match self {
            GenericRuntimeFunction::QAlloc => "___qalloc",
            GenericRuntimeFunction::QFree => "___qfree",
            GenericRuntimeFunction::LazyMeasureLeaked => "___lazy_measure_leaked",
            GenericRuntimeFunction::LazyMeasure => "___lazy_measure",
            GenericRuntimeFunction::Reset => "___reset",
            GenericRuntimeFunction::Rz => "___rz",
        }
    }

    fn func_type<'c>(
        &self,
        context: &EmitFuncContext<'c, '_, impl HugrView<Node = Node>>,
        pcg: &impl PreludeCodegen,
    ) -> FunctionType<'c> {
        let qb_type = pcg
            .qubit_type(&context.typing_session())
            .as_basic_type_enum();
        let iwc = context.iw_context();
        match self {
            GenericRuntimeFunction::QAlloc => qb_type.fn_type(&[], false),
            GenericRuntimeFunction::QFree => iwc.void_type().fn_type(&[qb_type.into()], false),
            GenericRuntimeFunction::LazyMeasureLeaked => {
                future_type(iwc).fn_type(&[qb_type.into()], false)
            }
            GenericRuntimeFunction::LazyMeasure => {
                future_type(iwc).fn_type(&[qb_type.into()], false)
            }
            GenericRuntimeFunction::Reset => iwc.void_type().fn_type(&[qb_type.into()], false),
            GenericRuntimeFunction::Rz => iwc
                .void_type()
                .fn_type(&[qb_type.into(), iwc.f64_type().into()], false),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum HeliosGateFunction {
    Rzz,
    Rxy,
}
impl QSystemRuntimeFunction for HeliosGateFunction {
    fn name(&self) -> &str {
        match self {
            HeliosGateFunction::Rzz => "___rzz",
            HeliosGateFunction::Rxy => "___rxy",
        }
    }

    fn func_type<'c>(
        &self,
        context: &EmitFuncContext<'c, '_, impl HugrView<Node = Node>>,
        pcg: &impl PreludeCodegen,
    ) -> FunctionType<'c> {
        let qb_type = pcg
            .qubit_type(&context.typing_session())
            .as_basic_type_enum();
        let iwc = context.iw_context();
        match self {
            HeliosGateFunction::Rzz => iwc.void_type().fn_type(
                &[qb_type.into(), qb_type.into(), iwc.f64_type().into()],
                false,
            ),
            HeliosGateFunction::Rxy => iwc.void_type().fn_type(
                &[qb_type.into(), iwc.f64_type().into(), iwc.f64_type().into()],
                false,
            ),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SolGateFunction {
    Rp,
    Rpp,
}

impl QSystemRuntimeFunction for SolGateFunction {
    fn name(&self) -> &str {
        match self {
            SolGateFunction::Rp => "___rp",
            SolGateFunction::Rpp => "___rpp",
        }
    }

    fn func_type<'c>(
        &self,
        context: &EmitFuncContext<'c, '_, impl HugrView<Node = Node>>,
        pcg: &impl PreludeCodegen,
    ) -> FunctionType<'c> {
        let qubit = pcg
            .qubit_type(&context.typing_session())
            .as_basic_type_enum()
            .into();
        let iwc = context.iw_context();
        let float = iwc.f64_type().into();
        match self {
            SolGateFunction::Rp => iwc.void_type().fn_type(&[qubit, float, float], false),
            SolGateFunction::Rpp => iwc
                .void_type()
                .fn_type(&[qubit, qubit, float, float], false),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RuntimeFunction {
    Generic(GenericRuntimeFunction),
    HeliosGate(HeliosGateFunction),
    SolGate(SolGateFunction),
}

impl<PCG: PreludeCodegen> QSystemCodegenExtension<PCG> {
    fn runtime_func<'c>(
        &self,
        context: &EmitFuncContext<'c, '_, impl HugrView<Node = Node>>,
        rf: RuntimeFunction,
    ) -> Result<FunctionValue<'c>> {
        match (self.platform, rf) {
            (QSystemPlatform::Helios, RuntimeFunction::HeliosGate(gf)) => {
                gf.get_func(context, &self.codegen)
            }
            (QSystemPlatform::Sol, RuntimeFunction::SolGate(gf)) => {
                gf.get_func(context, &self.codegen)
            }
            (_, RuntimeFunction::Generic(gf)) => gf.get_func(context, &self.codegen),
            (QSystemPlatform::Helios, RuntimeFunction::SolGate(_)) => {
                bail!("Sol gate function called on Helios platform")
            }
            (QSystemPlatform::Sol, RuntimeFunction::HeliosGate(_)) => {
                bail!("Helios gate function called on Sol platform")
            }
        }
    }
    fn runtime_func_name<'c>(&self, rf: &'c RuntimeFunction) -> Result<&'c str> {
        match (self.platform, rf) {
            (QSystemPlatform::Helios, RuntimeFunction::HeliosGate(gf)) => Ok(gf.name()),
            (QSystemPlatform::Sol, RuntimeFunction::SolGate(gf)) => Ok(gf.name()),
            (_, RuntimeFunction::Generic(gf)) => Ok(gf.name()),
            (QSystemPlatform::Helios, RuntimeFunction::SolGate(_)) => {
                bail!("Sol gate function called on Helios platform")
            }
            (QSystemPlatform::Sol, RuntimeFunction::HeliosGate(_)) => {
                bail!("Helios gate function called on Sol platform")
            }
        }
    }

    /// Helper function to `emit` a qsystem operation.
    fn emit_impl<'c, H: HugrView<Node = Node>>(
        &self,
        context: &mut EmitFuncContext<'c, '_, impl HugrView<Node = Node>>,
        args: EmitOpArgs<'c, '_, ExtensionOp, H>,
        runtime_func: RuntimeFunction,
        input_indices: &[usize],
        output_indices: &[usize],
    ) -> Result<()> {
        let inputs = input_indices
            .iter()
            .map(|&i| args.inputs[i].into())
            .collect_vec();
        let outputs = output_indices.iter().map(|&i| args.inputs[i]).collect_vec();
        let func = self.runtime_func(context, runtime_func)?;
        let func_name = self.runtime_func_name(&runtime_func)?;
        context.builder().build_call(func, &inputs, func_name)?;
        args.outputs.finish(context.builder(), outputs)
    }

    /// Function to help lower the `tket.qsystem.helios` extension.
    fn emit_helios<'c, H: HugrView<Node = Node>>(
        &self,
        context: &mut EmitFuncContext<'c, '_, H>,
        args: EmitOpArgs<'c, '_, ExtensionOp, H>,
        op: HeliosOp,
    ) -> Result<()> {
        match op {
            // PhasedX uses a different LLVM function on Helios (___rxy) vs Sol (___rp).
            HeliosOp::PhasedX => self.emit_impl(
                context,
                args,
                RuntimeFunction::HeliosGate(HeliosGateFunction::Rxy),
                &[0, 1, 2],
                &[0],
            ),
            HeliosOp::ZZPhase => self.emit_impl(
                context,
                args,
                RuntimeFunction::HeliosGate(HeliosGateFunction::Rzz),
                &[0, 1, 2],
                &[0, 1],
            ),
            _ => {
                let shared = SharedOp::try_from(op).map_err(|e| anyhow!(e))?;
                self.emit_shared(context, args, shared)
            }
        }
    }

    /// Function to help lower the `tket.qsystem.sol` extension.
    fn emit_sol<'c, H: HugrView<Node = Node>>(
        &self,
        context: &mut EmitFuncContext<'c, '_, H>,
        args: EmitOpArgs<'c, '_, ExtensionOp, H>,
        op: SolOp,
    ) -> Result<()> {
        match op {
            // PhasedX uses a different LLVM function on Sol (___rp) vs Helios (___rxy).
            SolOp::PhasedX => self.emit_impl(
                context,
                args,
                RuntimeFunction::SolGate(SolGateFunction::Rp),
                &[0, 1, 2],
                &[0],
            ),
            SolOp::PhasedXX => self.emit_impl(
                context,
                args,
                RuntimeFunction::SolGate(SolGateFunction::Rpp),
                &[0, 1, 2, 3],
                &[0, 1],
            ),
            _ => {
                let shared = SharedOp::try_from(op).map_err(|e| anyhow!(e))?;
                self.emit_shared(context, args, shared)
            }
        }
    }

    /// Lower a [`SharedOp`] that has identical LLVM behaviour on all platforms.
    ///
    /// Note: [`SharedOp::PhasedX`] is excluded — it uses different runtime functions
    /// per platform (`___rxy` on Helios, `___rp` on Sol) and must be handled by the
    /// platform-specific method before calling this one.
    fn emit_shared<'c, H: HugrView<Node = Node>>(
        &self,
        context: &mut EmitFuncContext<'c, '_, H>,
        args: EmitOpArgs<'c, '_, ExtensionOp, H>,
        op: SharedOp,
    ) -> Result<()> {
        match op {
            // Rz uses the same runtime function (___rz) on all platforms.
            SharedOp::Rz => self.emit_impl(
                context,
                args,
                RuntimeFunction::Generic(GenericRuntimeFunction::Rz),
                &[0, 1],
                &[0],
            ),
            // Measure qubit in Z basis, not forcing to a boolean.
            SharedOp::LazyMeasure => {
                let builder = context.builder();
                let [qb] = args
                    .inputs
                    .try_into()
                    .map_err(|_| anyhow!("LazyMeasure expects one input"))?;
                let result = builder
                    .build_call(
                        self.runtime_func(
                            context,
                            RuntimeFunction::Generic(GenericRuntimeFunction::LazyMeasure),
                        )?,
                        &[qb.into()],
                        "lazy_measure",
                    )?
                    .try_as_basic_value()
                    .unwrap_basic();
                builder.build_call(
                    self.runtime_func(
                        context,
                        RuntimeFunction::Generic(GenericRuntimeFunction::QFree),
                    )?,
                    &[qb.into()],
                    "qfree",
                )?;
                args.outputs.finish(builder, [result])
            }
            // Measure qubit in Z basis or detect leakage, not forcing to a boolean.
            SharedOp::LazyMeasureLeaked => {
                let builder = context.builder();
                let [qb] = args
                    .inputs
                    .try_into()
                    .map_err(|_| anyhow!("LazyMeasureLeaked expects one input"))?;
                let result = builder
                    .build_call(
                        self.runtime_func(
                            context,
                            RuntimeFunction::Generic(GenericRuntimeFunction::LazyMeasureLeaked),
                        )?,
                        &[qb.into()],
                        "lazy_measure_leaked",
                    )?
                    .try_as_basic_value()
                    .unwrap_basic();
                builder.build_call(
                    self.runtime_func(
                        context,
                        RuntimeFunction::Generic(GenericRuntimeFunction::QFree),
                    )?,
                    &[qb.into()],
                    "qfree",
                )?;
                args.outputs.finish(builder, [result])
            }
            SharedOp::LazyMeasureReset => {
                let builder = context.builder();
                let [qb] = args
                    .inputs
                    .try_into()
                    .map_err(|_| anyhow!("LazyMeasureReset expects one input"))?;
                let result = builder
                    .build_call(
                        self.runtime_func(
                            context,
                            RuntimeFunction::Generic(GenericRuntimeFunction::LazyMeasure),
                        )?,
                        &[qb.into()],
                        "lazy_measure",
                    )?
                    .try_as_basic_value()
                    .unwrap_basic();
                builder.build_call(
                    self.runtime_func(
                        context,
                        RuntimeFunction::Generic(GenericRuntimeFunction::Reset),
                    )?,
                    &[qb.into()],
                    "reset",
                )?;
                args.outputs.finish(builder, [qb, result])
            }
            // Reset a qubit.
            SharedOp::Reset => self.emit_impl(
                context,
                args,
                RuntimeFunction::Generic(GenericRuntimeFunction::Reset),
                &[0],
                &[0],
            ),
            SharedOp::TryQAlloc => {
                let [] = args
                    .inputs
                    .try_into()
                    .map_err(|_| anyhow!("QAlloc expects no inputs"))?;
                let qb = context
                    .builder()
                    .build_call(
                        self.runtime_func(
                            context,
                            RuntimeFunction::Generic(GenericRuntimeFunction::QAlloc),
                        )?,
                        &[],
                        "qalloc",
                    )?
                    .try_as_basic_value()
                    .unwrap_basic();
                let max_qb = self
                    .codegen
                    .qubit_type(&context.typing_session())
                    .as_basic_type_enum()
                    .into_int_type()
                    .const_int(u64::MAX, false);
                let not_max = context.builder().build_int_compare(
                    inkwell::IntPredicate::NE,
                    qb.into_int_value(),
                    max_qb,
                    "not_max",
                )?;
                self.reset_if_qb(context, qb, not_max)?;
                let result = build_option(context, not_max, qb, qb_t())?;
                args.outputs.finish(context.builder(), [result])
            }
            SharedOp::QFree => self.emit_impl(
                context,
                args,
                RuntimeFunction::Generic(GenericRuntimeFunction::QFree),
                &[0],
                &[],
            ),
            SharedOp::PhasedX => {
                unreachable!(
                    "PhasedX uses different LLVM functions per platform \
                     and must be handled before dispatching to emit_shared"
                )
            }
            SharedOp::FutureToMeasurement => {
                unreachable!("FutureToMeasurement should have been removed before codegen")
            }
        }
    }

    /// Reset a qubit if it is successfully allocated (not max value)
    fn reset_if_qb<'c>(
        &self,
        context: &mut EmitFuncContext<'c, '_, impl HugrView<Node = Node>>,
        qb: BasicValueEnum<'c>,
        not_max: IntValue<'c>,
    ) -> Result<()> {
        let orig_bb = context
            .builder()
            .get_insert_block()
            .ok_or_else(|| anyhow!("No current insertion point"))?;

        let id_bb = context
            .iw_context()
            .insert_basic_block_after(orig_bb, "id_bb");

        let reset_bb =
            context.build_positioned_new_block("reset_bb", Some(id_bb), |context, bb| {
                context.builder().build_call(
                    self.runtime_func(
                        context,
                        RuntimeFunction::Generic(GenericRuntimeFunction::Reset),
                    )?,
                    &[qb.into()],
                    "reset",
                )?;
                context.builder().build_unconditional_branch(id_bb)?;
                anyhow::Ok(bb)
            })?;
        context
            .builder()
            .build_conditional_branch(not_max, reset_bb, id_bb)?;
        context.builder().position_at_end(id_bb);
        Ok(())
    }
}

#[cfg(test)]
mod test {
    use crate::extension::qsystem::helios::HeliosOp;
    use crate::extension::qsystem::sol::SolOp;

    use hugr::extension::simple_op::MakeRegisteredOp;
    use hugr::llvm::check_emission;
    use hugr::llvm::test::TestContext;
    use hugr::llvm::test::llvm_ctx;
    use hugr::llvm::test::single_op_hugr;
    use rstest::rstest;

    use super::*;

    #[rstest]
    #[case::rz(1, HeliosOp::Rz)]
    #[case::zzphase(2, HeliosOp::ZZPhase)]
    #[case::phased_x(3, HeliosOp::PhasedX)]
    #[case::lazy_measure(5, HeliosOp::LazyMeasure)]
    #[case::try_qalloc(6, HeliosOp::TryQAlloc)]
    #[case::qfree(7, HeliosOp::QFree)]
    #[case::reset(8, HeliosOp::Reset)]
    #[case::lazy_measure_leaked(10, HeliosOp::LazyMeasureLeaked)]
    fn emit_helios_codegen(
        #[case] _i: i32,
        #[with(_i)] mut llvm_ctx: TestContext,
        #[case] op: HeliosOp,
    ) {
        use crate::llvm::{futures::FuturesCodegenExtension, prelude::QISPreludeCodegen};

        llvm_ctx.add_extensions(|ceb| {
            ceb.add_extension(QSystemCodegenExtension::new(
                QSystemPlatform::Helios,
                QISPreludeCodegen,
            ))
            .add_extension(FuturesCodegenExtension)
            .add_prelude_extensions(QISPreludeCodegen)
            .add_float_extensions()
            .add_logic_extensions()
        });
        let ext_op = op.to_extension_op().unwrap().into();
        let hugr = single_op_hugr(ext_op);
        check_emission!(hugr, llvm_ctx);
    }

    #[rstest]
    #[case::rz(1, SolOp::Rz)]
    #[case::phased_x(2, SolOp::PhasedX)]
    #[case::phased_xx(3, SolOp::PhasedXX)]
    #[case::lazy_measure(5, SolOp::LazyMeasure)]
    #[case::try_qalloc(6, SolOp::TryQAlloc)]
    #[case::qfree(7, SolOp::QFree)]
    #[case::reset(8, SolOp::Reset)]
    #[case::lazy_measure_leaked(10, SolOp::LazyMeasureLeaked)]
    fn emit_sol_codegen(#[case] _i: i32, #[with(_i)] mut llvm_ctx: TestContext, #[case] op: SolOp) {
        use crate::llvm::{futures::FuturesCodegenExtension, prelude::QISPreludeCodegen};

        llvm_ctx.add_extensions(|ceb| {
            ceb.add_extension(QSystemCodegenExtension::new(
                QSystemPlatform::Sol,
                QISPreludeCodegen,
            ))
            .add_extension(FuturesCodegenExtension)
            .add_prelude_extensions(QISPreludeCodegen)
            .add_float_extensions()
            .add_logic_extensions()
        });
        let ext_op = op.to_extension_op().unwrap().into();
        let hugr = single_op_hugr(ext_op);
        check_emission!(hugr, llvm_ctx);
    }
}
