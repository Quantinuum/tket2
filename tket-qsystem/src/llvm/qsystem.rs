//! LLVM lowering implementations for "tket.qystem" operations.

use hugr::llvm::extension::PreludeCodegen;
use hugr::llvm::inkwell::types::FunctionType;
use itertools::Itertools as _;
use tket::hugr::{self, llvm::inkwell};

use crate::extension::qsystem::{self, QSystemOp};
use anyhow::{Result, anyhow};
use hugr::extension::prelude::qb_t;
use hugr::llvm::custom::CodegenExtension;
use hugr::llvm::emit::func::{EmitFuncContext, build_option};
use hugr::llvm::emit::{EmitOpArgs, emit_value};
use inkwell::types::BasicType;
use inkwell::values::{BasicValueEnum, FunctionValue, IntValue};
use tket::hugr::llvm::CodegenExtsBuilder;
use tket::hugr::ops::ExtensionOp;
use tket::hugr::ops::constant::Value;
use tket::hugr::{HugrView, Node};

use super::futures::future_type;

/// Codegen extension for quantum operations.
#[derive(derive_more::From)]
pub struct QSystemCodegenExtension<PCG>(PCG);

impl<PCG: PreludeCodegen> CodegenExtension for QSystemCodegenExtension<PCG> {
    fn add_extension<'a, H: HugrView<Node = Node> + 'a>(
        self,
        builder: CodegenExtsBuilder<'a, H>,
    ) -> CodegenExtsBuilder<'a, H>
    where
        Self: 'a,
    {
        builder
            .simple_extension_op(move |context, args, op| self.emit(context, args, op))
            .extension_op(qsystem::EXTENSION_ID, qsystem::RUNTIME_BARRIER_NAME, {
                move |context, args| {
                    // Do nothing
                    // TODO don't lower to RuntimeBarrier
                    args.outputs.finish(context.builder(), args.inputs)
                }
            })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RuntimeFunction {
    Rz,
    Rzz,
    Rxy,
    QAlloc,
    QFree,
    Measure,
    LazyMeasureLeaked,
    LazyMeasure,
    Reset,
}

impl RuntimeFunction {
    fn name(&self) -> &str {
        match self {
            RuntimeFunction::Rz => "___rz",
            RuntimeFunction::Rzz => "___rzz",
            RuntimeFunction::Rxy => "___rxy",
            RuntimeFunction::QAlloc => "___qalloc",
            RuntimeFunction::QFree => "___qfree",
            RuntimeFunction::Measure => "___measure",
            RuntimeFunction::LazyMeasureLeaked => "___lazy_measure_leaked",
            RuntimeFunction::LazyMeasure => "___lazy_measure",
            RuntimeFunction::Reset => "___reset",
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
            RuntimeFunction::Rz => iwc
                .void_type()
                .fn_type(&[qb_type.into(), iwc.f64_type().into()], false),
            RuntimeFunction::Rzz => iwc.void_type().fn_type(
                &[qb_type.into(), qb_type.into(), iwc.f64_type().into()],
                false,
            ),
            RuntimeFunction::Rxy => iwc.void_type().fn_type(
                &[qb_type.into(), iwc.f64_type().into(), iwc.f64_type().into()],
                false,
            ),
            RuntimeFunction::QAlloc => qb_type.fn_type(&[], false),
            RuntimeFunction::QFree => iwc.void_type().fn_type(&[qb_type.into()], false),
            RuntimeFunction::Measure => iwc.bool_type().fn_type(&[qb_type.into()], false),
            RuntimeFunction::LazyMeasureLeaked => {
                future_type(iwc).fn_type(&[qb_type.into()], false)
            }
            RuntimeFunction::LazyMeasure => future_type(iwc).fn_type(&[qb_type.into()], false),
            RuntimeFunction::Reset => iwc.void_type().fn_type(&[qb_type.into()], false),
        }
    }

    fn get_func<'c, H: HugrView<Node = Node>>(
        &self,
        context: &EmitFuncContext<'c, '_, H>,
        pcg: &impl PreludeCodegen,
    ) -> Result<FunctionValue<'c>> {
        let func_type = self.func_type(context, pcg);
        context.get_extern_func(self.name(), func_type)
    }
}

impl<PCG: PreludeCodegen> QSystemCodegenExtension<PCG> {
    fn runtime_func<'c>(
        &self,
        context: &EmitFuncContext<'c, '_, impl HugrView<Node = Node>>,
        rf: RuntimeFunction,
    ) -> Result<FunctionValue<'c>> {
        rf.get_func(context, &self.0)
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
        context
            .builder()
            .build_call(func, &inputs, runtime_func.name())?;
        args.outputs.finish(context.builder(), outputs)
    }

    /// Function to help lower the `tket.qsystem` extension.
    fn emit<'c, H: HugrView<Node = Node>>(
        &self,
        context: &mut EmitFuncContext<'c, '_, H>,
        args: EmitOpArgs<'c, '_, ExtensionOp, H>,
        op: QSystemOp,
    ) -> Result<()> {
        match op {
            // Rotation about Z
            QSystemOp::Rz => self.emit_impl(context, args, RuntimeFunction::Rz, &[0, 1], &[0]),
            // Rotation about ZZ (aka "ZZPhase")
            QSystemOp::ZZPhase => {
                self.emit_impl(context, args, RuntimeFunction::Rzz, &[0, 1, 2], &[0, 1])
            }
            // Rotation in X and Y
            // (α,β) ↦ Rz(β)Rx(α)Rz(−β)
            //
            // aka "PhasedX", "R1XY", "U1q"
            QSystemOp::PhasedX => {
                self.emit_impl(context, args, RuntimeFunction::Rxy, &[0, 1, 2], &[0])
            }
            // Measure qubit in Z basis
            QSystemOp::Measure | QSystemOp::MeasureReset => {
                let true_val = emit_value(context, &Value::true_val())?;
                let false_val = emit_value(context, &Value::false_val())?;
                let builder = context.builder();
                let [qb] = args
                    .inputs
                    .try_into()
                    .map_err(|_| anyhow!("Measure expects one input"))?;
                let result_i1 = builder
                    .build_call(
                        self.runtime_func(context, RuntimeFunction::Measure)?,
                        &[qb.into()],
                        "measure_i1",
                    )?
                    .try_as_basic_value()
                    .unwrap_basic()
                    .into_int_value();
                let result = builder.build_select(result_i1, true_val, false_val, "measure")?;
                if op == QSystemOp::Measure {
                    // normal measure may put the qubit in invalid state, so assume
                    // deallocation, don't return it
                    builder.build_call(
                        self.runtime_func(context, RuntimeFunction::QFree)?,
                        &[qb.into()],
                        "qfree",
                    )?;
                    args.outputs.finish(builder, [result])
                } else {
                    // MeasureReset will reset the qubit after measurement so safe to return
                    builder.build_call(
                        self.runtime_func(context, RuntimeFunction::Reset)?,
                        &[qb.into()],
                        "reset",
                    )?;
                    args.outputs.finish(builder, [qb, result])
                }
            }
            // Measure qubit in Z basis, not forcing to a boolean
            QSystemOp::LazyMeasure => {
                let builder = context.builder();
                let [qb] = args
                    .inputs
                    .try_into()
                    .map_err(|_| anyhow!("LazyMeasure expects one input"))?;
                let result = builder
                    .build_call(
                        self.runtime_func(context, RuntimeFunction::LazyMeasure)?,
                        &[qb.into()],
                        "lazy_measure",
                    )?
                    .try_as_basic_value()
                    .unwrap_basic();
                builder.build_call(
                    self.runtime_func(context, RuntimeFunction::QFree)?,
                    &[qb.into()],
                    "qfree",
                )?;
                args.outputs.finish(builder, [result])
            }
            // Measure qubit in Z basis or detect leakage, not forcing to a boolean
            QSystemOp::LazyMeasureLeaked => {
                let builder = context.builder();
                let [qb] = args
                    .inputs
                    .try_into()
                    .map_err(|_| anyhow!("LazyMeasureLeaked expects one input"))?;
                let result = builder
                    .build_call(
                        self.runtime_func(context, RuntimeFunction::LazyMeasureLeaked)?,
                        &[qb.into()],
                        "lazy_measure_leaked",
                    )?
                    .try_as_basic_value()
                    .unwrap_basic();
                builder.build_call(
                    self.runtime_func(context, RuntimeFunction::QFree)?,
                    &[qb.into()],
                    "qfree",
                )?;
                args.outputs.finish(builder, [result])
            }
            QSystemOp::LazyMeasureReset => {
                let builder = context.builder();
                let [qb] = args
                    .inputs
                    .try_into()
                    .map_err(|_| anyhow!("LazyMeasureReset expects one input"))?;
                let result = builder
                    .build_call(
                        self.runtime_func(context, RuntimeFunction::LazyMeasure)?,
                        &[qb.into()],
                        "lazy_measure",
                    )?
                    .try_as_basic_value()
                    .unwrap_basic();
                builder.build_call(
                    self.runtime_func(context, RuntimeFunction::Reset)?,
                    &[qb.into()],
                    "reset",
                )?;
                args.outputs.finish(builder, [qb, result])
            }
            // Reset a qubit
            QSystemOp::Reset => self.emit_impl(context, args, RuntimeFunction::Reset, &[0], &[0]),
            QSystemOp::TryQAlloc => {
                let [] = args
                    .inputs
                    .try_into()
                    .map_err(|_| anyhow!("QAlloc expects no inputs"))?;
                let qb = context
                    .builder()
                    .build_call(
                        self.runtime_func(context, RuntimeFunction::QAlloc)?,
                        &[],
                        "qalloc",
                    )?
                    .try_as_basic_value()
                    .unwrap_basic();

                let max_qb = self
                    .0
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
            QSystemOp::QFree => self.emit_impl(context, args, RuntimeFunction::QFree, &[0], &[]),
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
                    self.runtime_func(context, RuntimeFunction::Reset)?,
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
    use crate::extension::qsystem::QSystemOp;

    use hugr::extension::simple_op::MakeRegisteredOp;
    use hugr::llvm::check_emission;
    use hugr::llvm::test::TestContext;
    use hugr::llvm::test::llvm_ctx;
    use hugr::llvm::test::single_op_hugr;
    use rstest::rstest;

    use super::*;

    #[rstest]
    #[case::rz(1, QSystemOp::Rz)]
    #[case::zzphase(2, QSystemOp::ZZPhase)]
    #[case::phased_x(3, QSystemOp::PhasedX)]
    #[case::measure(4, QSystemOp::Measure)]
    #[case::lazy_measure(5, QSystemOp::LazyMeasure)]
    #[case::try_qalloc(6, QSystemOp::TryQAlloc)]
    #[case::qfree(7, QSystemOp::QFree)]
    #[case::reset(8, QSystemOp::Reset)]
    #[case::measure_reset(9, QSystemOp::MeasureReset)]
    #[case::lazy_measure_leaked(10, QSystemOp::LazyMeasureLeaked)]
    fn emit_qsystem_codegen(
        #[case] _i: i32,
        #[with(_i)] mut llvm_ctx: TestContext,
        #[case] op: QSystemOp,
    ) {
        use hugr::algorithms::ComposablePass;

        use crate::llvm::{futures::FuturesCodegenExtension, prelude::QISPreludeCodegen};

        llvm_ctx.add_extensions(|ceb| {
            ceb.add_extension(QSystemCodegenExtension(QISPreludeCodegen))
                .add_extension(FuturesCodegenExtension)
                .add_prelude_extensions(QISPreludeCodegen)
                .add_float_extensions()
                .add_logic_extensions()
        });
        let ext_op = op.to_extension_op().unwrap().into();
        let mut hugr = single_op_hugr(ext_op);
        crate::replace_bools::ReplaceBoolPass
            .run(&mut hugr)
            .unwrap();
        check_emission!(hugr, llvm_ctx);
    }
}
