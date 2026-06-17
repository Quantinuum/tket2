#![allow(missing_docs)]

use crate::extension::globals::{GlobalsOp, GlobalsOpDef};
use anyhow::{Result, bail, ensure};
use hugr::extension::prelude::ConstError;
use hugr::llvm::emit::{deaggregate_call_result, emit_value};
use hugr::llvm::extension::PreludeCodegen;
use hugr::llvm::inkwell::builder::Builder;
use hugr::llvm::inkwell::module::Module;
use hugr::llvm::inkwell::types::BasicTypeEnum;
use hugr::llvm::inkwell::values::{BasicMetadataValueEnum, GlobalValue, PointerValue};
use hugr::llvm::sum::LLVMSumType;
use hugr::llvm::{
    CodegenExtension, CodegenExtsBuilder,
    emit::{EmitFuncContext, EmitOpArgs},
    inkwell::{AddressSpace, types::BasicType as _},
};
use hugr::ops::Value;
use hugr::{
    HugrView, Node,
    extension::{prelude::option_type, simple_op::HasConcrete as _},
    ops::ExtensionOp,
};
use hugr_core::types::{FuncValueType, Signature, Type, TypeRowRV};
use itertools::Itertools;

pub struct GlobalsCodegenExtension<PCG> {
    pcg: PCG,
    no_global_error: ConstError,
}

impl<PCG: PreludeCodegen> CodegenExtension for GlobalsCodegenExtension<PCG> {
    fn add_extension<'a, H: HugrView<Node = Node> + 'a>(
        self,
        builder: CodegenExtsBuilder<'a, H>,
    ) -> CodegenExtsBuilder<'a, H>
    where
        Self: 'a,
    {
        builder
            .simple_extension_op(move |context, args, op| self.emit_globals_op(context, args, op))
    }
}

impl<PCG: PreludeCodegen> GlobalsCodegenExtension<PCG> {
    pub fn new(pcg: PCG) -> Self {
        Self {
            pcg,
            no_global_error: ConstError::new_default_signal(
                "No global provided for GlobalsOp::With",
            ),
        }
    }

    pub fn with_no_global_error(self, err: ConstError) -> Self {
        Self {
            no_global_error: err,
            ..self
        }
    }

    fn emit_globals_op<'c, H: HugrView<Node = Node>>(
        &self,
        context: &mut EmitFuncContext<'c, '_, H>,
        args: EmitOpArgs<'c, '_, ExtensionOp, H>,
        op: GlobalsOpDef,
    ) -> Result<()> {
        let op = op.instantiate(args.node().args())?;
        const PREFIX: &str = "__globals__";

        match op {
            GlobalsOp::With {
                name,
                ty_arg,
                inputs,
                outputs,
            } => {
                let sym = format!("{PREFIX}.{name}");
                let global_ty_base = Type::try_from(ty_arg)?;
                let sym_ty = context.llvm_sum_type(option_type([global_ty_base.clone()]))?;

                let [init_global_value, func, func_args @ ..] = &args.inputs[..] else {
                    bail!("No function provided as input for GlobalsOp::With")
                };

                let module = context.get_current_module();
                let builder = context.builder();

                let global = get_global_value(module, builder, sym.clone(), sym_ty.clone())?;

                let global_ty: BasicTypeEnum = global
                    .get_value_type()
                    .try_into()
                    .map_err(|e| anyhow::anyhow!("Global {sym} has non-basic LLVM type: {e:?}"))?;
                ensure!(
                    global_ty == sym_ty.as_basic_type_enum(),
                    "Input type does not match global variable type. Found {global_ty}, Expected {sym_ty}"
                );
                let start_value =
                    builder.build_load(sym_ty.clone(), global.as_pointer_value(), "start_value")?;

                let new_value = sym_ty.build_tag(builder, 1, vec![*init_global_value])?;

                let _ = builder.build_store(global.as_pointer_value(), new_value)?;

                let real_args = func_args.iter().copied().map_into().collect_vec();
                let func_ptr = PointerValue::try_from(*func).map_err(|e| {
                    anyhow::anyhow!("Invalid function pointer provided to With: {e:?}")
                })?;

                let hugr_func_ty: Signature =
                    FuncValueType::new(inputs.clone(), outputs).try_into()?;
                let func_ty = context.llvm_func_type(&hugr_func_ty)?;

                let func_call =
                    builder.build_indirect_call(func_ty, func_ptr, &real_args, "call_func")?;

                let end_value =
                    builder.build_load(sym_ty.clone(), global.as_pointer_value(), "end_value")?;

                let end_value = sym_ty.value(end_value)?;
                let end_value = end_value.build_untag(builder, 1)?[0];

                let _ = builder.build_store(global.as_pointer_value(), start_value)?;

                let mut call_results =
                    deaggregate_call_result(builder, func_call, hugr_func_ty.output.len())?;
                call_results.insert(0, end_value);

                // Return results from function
                args.outputs.finish(builder, call_results)?
            }
            GlobalsOp::Map {
                name,
                ty_arg,
                inputs,
                outputs,
            } => {
                let sym = format!("{PREFIX}.{name}");
                let global_hugr_ty = Type::try_from(ty_arg)?;
                let sym_ty = context.llvm_sum_type(option_type([global_hugr_ty.clone()]))?;

                // Get function and args
                let [func, func_args @ ..] = &args.inputs[..] else {
                    bail!("No function provided as input for GlobalsOp::Map")
                };

                // we'll branch to this if the global is `none`
                let global_is_none_block =
                    context.build_positioned_new_block("global_is_none", None, |context, bb| {
                        let err = emit_value(context, &Value::from(self.no_global_error.clone()))?;
                        self.pcg.emit_panic(context, err)?;
                        context.builder().build_unreachable()?;
                        anyhow::Ok(bb)
                    })?;

                let mailbox = context.new_row_mail_box([&global_hugr_ty], "global_value")?;

                // We'll branch to this if the global is `some`. We'll fill it in further down.
                let global_is_some_block = context.build_positioned_new_block(
                    "global_is_some",
                    None,
                    |_context, bb| anyhow::Ok(bb),
                )?;

                let global = get_global_value(
                    context.get_current_module(),
                    context.builder(),
                    sym.clone(),
                    sym_ty.clone(),
                )?;
                let global_ty: BasicTypeEnum = global
                    .get_value_type()
                    .try_into()
                    .map_err(|e| anyhow::anyhow!("Global {sym} has non-basic LLVM type: {e:?}"))?;
                ensure!(
                    global_ty == sym_ty.as_basic_type_enum(),
                    "Input type does not match global variable type. Found {global_ty}, Expected {sym_ty}"
                );

                let start_value = {
                    let v = context.builder().build_load(
                        sym_ty.clone(),
                        global.as_pointer_value(),
                        "start_value",
                    )?;
                    sym_ty.value(v)?
                };

                start_value.build_destructure(context.builder(), |builder, tag, values| {
                    if tag == 0 {
                        // Global is None, branch to error block
                        builder.build_unconditional_branch(global_is_none_block)?;
                        return Ok(());
                    } else if tag == 1 {
                        // Global is Some. Write the value to the mailbox prepared earlier, and branch to the success block
                        mailbox.promise().finish(builder, [values[0]])?;
                        builder.build_unconditional_branch(global_is_some_block)?;
                    }
                    Ok(())
                })?;

                let builder = context.builder();
                builder.position_at_end(global_is_some_block);
                let global_val = mailbox.read_vec(builder, ["global"])?[0];

                // Store `none` in the global while `func` executes
                let none = sym_ty.build_tag(builder, 0, vec![])?;
                builder.build_store(global.as_pointer_value(), none)?;

                // real_args should be [global, *func_args]
                let real_args: Vec<BasicMetadataValueEnum> = {
                    let mut v = vec![global_val.into()];
                    v.extend(
                        func_args
                            .iter()
                            .copied()
                            .map_into::<BasicMetadataValueEnum>(),
                    );
                    v
                };

                let func_ptr = PointerValue::try_from(*func).map_err(|e| {
                    anyhow::anyhow!("Invalid function pointer provided to Map: {e:?}")
                })?;

                let in_types = TypeRowRV::from([global_hugr_ty.clone()]).concat(inputs.clone());

                let out_types = TypeRowRV::from([global_hugr_ty.clone()]).concat(outputs.clone());

                let hugr_func_ty = FuncValueType::new(in_types, out_types).try_into()?;
                let func_ty = context.llvm_func_type(&hugr_func_ty)?;

                let func_call =
                    builder.build_indirect_call(func_ty, func_ptr, &real_args, "call_func")?;

                let call_results =
                    deaggregate_call_result(builder, func_call, hugr_func_ty.output.len())?;

                let [end_value, results @ ..] = &call_results[..] else {
                    bail!("Global '{sym}' was not returned from function call")
                };

                let end_value = sym_ty.build_tag(builder, 1, vec![*end_value])?;
                builder.build_store(global.as_pointer_value(), end_value)?;

                args.outputs
                    .finish(builder, results.iter().copied().map_into().collect_vec())?;
            }
        }
        Ok(())
    }
}

fn get_global_value<'a>(
    module: &Module<'a>,
    builder: &Builder,
    sym: String,
    sym_ty: LLVMSumType<'a>,
) -> Result<GlobalValue<'a>> {
    if let Some(global) = module.get_global(&sym) {
        return Ok(global);
    }

    let none_value = sym_ty
        .build_tag(builder, 0, vec![])
        .map_err(|e| anyhow::anyhow!("Failed to build None value for global '{sym}': {e:?}"))?;
    let global = module.add_global(sym_ty.clone(), Some(AddressSpace::default()), &sym);
    global.set_initializer(&none_value);
    Ok(global)
}

#[cfg(test)]
mod test {
    use super::*;
    use hugr::extension::prelude::{bool_t, qb_t};
    use hugr::llvm::extension::DefaultPreludeCodegen;
    use hugr::llvm::{
        check_emission,
        test::{TestContext, llvm_ctx, single_op_hugr},
    };
    use hugr_core::extension::simple_op::MakeRegisteredOp;

    #[rstest::rstest]
    #[case::with(1,
        GlobalsOp::With{ name: "my_global".to_string(), ty_arg: qb_t().into(), inputs: [bool_t(), qb_t()].into(), outputs: [bool_t()].into() }
    )]
    #[case::map(2,
        GlobalsOp::Map{ name: "my_global".to_string(), ty_arg: qb_t().into(), inputs: TypeRowRV::new(), outputs: TypeRowRV::new() }
    )]
    fn emit_globals_codegen(
        #[case] _i: i32,
        #[with(_i)] mut llvm_ctx: TestContext,
        #[case] op: GlobalsOp,
    ) {
        llvm_ctx.add_extensions(|ceb| {
            ceb.add_extension(GlobalsCodegenExtension::new(DefaultPreludeCodegen))
                .add_default_prelude_extensions()
        });
        let mut hugr = single_op_hugr(op.to_extension_op().unwrap().into());
        check_emission!(hugr, llvm_ctx);
    }
}
