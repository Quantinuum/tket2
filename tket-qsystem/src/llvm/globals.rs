#![allow(missing_docs)]

use crate::extension::globals::{GlobalsOp, GlobalsOpDef, TYPE_PARAM};
use anyhow::{Result, bail, ensure};
use hugr::llvm::emit::deaggregate_call_result;
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
use hugr::{
    HugrView, Node,
    extension::{prelude::option_type, simple_op::HasConcrete as _},
    ops::ExtensionOp,
};
use hugr_core::extension::SignatureError;
use hugr_core::types::type_param::TermTypeError;
use hugr_core::types::{FuncValueType, Signature};
use itertools::Itertools;

pub struct GlobalsCodegenExtension;

impl CodegenExtension for GlobalsCodegenExtension {
    fn add_extension<'a, H: HugrView<Node = Node> + 'a>(
        self,
        builder: CodegenExtsBuilder<'a, H>,
    ) -> CodegenExtsBuilder<'a, H>
    where
        Self: 'a,
    {
        builder.simple_extension_op(emit_globals_op)
    }
}

fn emit_globals_op<'c, H: HugrView<Node = Node>>(
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
            impl_outputs,
        } => {
            let sym = format!("{PREFIX}.{name}");
            let Some(global_ty_base) = ty_arg.as_runtime() else {
                Err(SignatureError::from(TermTypeError::TypeMismatch {
                    term: ty_arg.clone().into(),
                    type_: TYPE_PARAM.to_owned().into(),
                }))?
            };
            let sym_ty = context.llvm_sum_type(option_type([global_ty_base]))?;

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
            let func_ptr = PointerValue::try_from(*func)
                .map_err(|e| anyhow::anyhow!("Invalid function pointer provided to With: {e:?}"))?;

            let mut out_types = outputs.iter().cloned().collect_vec();
            out_types.extend(impl_outputs.iter().cloned().map_into());

            let hugr_func_ty: Signature =
                FuncValueType::new(inputs.clone(), out_types).try_into()?;
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

            let explicit_outputs_len = outputs.len();
            call_results.insert(explicit_outputs_len, end_value);

            // Return results from function
            args.outputs.finish(builder, call_results)?
        }
        GlobalsOp::Map {
            name,
            ty_arg,
            inputs,
            outputs,
            impl_outputs,
        } => {
            let sym = format!("{PREFIX}.{name}");
            let Some(global_ty_base) = ty_arg.as_runtime() else {
                Err(SignatureError::from(TermTypeError::TypeMismatch {
                    term: ty_arg.clone().into(),
                    type_: TYPE_PARAM.to_owned().into(),
                }))?
            };
            let sym_ty = context.llvm_sum_type(option_type([global_ty_base.clone()]))?;

            // Get function and args
            let [func, func_args @ ..] = &args.inputs[..] else {
                bail!("No function provided as input for GlobalsOp::Map")
            };

            let module = context.get_current_module();
            let builder = context.builder();

            // Get global variable
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
            let start_value = sym_ty.value(start_value)?;
            let start_value = start_value.build_untag(builder, 1)?[0];

            // real_args should be [global, *func_args]
            let mut real_args: Vec<BasicMetadataValueEnum> =
                func_args.iter().copied().map_into().collect_vec();
            real_args.insert(0, start_value.into());

            let func_ptr = PointerValue::try_from(*func)
                .map_err(|e| anyhow::anyhow!("Invalid function pointer provided to Map: {e:?}"))?;

            let mut in_types = inputs.iter().cloned().collect_vec();
            in_types.insert(0, global_ty_base.clone().into());

            let mut out_types = outputs.iter().cloned().collect_vec();
            out_types.push(global_ty_base.clone().into());
            out_types.extend(impl_outputs.iter().cloned().map_into());

            let global_position = outputs.len();

            let hugr_func_ty = FuncValueType::new(in_types, out_types).try_into()?;
            let func_ty = context.llvm_func_type(&hugr_func_ty)?;

            let func_call =
                builder.build_indirect_call(func_ty, func_ptr, &real_args, "call_func")?;

            let call_results =
                deaggregate_call_result(builder, func_call, hugr_func_ty.output.len())?;

            let (explicit_returns, rest) = call_results.split_at(global_position);
            let [end_value, implicit_returns @ ..] = &rest else {
                bail!("Global '{sym}' was not returned from function call")
            };

            let mut results = explicit_returns.to_vec();
            results.extend_from_slice(implicit_returns);

            let end_value = sym_ty.build_tag(builder, 1, vec![*end_value])?;
            let _ = builder.build_store(global.as_pointer_value(), end_value)?;

            args.outputs.finish(builder, results)?
        }
    }

    Ok(())
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
    use hugr::llvm::{
        check_emission,
        test::{TestContext, llvm_ctx, single_op_hugr},
    };
    use hugr_core::extension::simple_op::MakeRegisteredOp;

    #[rstest::rstest]
    #[case::with(1,
        GlobalsOp::With{ name: "my_global".to_string(), ty_arg: qb_t().into(), inputs: [bool_t(), qb_t()].into(), outputs: [bool_t()].into(), impl_outputs: [qb_t()].into() }
    )]
    #[case::map(2,
        GlobalsOp::Map{ name: "my_global".to_string(), ty_arg: qb_t().into(), inputs: [bool_t(), qb_t()].into(), outputs: [bool_t()].into(), impl_outputs: [qb_t()].into() }
    )]
    fn emit_globals_codegen(
        #[case] _i: i32,
        #[with(_i)] mut llvm_ctx: TestContext,
        #[case] op: GlobalsOp,
    ) {
        llvm_ctx.add_extensions(|ceb| {
            ceb.add_extension(GlobalsCodegenExtension)
                .add_default_prelude_extensions()
        });
        let hugr = single_op_hugr(op.to_extension_op().unwrap().into());
        check_emission!(hugr, llvm_ctx);
    }
}
