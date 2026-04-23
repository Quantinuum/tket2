#![allow(missing_docs)]

use crate::extension::globals::{GlobalsOp, GlobalsOpDef};
use anyhow::{Result, bail, ensure};
use hugr::llvm::emit::deaggregate_call_result;
use hugr::llvm::inkwell::types::BasicTypeEnum;
use hugr::llvm::inkwell::values::{
    AsValueRef, BasicMetadataValueEnum, BasicValueEnum, PointerValue,
};
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
        GlobalsOp::Swap { name, ty } => {
            let sym = format!("{PREFIX}.{name}");
            let sym_ty = context.llvm_sum_type(option_type([ty.clone()]))?;

            let [new_value] = &args.inputs[..] else {
                bail!("Expected one input for GlobalsOp::Swap")
            };
            let new_value_ty = new_value.get_type();
            ensure!(
                new_value_ty == sym_ty.as_basic_type_enum(),
                "Input type does not match global variable type. Found {new_value_ty}, Expected {sym_ty}"
            );

            let module = context.get_current_module();
            let builder = context.builder();
            let none_value = sym_ty.build_tag(builder, 0, vec![])?;

            let global = module.get_global(&sym).unwrap_or_else(|| {
                let global = module.add_global(sym_ty.clone(), Some(AddressSpace::default()), &sym);
                global.set_initializer(&none_value);
                global
            });

            let result = builder.build_load(sym_ty, global.as_pointer_value(), "current_value")?;
            let _ = builder.build_store(global.as_pointer_value(), *new_value)?;
            args.outputs.finish(builder, [result])?
        }
        GlobalsOp::With {
            name,
            ty_arg,
            inputs,
            outputs,
        } => {
            let sym = format!("{PREFIX}.{name}");
            // TODO fix unwrap
            let sym_ty = context.llvm_sum_type(option_type([ty_arg.as_runtime().unwrap()]))?;

            let [init_global_value, func, func_args @ ..] = &args.inputs[..] else {
                bail!("No function provided as input for GlobalsOp::Map")
            };

            let module = context.get_current_module();
            let builder = context.builder();
            let none_value = sym_ty.build_tag(builder, 0, vec![])?;

            let global = module.get_global(&sym).unwrap_or_else(|| {
                let global = module.add_global(sym_ty.clone(), Some(AddressSpace::default()), &sym);
                global.set_initializer(&none_value);
                global
            });

            let global_ty: BasicTypeEnum = global.get_value_type().try_into().unwrap();
            ensure!(
                global_ty == sym_ty.as_basic_type_enum(),
                "Input type does not match global variable type. Found {global_ty}, Expected {sym_ty}"
            );
            let old_value =
                builder.build_load(sym_ty.clone(), global.as_pointer_value(), "current_value")?;

            let new_value = sym_ty.build_tag(builder, 1, vec![*init_global_value])?;

            let _ = builder.build_store(global.as_pointer_value(), new_value)?;

            let real_args = func_args.iter().cloned().map_into().collect_vec();
            let func_ptr = PointerValue::try_from(*func).unwrap();
            let hugr_func_ty: Signature =
                FuncValueType::new(inputs.clone(), outputs.clone()).try_into()?;
            let func_ty = context.llvm_func_type(&hugr_func_ty)?;

            let func_call =
                builder.build_indirect_call(func_ty, func_ptr, &real_args, "call_func")?;
            dbg!(&func_call);

            // Put global back

            let end_value =
                builder.build_load(sym_ty.clone(), global.as_pointer_value(), "current_value")?;

            let end_value = sym_ty.value(end_value)?;
            let end_value = end_value.build_untag(builder, 1)?[0];

            let _ = builder.build_store(global.as_pointer_value(), old_value)?;

            // extract results
            let call_results =
                deaggregate_call_result(builder, func_call, hugr_func_ty.output.len())?;

            let mut results = vec![end_value];
            results.extend(call_results);

            // Return results from function
            args.outputs.finish(builder, results)?
        }
        GlobalsOp::Map {
            name,
            ty_arg,
            inputs,
            outputs,
        } => unimplemented!(),
    }

    Ok(())
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
    #[case::with(1,GlobalsOp::With { name: "my_global".to_string(), ty_arg: qb_t().into(), inputs: [qb_t(), bool_t()].into(), outputs: [qb_t(), bool_t()].into() })]
    // #[case::map(2,GlobalsOp::Map { name: "my_global".to_string(), ty_arg: qb_t().into(), inputs: [qb_t(), bool_t()].into(), outputs: [qb_t(), bool_t()].into() })]
    fn emit_futures_codegen(
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
