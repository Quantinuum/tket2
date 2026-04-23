#![allow(missing_docs)]

use crate::extension::globals::{GlobalsOp, GlobalsOpDef};
use anyhow::{Result, bail, ensure};
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
        GlobalsOp::With { .. } => unimplemented!(),
        GlobalsOp::Map { .. } => unimplemented!(),
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
    #[case::map(2,GlobalsOp::Map { name: "my_global".to_string(), ty_arg: qb_t().into(), inputs: [qb_t(), bool_t()].into(), outputs: [qb_t(), bool_t()].into() })]
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
