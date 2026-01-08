#![allow(missing_docs)]

use anyhow::{bail, ensure, Result};
use hugr::{
    extension::{prelude::option_type, simple_op::HasConcrete as _},
    llvm::{
        emit::{EmitFuncContext, EmitOpArgs},
        inkwell::{types::BasicType as _, AddressSpace},
        CodegenExtension, CodegenExtsBuilder,
    },
    ops::ExtensionOp,
    HugrView, Node,
};
use tket::extension::globals::{GlobalsOp, GlobalsOpDef};

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
            let sym_ty = context.llvm_sum_type(option_type(ty.clone()))?;

            let [new_value] = &args.inputs[..] else {
                bail!("Expected one input for GlobalsOp::Swap")
            };
            let new_value_ty = new_value.get_type();
            ensure!(new_value_ty == sym_ty.as_basic_type_enum(), "Input type does not match global variable type. Found {new_value_ty}, Expected {sym_ty}");

            let module = context.get_current_module();
            let builder = context.builder();
            let none_value = sym_ty.build_tag(builder, 0, vec![])?;

            let global = module.get_global(&sym).unwrap_or_else(|| {
                let global = module.add_global(sym_ty, Some(AddressSpace::default()), &sym);
                global.set_initializer(&none_value);
                global.set_thread_local(true);
                global
            });

            let result = builder.build_load(global.as_pointer_value(), "current_value")?;
            let _ = builder.build_store(global.as_pointer_value(), *new_value)?;
            args.outputs.finish(builder, [result])?
        }
    }

    Ok(())
}

#[cfg(test)]
mod test {
    use super::*;
    use hugr::extension::prelude::usize_t;
    
    use hugr::llvm::check_emission;
    use hugr::llvm::test::llvm_ctx;
    use hugr::llvm::test::single_op_hugr;
    use hugr::llvm::test::TestContext;
    

    #[rstest::rstest]
    fn emit_debug_codegen(mut llvm_ctx: TestContext) {
        llvm_ctx.add_extensions(move |ceb| {
            ceb.add_default_prelude_extensions()
                .add_extension(GlobalsCodegenExtension)
        });
        let hugr = single_op_hugr(
            GlobalsOp::Swap {
                name: "my_global".to_string(),
                ty: usize_t(),
            }
            .into(),
        );
        check_emission!(hugr, llvm_ctx);
    }
}
