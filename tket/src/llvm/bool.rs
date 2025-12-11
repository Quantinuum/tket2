//! `hugr-llvm` codegen extension for `tket.bool`.

use hugr::HugrView;
use hugr::Node;
use hugr::llvm::CodegenExtension;
use hugr::llvm::emit::EmitOpArgs;
use hugr::llvm::emit::emit_value;
use hugr::llvm::emit::func::EmitFuncContext;
use hugr::llvm::inkwell;
use hugr::llvm::sum::LLVMSumValue;
use hugr::llvm::types::TypingSession;
use hugr::ops::ExtensionOp;
use hugr::ops::Value;
use hugr::types::SumType;
use hugr::types::TypeName;

use crate::extension::bool::{BOOL_EXTENSION_ID, BoolOp, ConstBool};
use anyhow::{Result, anyhow};
use inkwell::IntPredicate;
use inkwell::types::IntType;

const BOOL_TYPE_ID: TypeName = TypeName::new_inline("bool");

fn llvm_bool_type<'c>(ts: &TypingSession<'c, '_>) -> IntType<'c> {
    ts.iw_context().bool_type()
}

/// A codegen extension for the `tket.bool` extension.
#[derive(Clone)]
pub struct BoolCodegenExtension;

impl BoolCodegenExtension {
    fn emit_bool_op<'c, H: HugrView<Node = Node>>(
        &self,
        context: &mut EmitFuncContext<'c, '_, H>,
        args: EmitOpArgs<'c, '_, ExtensionOp, H>,
        op: BoolOp,
    ) -> Result<()> {
        match op {
            BoolOp::read => {
                let [inp] = args
                    .inputs
                    .try_into()
                    .map_err(|_| anyhow!("BoolOp::read expects one argument"))?;
                let res = inp.into_int_value();
                let true_val = emit_value(context, &Value::true_val())?;
                let false_val = emit_value(context, &Value::false_val())?;
                let res = context
                    .builder()
                    .build_select(res, true_val, false_val, "")?;
                args.outputs.finish(context.builder(), vec![res])
            }
            BoolOp::make_opaque => {
                let [inp] = args
                    .inputs
                    .try_into()
                    .map_err(|_| anyhow!("BoolOp::make_opaque expects one argument"))?;
                let bool_ty = context.llvm_sum_type(SumType::new_unary(2))?;
                let bool_val = LLVMSumValue::try_new(inp, bool_ty)?;
                let res = bool_val.build_get_tag(context.builder())?;
                args.outputs.finish(context.builder(), vec![res.into()])
            }
            BoolOp::not => {
                let [inp] = args
                    .inputs
                    .try_into()
                    .map_err(|_| anyhow!("BoolOp::not expects one argument"))?;
                let res = inp.into_int_value();
                let res = context.builder().build_not(res, "")?;
                args.outputs.finish(context.builder(), vec![res.into()])
            }
            binary_op => {
                let [inp1, inp2] = args
                    .inputs
                    .try_into()
                    .map_err(|_| anyhow!("BoolOp::{binary_op:?} expects two arguments"))?;
                let inp1_val = inp1.into_int_value();
                let inp2_val = inp2.into_int_value();
                let res = match binary_op {
                    BoolOp::and => context.builder().build_and(inp1_val, inp2_val, "")?,
                    BoolOp::or => context.builder().build_or(inp1_val, inp2_val, "")?,
                    BoolOp::xor => context.builder().build_xor(inp1_val, inp2_val, "")?,
                    BoolOp::eq => context.builder().build_int_compare(
                        IntPredicate::EQ,
                        inp1_val,
                        inp2_val,
                        "",
                    )?,
                    _ => return Err(anyhow!("Unsupported binary bool operation")),
                };
                args.outputs.finish(context.builder(), vec![res.into()])
            }
        }
    }
}

impl CodegenExtension for BoolCodegenExtension {
    fn add_extension<'a, H: hugr::HugrView<Node = hugr::Node> + 'a>(
        self,
        builder: hugr::llvm::CodegenExtsBuilder<'a, H>,
    ) -> hugr::llvm::CodegenExtsBuilder<'a, H>
    where
        Self: 'a,
    {
        builder
            .custom_type((BOOL_EXTENSION_ID, BOOL_TYPE_ID), |ts, _| {
                Ok(llvm_bool_type(&ts).into())
            })
            .custom_const::<ConstBool>(|context, val| {
                let bool_ty = llvm_bool_type(&context.typing_session());
                Ok(bool_ty.const_int(val.value().into(), false).into())
            })
            .simple_extension_op(move |context, args, op| self.emit_bool_op(context, args, op))
    }
}

#[cfg(test)]
mod test {
    use rstest::rstest;

    use super::*;

    use hugr::extension::simple_op::MakeRegisteredOp;
    use hugr::llvm::check_emission;
    use hugr::llvm::extension::DefaultPreludeCodegen;
    use hugr::llvm::test::{TestContext, llvm_ctx, single_op_hugr};

    #[rstest]
    #[case::read(1, BoolOp::read)]
    #[case::make_opaque(2, BoolOp::make_opaque)]
    #[case::not(3, BoolOp::not)]
    #[case::and(4, BoolOp::and)]
    #[case::or(5, BoolOp::or)]
    #[case::xor(6, BoolOp::xor)]
    #[case::eq(7, BoolOp::eq)]
    fn emit_all_ops(#[case] _id: i32, #[with(_id)] mut llvm_ctx: TestContext, #[case] op: BoolOp) {
        let pcg = DefaultPreludeCodegen;
        llvm_ctx.add_extensions(move |ceb| {
            ceb.add_extension(BoolCodegenExtension)
                .add_prelude_extensions(pcg.clone())
                .add_default_int_extensions()
        });
        let ext_op = op.to_extension_op().unwrap().into();
        let hugr = single_op_hugr(ext_op);
        check_emission!(hugr, llvm_ctx);
    }
}
