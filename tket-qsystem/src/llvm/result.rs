//! LLVM lowering implementations for "tket.results" extension.

use crate::extension::result::{ResultArgs, ResultOp, ResultOpDef};
use crate::llvm::prelude::emit_global_string;
use anyhow::{Result, anyhow, bail};
use hugr::llvm::CodegenExtsBuilder;
use hugr::llvm::custom::CodegenExtension;
use hugr::llvm::emit::{EmitFuncContext, EmitOpArgs};
use hugr::llvm::inkwell;
use hugr::llvm::sum::LLVMSumValue;
use hugr::llvm::types::HugrSumType;
use inkwell::AddressSpace;
use inkwell::builder::Builder;
use inkwell::context::Context;
use inkwell::types::{FloatType, IntType, PointerType, VoidType};
use inkwell::values::{BasicValueEnum, FunctionValue, IntValue};
use tket::hugr::extension::simple_op::MakeExtensionOp;
use tket::hugr::ops::ExtensionOp;
use tket::hugr::{HugrView, Node};

use super::array_utils::{ArrayLowering, ElemType, struct_1d_arr_alloc, struct_1d_arr_ptr_t};

static TAG_PREFIX: &str = "USER:";

/// Codegen extension for results
#[derive(Default)]
pub struct ResultsCodegenExtension<AL: ArrayLowering> {
    array_lowering: AL,
}

impl<AL: ArrayLowering> ResultsCodegenExtension<AL> {
    /// Creates a new [ResultsCodegenExtension] with specified array lowering.
    pub const fn new(array_lowering: AL) -> Self {
        Self { array_lowering }
    }
}

impl<AL: ArrayLowering + Clone> CodegenExtension for ResultsCodegenExtension<AL> {
    fn add_extension<'a, H: HugrView<Node = Node> + 'a>(
        self,
        builder: CodegenExtsBuilder<'a, H>,
    ) -> CodegenExtsBuilder<'a, H>
    where
        Self: 'a,
    {
        builder.simple_extension_op::<ResultOpDef>(move |context, args, _op| {
            let op = ResultOp::from_extension_op(args.node().as_ref())?;
            ResultEmitter(context, self.array_lowering.clone()).emit(args, &op)
        })
    }
}

struct ResultEmitter<'c, 'd, 'e, H: HugrView<Node = Node>, AL: ArrayLowering>(
    &'d mut EmitFuncContext<'c, 'e, H>,
    AL,
);

impl<'c, H: HugrView<Node = Node>, AL: ArrayLowering + Clone> ResultEmitter<'c, '_, '_, H, AL> {
    fn iw_context(&self) -> &'c Context {
        self.0.typing_session().iw_context()
    }

    fn int_t(&self) -> IntType<'c> {
        self.iw_context().i64_type()
    }

    fn float_t(&self) -> FloatType<'c> {
        self.iw_context().f64_type()
    }

    fn bool_t(&self) -> IntType<'c> {
        self.iw_context().bool_type()
    }

    fn i8_ptr_t(&self) -> PointerType<'c> {
        self.iw_context()
            .i8_type()
            .ptr_type(AddressSpace::default())
    }

    fn void_t(&self) -> VoidType<'c> {
        self.iw_context().void_type()
    }

    fn builder(&self) -> &Builder<'c> {
        self.0.builder()
    }

    fn get_func_print(&self, op: &ResultOp) -> Result<FunctionValue<'c>> {
        // The first two parameters are the same for all print function variants
        let mut params = vec![self.i8_ptr_t().into(), self.int_t().into()];
        let symbol = match op.result_op {
            ResultOpDef::Bool => {
                params.push(self.bool_t().into());
                "print_bool"
            }
            ResultOpDef::Int => {
                params.push(self.int_t().into());
                "print_int"
            }
            ResultOpDef::UInt => {
                params.push(self.int_t().into());
                "print_uint"
            }
            ResultOpDef::F64 => {
                params.push(self.float_t().into());
                "print_float"
            }
            ResultOpDef::ArrBool => {
                params.push(struct_1d_arr_ptr_t(self.iw_context(), &ElemType::Bool).into());
                "print_bool_arr"
            }
            ResultOpDef::ArrInt => {
                params.push(struct_1d_arr_ptr_t(self.iw_context(), &ElemType::Int).into());
                "print_int_arr"
            }
            ResultOpDef::ArrUInt => {
                params.push(struct_1d_arr_ptr_t(self.iw_context(), &ElemType::Uint).into());
                "print_uint_arr"
            }
            ResultOpDef::ArrF64 => {
                params.push(struct_1d_arr_ptr_t(self.iw_context(), &ElemType::Float).into());
                "print_float_arr"
            }
        };

        let func_t = self.void_t().fn_type(&params, false);
        self.0.get_extern_func(symbol, func_t)
    }

    fn generate_global_tag(
        &self,
        args: &EmitOpArgs<'c, '_, ExtensionOp, H>,
        type_tag: impl AsRef<str>,
    ) -> Result<(BasicValueEnum<'_>, IntValue<'_>)> {
        let type_tag = type_tag.as_ref();
        let result_op = ResultOp::from_extension_op(args.node().as_ref())?;

        let tag = result_op.tag;
        if tag.is_empty() {
            bail!("Empty result tag received");
        }

        let tag_ptr = emit_global_string(self.0, tag, "res_", format!("{TAG_PREFIX}{type_tag}"))?;
        let tag_len = {
            let mut l = self
                .0
                .builder()
                .build_load(tag_ptr.into_pointer_value(), "tag_len")?
                .into_int_value();
            if self.int_t() != l.get_type() {
                l = self
                    .builder()
                    .build_int_z_extend(l, self.int_t(), "tag_len")?;
            }
            l
        };

        Ok((tag_ptr, tag_len))
    }

    fn build_print_array_call(
        &self,
        val: BasicValueEnum,
        op: &ResultOp,
        data_type: &ElemType,
        tag_ptr: BasicValueEnum,
        tag_len: IntValue,
    ) -> Result<()> {
        // TODO update to return array after https://github.com/quantinuum/tket2/issues/922
        let ResultArgs::Array(_, length) = op.args else {
            bail!("Expected array argument")
        };

        let print_fn = self.get_func_print(op)?;
        let array = self.1.array_to_ptr(self.builder(), val)?;
        let (array_ptr, _) = struct_1d_arr_alloc(
            self.iw_context(),
            self.builder(),
            length.try_into()?,
            data_type,
            array,
        )?;
        self.builder().build_call(
            print_fn,
            &[tag_ptr.into(), tag_len.into(), array_ptr.into()],
            "",
        )?;
        Result::Ok(())
    }

    /// Function to help lower the tket result extension.
    fn emit(&self, args: EmitOpArgs<'c, '_, ExtensionOp, H>, op: &ResultOp) -> Result<()> {
        let print_fn = self.get_func_print(op)?;
        match op.result_op {
            ResultOpDef::Bool => {
                let (tag_ptr, tag_len) = self.generate_global_tag(&args, "BOOL:").unwrap();
                let [val] = args
                    .inputs
                    .try_into()
                    .map_err(|_| anyhow!("result_bool expects one input"))?;
                let bool_type = self.0.llvm_sum_type(HugrSumType::new_unary(2))?;
                let val = LLVMSumValue::try_new(val, bool_type)
                    .map_err(|_| anyhow!("bool_type expects a value"))?
                    .build_get_tag(self.builder())?;
                let b_type = self.bool_t();
                let trunc_val = self.builder().build_int_truncate(val, b_type, "")?;
                self.builder().build_call(
                    print_fn,
                    &[tag_ptr.into(), tag_len.into(), trunc_val.into()],
                    "print_bool",
                )?;
            }
            ResultOpDef::Int => {
                let (tag_ptr, tag_len) = self.generate_global_tag(&args, "INT:").unwrap();
                let [val] = args
                    .inputs
                    .try_into()
                    .map_err(|_| anyhow!("result_int expects one input"))?;
                self.builder().build_call(
                    print_fn,
                    &[tag_ptr.into(), tag_len.into(), val.into()],
                    "print_int",
                )?;
            }
            ResultOpDef::UInt => {
                let (tag_ptr, tag_len) = self.generate_global_tag(&args, "INT:").unwrap();
                let [val] = args
                    .inputs
                    .try_into()
                    .map_err(|_| anyhow!("result_uint expects one input"))?;
                self.builder().build_call(
                    print_fn,
                    &[tag_ptr.into(), tag_len.into(), val.into()],
                    "print_uint",
                )?;
            }
            ResultOpDef::F64 => {
                let (tag_ptr, tag_len) = self.generate_global_tag(&args, "FLOAT:").unwrap();
                let [val] = args
                    .inputs
                    .try_into()
                    .map_err(|_| anyhow!("result_float expects one input"))?;
                self.builder().build_call(
                    print_fn,
                    &[tag_ptr.into(), tag_len.into(), val.into()],
                    "print_int",
                )?;
            }
            ResultOpDef::ArrBool => {
                let (tag_ptr, tag_len) = self.generate_global_tag(&args, "BOOLARR:").unwrap();
                let [val] = args
                    .inputs
                    .try_into()
                    .map_err(|_| anyhow!("result_arr_bool expects one input"))?;
                self.build_print_array_call(val, op, &ElemType::Bool, tag_ptr, tag_len)?;
            }
            ResultOpDef::ArrInt => {
                let (tag_ptr, tag_len) = self.generate_global_tag(&args, "INTARR:").unwrap();
                let [val] = args
                    .inputs
                    .try_into()
                    .map_err(|_| anyhow!("result_arr_int expects one input"))?;
                self.build_print_array_call(val, op, &ElemType::Int, tag_ptr, tag_len)?;
            }
            ResultOpDef::ArrUInt => {
                let (tag_ptr, tag_len) = self.generate_global_tag(&args, "INTARR:").unwrap();
                let [val] = args
                    .inputs
                    .try_into()
                    .map_err(|_| anyhow!("result_arr_uint expects one input"))?;
                self.build_print_array_call(val, op, &ElemType::Uint, tag_ptr, tag_len)?;
            }
            ResultOpDef::ArrF64 => {
                let (tag_ptr, tag_len) = self.generate_global_tag(&args, "FLOATARR:").unwrap();
                let [val] = args
                    .inputs
                    .try_into()
                    .map_err(|_| anyhow!("result_arr_float expects one input"))?;
                self.build_print_array_call(val, op, &ElemType::Float, tag_ptr, tag_len)?;
            }
        }
        args.outputs.finish(self.builder(), [])
    }
}

#[cfg(test)]
mod test {
    use crate::extension::result::ResultOp;
    use crate::llvm::array_utils::DEFAULT_HEAP_ARRAY_LOWERING;
    use crate::llvm::array_utils::DEFAULT_STACK_ARRAY_LOWERING;

    use hugr::extension::simple_op::MakeRegisteredOp;
    use hugr::llvm::check_emission;
    use hugr::llvm::test::TestContext;
    use hugr::llvm::test::llvm_ctx;
    use hugr::llvm::test::single_op_hugr;

    use crate::llvm::prelude::QISPreludeCodegen;

    use rstest::rstest;

    use super::*;

    #[rstest]
    #[case::bool(1, ResultOp::new_bool("test_bool"), &DEFAULT_STACK_ARRAY_LOWERING)]
    #[case::int(2, ResultOp::new_int("test_int", 6), &DEFAULT_STACK_ARRAY_LOWERING)]
    #[case::uint(3, ResultOp::new_uint("test_uint", 6), &DEFAULT_STACK_ARRAY_LOWERING)]
    #[case::f64(4, ResultOp::new_f64("test_f64"), &DEFAULT_STACK_ARRAY_LOWERING)]
    #[case::arr_bool(5, ResultOp::new_bool("test_arr_bool").array_op(10), &DEFAULT_HEAP_ARRAY_LOWERING)]
    #[case::arr_bool(6, ResultOp::new_bool("test_arr_bool").array_op(10), &DEFAULT_STACK_ARRAY_LOWERING)]
    #[case::arr_int(7, ResultOp::new_int("test_arr_int", 6).array_op(10), &DEFAULT_HEAP_ARRAY_LOWERING)]
    #[case::arr_int(8, ResultOp::new_int("test_arr_int", 6).array_op(10), &DEFAULT_STACK_ARRAY_LOWERING)]
    #[case::arr_uint(9, ResultOp::new_uint("test_arr_uint", 6).array_op(10), &DEFAULT_HEAP_ARRAY_LOWERING)]
    #[case::arr_int(10, ResultOp::new_int("test_arr_int", 6).array_op(10), &DEFAULT_STACK_ARRAY_LOWERING)]
    #[case::arr_f64(11, ResultOp::new_f64("test_arr_f64").array_op(10), &DEFAULT_HEAP_ARRAY_LOWERING)]
    // test cases for various tags
    #[case::unicode_tag(12, ResultOp::new_int("测试字符串", 6), &DEFAULT_STACK_ARRAY_LOWERING)]
    #[case::special_chars(13, ResultOp::new_uint("test!@#$%^&*()", 6), &DEFAULT_STACK_ARRAY_LOWERING)]
    #[should_panic(expected = "Constant string too long")]
    #[case::very_long_tag(14, ResultOp::new_f64("x".repeat(256)), &DEFAULT_STACK_ARRAY_LOWERING)]
    #[case::whitespace(15, ResultOp::new_bool("   spaces   tabs\t\t\tnewlines\n\n\n"), &DEFAULT_STACK_ARRAY_LOWERING)]
    #[case::emoji(16, ResultOp::new_bool("🚀👨‍👩‍👧‍👦🌍"), &DEFAULT_STACK_ARRAY_LOWERING)]
    #[should_panic(expected = "Empty result tag received")]
    #[case::actually_empty(17, ResultOp::new_bool(""), &DEFAULT_STACK_ARRAY_LOWERING)]
    fn emit_result_codegen(
        #[case] _i: i32,
        #[with(_i)] mut llvm_ctx: TestContext,
        #[case] op: ResultOp,
        #[case] array_lowering: &'static (impl ArrayLowering + Clone),
    ) {
        let pcg = QISPreludeCodegen;
        llvm_ctx.add_extensions(move |ceb| {
            ceb.add_extension(ResultsCodegenExtension::new(array_lowering.clone()))
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
