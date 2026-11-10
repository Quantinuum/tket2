//! LLVM lowering implementations for HUGR prelude operations.

use std::hash::{DefaultHasher, Hash as _, Hasher as _};
use tket::hugr::{self, llvm::inkwell};

use anyhow::{Result, anyhow, ensure};
use hugr::llvm::emit::func::EmitFuncContext;
use hugr::llvm::extension::prelude::{DefaultPreludeCodegen, PreludeCodegen};
use hugr::llvm::types::TypingSession;
use inkwell::AddressSpace;
use inkwell::attributes::{Attribute, AttributeLoc};
use inkwell::module::Linkage;
use inkwell::types::{BasicType, IntType};
use inkwell::values::{BasicValue, BasicValueEnum, FunctionValue, StructValue};
use tket::hugr::{HugrView, Node};

#[derive(Clone)]
/// Codegen extension for the QIS prelude.
///
/// `MakeError` requires a message loaded from a HUGR string constant;
/// its signal may be computed at runtime.
pub struct QISPreludeCodegen;

impl PreludeCodegen for QISPreludeCodegen {
    fn usize_type<'c>(&self, session: &TypingSession<'c, '_>) -> IntType<'c> {
        session.iw_context().i64_type()
    }

    /// Return the llvm type of [`tket::hugr::extension::prelude::qb_t`].
    fn qubit_type<'c>(&self, session: &TypingSession<'c, '_>) -> impl BasicType<'c> {
        session.iw_context().i64_type()
    }

    /// A panic ends all shots, this is done by using a return code > 1000.
    /// The lowering of the panic HUGR op shifts the return code held by the op by 1000.
    /// So valid values for `signal` in the panic HUGR op are 1-(2^31 - 1001).
    fn emit_panic<H: HugrView<Node = Node>>(
        &self,
        ctx: &mut EmitFuncContext<H>,
        err: BasicValueEnum,
    ) -> Result<()> {
        let err = StructValue::try_from(err)
            .map_err(|()| anyhow!("emit_panic: Expected err to have struct type"))?;
        let return_code = ctx.builder().build_extract_value(err, 0, "")?;
        // TODO check if code is in valid range 1-(2^31 - 1001), otherwise error.
        // https://github.com/quantinuum-dev/eldarion/issues/287
        let return_code = {
            let shift = ctx.iw_context().i32_type().const_int(1000, false);
            ctx.builder()
                .build_int_add(return_code.into_int_value(), shift, "shift_code")?
        };

        let msg = ctx.builder().build_extract_value(err, 1, "")?;
        let panic = Self::get_panic(ctx)?;
        ctx.builder()
            .build_call(panic, &[return_code.into(), msg.into()], "")?;
        Ok(())
    }

    /// An exit ends the current shot using a return code `rc`, where `1 <= rc <= 1000`.
    fn emit_exit<H: HugrView<Node = Node>>(
        &self,
        ctx: &mut EmitFuncContext<H>,
        err: BasicValueEnum,
    ) -> Result<()> {
        let err = StructValue::try_from(err)
            .map_err(|()| anyhow!("emit_exit: Expected err to have struct type"))?;
        let return_code = ctx.builder().build_extract_value(err, 0, "")?;
        // TODO check if code is in valid range 1-1000, otherwise error.
        // https://github.com/quantinuum-dev/eldarion/issues/287
        let msg = ctx.builder().build_extract_value(err, 1, "")?;
        let panic = Self::get_panic(ctx)?;
        ctx.builder()
            .build_call(panic, &[return_code.into(), msg.into()], "")?;
        Ok(())
    }

    fn emit_print<H: HugrView<Node = Node>>(
        &self,
        _ctx: &mut EmitFuncContext<H>,
        _text: inkwell::values::BasicValueEnum,
    ) -> Result<()> {
        Ok(())
    }

    fn emit_const_error<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &mut EmitFuncContext<'c, '_, H>,
        err: &hugr::extension::prelude::ConstError,
    ) -> Result<BasicValueEnum<'c>> {
        let err_ty = ctx
            .llvm_type(&hugr::extension::prelude::error_type())?
            .into_struct_type();
        let signal = err_ty
            .get_field_type_at_index(0)
            .unwrap()
            .into_int_type()
            .const_int(u64::from(err.signal), false);

        let message = Self::emit_error_message(ctx, &err.message)?;
        let err = err_ty.const_named_struct(&[signal.into(), message]);
        Ok(err.into())
    }

    fn emit_const_string<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &mut EmitFuncContext<'c, '_, H>,
        str: &hugr::extension::prelude::ConstString,
    ) -> Result<BasicValueEnum<'c>> {
        emit_global_string(ctx, str.value(), "s_", "")
    }

    fn emit_make_error<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &mut EmitFuncContext<'c, '_, H>,
        signal: BasicValueEnum<'c>,
        _message: BasicValueEnum<'c>,
        const_message: Option<&hugr::extension::prelude::ConstString>,
    ) -> Result<BasicValueEnum<'c>> {
        let message = const_message
            .ok_or_else(|| anyhow!("QIS MakeError requires a statically known string message"))?;
        let message = Self::emit_error_message(ctx, message.value())?;
        DefaultPreludeCodegen.emit_make_error(ctx, signal, message, None)
    }
}

impl QISPreludeCodegen {
    fn emit_error_message<'c, H: HugrView<Node = Node>>(
        ctx: &EmitFuncContext<'c, '_, H>,
        message: &str,
    ) -> Result<BasicValueEnum<'c>> {
        emit_global_string(ctx, message, "e_", "EXIT:INT:")
    }

    fn get_panic<'c, H: HugrView<Node = Node>>(
        ctx: &EmitFuncContext<'c, '_, H>,
    ) -> Result<FunctionValue<'c>> {
        let iwc = ctx.iw_context();
        let func_type = iwc.void_type().fn_type(
            &[
                iwc.i32_type().into(),
                iwc.ptr_type(AddressSpace::default()).into(),
            ],
            false,
        );
        let f = ctx.get_extern_func("panic", func_type)?;
        let noreturn = Attribute::get_named_enum_kind_id("noreturn");
        debug_assert!(noreturn != 0);
        f.add_attribute(
            AttributeLoc::Function,
            iwc.create_enum_attribute(noreturn, 0),
        );

        Ok(f)
    }
}

/// Emit a global string constant with a unique name.
///
/// The symbol name is constructed with `symbol_prefix` and ``str`.
///
/// The string is prefixed with `str_prefix`.
pub fn emit_global_string<'c, H: HugrView<Node = Node>>(
    ctx: &EmitFuncContext<'c, '_, H>,
    str: impl AsRef<str>,
    symbol_prefix: impl AsRef<str>,
    str_prefix: impl AsRef<str>,
) -> Result<BasicValueEnum<'c>> {
    let (str, symbol_prefix, str_prefix) =
        (str.as_ref(), symbol_prefix.as_ref(), str_prefix.as_ref());
    let tagged_str = format!("{str_prefix}{str}");
    let tagged_str_bytes = tagged_str.as_bytes();
    let tagged_str_len = tagged_str_bytes.len();
    ensure!(
        tagged_str_len < 256,
        "Constant string too long: {tagged_str_len} >= 256"
    );

    let tgt_str_bytes = [&[u8::try_from(tagged_str_len)?], tagged_str_bytes].concat();
    let tgt_str_value = ctx.iw_context().const_string(&tgt_str_bytes, false);

    let global = {
        // we need a unique name for the global. We use a prefix(of length
        // 10) of the string, followed by the hash of the whole string.
        //
        // This should be enough, but for absolute guarantee of success we
        // append ".i", i an integer and find an available name.
        let hash = {
            let mut s = DefaultHasher::default();
            tgt_str_bytes.hash(&mut s);
            let h = s.finish();
            let lower32 = h as u32;
            format!("{lower32:08X}")
        };
        // Use unicode-aware character slicing to prevent slicing through multi-byte characters
        let prefix_chars: String = str.chars().take(10).collect();
        let name_prefix = format!("{symbol_prefix}{prefix_chars}.{hash}");

        (0..)
            .find_map(|i| {
                let name = format!("{name_prefix}.{i}");
                if let Some(global) = ctx.get_current_module().get_global(&name) {
                    if let Some(v) = global.get_initializer()
                        && v == tgt_str_value
                    {
                        // We've found a global with the same value, use that.
                        return Some(global);
                    }
                } else {
                    let global =
                        ctx.get_current_module()
                            .add_global(tgt_str_value.get_type(), None, &name);
                    global.set_initializer(&tgt_str_value);
                    global.set_linkage(Linkage::Private);
                    global.set_constant(true);
                    return Some(global);
                }
                None
            })
            .unwrap()
    };
    Ok(global.as_pointer_value().as_basic_value_enum())
}

#[cfg(test)]
mod test {
    use hugr::builder::{Dataflow, DataflowHugr};
    use hugr::extension::PRELUDE;
    use hugr::extension::prelude::{
        self, ConstError, ConstString, ConstUsize, EXIT_OP_ID, MAKE_ERROR_OP_ID, PANIC_OP_ID, qb_t,
    };
    use hugr::llvm::check_emission;
    use hugr::llvm::emit::test::{Emission, SimpleHugrConfig, TEST_EMIT_DEBUG};
    use hugr::llvm::utils::fat::FatExt as _;

    use hugr::llvm::test::llvm_ctx;
    use hugr::types::TypeArg;
    use rstest::rstest;

    use super::*;

    #[rstest]
    fn test_panic_emit() {
        let mut llvm_ctx = llvm_ctx(0);
        let prelude_codegen = QISPreludeCodegen;
        llvm_ctx.add_extensions(move |ceb| ceb.add_prelude_extensions(prelude_codegen.clone()));

        // Create a hugr that has a panic message
        let error_val = ConstError::new(42, "PANIC");
        let type_arg_q = TypeArg::from(qb_t());
        let type_arg_2q = TypeArg::List(vec![type_arg_q.clone(), type_arg_q]);
        let panic_op = PRELUDE
            .instantiate_extension_op(&PANIC_OP_ID, [type_arg_2q.clone(), type_arg_2q.clone()])
            .unwrap();

        let mut hugr = SimpleHugrConfig::new()
            .with_ins(vec![qb_t(), qb_t()])
            .with_outs(vec![qb_t(), qb_t()])
            .with_extensions(prelude::PRELUDE_REGISTRY.to_owned())
            .finish(|mut builder| {
                let [q0, q1] = builder.input_wires_arr();
                let err = builder.add_load_value(error_val);
                let [q0, q1] = builder
                    .add_dataflow_op(panic_op, [err, q0, q1])
                    .unwrap()
                    .outputs_arr();
                builder.finish_hugr_with_outputs([q0, q1]).unwrap()
            });

        check_emission!(hugr, llvm_ctx);
    }

    #[rstest]
    fn test_exit_emit() {
        let mut llvm_ctx = llvm_ctx(0);
        let prelude_codegen = QISPreludeCodegen;
        llvm_ctx.add_extensions(move |ceb| ceb.add_prelude_extensions(prelude_codegen.clone()));

        let error_val = ConstError::new(42, "EXIT");
        let type_arg_q = TypeArg::from(qb_t());
        let type_arg_2q = TypeArg::List(vec![type_arg_q; 2]);
        let exit_op = PRELUDE
            .instantiate_extension_op(&EXIT_OP_ID, vec![type_arg_2q; 2])
            .unwrap();

        let mut hugr = SimpleHugrConfig::new()
            .with_ins(vec![qb_t(), qb_t()])
            .with_outs(vec![qb_t(), qb_t()])
            .with_extensions(prelude::PRELUDE_REGISTRY.to_owned())
            .finish(|mut builder| {
                let [q0, q1] = builder.input_wires_arr();
                let err = builder.add_load_value(error_val);
                let [q0, q1] = builder
                    .add_dataflow_op(exit_op, [err, q0, q1])
                    .unwrap()
                    .outputs_arr();
                builder.finish_hugr_with_outputs([q0, q1]).unwrap()
            });

        check_emission!(hugr, llvm_ctx);
    }

    #[rstest]
    #[case::exit(EXIT_OP_ID)]
    #[case::panic(PANIC_OP_ID)]
    fn test_make_error_emit(#[case] op_id: hugr::ops::OpName) {
        let mut llvm_ctx = llvm_ctx(0);
        llvm_ctx.add_extensions(|ceb| ceb.add_prelude_extensions(QISPreludeCodegen));

        let make_error_op = PRELUDE
            .instantiate_extension_op(&MAKE_ERROR_OP_ID, [])
            .unwrap();
        let terminate_op = PRELUDE
            .instantiate_extension_op(&op_id, [TypeArg::List(vec![]), TypeArg::List(vec![])])
            .unwrap();
        let hugr = SimpleHugrConfig::new()
            .with_extensions(prelude::PRELUDE_REGISTRY.to_owned())
            .finish(|mut builder| {
                let signal = builder.add_load_value(ConstUsize::new(42));
                let message =
                    builder.add_load_value(ConstString::new("Postselection failed".into()));
                let [err] = builder
                    .add_dataflow_op(make_error_op, [signal, message])
                    .unwrap()
                    .outputs_arr();
                builder.add_dataflow_op(terminate_op, [err]).unwrap();
                builder.finish_hugr_with_outputs([]).unwrap()
            });

        // Match Guppy's make_error path, rather than loading a ConstError.
        let emission = Emission::emit_hugr(
            hugr.fat_root().unwrap(),
            llvm_ctx.get_emit_hugr(),
            TEST_EMIT_DEBUG,
        )
        .unwrap();
        emission.verify().unwrap();
        let ir = emission.module().to_string();
        assert!(
            ir.contains(r#"[30 x i8] c"\1DEXIT:INT:Postselection failed""#),
            "{ir}"
        );
    }

    #[rstest]
    #[case::empty("")]
    #[case::unicode_and_nul("é\0oops")]
    #[case::maximum_error(&"x".repeat(246))]
    fn test_make_error_dynamic_signal(#[case] message: &str) {
        let mut llvm_ctx = llvm_ctx(0);
        llvm_ctx.add_extensions(|ceb| ceb.add_prelude_extensions(QISPreludeCodegen));
        let make_error = PRELUDE
            .instantiate_extension_op(&MAKE_ERROR_OP_ID, [])
            .unwrap();
        let hugr = SimpleHugrConfig::new()
            .with_ins([prelude::usize_t()])
            .with_outs([prelude::error_type(), prelude::string_type()])
            .with_extensions(prelude::PRELUDE_REGISTRY.to_owned())
            .finish(|mut builder| {
                let [signal] = builder.input_wires_arr();
                let text = builder.add_load_value(ConstString::new(message.into()));
                let [err] = builder
                    .add_dataflow_op(make_error, [signal, text])
                    .unwrap()
                    .outputs_arr();
                builder.finish_hugr_with_outputs([err, text]).unwrap()
            });
        let emission = Emission::emit_hugr(
            hugr.fat_root().unwrap(),
            llvm_ctx.get_emit_hugr(),
            TEST_EMIT_DEBUG,
        )
        .unwrap();
        emission.verify().unwrap();
        // The same source string is used as ordinary text and as an error.
        // Only the error global receives the tag; a computed signal is allowed.
        for (prefix, expected) in [
            ("s_", message.to_owned()),
            ("e_", format!("EXIT:INT:{message}")),
        ] {
            let global = emission
                .module()
                .get_globals()
                .find(|global| global.get_name().to_bytes().starts_with(prefix.as_bytes()))
                .unwrap();
            let initializer = global.get_initializer().unwrap();
            assert_eq!(
                initializer,
                llvm_ctx
                    .iw_context()
                    .const_string(
                        &[
                            &[u8::try_from(expected.len()).unwrap()],
                            expected.as_bytes()
                        ]
                        .concat(),
                        false
                    )
                    .as_basic_value_enum()
            );
        }
    }

    #[rstest]
    #[case::ordinary_long_string(false, "x".repeat(255))]
    #[case::oversize_error(true, "x".repeat(247))]
    fn test_string_length_limits(#[case] as_error: bool, #[case] message: String) {
        let mut llvm_ctx = llvm_ctx(0);
        llvm_ctx.add_extensions(|ceb| ceb.add_prelude_extensions(QISPreludeCodegen));
        let hugr = SimpleHugrConfig::new()
            .with_outs([if as_error {
                prelude::error_type()
            } else {
                prelude::string_type()
            }])
            .with_extensions(prelude::PRELUDE_REGISTRY.to_owned())
            .finish(|mut builder| {
                let text = builder.add_load_value(ConstString::new(message));
                let output = if as_error {
                    let signal = builder.add_load_value(ConstUsize::new(42));
                    let op = PRELUDE
                        .instantiate_extension_op(&MAKE_ERROR_OP_ID, [])
                        .unwrap();
                    builder
                        .add_dataflow_op(op, [signal, text])
                        .unwrap()
                        .out_wire(0)
                } else {
                    text
                };
                builder.finish_hugr_with_outputs([output]).unwrap()
            });
        let result = Emission::emit_hugr(
            hugr.fat_root().unwrap(),
            llvm_ctx.get_emit_hugr(),
            TEST_EMIT_DEBUG,
        );
        if as_error {
            let err = result
                .err()
                .expect("An error message plus its tag must fit in 255 bytes");
            assert!(format!("{err:#}").contains("Constant string too long: 256 >= 256"));
        } else {
            result.unwrap().verify().unwrap();
        }
    }

    #[rstest]
    fn test_make_error_unknown_message() {
        let mut llvm_ctx = llvm_ctx(0);
        llvm_ctx.add_extensions(|ceb| ceb.add_prelude_extensions(QISPreludeCodegen));
        let hugr = SimpleHugrConfig::new()
            .with_ins([prelude::usize_t(), prelude::string_type()])
            .with_outs([prelude::error_type()])
            .with_extensions(prelude::PRELUDE_REGISTRY.to_owned())
            .finish(|mut builder| {
                let [signal, message] = builder.input_wires_arr();
                let op = PRELUDE
                    .instantiate_extension_op(&MAKE_ERROR_OP_ID, [])
                    .unwrap();
                let [err] = builder
                    .add_dataflow_op(op, [signal, message])
                    .unwrap()
                    .outputs_arr();
                builder.finish_hugr_with_outputs([err]).unwrap()
            });
        let err = Emission::emit_hugr(
            hugr.fat_root().unwrap(),
            llvm_ctx.get_emit_hugr(),
            TEST_EMIT_DEBUG,
        )
        .err()
        .expect("Unknown messages must not silently bypass the error tag");
        assert!(
            format!("{err:#}").contains("QIS MakeError requires a statically known string message")
        );
    }
}
