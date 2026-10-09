use super::*;
use crate::hugr;
use hugr::builder::{Dataflow, DataflowHugr, DataflowSubContainer, HugrBuilder};
use hugr::extension::prelude::bool_t;
use hugr::llvm::emit::{EmitDebugInfo, EmitFuncContext, Namer, test::SimpleHugrConfig};
use hugr::llvm::extension::PreludeCodegen;
use hugr::llvm::inkwell::{self, context::Context, module::Module, values::PointerValue};
use hugr::llvm::utils::IntOpBuilder;
use hugr::ops::handle::NodeHandle;
use hugr::std_extensions::arithmetic::int_types::{ConstInt, int_type};
use hugr::types::Signature;
use hugr::{HugrView, Node};
use std::{cell::RefCell, rc::Rc};
use tket_qsystem::extension::globals::GlobalsOp;
use tket_qsystem::llvm::globals::{DefaultGlobalsLockCodegen, GlobalsLockCodegen};

fn op(with: bool) -> GlobalsOp {
    if with {
        GlobalsOp::With {
            name: "state".into(),
            ty_arg: int_type(6).into(),
            inputs: [].into(),
            outputs: [].into(),
        }
    } else {
        GlobalsOp::Map {
            name: "state".into(),
            ty_arg: int_type(6).into(),
            inputs: [].into(),
            outputs: [].into(),
        }
    }
}

fn graph(reenter: &str) -> Hugr {
    let ty = int_type(6);
    SimpleHugrConfig::new()
        .with_extensions(REGISTRY.to_owned())
        .with_outs([bool_t()])
        .finish(|mut b| {
            let mut mb = b.module_root_builder();
            let cb = mb
                .define_function("identity", Signature::new_endo([ty.clone()]))
                .unwrap();
            let [v] = cb.input_wires_arr();
            let identity = cb.finish_with_outputs([v]).unwrap();
            let empty = mb
                .define_function("empty", Signature::new_endo([]))
                .unwrap()
                .finish_with_outputs([])
                .unwrap();
            let mut cb = mb
                .define_function("update", Signature::new_endo([ty.clone()]))
                .unwrap();
            let [v] = cb.input_wires_arr();
            if reenter == "map" {
                let f = cb.load_func(identity.handle(), &[]).unwrap();
                let inner = cb.add_dataflow_op(op(false), [f]).unwrap();
                cb.set_order(&inner.node(), &cb.output().node());
            } else if reenter == "with" {
                let f = cb.load_func(empty.handle(), &[]).unwrap();
                let nested = cb.add_dataflow_op(op(true), [v, f]).unwrap();
                let [v] = nested.outputs_arr();
                let one = cb.add_load_value(ConstInt::new_u(6, 1).unwrap());
                let v = cb.add_iadd(6, v, one).unwrap();
                // Only reached if the lock incorrectly permits replacing a borrowed slot.
                let update = cb.finish_with_outputs([v]).unwrap();
                return finish_graph(b, update.handle());
            }
            let one = cb.add_load_value(ConstInt::new_u(6, 1).unwrap());
            let v = cb.add_iadd(6, v, one).unwrap();
            let update = cb.finish_with_outputs([v]).unwrap();
            finish_graph(b, update.handle())
        })
}

fn finish_graph(
    mut b: hugr::llvm::emit::test::DFGW,
    update: &hugr::ops::handle::FuncID<true>,
) -> Hugr {
    let mut mb = b.module_root_builder();
    let mut cb = mb
        .define_function("scope", Signature::new_endo([]))
        .unwrap();
    let f = cb.load_func(update, &[]).unwrap();
    let first = cb.add_dataflow_op(op(false), [f]).unwrap();
    let second = cb.add_dataflow_op(op(false), [f]).unwrap();
    cb.set_order(&first.node(), &second.node());
    let scope = cb.finish_with_outputs([]).unwrap();
    let f = b.load_func(scope.handle(), &[]).unwrap();
    let initial = b.add_load_value(ConstInt::new_u(6, 41).unwrap());
    let [result] = b
        .add_dataflow_op(op(true), [initial, f])
        .unwrap()
        .outputs_arr();
    let expected = b.add_load_value(ConstInt::new_u(6, 43).unwrap());
    let ok = b.add_ieq(6, result, expected).unwrap();
    b.finish_hugr_with_outputs([ok]).unwrap()
}

fn emit<'c>(
    ctx: &'c Context,
    platform: QSystemPlatform,
    graph: &Hugr,
    locks: Option<TracingLocks>,
) -> Module<'c> {
    let extensions = if let Some(locks) = locks {
        CodegenExtsBuilder::default()
            .add_prelude_extensions(QISPreludeCodegen)
            .add_extension(IntCodegenExtension::new(QISPreludeCodegen))
            .add_extension(GlobalsCodegenExtension::new(QISPreludeCodegen).with_lock_codegen(locks))
            .finish()
    } else {
        codegen_extensions(platform)
    };
    let (module, _) = crate::get_hugr_llvm_module(
        ctx,
        Rc::new(Namer::new("", false)),
        graph,
        "globals",
        Rc::new(extensions),
        EmitDebugInfo::Exclude,
    )
    .unwrap();
    module.verify().unwrap();
    module
        .get_function("main")
        .unwrap()
        .set_linkage(inkwell::module::Linkage::External);
    module
}

thread_local! { static EVENTS: RefCell<Vec<(usize, bool, u64)>> = const { RefCell::new(Vec::new()) }; }
extern "C" fn event(global: *const u8, locked: bool, value: u64) {
    EVENTS.with_borrow_mut(|events| events.push((global as usize, locked, value)));
}
unsafe extern "C" fn runtime_panic(code: u32, message: *const u8) {
    // QIS strings contain a length byte followed by tagged bytes, without NUL.
    let bytes = unsafe { std::slice::from_raw_parts(message.add(1), usize::from(*message)) };
    eprintln!("globals panic {code}: {}", String::from_utf8_lossy(bytes));
    std::process::exit(86);
}
fn execute(module: &Module<'_>, repetitions: usize) -> bool {
    let engine = module
        .create_jit_execution_engine(inkwell::OptimizationLevel::None)
        .unwrap();
    for (name, address) in [
        ("panic", runtime_panic as *const () as usize),
        ("global_lock_event", event as *const () as usize),
    ] {
        if let Some(function) = module.get_function(name) {
            engine.add_global_mapping(&function, address);
        }
    }
    unsafe {
        let main = engine
            .get_function::<unsafe extern "C" fn() -> bool>("main")
            .unwrap();
        (0..repetitions).all(|_| main.call())
    }
}

struct TracingLocks {
    double_unlock: bool,
}
impl TracingLocks {
    fn record<'c, H: HugrView<Node = Node>>(
        context: &mut EmitFuncContext<'c, '_, H>,
        global: PointerValue<'c>,
        locked: bool,
    ) -> Result<()> {
        let ty = context.iw_context().bool_type();
        let value_ty =
            context.llvm_sum_type(hugr::extension::prelude::option_type([int_type(6)]))?;
        let module = context.get_current_module();
        assert_eq!(
            global,
            module
                .get_global("__globals__.state")
                .unwrap()
                .as_pointer_value(),
            "hooks must receive the global slot, not its payload or a fabricated identifier"
        );
        let value = context
            .builder()
            .build_load(value_ty.clone(), global, "observed_global")?;
        let value = value_ty.value(value)?;
        let tag = value.build_get_tag(context.builder())?;
        let present = context.builder().build_int_compare(
            inkwell::IntPredicate::EQ,
            tag,
            tag.get_type().const_int(1, false),
            "present",
        )?;
        let payload = value.build_untag(context.builder(), 1)?[0];
        let payload_ty = context.iw_context().i64_type();
        let observed = context.builder().build_select(
            present,
            payload,
            payload_ty.const_zero().into(),
            "observed_value",
        )?;
        let f = module.get_function("global_lock_event").unwrap_or_else(|| {
            module.add_function(
                "global_lock_event",
                context.iw_context().void_type().fn_type(
                    &[global.get_type().into(), ty.into(), payload_ty.into()],
                    false,
                ),
                None,
            )
        });
        context.builder().build_call(
            f,
            &[
                global.into(),
                ty.const_int(u64::from(locked), false).into(),
                observed.into(),
            ],
            "",
        )?;
        Ok(())
    }
}
impl GlobalsLockCodegen for TracingLocks {
    fn emit_lock<'c, H: HugrView<Node = Node>, PCG: PreludeCodegen>(
        &self,
        context: &mut EmitFuncContext<'c, '_, H>,
        global: PointerValue<'c>,
        prelude: &PCG,
    ) -> Result<()> {
        DefaultGlobalsLockCodegen::default().emit_lock(context, global, prelude)?;
        Self::record(context, global, true)
    }
    fn emit_unlock<'c, H: HugrView<Node = Node>, PCG: PreludeCodegen>(
        &self,
        context: &mut EmitFuncContext<'c, '_, H>,
        global: PointerValue<'c>,
        prelude: &PCG,
    ) -> Result<()> {
        DefaultGlobalsLockCodegen::default().emit_unlock(context, global, prelude)?;
        if self.double_unlock {
            DefaultGlobalsLockCodegen::default().emit_unlock(context, global, prelude)?;
        }
        Self::record(context, global, false)
    }
}

#[test]
fn globals_default_map_restores_value_and_unlocks() {
    for platform in [QSystemPlatform::Sol, QSystemPlatform::Helios] {
        let ctx = Context::create();
        let graph = graph("");
        let module = emit(&ctx, platform, &graph, None);
        let ir = module.print_to_string().to_string();
        assert!(module.get_function("panic").is_some());
        assert!(ir.contains("i32 1001"));
        assert!(!ir.contains("atomic"));
        assert!(
            execute(&module, 2),
            "With restores the original scope and releases its lock"
        );
    }
}

#[test]
fn globals_custom_hooks_cover_scoped_install_map_and_restore() {
    EVENTS.with_borrow_mut(Vec::clear);
    let ctx = Context::create();
    let graph = graph("");
    let module = emit(
        &ctx,
        QSystemPlatform::Helios,
        &graph,
        Some(TracingLocks {
            double_unlock: false,
        }),
    );
    assert!(execute(&module, 1));
    EVENTS.with_borrow(|events| {
        let address = events[0].0;
        assert_ne!(address, 0);
        assert!(
            events.iter().all(|event| event.0 == address),
            "With and Map lock/unlock must use one stable slot address"
        );
        let transitions = events
            .iter()
            .map(|&(_, locked, value)| (locked, value))
            .collect::<Vec<_>>();
        assert_eq!(
            transitions,
            &[
                (true, 0),
                (false, 41),
                (true, 41),
                (false, 42),
                (true, 42),
                (false, 43),
                (true, 43),
                (false, 0)
            ]
        )
    });
}

#[test]
fn globals_lock_errors_use_qis_panic() {
    const ENV: &str = "TKET_GLOBAL_LOCK_PANIC_CASE";
    if let Ok(case) = std::env::var(ENV) {
        let ctx = Context::create();
        let graph = graph(if case == "unlock" { "" } else { &case });
        let module = emit(
            &ctx,
            QSystemPlatform::Helios,
            &graph,
            (case == "unlock").then_some(TracingLocks {
                double_unlock: true,
            }),
        );
        execute(&module, 1);
        panic!("expected nonreturning QIS panic");
    }
    for (case, message) in [
        ("map", "Global already locked"),
        ("with", "Global already locked"),
        ("unlock", "Global not locked"),
    ] {
        let out = std::process::Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "extensions::globals_tests::globals_lock_errors_use_qis_panic",
                "--nocapture",
            ])
            .env(ENV, case)
            .output()
            .unwrap();
        assert_eq!(
            out.status.code(),
            Some(86),
            "{}",
            String::from_utf8_lossy(&out.stderr)
        );
        assert!(
            String::from_utf8_lossy(&out.stderr)
                .contains(&format!("globals panic 1001: EXIT:INT:{message}")),
            "{}",
            String::from_utf8_lossy(&out.stderr)
        );
    }
}
