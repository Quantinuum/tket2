use super::*;
use crate::hugr;
use hugr::builder::{Dataflow, DataflowHugr, DataflowSubContainer, HugrBuilder};
use hugr::extension::prelude::{UnwrapBuilder, bool_t, option_type};
use hugr::llvm::emit::{EmitDebugInfo, Namer, test::SimpleHugrConfig};
use hugr::llvm::inkwell::{self, context::Context, module::Module};
use hugr::llvm::utils::{IntOpBuilder, LogicOpBuilder};
use hugr::std_extensions::{
    arithmetic::int_types::{ConstInt, int_type},
    ptr::{self, PtrOpBuilder},
};
use hugr::types::Signature;
use std::{
    alloc::{Layout, alloc, dealloc},
    cell::RefCell,
    collections::HashMap,
    rc::Rc,
};

fn emit<'c>(ctx: &'c Context, platform: QSystemPlatform, mut graph: Hugr) -> Module<'c> {
    crate::process_hugr(platform, &mut graph).unwrap();
    let (module, _) = crate::get_hugr_llvm_module(
        ctx,
        Rc::new(Namer::new("", false)),
        &graph,
        "ptr",
        Rc::new(codegen_extensions(platform)),
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

fn lifecycle() -> Hugr {
    let ty = int_type(6);
    SimpleHugrConfig::new()
        .with_extensions(REGISTRY.to_owned())
        .with_outs([bool_t()])
        .finish(|mut b| {
            let one = b.add_load_value(ConstInt::new_u(6, 1).unwrap());
            let two = b.add_load_value(ConstInt::new_u(6, 2).unwrap());
            let three = b.add_load_value(ConstInt::new_u(6, 3).unwrap());
            let p = b.add_new_ptr(one).unwrap();
            let (a, other) = b.add_dup_ptr(p, ty.clone()).unwrap();
            let (a, other, same) = b.add_eq_ptr(a, other, ty.clone()).unwrap();
            let a = b.add_write_ptr(a, two).unwrap();
            let (a, read) = b.add_read_ptr(a, ty.clone()).unwrap();
            let (a, old) = b.add_swap_ptr(a, three).unwrap();
            let none = b.add_free_ptr(a, ty.clone()).unwrap();
            b.build_unwrap_sum::<0>(0, option_type([ty.clone()]), none)
                .unwrap();
            let some = b.add_free_ptr(other, ty.clone()).unwrap();
            b.set_order(&none.node(), &some.node());
            let [last] = b.build_unwrap_sum(1, option_type([ty]), some).unwrap();
            let read_ok = b.add_ieq(6, read, two).unwrap();
            let old_ok = b.add_ieq(6, old, two).unwrap();
            let last_ok = b.add_ieq(6, last, three).unwrap();
            let ok = b.add_and(read_ok, old_ok).unwrap();
            let ok = b.add_and(ok, last_ok).unwrap();
            let ok = b.add_and(ok, same).unwrap();
            let left = b.add_new_ptr(two).unwrap();
            let right = b.add_new_ptr(two).unwrap();
            let (left, right, equal) = b.add_eq_ptr(left, right, int_type(6)).unwrap();
            let different = b.add_not(equal).unwrap();
            let left = b.add_free_ptr(left, int_type(6)).unwrap();
            let right = b.add_free_ptr(right, int_type(6)).unwrap();
            let [lv] = b
                .build_unwrap_sum(1, option_type([int_type(6)]), left)
                .unwrap();
            let [rv] = b
                .build_unwrap_sum(1, option_type([int_type(6)]), right)
                .unwrap();
            let same_payload = b.add_ieq(6, lv, rv).unwrap();
            let ok = b.add_and(ok, different).unwrap();
            let ok = b.add_and(ok, same_payload).unwrap();
            b.finish_hugr_with_outputs([ok]).unwrap()
        })
}

#[derive(Default)]
struct Heap {
    live: HashMap<usize, Layout>,
    allocations: usize,
    frees: usize,
    fail: bool,
}
thread_local! { static HEAP: RefCell<Heap> = RefCell::default(); }
extern "C" fn heap_alloc(size: u64) -> *mut u8 {
    HEAP.with_borrow_mut(|heap| {
        if heap.fail {
            return std::ptr::null_mut();
        }
        let layout = Layout::from_size_align(usize::try_from(size).unwrap().max(1), 16).unwrap();
        let p = unsafe { alloc(layout) };
        assert!(!p.is_null());
        assert!(heap.live.insert(p as usize, layout).is_none());
        heap.allocations += 1;
        p
    })
}
unsafe extern "C" fn heap_free(p: *mut u8) {
    HEAP.with_borrow_mut(|heap| {
        let layout = heap
            .live
            .remove(&(p as usize))
            .expect("exactly one final free");
        unsafe { dealloc(p, layout) };
        heap.frees += 1;
    });
}
unsafe extern "C" fn runtime_panic(code: u32, message: *const u8) {
    // QIS strings have a byte length followed by the tagged message, without NUL.
    let bytes = unsafe { std::slice::from_raw_parts(message.add(1), usize::from(*message)) };
    eprintln!("pointer panic {code}: {}", String::from_utf8_lossy(bytes));
    std::process::exit(86);
}

fn execute(module: &Module<'_>) -> bool {
    let engine = module
        .create_jit_execution_engine(inkwell::OptimizationLevel::None)
        .unwrap();
    for (name, address) in [
        ("heap_alloc", heap_alloc as *const () as usize),
        ("heap_free", heap_free as *const () as usize),
        ("panic", runtime_panic as *const () as usize),
    ] {
        if let Some(function) = module.get_function(name) {
            engine.add_global_mapping(&function, address);
        }
    }
    unsafe {
        engine
            .get_function::<unsafe extern "C" fn() -> bool>("main")
            .unwrap()
            .call()
    }
}

#[test]
fn pointer_lifecycle_uses_configured_heap_and_panic() {
    assert!(has_compatible_extension(&ptr::EXTENSION_ID, &ptr::VERSION.to_string()).unwrap());
    for platform in [QSystemPlatform::Sol, QSystemPlatform::Helios] {
        HEAP.with_borrow_mut(|h| *h = Heap::default());
        let ctx = Context::create();
        let module = emit(&ctx, platform, lifecycle());
        for symbol in ["heap_alloc", "heap_free", "panic"] {
            assert!(module.get_function(symbol).is_some(), "missing {symbol}");
        }
        for symbol in [
            "malloc",
            "free",
            "___ptr_create",
            "___ptr_inc_refcount",
            "___ptr_lock",
            "___ptr_unlock",
        ] {
            assert!(module.get_function(symbol).is_none(), "unexpected {symbol}");
        }
        let ir = module.print_to_string().to_string();
        assert!(
            ir.contains("i32 1002"),
            "pointer errors use the normal QIS signal offset"
        );
        assert!(!ir.contains("atomic"));
        assert!(execute(&module));
        HEAP.with_borrow(|h| {
            assert!(h.live.is_empty());
            assert_eq!((h.allocations, h.frees), (3, 3));
        });
    }
}

#[test]
fn pointer_equality_preserves_linear_handles_without_heap_or_panic() {
    let ty = ptr::ptr_type(int_type(6));
    let pointer = ptr::ptr_type(ty.clone());
    let graph = SimpleHugrConfig::new()
        .with_extensions(REGISTRY.to_owned())
        .with_ins([pointer.clone(), pointer.clone()])
        .with_outs([pointer.clone(), pointer, bool_t()])
        .finish(|mut b| {
            let [a, bp] = b.input_wires_arr();
            let (a, bp, equal) = b.add_eq_ptr(a, bp, ty).unwrap();
            b.finish_hugr_with_outputs([a, bp, equal]).unwrap()
        });
    for platform in [QSystemPlatform::Sol, QSystemPlatform::Helios] {
        let ctx = Context::create();
        let module = emit(&ctx, platform, graph.clone());
        let ir = module.print_to_string().to_string();
        assert!(ir.contains("icmp eq ptr"));
        for forbidden in ["getelementptr", "atomic", "@heap_", "@panic", "@___ptr_"] {
            assert!(!ir.contains(forbidden));
        }
    }
}

fn mapped(reenter: bool) -> Hugr {
    let ty = int_type(6);
    let ptr_ty = ptr::ptr_type(ty.clone());
    SimpleHugrConfig::new()
        .with_extensions(REGISTRY.to_owned())
        .with_outs([bool_t()])
        .finish(|mut b| {
            let mut mb = b.module_root_builder();
            let mut cb = mb
                .define_function("update", Signature::new_endo([ty.clone(), ptr_ty.clone()]))
                .unwrap();
            let [value, alias] = cb.input_wires_arr();
            let alias = if reenter {
                cb.add_read_ptr(alias, ty.clone()).unwrap().0
            } else {
                alias
            };
            let one = cb.add_load_value(ConstInt::new_u(6, 1).unwrap());
            let value = cb.add_iadd(6, value, one).unwrap();
            let cb = cb.finish_with_outputs([value, alias]).unwrap();
            let f = b.load_func(cb.handle(), &[]).unwrap();
            let initial = b.add_load_value(ConstInt::new_u(6, 41).unwrap());
            let p = b.add_new_ptr(initial).unwrap();
            let (p, alias) = b.add_dup_ptr(p, ty.clone()).unwrap();
            let (p, extras) = b.add_map_ptr(p, f, ty.clone(), [alias], [ptr_ty]).unwrap();
            let a = b.add_free_ptr(p, ty.clone()).unwrap();
            let last = b.add_free_ptr(extras[0], ty.clone()).unwrap();
            b.set_order(&a.node(), &last.node());
            let [value] = b.build_unwrap_sum(1, option_type([ty]), last).unwrap();
            let expected = b.add_load_value(ConstInt::new_u(6, 42).unwrap());
            let ok = b.add_ieq(6, value, expected).unwrap();
            b.finish_hugr_with_outputs([ok]).unwrap()
        })
}

#[test]
fn pointer_map_releases_checked_lock() {
    HEAP.with_borrow_mut(|h| *h = Heap::default());
    let ctx = Context::create();
    let module = emit(&ctx, QSystemPlatform::Helios, mapped(false));
    assert!(execute(&module));
    HEAP.with_borrow(|h| {
        assert!(h.live.is_empty());
        assert_eq!((h.allocations, h.frees), (1, 1));
    });
}

#[test]
fn pointer_errors_use_runtime_panic() {
    const ENV: &str = "TKET_DEFAULT_POINTER_PANIC_CASE";
    if let Ok(case) = std::env::var(ENV) {
        HEAP.with_borrow_mut(|h| {
            *h = Heap::default();
            h.fail = case == "allocation";
        });
        let ctx = Context::create();
        let module = emit(
            &ctx,
            QSystemPlatform::Helios,
            if case == "allocation" {
                lifecycle()
            } else {
                mapped(true)
            },
        );
        execute(&module);
        panic!("expected non-returning runtime panic");
    }
    for (case, message) in [
        ("allocation", "Pointer allocation failed"),
        ("reentrant", "Pointer cell is already locked"),
    ] {
        let out = std::process::Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "extensions::ptr_tests::pointer_errors_use_runtime_panic",
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
                .contains(&format!("pointer panic 1002: EXIT:INT:{message}")),
            "{}",
            String::from_utf8_lossy(&out.stderr)
        );
    }
}

#[test]
fn pointer_final_release_recovers_linear_payload() {
    let ty = int_type(6);
    let inner_ty = ptr::ptr_type(ty.clone());
    let graph = SimpleHugrConfig::new()
        .with_extensions(REGISTRY.to_owned())
        .with_outs([bool_t()])
        .finish(|mut b| {
            let value = b.add_load_value(ConstInt::new_u(6, 42).unwrap());
            let inner = b.add_new_ptr(value).unwrap();
            let outer = b.add_new_ptr(inner).unwrap();
            let outer = b.add_free_ptr(outer, inner_ty.clone()).unwrap();
            let [inner] = b
                .build_unwrap_sum(1, option_type([inner_ty]), outer)
                .unwrap();
            let inner = b.add_free_ptr(inner, ty.clone()).unwrap();
            let [restored] = b.build_unwrap_sum(1, option_type([ty]), inner).unwrap();
            let ok = b.add_ieq(6, value, restored).unwrap();
            b.finish_hugr_with_outputs([ok]).unwrap()
        });
    HEAP.with_borrow_mut(|h| *h = Heap::default());
    let ctx = Context::create();
    let module = emit(&ctx, QSystemPlatform::Helios, graph);
    assert!(execute(&module));
    HEAP.with_borrow(|h| {
        assert!(h.live.is_empty());
        assert_eq!((h.allocations, h.frees), (2, 2));
    });
}

#[test]
fn pointer_equality_returns_original_handles_in_order() {
    let ty = int_type(6);
    let graph = SimpleHugrConfig::new()
        .with_extensions(REGISTRY.to_owned())
        .with_outs([bool_t()])
        .finish(|mut b| {
            let seven = b.add_load_value(ConstInt::new_u(6, 7).unwrap());
            let nine = b.add_load_value(ConstInt::new_u(6, 9).unwrap());
            let left = b.add_new_ptr(seven).unwrap();
            let right = b.add_new_ptr(nine).unwrap();
            let (left, right, equal) = b.add_eq_ptr(left, right, ty.clone()).unwrap();
            let different = b.add_not(equal).unwrap();
            let left = b.add_free_ptr(left, ty.clone()).unwrap();
            let right = b.add_free_ptr(right, ty.clone()).unwrap();
            let [left] = b
                .build_unwrap_sum(1, option_type([ty.clone()]), left)
                .unwrap();
            let [right] = b.build_unwrap_sum(1, option_type([ty]), right).unwrap();
            let left_ok = b.add_ieq(6, left, seven).unwrap();
            let right_ok = b.add_ieq(6, right, nine).unwrap();
            let ok = b.add_and(left_ok, right_ok).unwrap();
            let ok = b.add_and(ok, different).unwrap();
            b.finish_hugr_with_outputs([ok]).unwrap()
        });
    HEAP.with_borrow_mut(|h| *h = Heap::default());
    let ctx = Context::create();
    let module = emit(&ctx, QSystemPlatform::Helios, graph);
    assert!(execute(&module));
    HEAP.with_borrow(|h| {
        assert!(h.live.is_empty());
        assert_eq!((h.allocations, h.frees), (2, 2));
    });
}
