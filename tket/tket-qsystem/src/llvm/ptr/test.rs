use super::*;
use hugr::llvm::{
    emit::{
        EmitDebugInfo,
        test::{Emission, SimpleHugrConfig},
    },
    test::{TestContext, exec_ctx},
    utils::{IntOpBuilder, LogicOpBuilder, fat::FatExt},
};
use hugr::{
    Hugr,
    builder::{Dataflow, DataflowHugr, DataflowSubContainer, HugrBuilder},
    extension::prelude::{UnwrapBuilder, bool_t, option_type},
    std_extensions::{
        STD_REG,
        arithmetic::int_types::{ConstInt, int_type},
        ptr::{self, PtrOpBuilder},
    },
};
use rstest::rstest;

use hugr::llvm::inkwell;
use hugr::types::Signature;
fn configure(ctx: &mut TestContext) {
    ctx.add_extensions(|b| {
        b.add_default_prelude_extensions()
            .add_default_int_extensions()
            .add_logic_extensions()
            .add_ptr_extensions(QisPtrCodegen)
    });
}

fn lifecycle() -> Hugr {
    let ty = int_type(6);
    SimpleHugrConfig::new()
        .with_extensions(STD_REG.to_owned())
        .with_outs([bool_t()])
        .finish(|mut b| {
            let one = b.add_load_value(ConstInt::new_u(6, 1).unwrap());
            let two = b.add_load_value(ConstInt::new_u(6, 2).unwrap());
            let three = b.add_load_value(ConstInt::new_u(6, 3).unwrap());
            let ptr = b.add_new_ptr(one).unwrap();
            let (a, other) = b.add_dup_ptr(ptr, ty.clone()).unwrap();
            let a = b.add_write_ptr(a, two).unwrap();
            let (a, read) = b.add_read_ptr(a, ty.clone()).unwrap();
            let (a, old) = b.add_swap_ptr(a, three).unwrap();
            let mut mb = b.module_root_builder();
            let mut cb = mb
                .define_function("increment", Signature::new_endo([ty.clone()]))
                .unwrap();
            let [v] = cb.input_wires_arr();
            let one = cb.add_load_value(ConstInt::new_u(6, 1).unwrap());
            let v = cb.add_iadd(6, v, one).unwrap();
            let cb = cb.finish_with_outputs([v]).unwrap();
            let f = b.load_func(cb.handle(), &[]).unwrap();
            let (a, _) = b.add_map_ptr(a, f, ty.clone(), [], []).unwrap();
            let none = b.add_free_ptr(a, ty.clone()).unwrap();
            b.build_unwrap_sum::<0>(0, option_type([ty.clone()]), none)
                .unwrap();
            let some = b.add_free_ptr(other, ty.clone()).unwrap();
            // The two handles alone do not order their Free operations.
            b.set_order(&none.node(), &some.node());
            let [last] = b.build_unwrap_sum(1, option_type([ty]), some).unwrap();
            let read_ok = b.add_ieq(6, read, two).unwrap();
            let old_ok = b.add_ieq(6, old, two).unwrap();
            let four = b.add_load_value(ConstInt::new_u(6, 4).unwrap());
            let last_ok = b.add_ieq(6, last, four).unwrap();
            let ok = b.add_and(read_ok, old_ok).unwrap();
            let ok = b.add_and(ok, last_ok).unwrap();
            b.finish_hugr_with_outputs([ok]).unwrap()
        })
}

fn emit<'c>(ctx: &'c TestContext, hugr: &'c Hugr) -> Emission<'c> {
    let emission = Emission::emit_hugr(
        hugr.fat_root().unwrap(),
        ctx.get_emit_hugr(),
        EmitDebugInfo::Exclude,
    )
    .unwrap();
    emission.verify().unwrap();
    emission
}

thread_local! {
    static EVENTS: std::cell::RefCell<Vec<&'static str>> = const { std::cell::RefCell::new(Vec::new()) };
    static ALLOCATIONS: std::cell::RefCell<std::collections::BTreeMap<usize, (std::alloc::Layout, usize)>> = const { std::cell::RefCell::new(std::collections::BTreeMap::new()) };
}
fn event(name: &'static str) {
    EVENTS.with_borrow_mut(|events| events.push(name));
}
extern "C" fn hook_alloc(size: u64, align: u64) -> *mut u8 {
    let payload = std::alloc::Layout::from_size_align(size as usize, align as usize).unwrap();
    let (layout, offset) = std::alloc::Layout::new::<u64>().extend(payload).unwrap();
    // LLVM supplies a nonzero sized cell and its power-of-two ABI alignment.
    let ptr = unsafe { std::alloc::alloc(layout) };
    assert!(!ptr.is_null());
    ALLOCATIONS.with_borrow_mut(|allocs| allocs.insert(ptr as usize, (layout, offset)));
    event("alloc");
    hook_init(ptr.cast());
    ptr
}
extern "C" fn hook_free(ptr: *mut u8) {
    hook_destroy(ptr.cast());
    let (layout, _) = ALLOCATIONS
        .with_borrow_mut(|allocs| allocs.remove(&(ptr as usize)))
        .unwrap();
    // The last Free returns the value and destroys the mutex before freeing.
    unsafe {
        std::alloc::dealloc(ptr, layout);
    }
    event("free");
}
extern "C" fn hook_get_ptr(ptr: *mut u8) -> *mut u8 {
    let offset = ALLOCATIONS.with_borrow(|allocs| allocs[&(ptr as usize)].1);
    // Payload storage is a separate, aligned region after the runtime mutex.
    assert!(offset > 0);
    EVENTS.with_borrow(|events| assert!(matches!(events.last(), Some(&"lock" | &"init"))));
    event("get_ptr");
    unsafe { ptr.add(offset) }
}
extern "C" fn hook_init(ptr: *mut u64) {
    unsafe {
        ptr.write(0);
    }
    event("init");
}
extern "C" fn hook_lock(ptr: *mut u64) {
    assert_eq!(unsafe { ptr.replace(1) }, 0);
    event("lock");
}
extern "C" fn hook_unlock(ptr: *mut u64) {
    assert_eq!(unsafe { ptr.replace(0) }, 1);
    event("unlock");
}
extern "C" fn hook_destroy(ptr: *mut u64) {
    assert_eq!(unsafe { ptr.read() }, 0);
    event("destroy");
}

fn nested_linear_payload() -> Hugr {
    let ty = int_type(6);
    SimpleHugrConfig::new()
        .with_extensions(STD_REG.to_owned())
        .with_outs([bool_t()])
        .finish(|mut b| {
            let expected = b.add_load_value(ConstInt::new_u(6, 42).unwrap());
            let inner = b.add_new_ptr(expected).unwrap();
            let outer = b.add_new_ptr(inner).unwrap();
            let inner_ty = ptr::ptr_type(ty.clone());
            let freed = b.add_free_ptr(outer, inner_ty.clone()).unwrap();
            let [inner] = b
                .build_unwrap_sum(1, option_type([inner_ty]), freed)
                .unwrap();
            let freed = b.add_free_ptr(inner, ty.clone()).unwrap();
            let [value] = b.build_unwrap_sum(1, option_type([ty]), freed).unwrap();
            let ok = b.add_ieq(6, value, expected).unwrap();
            b.finish_hugr_with_outputs([ok]).unwrap()
        })
}

fn equality(aliases: bool, rhs_value: u64) -> Hugr {
    let ty = int_type(6);
    SimpleHugrConfig::new()
        .with_extensions(STD_REG.to_owned())
        .with_outs([bool_t()])
        .finish(|mut b| {
            let seven = b.add_load_value(ConstInt::new_u(6, 7).unwrap());
            let rhs_value = b.add_load_value(ConstInt::new_u(6, rhs_value).unwrap());
            let lhs = b.add_new_ptr(seven).unwrap();
            let (lhs, rhs) = if aliases {
                b.add_dup_ptr(lhs, ty.clone()).unwrap()
            } else {
                (lhs, b.add_new_ptr(rhs_value).unwrap())
            };
            let (lhs, rhs, equal) = b.add_eq_ptr(lhs, rhs, ty.clone()).unwrap();
            let identity_ok = if aliases {
                equal
            } else {
                b.add_not(equal).unwrap()
            };
            // Read both returned handles: different values expose swapped outputs.
            let (lhs, left_value) = b.add_read_ptr(lhs, ty.clone()).unwrap();
            let (rhs, right_value) = b.add_read_ptr(rhs, ty.clone()).unwrap();
            let left_ok = b.add_ieq(6, left_value, seven).unwrap();
            let right_ok = b.add_ieq(6, right_value, rhs_value).unwrap();
            let first = b.add_free_ptr(lhs, ty.clone()).unwrap();
            let last = b.add_free_ptr(rhs, ty.clone()).unwrap();
            b.set_order(&first.node(), &last.node());
            // Eq must not change the reference count or release either handle.
            if aliases {
                b.build_unwrap_sum::<0>(0, option_type([ty.clone()]), first)
                    .unwrap();
            } else {
                let [_] = b
                    .build_unwrap_sum(1, option_type([ty.clone()]), first)
                    .unwrap();
            }
            let [_] = b.build_unwrap_sum(1, option_type([ty]), last).unwrap();
            let ok = b.add_and(identity_ok, left_ok).unwrap();
            let ok = b.add_and(ok, right_ok).unwrap();
            b.finish_hugr_with_outputs([ok]).unwrap()
        })
}

#[rstest]
#[case(lifecycle(), 1, 7)]
#[case(nested_linear_payload(), 2, 2)]
#[case::eq_aliases(equality(true, 7), 1, 5)]
#[case::eq_distinct_same_payload(equality(false, 7), 2, 4)]
#[case::eq_distinct_different_payload(equality(false, 19), 2, 4)]
fn custom_hooks_manage_cell_once(
    mut exec_ctx: TestContext,
    #[case] hugr: Hugr,
    #[case] allocations: usize,
    #[case] locks: usize,
) {
    configure(&mut exec_ctx);
    let emission = emit(&exec_ctx, &hugr);
    emission
        .module()
        .get_function("main")
        .unwrap()
        .set_linkage(inkwell::module::Linkage::External);
    let engine = emission
        .module()
        .create_jit_execution_engine(inkwell::OptimizationLevel::None)
        .unwrap();
    for (name, address) in [
        ("___ptr_alloc", hook_alloc as *const () as usize),
        ("___ptr_free", hook_free as *const () as usize),
        ("___ptr_get_ptr", hook_get_ptr as *const () as usize),
        ("___ptr_lock", hook_lock as *const () as usize),
        ("___ptr_unlock", hook_unlock as *const () as usize),
    ] {
        engine.add_global_mapping(&emission.module().get_function(name).unwrap(), address);
    }
    EVENTS.with_borrow_mut(Vec::clear);
    // This test's entry has no arguments and returns the LLVM boolean type.
    let main = unsafe {
        engine
            .get_function::<unsafe extern "C" fn() -> bool>("main")
            .unwrap()
    };
    assert!(unsafe { main.call() });
    EVENTS.with_borrow(|events| {
        assert_eq!(
            events.iter().filter(|e| **e == "alloc").count(),
            allocations
        );
        assert_eq!(events.iter().filter(|e| **e == "free").count(), allocations);
        assert_eq!(&events[..3], &["alloc", "init", "get_ptr"]);
        assert_eq!(&events[events.len() - 2..], &["destroy", "free"]);
        assert_eq!(events.iter().filter(|e| **e == "lock").count(), locks);
        assert_eq!(events.iter().filter(|e| **e == "unlock").count(), locks);
    });
    ALLOCATIONS.with_borrow(|allocs| assert!(allocs.is_empty()));
}

use std::{
    cell::UnsafeCell,
    sync::atomic::{AtomicU8, Ordering},
};

// A test runtime whose opaque handle and payload have different addresses.
#[repr(C)]
struct Cell {
    mutex: AtomicU8,
    count: u64,
    value: UnsafeCell<u64>,
}
extern "C" fn concurrent_lock(ptr: *const Cell) {
    // The test retains storage until all JIT calls join.
    let lock = unsafe { &(*ptr).mutex };
    while lock
        .compare_exchange_weak(0, 1, Ordering::Acquire, Ordering::Relaxed)
        .is_err()
    {
        std::hint::spin_loop();
    }
}
extern "C" fn concurrent_unlock(ptr: *const Cell) {
    unsafe { &(*ptr).mutex }.store(0, Ordering::Release);
}
extern "C" fn concurrent_get_ptr(ptr: *const Cell) -> *const u64 {
    // Payload access occurs only while the caller holds the mutex.
    unsafe { std::ptr::addr_of!((*ptr).count) }
}
#[rstest]
fn custom_mutex_makes_concurrent_map_exclusive(mut exec_ctx: TestContext) {
    configure(&mut exec_ctx);
    let ty = int_type(6);
    let ptr_ty = ptr::ptr_type(ty.clone());
    let hugr = SimpleHugrConfig::new()
        .with_extensions(STD_REG.to_owned())
        .with_ins([ptr_ty.clone()])
        .with_outs([ptr_ty])
        .finish(|mut b| {
            let mut mb = b.module_root_builder();
            let mut callback = mb
                .define_function("increment", Signature::new_endo([ty.clone()]))
                .unwrap();
            let [value] = callback.input_wires_arr();
            let one = callback.add_load_value(ConstInt::new_u(6, 1).unwrap());
            let value = callback.add_iadd(6, value, one).unwrap();
            let callback = callback.finish_with_outputs([value]).unwrap();
            let function = b.load_func(callback.handle(), &[]).unwrap();
            let [ptr] = b.input_wires_arr();
            let (ptr, _) = b.add_map_ptr(ptr, function, ty, [], []).unwrap();
            b.finish_hugr_with_outputs([ptr]).unwrap()
        });
    let emission = emit(&exec_ctx, &hugr);
    let function = emission.module().get_function("main").unwrap();
    function.set_linkage(inkwell::module::Linkage::External);
    let engine = emission
        .module()
        .create_jit_execution_engine(inkwell::OptimizationLevel::Aggressive)
        .unwrap();
    for (name, address) in [
        ("___ptr_get_ptr", concurrent_get_ptr as *const () as usize),
        ("___ptr_lock", concurrent_lock as *const () as usize),
        ("___ptr_unlock", concurrent_unlock as *const () as usize),
    ] {
        engine.add_global_mapping(&emission.module().get_function(name).unwrap(), address);
    }
    let address = engine.get_function_address("main").unwrap();
    // Mirror the test spin-mutex cell layout. Each worker owns one of four handles;
    // storage is owned by this test and stays alive until every worker joins.
    let cell = Cell {
        count: 4,
        mutex: AtomicU8::new(0),
        value: UnsafeCell::new(0),
    };
    let ptr = (&cell as *const Cell) as usize;
    let start = std::sync::Barrier::new(4);
    std::thread::scope(|scope| {
        for _ in 0..4 {
            let start = &start;
            scope.spawn(move || {
                // The emitted function takes and returns an opaque cell pointer.
                // Its mutex protects the only accesses to value while threads run.
                let map: unsafe extern "C" fn(usize) -> usize =
                    unsafe { std::mem::transmute(address) };
                start.wait();
                for _ in 0..2000 {
                    assert_eq!(unsafe { map(ptr) }, ptr);
                }
            });
        }
    });
    // All workers have joined, so no concurrent access to the value remains.
    assert_eq!(unsafe { *cell.value.get() }, 8000);
    assert_eq!(cell.count, 4);
}
