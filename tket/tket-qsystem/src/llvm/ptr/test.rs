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

mod runtime;
use runtime::{EVENTS, Event};

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
#[case(lifecycle(), 1, 4)]
#[case(nested_linear_payload(), 2, 0)]
#[case::eq_aliases(equality(true, 7), 1, 2)]
#[case::eq_distinct_same_payload(equality(false, 7), 2, 2)]
#[case::eq_distinct_different_payload(equality(false, 19), 2, 2)]
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
        ("___ptr_create", runtime::create as *const () as usize),
        ("___ptr_inc_refcount", runtime::adjust as *const () as usize),
        ("___ptr_get_ptr", runtime::data as *const () as usize),
        ("___ptr_lock", runtime::lock as *const () as usize),
        ("___ptr_unlock", runtime::unlock as *const () as usize),
    ] {
        if let Some(function) = emission.module().get_function(name) {
            engine.add_global_mapping(&function, address);
        }
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
        let creates: Vec<_> = events
            .iter()
            .filter_map(|event| match event {
                Event::Create { size, alignment } => Some((*size, *alignment)),
                _ => None,
            })
            .collect();
        // All these payloads are i64 or a pointer, never a {count, value} pair.
        assert_eq!(creates, vec![(8, 8); allocations]);
        assert_eq!(
            events.iter().filter(|e| **e == Event::Destroy).count(),
            allocations
        );
        assert!(matches!(events.first(), Some(Event::Create { .. })));
        assert_eq!(events.last(), Some(&Event::Destroy));
        assert_eq!(events.iter().filter(|e| **e == Event::Lock).count(), locks);
        assert_eq!(
            events.iter().filter(|e| **e == Event::Unlock).count(),
            locks
        );
        assert_eq!(
            events.iter().filter(|e| **e == Event::Project).count(),
            locks
        );
        let adjustments: Vec<_> = events
            .iter()
            .filter_map(|event| match event {
                Event::Adjust { delta, final_owner } => Some((*delta, *final_owner)),
                _ => None,
            })
            .collect();
        assert_eq!(
            adjustments.iter().map(|(delta, _)| delta).sum::<i64>(),
            -(allocations as i64)
        );
        assert_eq!(
            adjustments.iter().filter(|(_, last)| *last).count(),
            allocations
        );
    });
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
        ("___ptr_get_ptr", runtime::data as *const () as usize),
        ("___ptr_lock", runtime::lock as *const () as usize),
        ("___ptr_unlock", runtime::unlock as *const () as usize),
    ] {
        if let Some(function) = emission.module().get_function(name) {
            engine.add_global_mapping(&function, address);
        }
    }
    let address = engine.get_function_address("main").unwrap();
    let initial = 0u64;
    let cell = unsafe { runtime::create(8, 8, std::ptr::from_ref(&initial).cast()) };
    for _ in 0..3 {
        assert!(!unsafe { runtime::adjust(cell, 1, std::ptr::null_mut()) });
    }
    let ptr = cell as usize;
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
    // The final extraction returns the value once, without an extra lock or
    // caller-visible payload counter. Nonfinal releases leave output untouched.
    let mut output = u64::MAX;
    for _ in 0..3 {
        assert!(!unsafe { runtime::adjust(cell, -1, std::ptr::from_mut(&mut output).cast()) });
        assert_eq!(output, u64::MAX);
    }
    assert!(unsafe { runtime::adjust(cell, -1, std::ptr::from_mut(&mut output).cast()) });
    assert_eq!(output, 8000);
}
