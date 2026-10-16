//! Integration tests through the production QIS extension map and fresh LLVM artifacts.
use crate::array::SeleneHeapListCodegen;
use hugr::builder::{Dataflow, DataflowHugr};
use hugr::extension::prelude::{ConstUsize, qb_t, usize_t};
use hugr::llvm::CodegenExtsBuilder;
use hugr::llvm::emit::test::{Emission, PanicTestPreludeCodegen, SimpleHugrConfig};
use hugr::llvm::emit::{EmitDebugInfo, EmitHugr, Namer};
use hugr::llvm::inkwell::{
    AddressSpace, OptimizationLevel,
    context::Context,
    module::{Linkage, Module},
};
use hugr::llvm::utils::fat::FatExt as _;
use hugr::std_extensions::collections::list::ListOp;
use hugr::{Hugr, HugrView, Wire, types::SumType};
use rstest::rstest;
use std::rc::Rc;
use tket::hugr;
use tket_qsystem::QSystemPlatform;

fn emit<'c>(context: &'c Context, hugr: &Hugr, platform: QSystemPlatform) -> Module<'c> {
    hugr.validate().unwrap();
    let emit = EmitHugr::new(
        context,
        context.create_module("list_integration"),
        Rc::new(Namer::new("", false)),
        Rc::new(crate::extensions::codegen_extensions(platform)),
    );
    let (module, _) = emit
        .emit_module(hugr.fat_root().unwrap(), EmitDebugInfo::Exclude)
        .unwrap()
        .finish();
    module.verify().unwrap();
    module
}

fn assert_selected_hooks(module: &Module) {
    for name in ["heap_alloc", "heap_free", "panic"] {
        assert!(
            module.get_function(name).is_some(),
            "Missing selected hook {name}"
        );
    }
    for name in ["malloc", "realloc", "free", "abort"] {
        assert!(
            module.get_function(name).is_none(),
            "Unexpected default hook {name}"
        );
    }
    for name in [
        "__rt__list__new",
        "__rt__list__push",
        "__rt__list__discard_empty",
    ] {
        assert!(module.get_function(name).unwrap().count_basic_blocks() > 0);
    }
    let ir = module.print_to_string().to_string();
    assert!(ir.contains("Cannot discard a nonempty list"));
    assert!(
        ir.contains("i32 1001"),
        "QIS panic must shift the runtime signal"
    );
}

fn save_artifacts(module: &Module, name: &str) {
    if let Some(dir) = std::env::var_os("LIST_TEST_ARTIFACT_DIR") {
        let dir = std::path::PathBuf::from(dir);
        std::fs::create_dir_all(&dir).unwrap();
        module
            .print_to_file(dir.join(format!("{name}.ll")))
            .unwrap();
        assert!(module.write_bitcode_to_path(dir.join(format!("{name}.bc"))));
    }
}

// Test heap definitions retain the production ABI while using host libc for JIT execution.
// They do not establish downstream Selene runtime acceptance.
fn install_test_heap(module: &Module, fail: bool) {
    unsafe extern "C" {
        fn malloc(size: usize) -> *mut std::ffi::c_void;
        fn free(ptr: *mut std::ffi::c_void);
        fn memcpy(
            dst: *mut std::ffi::c_void,
            src: *const std::ffi::c_void,
            n: usize,
        ) -> *mut std::ffi::c_void;
        fn memmove(
            dst: *mut std::ffi::c_void,
            src: *const std::ffi::c_void,
            n: usize,
        ) -> *mut std::ffi::c_void;
    }
    let context = module.get_context();
    let builder = context.create_builder();
    let alloc = module.get_function("heap_alloc").unwrap();
    builder.position_at_end(context.append_basic_block(alloc, "test_heap"));
    let ptr = if fail {
        context
            .ptr_type(AddressSpace::default())
            .const_null()
            .into()
    } else {
        // Bind the host function address directly: this static LLVM build does
        // not automatically resolve libc declarations in the JIT.
        let malloc_ptr = context
            .i64_type()
            .const_int(malloc as *const () as u64, false)
            .const_to_pointer(context.ptr_type(AddressSpace::default()));
        builder
            .build_indirect_call(
                alloc.get_type(),
                malloc_ptr,
                &[alloc.get_first_param().unwrap().into()],
                "allocated",
            )
            .unwrap()
            .try_as_basic_value()
            .unwrap_basic()
    };
    builder.build_return(Some(&ptr)).unwrap();
    let heap_free = module.get_function("heap_free").unwrap();
    builder.position_at_end(context.append_basic_block(heap_free, "test_heap"));
    let free_ptr = context
        .i64_type()
        .const_int(free as *const () as u64, false)
        .const_to_pointer(context.ptr_type(AddressSpace::default()));
    builder
        .build_indirect_call(
            heap_free.get_type(),
            free_ptr,
            &[heap_free.get_first_param().unwrap().into()],
            "",
        )
        .unwrap();
    builder.build_return(None).unwrap();
    // LLVM may lower the runtime's memory intrinsics to libc calls. Define
    // their test bindings too, so execution is independent of symbol discovery.
    let ptr_ty = context.ptr_type(AddressSpace::default());
    let copy_ty = ptr_ty.fn_type(
        &[ptr_ty.into(), ptr_ty.into(), context.i64_type().into()],
        false,
    );
    for (name, address) in [
        ("memcpy", memcpy as *const () as u64),
        ("memmove", memmove as *const () as u64),
    ] {
        let function = module.add_function(name, copy_ty, None);
        builder.position_at_end(context.append_basic_block(function, "host_copy"));
        let pointer = context
            .i64_type()
            .const_int(address, false)
            .const_to_pointer(ptr_ty);
        let args = function
            .get_param_iter()
            .map(Into::into)
            .collect::<Vec<_>>();
        let result = builder
            .build_indirect_call(copy_ty, pointer, &args, "copied")
            .unwrap()
            .try_as_basic_value()
            .unwrap_basic();
        builder.build_return(Some(&result)).unwrap();
    }
    module.verify().unwrap();
}

// Convert a result sum into an observable integer. Empty rows return 0;
// populated rows return their element. This pins both tags and payloads.
fn sum_value(builder: &mut hugr::llvm::emit::test::DFGW, value: Wire, sum: SumType) -> Wire {
    use hugr::builder::{DataflowSubContainer, SubContainer};
    let zero = builder.add_load_value(ConstUsize::new(0));
    let rows: Vec<hugr::types::TypeRow> = sum
        .variants()
        .map(|row| row.clone().try_into().unwrap())
        .collect();
    let mut cond = builder
        .conditional_builder((rows, value), [], [usize_t()].into())
        .unwrap();
    for tag in 0..sum.num_variants() {
        let case = cond.case_builder(tag).unwrap();
        let mut input = case.input_wires();
        case.finish_with_outputs([input.next().unwrap_or(zero)])
            .unwrap();
    }
    cond.finish_sub_container().unwrap().out_wire(0)
}

#[rstest]
#[case::get_first_helios(QSystemPlatform::Helios, ListOp::get, 0, 2, false, 10)]
#[case::get_first_sol(QSystemPlatform::Sol, ListOp::get, 0, 2, false, 10)]
#[case::get_last_helios(QSystemPlatform::Helios, ListOp::get, 1, 2, false, 20)]
#[case::get_last_sol(QSystemPlatform::Sol, ListOp::get, 1, 2, false, 20)]
#[case::get_end_helios(QSystemPlatform::Helios, ListOp::get, 2, 2, false, 0)]
#[case::get_end_sol(QSystemPlatform::Sol, ListOp::get, 2, 2, false, 0)]
#[case::get_huge_helios(QSystemPlatform::Helios, ListOp::get, u64::MAX, 2, false, 0)]
#[case::get_huge_sol(QSystemPlatform::Sol, ListOp::get, u64::MAX, 2, false, 0)]
#[case::get_empty_helios(QSystemPlatform::Helios, ListOp::get, 0, 0, false, 0)]
#[case::get_empty_sol(QSystemPlatform::Sol, ListOp::get, 0, 0, false, 0)]
#[case::get_keeps_list_helios(QSystemPlatform::Helios, ListOp::get, 0, 2, true, 2)]
#[case::get_keeps_list_sol(QSystemPlatform::Sol, ListOp::get, 0, 2, true, 2)]
#[case::pop_helios(QSystemPlatform::Helios, ListOp::pop, 0, 2, false, 20)]
#[case::pop_sol(QSystemPlatform::Sol, ListOp::pop, 0, 2, false, 20)]
#[case::pop_empty_helios(QSystemPlatform::Helios, ListOp::pop, 0, 0, false, 0)]
#[case::pop_empty_sol(QSystemPlatform::Sol, ListOp::pop, 0, 0, false, 0)]
#[case::pop_keeps_list_helios(QSystemPlatform::Helios, ListOp::pop, 0, 2, true, 1)]
#[case::pop_keeps_list_sol(QSystemPlatform::Sol, ListOp::pop, 0, 2, true, 1)]
#[case::set_old_helios(QSystemPlatform::Helios, ListOp::set, 1, 2, false, 20)]
#[case::set_old_sol(QSystemPlatform::Sol, ListOp::set, 1, 2, false, 20)]
#[case::set_invalid_helios(QSystemPlatform::Helios, ListOp::set, 2, 2, false, 99)]
#[case::set_invalid_sol(QSystemPlatform::Sol, ListOp::set, 2, 2, false, 99)]
#[case::set_huge_helios(QSystemPlatform::Helios, ListOp::set, u64::MAX, 2, false, 99)]
#[case::set_huge_sol(QSystemPlatform::Sol, ListOp::set, u64::MAX, 2, false, 99)]
#[case::set_new_helios(QSystemPlatform::Helios, ListOp::set, 1, 2, true, 99)]
#[case::set_new_sol(QSystemPlatform::Sol, ListOp::set, 1, 2, true, 99)]
#[case::set_failed_unchanged_helios(QSystemPlatform::Helios, ListOp::set, 2, 2, true, 10)]
#[case::set_failed_unchanged_sol(QSystemPlatform::Sol, ListOp::set, 2, 2, true, 10)]
#[case::insert_front_helios(QSystemPlatform::Helios, ListOp::insert, 0, 2, true, 99)]
#[case::insert_front_sol(QSystemPlatform::Sol, ListOp::insert, 0, 2, true, 99)]
#[case::insert_middle_helios(QSystemPlatform::Helios, ListOp::insert, 1, 2, true, 99)]
#[case::insert_middle_sol(QSystemPlatform::Sol, ListOp::insert, 1, 2, true, 99)]
#[case::insert_end_helios(QSystemPlatform::Helios, ListOp::insert, 2, 2, true, 99)]
#[case::insert_end_sol(QSystemPlatform::Sol, ListOp::insert, 2, 2, true, 99)]
#[case::insert_empty_helios(QSystemPlatform::Helios, ListOp::insert, 0, 0, true, 99)]
#[case::insert_empty_sol(QSystemPlatform::Sol, ListOp::insert, 0, 0, true, 99)]
#[case::insert_success_helios(QSystemPlatform::Helios, ListOp::insert, 2, 2, false, 0)]
#[case::insert_success_sol(QSystemPlatform::Sol, ListOp::insert, 2, 2, false, 0)]
#[case::insert_invalid_helios(QSystemPlatform::Helios, ListOp::insert, 3, 2, false, 99)]
#[case::insert_invalid_sol(QSystemPlatform::Sol, ListOp::insert, 3, 2, false, 99)]
#[case::insert_huge_helios(QSystemPlatform::Helios, ListOp::insert, u64::MAX, 2, false, 99)]
#[case::insert_huge_sol(QSystemPlatform::Sol, ListOp::insert, u64::MAX, 2, false, 99)]
#[case::insert_failed_unchanged_helios(QSystemPlatform::Helios, ListOp::insert, 3, 2, true, 10)]
#[case::insert_failed_unchanged_sol(QSystemPlatform::Sol, ListOp::insert, 3, 2, true, 10)]
#[case::growth_helios(QSystemPlatform::Helios, ListOp::length, 0, 100, false, 100)]
#[case::growth_sol(QSystemPlatform::Sol, ListOp::length, 0, 100, false, 100)]
#[case::length_empty_helios(QSystemPlatform::Helios, ListOp::length, 0, 0, false, 0)]
#[case::length_empty_sol(QSystemPlatform::Sol, ListOp::length, 0, 0, false, 0)]
fn exec_list(
    #[case] platform: QSystemPlatform,
    #[case] op: ListOp,
    #[case] index: u64,
    #[case] initial_len: u64,
    #[case] observe_list: bool,
    #[case] expected: u64,
) {
    use hugr::extension::prelude::{either_type, option_type};
    let hugr = SimpleHugrConfig::new()
        .with_outs([usize_t()])
        .finish(|mut b| {
            let mut list = b
                .add_dataflow_op(
                    ListOp::new.with_type(usize_t()).to_extension_op().unwrap(),
                    [],
                )
                .unwrap()
                .out_wire(0);
            for i in 1..=initial_len {
                let elem = b.add_load_value(ConstUsize::new(i * 10));
                list = b
                    .add_dataflow_op(
                        ListOp::push.with_type(usize_t()).to_extension_op().unwrap(),
                        [list, elem],
                    )
                    .unwrap()
                    .out_wire(0);
            }
            let idx = b.add_load_value(ConstUsize::new(index));
            let elem = b.add_load_value(ConstUsize::new(99));
            let inputs = match op {
                ListOp::get => vec![list, idx],
                ListOp::set | ListOp::insert => vec![list, idx, elem],
                _ => vec![list],
            };
            let result = b
                .add_dataflow_op(op.with_type(usize_t()).to_extension_op().unwrap(), inputs)
                .unwrap();
            list = result.out_wire(0);
            let value = result.out_wire(1);
            let result_value = match op {
                ListOp::get | ListOp::pop => sum_value(&mut b, value, option_type([usize_t()])),
                ListOp::set => sum_value(&mut b, value, either_type([usize_t()], [usize_t()])),
                ListOp::insert => sum_value(&mut b, value, either_type([usize_t()], [])),
                _ => value,
            };
            let out = if observe_list {
                if matches!(op, ListOp::get | ListOp::pop) {
                    let result = b
                        .add_dataflow_op(
                            ListOp::length
                                .with_type(usize_t())
                                .to_extension_op()
                                .unwrap(),
                            [list],
                        )
                        .unwrap();
                    list = result.out_wire(0);
                    result.out_wire(1)
                } else {
                    let read_index =
                        if index < initial_len || (op == ListOp::insert && index == initial_len) {
                            index
                        } else {
                            0
                        };
                    let idx = b.add_load_value(ConstUsize::new(read_index));
                    let result = b
                        .add_dataflow_op(
                            ListOp::get.with_type(usize_t()).to_extension_op().unwrap(),
                            [list, idx],
                        )
                        .unwrap();
                    list = result.out_wire(0);
                    sum_value(&mut b, result.out_wire(1), option_type([usize_t()]))
                }
            } else {
                result_value
            };
            for _ in 0..=initial_len {
                list = b
                    .add_dataflow_op(
                        ListOp::pop.with_type(usize_t()).to_extension_op().unwrap(),
                        [list],
                    )
                    .unwrap()
                    .out_wire(0);
            }
            b.add_dataflow_op(
                ListOp::discard_empty
                    .with_type(usize_t())
                    .to_extension_op()
                    .unwrap(),
                [list],
            )
            .unwrap();
            b.finish_hugr_with_outputs([out]).unwrap()
        });
    let context = Context::create();
    let module = emit(&context, &hugr, platform);
    assert_selected_hooks(&module);
    save_artifacts(
        &module,
        &format!("{platform:?}-{op:?}-{index}-{initial_len}-{observe_list}"),
    );
    // Execute a fresh bitcode round trip, rather than reusing the emitted module.
    let bitcode = module.write_bitcode_to_memory();
    let module = Module::parse_bitcode_from_buffer(&bitcode, &context).unwrap();
    install_test_heap(&module, false);
    module
        .get_function("main")
        .unwrap()
        .set_linkage(Linkage::External);
    let ee = module
        .create_jit_execution_engine(OptimizationLevel::None)
        .unwrap();
    // SAFETY: SimpleHugrConfig creates main with no inputs and one usize (i64) output.
    let result = unsafe {
        ee.get_function::<unsafe extern "C" fn() -> u64>("main")
            .unwrap()
            .call()
    };
    assert_eq!(result, expected);
}

#[rstest]
#[case::helios(QSystemPlatform::Helios)]
#[case::sol(QSystemPlatform::Sol)]
fn linear_after_draining(#[case] platform: QSystemPlatform) {
    use hugr::extension::prelude::{UnwrapBuilder, option_type};
    let hugr = SimpleHugrConfig::new()
        .with_ins([qb_t()])
        .with_outs([qb_t()])
        .finish(|mut b| {
            let qubit = b.input_wires().next().unwrap();
            let list = b
                .add_dataflow_op(ListOp::new.with_type(qb_t()).to_extension_op().unwrap(), [])
                .unwrap()
                .out_wire(0);
            let list = b
                .add_dataflow_op(
                    ListOp::push.with_type(qb_t()).to_extension_op().unwrap(),
                    [list, qubit],
                )
                .unwrap()
                .out_wire(0);
            let pop = b
                .add_dataflow_op(
                    ListOp::pop.with_type(qb_t()).to_extension_op().unwrap(),
                    [list],
                )
                .unwrap();
            b.add_dataflow_op(
                ListOp::discard_empty
                    .with_type(qb_t())
                    .to_extension_op()
                    .unwrap(),
                [pop.out_wire(0)],
            )
            .unwrap();
            let [value] = b
                .build_unwrap_sum(1, option_type([qb_t()]), pop.out_wire(1))
                .unwrap();
            b.finish_hugr_with_outputs([value]).unwrap()
        });

    let context = Context::create();
    let module = emit(&context, &hugr, platform);
    assert_selected_hooks(&module);
    save_artifacts(&module, &format!("{platform:?}-linear-drained"));
    install_test_heap(&module, false);
    module
        .get_function("main")
        .unwrap()
        .set_linkage(Linkage::External);
    let ee = module
        .create_jit_execution_engine(OptimizationLevel::None)
        .unwrap();
    // SAFETY: QIS qubits are i64 tokens; main takes and returns one such token.
    unsafe {
        assert_eq!(
            ee.get_function::<unsafe extern "C" fn(u64) -> u64>("main")
                .unwrap()
                .call(0xbeef),
            0xbeef
        );
    }
}

fn discard_graph(nonempty: bool, linear: bool) -> Hugr {
    let ty = if linear { qb_t() } else { usize_t() };
    SimpleHugrConfig::new()
        .with_outs([usize_t()])
        .finish(|mut b| {
            let mut list = b
                .add_dataflow_op(
                    ListOp::new.with_type(ty.clone()).to_extension_op().unwrap(),
                    [],
                )
                .unwrap()
                .out_wire(0);
            if nonempty {
                assert!(!linear);
                let elem = b.add_load_value(ConstUsize::new(42));
                list = b
                    .add_dataflow_op(
                        ListOp::push
                            .with_type(ty.clone())
                            .to_extension_op()
                            .unwrap(),
                        [list, elem],
                    )
                    .unwrap()
                    .out_wire(0);
            }
            b.add_dataflow_op(
                ListOp::discard_empty
                    .with_type(ty)
                    .to_extension_op()
                    .unwrap(),
                [list],
            )
            .unwrap();
            let out = b.add_load_value(ConstUsize::new(42));
            b.finish_hugr_with_outputs([out]).unwrap()
        })
}

#[rstest]
#[case::helios(QSystemPlatform::Helios)]
#[case::sol(QSystemPlatform::Sol)]
fn discard_empty_linear(#[case] platform: QSystemPlatform) {
    let context = Context::create();
    let graph = discard_graph(false, true);
    let module = emit(&context, &graph, platform);
    assert_selected_hooks(&module);
    save_artifacts(&module, &format!("{platform:?}-linear-empty"));
    install_test_heap(&module, false);
    module
        .get_function("main")
        .unwrap()
        .set_linkage(Linkage::External);
    let ee = module
        .create_jit_execution_engine(OptimizationLevel::None)
        .unwrap();
    // SAFETY: main has no parameters and returns a single i64.
    assert_eq!(
        unsafe {
            ee.get_function::<unsafe extern "C" fn() -> u64>("main")
                .unwrap()
                .call()
        },
        42
    );
}

#[rstest]
#[case::nonempty(true, false, "Cannot discard a nonempty list")]
#[case::allocation(false, true, "List allocation failed")]
fn selected_panic(#[case] nonempty: bool, #[case] fail_alloc: bool, #[case] expected: &str) {
    let context = Context::create();
    let graph = discard_graph(nonempty, false);
    graph.validate().unwrap();
    // The adapter must honor any selected prelude, including this catchable test prelude.
    let exts = CodegenExtsBuilder::default()
        .add_prelude_extensions(PanicTestPreludeCodegen)
        .add_list_extensions(SeleneHeapListCodegen(PanicTestPreludeCodegen))
        .finish();
    let emit = EmitHugr::new(
        &context,
        context.create_module("list_panic"),
        Rc::new(Namer::new("", false)),
        Rc::new(exts),
    );
    let (module, _) = emit
        .emit_module(graph.fat_root().unwrap(), EmitDebugInfo::Exclude)
        .unwrap()
        .finish();
    install_test_heap(&module, fail_alloc);
    let emission = Emission::new(module);
    assert_eq!(emission.exec_panicking("main").unwrap(), expected);
}

#[rstest]
#[case::new(ListOp::new)]
#[case::push(ListOp::push)]
#[case::pop(ListOp::pop)]
#[case::get(ListOp::get)]
#[case::set(ListOp::set)]
#[case::insert(ListOp::insert)]
#[case::length(ListOp::length)]
#[case::discard(ListOp::discard_empty)]
fn all_operations_emit(#[case] op: ListOp) {
    for platform in [QSystemPlatform::Helios, QSystemPlatform::Sol] {
        // get requires copyability; all other operations accept a linear element.
        let ty = if op == ListOp::get { usize_t() } else { qb_t() };
        let graph =
            hugr::llvm::test::single_op_hugr(op.with_type(ty).to_extension_op().unwrap().into());
        let context = Context::create();
        let module = emit(&context, &graph, platform);
        assert_selected_hooks(&module);
        save_artifacts(&module, &format!("{platform:?}-emit-{op:?}"));
    }
}

#[rstest]
#[case::helios(QSystemPlatform::Helios)]
#[case::sol(QSystemPlatform::Sol)]
fn constant_lists_compile(#[case] platform: QSystemPlatform) {
    use hugr::std_extensions::collections::list::ListValue;
    let graph = SimpleHugrConfig::new()
        .with_outs([usize_t()])
        .finish(|mut b| {
            let mut list = b.add_load_value(ListValue::new(
                usize_t(),
                [ConstUsize::new(10).into(), ConstUsize::new(20).into()],
            ));
            let length = b
                .add_dataflow_op(
                    ListOp::length
                        .with_type(usize_t())
                        .to_extension_op()
                        .unwrap(),
                    [list],
                )
                .unwrap();
            list = length.out_wire(0);
            for _ in 0..2 {
                list = b
                    .add_dataflow_op(
                        ListOp::pop.with_type(usize_t()).to_extension_op().unwrap(),
                        [list],
                    )
                    .unwrap()
                    .out_wire(0);
            }
            b.add_dataflow_op(
                ListOp::discard_empty
                    .with_type(usize_t())
                    .to_extension_op()
                    .unwrap(),
                [list],
            )
            .unwrap();
            b.finish_hugr_with_outputs([length.out_wire(1)]).unwrap()
        });
    let context = Context::create();
    let target = crate::get_native_target_machine(OptimizationLevel::None).unwrap();
    for level in [OptimizationLevel::None, OptimizationLevel::Aggressive] {
        let mut graph = graph.clone();
        graph.validate().unwrap();
        let args =
            crate::CompileArgs::new(&"list_constant", &target, level, platform, &context, false);
        let module = crate::compile(&args, &context, &mut graph).unwrap();
        module.verify().unwrap();
        save_artifacts(&module, &format!("{platform:?}-constant-{level:?}"));
    }
    let module = emit(&context, &graph, platform);
    assert_selected_hooks(&module);
    let bitcode = module.write_bitcode_to_memory();
    let module = Module::parse_bitcode_from_buffer(&bitcode, &context).unwrap();
    install_test_heap(&module, false);
    module
        .get_function("main")
        .unwrap()
        .set_linkage(Linkage::External);
    let ee = module
        .create_jit_execution_engine(OptimizationLevel::None)
        .unwrap();
    // SAFETY: the emitted main has no inputs and one i64 output.
    assert_eq!(
        unsafe {
            ee.get_function::<unsafe extern "C" fn() -> u64>("main")
                .unwrap()
                .call()
        },
        2
    );
}
