//! Benchmarks for pathological constant-folding graph shapes.

use std::hint::black_box;

use criterion::{BatchSize, BenchmarkId, Criterion, criterion_group};
use hugr::builder::{Dataflow, DataflowSubContainer, HugrBuilder, ModuleBuilder};
use hugr::core::Visibility;
use hugr::extension::prelude::MakeTuple;
use hugr::hugr::hugrmut::HugrMut;
use hugr::ops::Value;
use hugr::ops::handle::NodeHandle;
use hugr::std_extensions::arithmetic::int_ops::IntOpDef;
use hugr::std_extensions::arithmetic::int_types::{ConstInt, INT_TYPES};
use hugr::types::{Signature, Type};
use hugr::{Hugr, type_row};
use tket::passes::composable::{PassScope, WithScope};
use tket::passes::{ComposablePass, ConstantFoldPass};

/// Build a module whose entrypoint is tiny but which contains many unrelated functions.
fn module_with_unrelated_functions(function_count: usize) -> Hugr {
    let mut module = ModuleBuilder::new();
    let mut entrypoint = None;

    for index in 0..function_count {
        let mut function = module
            .define_function_vis(
                format!("function_{index}"),
                Signature::new(type_row![], [INT_TYPES[5].clone()]),
                Visibility::Private,
            )
            .unwrap();
        let lhs = function.add_load_value(Value::from(ConstInt::new_u(5, 7).unwrap()));
        let rhs = function.add_load_value(Value::from(ConstInt::new_u(5, 11).unwrap()));
        let sum = function
            .add_dataflow_op(IntOpDef::iadd.with_log_width(5), [lhs, rhs])
            .unwrap();
        let function = function.finish_with_outputs(sum.outputs()).unwrap();
        entrypoint.get_or_insert(function.node());
    }

    let mut hugr = module.finish_hugr().unwrap();
    hugr.set_entrypoint(entrypoint.unwrap());
    hugr
}

/// Build one constant `MakeTuple` with many inputs.
fn wide_constant_tuple(width: usize) -> Hugr {
    let element_type = INT_TYPES[5].clone();
    let tuple_row = vec![element_type; width];
    let mut module = ModuleBuilder::new();
    let mut function = module
        .define_function_vis(
            "wide_tuple",
            Signature::new(type_row![], [Type::new_tuple(tuple_row.clone())]),
            Visibility::Public,
        )
        .unwrap();
    let constants = (0..width)
        .map(|value| {
            function.add_load_value(Value::from(
                ConstInt::new_u(5, u64::try_from(value).unwrap()).unwrap(),
            ))
        })
        .collect::<Vec<_>>();
    let tuple = function
        .add_dataflow_op(MakeTuple::new(tuple_row.into()), constants)
        .unwrap();
    function.finish_with_outputs(tuple.outputs()).unwrap();
    module.finish_hugr().unwrap()
}

fn bench_unrelated_functions(c: &mut Criterion) {
    let mut group = c.benchmark_group("constant_fold/unrelated_functions");
    for function_count in [1, 10, 100, 1_000] {
        let hugr = module_with_unrelated_functions(function_count);
        group.bench_with_input(
            BenchmarkId::from_parameter(function_count),
            &hugr,
            |b, hugr| {
                b.iter_batched(
                    || hugr.clone(),
                    |mut hugr| {
                        ConstantFoldPass::default_with_scope(PassScope::EntrypointRecursive)
                            .run(&mut hugr)
                            .unwrap();
                        black_box(hugr)
                    },
                    BatchSize::LargeInput,
                );
            },
        );
    }
    group.finish();
}

fn bench_wide_nodes(c: &mut Criterion) {
    let mut group = c.benchmark_group("constant_fold/wide_tuple");
    for width in [8, 32, 128, 512] {
        let hugr = wide_constant_tuple(width);
        group.bench_with_input(BenchmarkId::from_parameter(width), &hugr, |b, hugr| {
            b.iter_batched(
                || hugr.clone(),
                |mut hugr| {
                    ConstantFoldPass::default().run(&mut hugr).unwrap();
                    black_box(hugr)
                },
                BatchSize::LargeInput,
            );
        });
    }
    group.finish();
}

criterion_group! {
    name = benches;
    config = Criterion::default();
    targets = bench_unrelated_functions, bench_wide_nodes,
}
