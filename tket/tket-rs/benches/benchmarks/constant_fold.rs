//! Benchmark constant folding on chains of conditionals.

use std::hint::black_box;

use criterion::{BatchSize, BenchmarkId, Criterion, criterion_group};
use hugr::{
    Hugr,
    builder::{DFGBuilder, Dataflow, DataflowHugr, DataflowSubContainer, SubContainer},
    extension::prelude::bool_t,
    ops::Value,
    type_row,
    types::Signature,
};
use tket::passes::{ComposablePass, const_fold::ConstantFoldPass};

/// Each conditional returns its input, forcing the analysis to propagate through the chain.
fn conditional_chain(length: usize) -> Hugr {
    let mut builder = DFGBuilder::new(Signature::new_endo(vec![bool_t()])).unwrap();
    let [mut value] = builder.input_wires_arr();
    for _ in 0..length {
        let mut conditional = builder
            .conditional_builder(
                (vec![type_row![], type_row![]], value),
                [],
                vec![bool_t()].into(),
            )
            .unwrap();
        for branch in 0..2 {
            let mut case = conditional.case_builder(branch).unwrap();
            let output = case.add_load_value(if branch == 0 {
                Value::false_val()
            } else {
                Value::true_val()
            });
            case.finish_with_outputs([output]).unwrap();
        }
        let operation = conditional.finish_sub_container().unwrap();
        [value] = operation.outputs_arr();
    }
    builder.finish_hugr_with_outputs([value]).unwrap()
}

fn bench_conditional_chain(c: &mut Criterion) {
    let mut group = c.benchmark_group("constant_fold/conditional_chain");
    group.sample_size(10);
    for length in [1_000, 2_000, 5_000] {
        let hugr = conditional_chain(length);
        group.bench_with_input(BenchmarkId::from_parameter(length), &hugr, |b, hugr| {
            b.iter_batched(
                || hugr.clone(),
                |mut hugr| {
                    ConstantFoldPass::default().run(&mut hugr).unwrap();
                    black_box(hugr)
                },
                BatchSize::PerIteration,
            )
        });
    }
    group.finish();
}

criterion_group!(benches, bench_conditional_chain);
