//! Benchmark the modifier resolver pass.
//!
//! Executing three benchmarks:
//! - modifier_passes[double_modifier.hugr]: Run the passes on `test_files/modifier_examples/double_modifier.hugr`,
//!   which contains a gate under a control and dagger modifier.
//! - modifier_passes[simple_higher_order.hugr]: Run the passes on `test_files/modifier_examples/simple_higher_order.hugr`,
//!   which contains two higher-order calls under a control and dagger modifier.
//! - modifier_passes[conditional_loop.hugr]: Run the passes on `test_files/guppy_examples/conditional_loop.hugr`, which contains no modifiers.
//!   This is to check the overhead of running the pass on a Hugr without modifiers.
//!

use std::hint::black_box;

use criterion::{AxisScale, BatchSize, Criterion, PlotConfiguration, criterion_group};
use hugr::Hugr;
use tket::passes::{ComposablePass, ModifierResolverPass};

fn load_hugr(bytes: &[u8]) -> Hugr {
    Hugr::load(bytes, None).unwrap()
}

fn bench_modifier_resolver(c: &mut Criterion) {
    let mut group = c.benchmark_group("modifier resolver");
    group.plot_config(PlotConfiguration::default().summary_scale(AxisScale::Logarithmic));

    let double_modifier_hugr = load_hugr(include_bytes!(
        "../../../test_files/modifier_examples/double_modifier.hugr"
    ));
    group.bench_function("modifier_passes[double_modifier.hugr]", |b| {
        b.iter_batched(
            || double_modifier_hugr.clone(),
            |mut hugr| {
                ModifierResolverPass::default().run(&mut hugr).unwrap();
                black_box(hugr)
            },
            BatchSize::SmallInput,
        )
    });

    let higher_order_hugr = load_hugr(include_bytes!(
        "../../../test_files/modifier_examples/simple_higher_order.hugr"
    ));
    group.bench_function("modifier_passes[simple_higher_order.hugr]", |b| {
        b.iter_batched(
            || higher_order_hugr.clone(),
            |mut hugr| {
                ModifierResolverPass::default().run(&mut hugr).unwrap();
                black_box(hugr)
            },
            BatchSize::SmallInput,
        )
    });

    let guppy_no_modifier_hugr = load_hugr(include_bytes!(
        "../../../test_files/guppy_examples/conditional_loop.hugr"
    ));
    group.bench_function("modifier_passes[conditional_loop.hugr]", |b| {
        b.iter_batched(
            || guppy_no_modifier_hugr.clone(),
            |mut hugr| {
                ModifierResolverPass::default().run(&mut hugr).unwrap();
                black_box(hugr)
            },
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

criterion_group! {
    name = benches;
    config = Criterion::default();
    targets =
        bench_modifier_resolver,
}
