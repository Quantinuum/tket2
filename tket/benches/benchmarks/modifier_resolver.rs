//! Benchmark the modifier resolver pass.

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
    group.bench_function("double_modifier_resolve", |b| {
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
        "../../../test_files/modifier_examples/higher_order_function_w_arrays.hugr"
    ));
    group.bench_function("higher_order_function_w_arrays_resolve", |b| {
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
    group.bench_function("conditional_loop_no_modifiers", |b| {
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
