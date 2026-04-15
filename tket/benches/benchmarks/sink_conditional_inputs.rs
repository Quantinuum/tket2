//! Benchmarks for sinking branch-local conditional inputs.

use std::hint::black_box;

use criterion::{BatchSize, BenchmarkId, Criterion, criterion_group};
use tket::passes::{ComposablePass, SinkConditionalInputsPass};

use super::guppy::load_guppy_example;

const CONDITIONAL_EXAMPLES: &[&str] = &[
    "conditional_loop",
    "loop_conditional",
    "shortcircuit",
    "complex_control",
];

/// Benchmark the standalone conditional-input sinking pass on checked-in
/// Guppy examples that contain conditionals.
fn bench_sink_conditional_inputs(c: &mut Criterion) {
    let pass = SinkConditionalInputsPass::default();

    for &example in CONDITIONAL_EXAMPLES {
        let template = load_guppy_example(example).expect("guppy fixture should load");
        let mut group = c.benchmark_group("sink_conditional_inputs");
        group.bench_with_input(
            BenchmarkId::from_parameter(example),
            &template,
            |b, template| {
                b.iter_batched(
                    || template.clone(),
                    |mut hugr| {
                        pass.run(&mut hugr).unwrap();
                        black_box(hugr);
                    },
                    BatchSize::SmallInput,
                );
            },
        );
        group.finish();
    }
}

criterion_group! {
    name = benches;
    config = Criterion::default();
    targets = bench_sink_conditional_inputs,
}
