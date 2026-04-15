//! Benchmarks for CFG structuralization strategies.
//!
//! The benchmark intentionally measures the public pass surface only. The two
//! strategies perform different private analysis pipelines, so benchmarking an
//! internal "analysis only" hook would compare unlike-for-like internals rather
//! than the supported structuralization API.

use std::hint::black_box;

use criterion::{BatchSize, BenchmarkId, Criterion, criterion_group};
use tket::control::structuralize::StructuralizationStrategy;
use tket::passes::ComposablePass;
use tket::passes::structuralize_cfgs::StructuralizeCfgsPass;

use super::guppy::load_guppy_example;

const WHOLE_PROGRAM_EXAMPLES: &[&str] = &["loop_and_branch", "complex_control"];

/// Benchmark the end-to-end structuralization rewrite for each strategy.
fn bench_structuralize_pass(c: &mut Criterion) {
    for &example in WHOLE_PROGRAM_EXAMPLES {
        let template = load_guppy_example(example).expect("guppy fixture should load");
        let mut group = c.benchmark_group(format!("structuralize/pass/{example}"));

        for &(name, strategy) in &[
            ("rvsdg", StructuralizationStrategy::Rvsdg),
            ("relooper", StructuralizationStrategy::Relooper),
        ] {
            group.bench_with_input(
                BenchmarkId::new("strategy", name),
                &strategy,
                |b, &strategy| {
                    b.iter_batched(
                        || template.clone(),
                        |mut hugr| {
                            StructuralizeCfgsPass::default()
                                .with_strategy(strategy)
                                .run(&mut hugr)
                                .unwrap();
                            black_box(hugr);
                        },
                        BatchSize::SmallInput,
                    );
                },
            );
        }

        group.finish();
    }
}

criterion_group! {
    name = benches;
    config = Criterion::default();
    targets = bench_structuralize_pass,
}
