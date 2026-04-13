//! Benchmarks for CFG structuralization strategies.
//!
//! These benchmarks compare the RVSDG and Beyond-Relooper strategies on the
//! same semi-complex Guppy-generated CFG fixture.

use std::hint::black_box;

use criterion::{BatchSize, BenchmarkId, Criterion, criterion_group};
use tket::control::structuralize::{StructuralizationStrategy, analyze_hugr_cfgs};
use tket::passes::ComposablePass;
use tket::passes::structuralize_cfgs::StructuralizeCfgsPass;

use super::guppy::load_guppy_example;

/// Benchmark the pure structural analysis cost for each strategy.
fn bench_structural_analysis(c: &mut Criterion) {
    let template =
        load_guppy_example("complex_control").expect("complex_control fixture should load");
    let mut group = c.benchmark_group("structuralize/analyze/complex_control");

    for &(name, strategy) in &[
        ("rvsdg", StructuralizationStrategy::Rvsdg),
        ("relooper", StructuralizationStrategy::BeyondRelooper),
    ] {
        group.bench_with_input(
            BenchmarkId::new("strategy", name),
            &strategy,
            |b, &strategy| {
                b.iter(|| black_box(analyze_hugr_cfgs(black_box(&template), strategy).unwrap()));
            },
        );
    }

    group.finish();
}

/// Benchmark the end-to-end structuralization rewrite for each strategy.
///
/// DFG inlining is disabled so the benchmark focuses on the strategy-specific
/// analysis and rewrite work rather than the shared cleanup pass.
fn bench_structuralize_pass(c: &mut Criterion) {
    let template =
        load_guppy_example("complex_control").expect("complex_control fixture should load");
    let mut group = c.benchmark_group("structuralize/pass/complex_control");

    for &(name, strategy) in &[
        ("rvsdg", StructuralizationStrategy::Rvsdg),
        ("relooper", StructuralizationStrategy::BeyondRelooper),
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
                            .inline_dfgs(false)
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

criterion_group! {
    name = benches;
    config = Criterion::default();
    targets =
        bench_structural_analysis,
        bench_structuralize_pass,
}
