//! Benchmarks for CFG structuralization strategies.
//!
//! These benchmarks compare the RVSDG and Beyond-Relooper strategies on the
//! same semi-complex Guppy-generated CFG fixture.

use std::hint::black_box;

use criterion::{BatchSize, BenchmarkId, Criterion, criterion_group};
use hugr::hugr::hugrmut::HugrMut;
use hugr::{Hugr, HugrView, Node};
use tket::control::structuralize::{StructuralizationStrategy, analyze_hugr_cfgs};
use tket::passes::ComposablePass;
use tket::passes::PassScope;
use tket::passes::composable::WithScope;
use tket::passes::structuralize_cfgs::StructuralizeCfgsPass;

use super::guppy::load_guppy_example;

const WHOLE_PROGRAM_EXAMPLES: &[&str] = &["complex_control"];

fn function_with_name(h: &Hugr, name: &str) -> Node {
    h.children(h.module_root())
        .find(|node| {
            h.get_optype(*node)
                .as_func_defn()
                .is_some_and(|defn| defn.func_name() == name)
        })
        .unwrap_or_else(|| panic!("missing function {name}"))
}

/// Returns helper CFGs that both strategies can structuralize successfully
/// when run as standalone entrypoints.
///
/// This keeps the comparison focused on shared supported coverage while
/// preserving the function boundaries emitted by Guppy.
fn supported_helper_functions(h: &Hugr, root_name: &str) -> Vec<String> {
    h.children(h.module_root())
        .filter_map(|node| {
            let defn = h.get_optype(node).as_func_defn()?;
            if defn.func_name() == root_name
                || !h.children(node).any(|child| h.get_optype(child).is_cfg())
            {
                return None;
            }

            let name = defn.func_name().to_string();
            let supported = [
                StructuralizationStrategy::Rvsdg,
                StructuralizationStrategy::BeyondRelooper,
            ]
            .into_iter()
            .all(|strategy| {
                let mut candidate = h.clone();
                let func = function_with_name(&candidate, &name);
                StructuralizeCfgsPass::default()
                    .with_strategy(strategy)
                    .with_scope(PassScope::EntrypointRecursive)
                    .run(&mut candidate.with_entrypoint_mut(func))
                    .is_ok()
            });

            supported.then_some(name)
        })
        .collect()
}

/// Benchmark the pure structural analysis cost for each strategy.
fn bench_structural_analysis(c: &mut Criterion) {
    for &example in WHOLE_PROGRAM_EXAMPLES {
        let template = load_guppy_example(example).expect("guppy fixture should load");
        let mut group = c.benchmark_group(format!("structuralize/analyze/{example}"));

        for &(name, strategy) in &[
            ("rvsdg", StructuralizationStrategy::Rvsdg),
            ("relooper", StructuralizationStrategy::BeyondRelooper),
        ] {
            group.bench_with_input(
                BenchmarkId::new("strategy", name),
                &strategy,
                |b, &strategy| {
                    b.iter(|| {
                        black_box(analyze_hugr_cfgs(black_box(&template), strategy).unwrap())
                    });
                },
            );
        }

        group.finish();
    }

    let template = load_guppy_example("control_pipeline").expect("guppy fixture should load");
    let helper_names = supported_helper_functions(&template, "control_pipeline");
    let mut group = c.benchmark_group("structuralize/analyze/control_pipeline_helpers");

    for &(name, strategy) in &[
        ("rvsdg", StructuralizationStrategy::Rvsdg),
        ("relooper", StructuralizationStrategy::BeyondRelooper),
    ] {
        group.bench_with_input(
            BenchmarkId::new("strategy", name),
            &strategy,
            |b, &strategy| {
                b.iter(|| {
                    for helper_name in &helper_names {
                        let func = function_with_name(&template, helper_name);
                        black_box(
                            analyze_hugr_cfgs(&template.with_entrypoint(func), strategy).unwrap(),
                        );
                    }
                });
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
    for &example in WHOLE_PROGRAM_EXAMPLES {
        let template = load_guppy_example(example).expect("guppy fixture should load");
        let mut group = c.benchmark_group(format!("structuralize/pass/{example}"));

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

    let template = load_guppy_example("control_pipeline").expect("guppy fixture should load");
    let helper_names = supported_helper_functions(&template, "control_pipeline");
    let mut group = c.benchmark_group("structuralize/pass/control_pipeline_helpers");

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
                        for helper_name in &helper_names {
                            let func = function_with_name(&hugr, helper_name);
                            StructuralizeCfgsPass::default()
                                .with_strategy(strategy)
                                .with_scope(PassScope::EntrypointRecursive)
                                .run(&mut hugr.with_entrypoint_mut(func))
                                .unwrap();
                        }
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
