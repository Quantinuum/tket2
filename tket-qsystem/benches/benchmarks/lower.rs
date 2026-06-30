//! Benchmark the QSystem lowering pass on HUGRs with many repeated quantum ops.

use std::hint::black_box;

use criterion::{AxisScale, BenchmarkId, Criterion, PlotConfiguration, criterion_group};
use tket::passes::ComposablePass;
use tket_qsystem::extension::qsystem::{LowerTketToQSystemPass, QSystemPlatform};

use super::generators::make_h_cx_rx_reset_layers;

const NUM_QUBITS: usize = 8;

fn bench_lower_platform(c: &mut Criterion, platform: QSystemPlatform, name: &str) {
    let mut g = c.benchmark_group(format!("lower to {name}"));
    g.plot_config(PlotConfiguration::default().summary_scale(AxisScale::Logarithmic));

    for layers in [10, 100, 1_000] {
        let hugr = make_h_cx_rx_reset_layers(NUM_QUBITS, layers);
        let pass = LowerTketToQSystemPass::new(platform);
        g.bench_with_input(BenchmarkId::new("layers", layers), &layers, |b, _| {
            b.iter_batched(
                || hugr.clone(),
                |mut h| {
                    pass.run(&mut h).unwrap();
                    black_box(h);
                },
                criterion::BatchSize::LargeInput,
            )
        });
    }
    g.finish();
}

fn bench_lower_helios(c: &mut Criterion) {
    bench_lower_platform(c, QSystemPlatform::Helios, "helios");
}

fn bench_lower_sol(c: &mut Criterion) {
    bench_lower_platform(c, QSystemPlatform::Sol, "sol");
}

criterion_group! {
    name = benches;
    config = Criterion::default();
    targets =
        bench_lower_helios,
        bench_lower_sol,
}
