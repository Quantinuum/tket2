use std::hint::black_box;

use criterion::{AxisScale, BenchmarkId, Criterion, PlotConfiguration, criterion_group};
use hugr::HugrView;
use tket::circuit::CircuitHash;

use super::generators::make_cnot_layers;

fn bench_hash_simple(c: &mut Criterion) {
    let mut g = c.benchmark_group("hash a simple circuit");
    g.plot_config(PlotConfiguration::default().summary_scale(AxisScale::Logarithmic));

    for size in [10, 100, 1_000] {
        g.bench_with_input(BenchmarkId::new("hash_simple", size), &size, |b, size| {
            let circ = make_cnot_layers(8, *size);
            b.iter(|| black_box(circ.circuit_hash(circ.entrypoint())))
        });
    }
    g.finish();
}

criterion_group! {
    name = benches;
    config = Criterion::default();
    targets =
        bench_hash_simple,
}
