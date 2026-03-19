//! Benchmark hashing hugrs.
//!
//! TODO: This is already being done in the Hugr repository, so we could remove it from here.
//! We don't delete it yet to keep an example for writing benchmarks. Remove it once other benchmarks are written.

use std::hint::black_box;

use criterion::{AxisScale, BenchmarkId, Criterion, PlotConfiguration, criterion_group};
use hugr::HugrView;
use hugr_passes::hash::HugrHash;

use super::generators::make_cnot_layers;

fn bench_hash_simple(c: &mut Criterion) {
    let mut g = c.benchmark_group("hash a simple circuit");
    g.plot_config(PlotConfiguration::default().summary_scale(AxisScale::Logarithmic));

    for size in [10, 100, 1_000] {
        g.bench_with_input(BenchmarkId::new("hash_simple", size), &size, |b, size| {
            let hugr = make_cnot_layers(8, *size);
            b.iter(|| black_box(hugr.region_hash(hugr.entrypoint())))
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
