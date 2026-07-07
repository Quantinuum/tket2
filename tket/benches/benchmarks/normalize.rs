//! Benchmark the NormalizeGuppy pass on a small Guppy-generated HUGR.

use std::hint::black_box;

use criterion::{AxisScale, BatchSize, Criterion, PlotConfiguration, criterion_group};
use hugr::Hugr;
use tket::passes::{ComposablePass, NormalizeGuppy};

fn load_hugr(bytes: &[u8]) -> Hugr {
    Hugr::load(bytes, None).unwrap()
}

fn bench_normalize(c: &mut Criterion) {
    let mut group = c.benchmark_group("normalize");
    group.plot_config(PlotConfiguration::default().summary_scale(AxisScale::Logarithmic));

    // Statically include the small Guppy-generated HUGR examples for benchmarking.
    let examples = [
        (
            "normalize[fn_calls.hugr]",
            include_bytes!("../../../test_files/guppy_examples/fn_calls.hugr").as_slice(),
        ),
        (
            "normalize[repeat_until_success.hugr]",
            include_bytes!("../../../test_files/guppy_examples/repeat_until_success.hugr")
                .as_slice(),
        ),
        (
            "normalize[t_factory.hugr]",
            include_bytes!("../../../test_files/guppy_examples/t_factory.hugr").as_slice(),
        ),
    ];

    for (name, bytes) in examples {
        let hugr = load_hugr(bytes);
        group.bench_function(name, |b| {
            b.iter_batched(
                || hugr.clone(),
                |mut hugr| {
                    NormalizeGuppy::default().run(&mut hugr).unwrap();
                    black_box(hugr)
                },
                BatchSize::SmallInput,
            )
        });
    }

    group.finish();
}

criterion_group! {
    name = benches;
    config = Criterion::default();
    targets =
        bench_normalize,
}
