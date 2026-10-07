//! Benchmarks for the tket-qsystem crate.

mod benchmarks;

use criterion::criterion_main;

criterion_main! {
    benchmarks::lower::benches,
}
