//! Benchmarks for the tket crate.

mod benchmarks;

use criterion::criterion_main;

criterion_main! {
    benchmarks::hash::benches,
    benchmarks::sink_conditional_inputs::benches,
    benchmarks::structuralize::benches,
}
