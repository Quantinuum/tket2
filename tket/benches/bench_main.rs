//! Benchmarks for the tket crate.

mod benchmarks;

use criterion::criterion_main;

criterion_main! {
    benchmarks::constant_fold::benches,
    benchmarks::hash::benches,
    benchmarks::modifier_resolver::benches,
    benchmarks::normalize::benches,
}
