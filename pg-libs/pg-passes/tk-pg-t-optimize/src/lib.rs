//! T-count optimization for Clifford+T Pauli graphs.
//!
//! Bit-vector operations are scalar by default. The `unstable_simd` feature enables
//! portable SIMD and requires a nightly Rust toolchain.

#![cfg_attr(feature = "unstable_simd", feature(portable_simd))]

mod clifford_propagation;
mod gadgetization;
mod hadamard_optimizer;
mod optimizer;
mod pass;
mod simd_vector;
mod t_optimizer;

pub use pass::TOptimizationPass;
