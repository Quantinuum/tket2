//! T-count optimization for Clifford+T Pauli graphs.

mod clifford_propagation;
mod gadgetization;
mod hadamard_optimizer;
mod optimizer;
mod pass;
mod simd_vector;
mod t_optimizer;

pub use pass::TOptimizationPass;
