//! T-count optimization for Clifford+T Pauli graphs.
//!
//! Bit-vector operations are scalar by default. The `unstable_simd` feature enables
//! portable SIMD and requires a nightly Rust toolchain.

#![cfg_attr(feature = "unstable_simd", feature(portable_simd))]

mod clifford_propagation;
mod hadamard_optimizer;
mod optimizer;
mod simd_vector;
mod t_optimizer;

use tk_pg_core::{PGPass, PauliGraph};

use crate::optimizer::optimize;

/// A T-count optimization pass for rotations, tableaux, and unconditional H gates.
#[derive(Default)]
pub struct TOptimizationPass {
    ancilla_budget: usize,
    first_bit: usize,
}

impl TOptimizationPass {
    /// Create a pass that uses no ancillas.
    pub fn new() -> Self {
        Self::default()
    }

    /// Reserve the last `budget` input qubits as idle ancillas prepared in zero.
    ///
    /// Using ancillas may introduce measurements, resets, and conditional corrections.
    pub fn with_ancilla_budget(mut self, budget: usize) -> Self {
        self.ancilla_budget = budget;
        self
    }

    /// Start numbering new measurement results at `first_bit` (default: zero).
    /// Set this to the number of existing classical bits to avoid overwriting them.
    pub fn with_first_bit(mut self, first_bit: usize) -> Self {
        self.first_bit = first_bit;
        self
    }

    /// Optimize a graph whose rotation angles are multiples of 0.25 half turns.
    ///
    /// Panics if the input contains unsupported operations or non-idle reserved ancillas.
    pub fn optimize(&self, graph: &PauliGraph) -> PauliGraph {
        optimize(graph, self.ancilla_budget, self.first_bit)
    }
}

impl PGPass for TOptimizationPass {
    fn transform(&self, pauli_graph: &PauliGraph) -> PauliGraph {
        self.optimize(pauli_graph)
    }
}
