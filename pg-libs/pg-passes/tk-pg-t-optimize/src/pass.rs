use tk_pg_core::{PGPass, PauliGraph};

use crate::optimizer::optimize;

#[derive(Default)]
/// A T-count optimization pass for rotations, tableaux, and unconditional H gates.
pub struct TOptimizationPass {
    ancilla_budget: usize,
}

impl TOptimizationPass {
    /// Create a pass that uses no ancillas.
    pub fn new() -> Self {
        Self::default()
    }

    /// Reserve the last `budget` input qubits as idle ancillas prepared in zero.
    pub fn with_ancilla_budget(mut self, budget: usize) -> Self {
        self.ancilla_budget = budget;
        self
    }

    /// Optimize a graph whose rotation angles are multiples of 0.25 half turns.
    ///
    /// Panics if the input contains unsupported operations or non-idle reserved ancillas.
    pub fn optimize(&self, graph: &PauliGraph) -> PauliGraph {
        optimize(graph, self.ancilla_budget)
    }
}

impl PGPass for TOptimizationPass {
    fn transform(&self, pauli_graph: &PauliGraph) -> PauliGraph {
        self.optimize(pauli_graph)
    }
}
