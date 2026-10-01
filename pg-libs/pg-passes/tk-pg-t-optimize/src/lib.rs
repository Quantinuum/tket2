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

#[cfg(test)]
mod tests {
    use super::*;
    use tk_pg_converter::compare_unitaries_via_tk;
    use tk_pg_core::{Op, Pauli, RotationData};

    #[test]
    fn reduces_t_count_without_ancillas() {
        for (input_t_count, max_t_count) in [(14, 1), (15, 0)] {
            // T rotations on all 15 nonzero four-qubit parities give identity up to phase.
            // Omitting one parity leaves a single inverse T rotation.
            let pg = PauliGraph::new(4).with_ops(
                (1..=input_t_count)
                    .map(|parity| Op::Rotation {
                        data: RotationData::new(
                            (0..4)
                                .map(|q| {
                                    if parity & (1 << q) == 0 {
                                        Pauli::I
                                    } else {
                                        Pauli::Z
                                    }
                                })
                                .collect(),
                            0.25,
                        ),
                    })
                    .collect(),
            );

            let optimized = TOptimizationPass::new().transform(&pg);
            optimized.try_validate().unwrap();
            assert_eq!(optimized.get_n_qubits(), pg.get_n_qubits());
            let rotations: Vec<_> = optimized
                .get_ops()
                .iter()
                .filter_map(|op| match op {
                    Op::Rotation { data } => Some(data),
                    _ => None,
                })
                .collect();
            assert!(rotations.iter().all(|data| data.get_angle().abs() == 0.25));
            assert!(
                rotations.len() <= max_t_count,
                "input T count: {input_t_count}"
            );
            assert!(compare_unitaries_via_tk(&pg, &optimized));
        }
    }
}
