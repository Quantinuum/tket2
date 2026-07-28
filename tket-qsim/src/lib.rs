//! Internal quantum circuit simulator for testing compilation correctness.
//!
//! This crate provides a simple unitary-based simulator that can compute the
//! overall unitary matrix of a quantum circuit represented as a HUGR. It is
//! intended exclusively for testing that compilation passes preserve
//! circuit semantics.
//!
//! # Architecture
//!
//! - [`UnitaryMatrix`]: A dense 2^n × 2^n complex matrix.
//! - [`Simulatable`]: A trait that each gate type implements to provide its
//!   unitary representation.
//! - [`simulate_circuit`]: Traverses a HUGR dataflow graph and computes the
//!   overall unitary by composing individual gates.

#![allow(missing_docs)]

mod helios_ops;
mod matrix;
mod simulate;
mod sol_ops;
mod tket_ops;

pub use matrix::UnitaryMatrix;
pub use simulate::simulate_circuit;

/// Trait for quantum operations that can be simulated via unitary matrices.
///
/// Each implementor provides the unitary for a specific gate, given its
/// floating-point parameters. The `num_qubits` method specifies the
/// dimension of the unitary: 2^n × 2^n).
pub trait Simulatable {
    /// Number of qubits this gate acts on.
    fn num_qubits(&self) -> usize;

    /// Compute the unitary matrix for this gate.
    ///
    /// `params`: floating-point parameters for parametric gates.
    ///           Non-parametric gates ignore this slice.
    fn unitary(&self, params: &[f64]) -> UnitaryMatrix;
}
