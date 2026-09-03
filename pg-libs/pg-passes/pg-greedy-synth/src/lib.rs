//! Greedy synthesis pass for Pauli graphs.
//!
//! [`GreedySynthPass`] lowers a canonical Pauli graph whose operations are
//! grouped into commuting sets. The output contains Clifford gates on
//! individual qubits, Pauli rotations, measurements, resets and entangling
//! gates between two qubits, known as TQEs. [`ParallelMode`] controls whether
//! Rayon evaluates candidate costs in parallel.
//!
//! The pass expects input produced by `CanonicalFormPass` followed by
//! `GroupCommutingOpsPass`.

#![cfg_attr(feature = "simd", feature(portable_simd))]
#![allow(clippy::upper_case_acronyms)]
mod backend;
mod frontier;
mod packed_pg_slice;
mod reducer;
mod synthesis;
mod tqe;
mod utils;

#[cfg(feature = "simd")]
pub use synthesis::GreedySynthSimdPass;
pub use synthesis::{GreedySynthPass, ParallelMode};
