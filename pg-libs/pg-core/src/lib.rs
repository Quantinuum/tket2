//! This crate contains the core data structures and traits for the `pg-libs` library.

mod errors;
mod gates;
mod graph;
mod ops;
mod traits;

pub use errors::PauliGraphError;
pub use gates::GateType;
pub use graph::{Pauli, PauliGraph};
pub use ops::{
    BlackBoxData, ConditionalBoxData, GateData, MeasureData, Op, ResetData, RotationData,
    TableauData,
};
pub use traits::PGPass;

pub(crate) use gates::{gate_type_n_args, gate_type_n_params};

#[cfg(test)]
mod tests;
