//! Optimisation passes for Pauli graphs.

mod commuting_sets;
mod packed_op;
mod rotation_merging;
pub use commuting_sets::GroupCommutingOpsPass;
pub use rotation_merging::RotationMergingPass;
