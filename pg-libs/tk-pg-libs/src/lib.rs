//! An umbrella crate for pg-libs, TKET's Pauli graph libraries. It re-exports
//! core types, transformation and synthesis passes, and supporting utilities.
//!
//! # Quick start
//!
//! This example optimizes the T count and synthesizes a two qubit Clifford+T circuit.
//!
//! ```
//! use tk_pg_libs::{GateData, GateType, Op, PauliGraph};
//! use tk_pg_libs::passes::{
//!     CanonicalFormPass, GreedySynthPass, GroupCommutingOpsPass, PGPass,
//!     RotationMergingPass, TOptimizationPass,
//! };
//!
//! let mut pg = PauliGraph::new(2);
//! for data in [
//!     GateData::new(GateType::RZ, vec![1]).with_params(vec![0.25]),
//!     GateData::new(GateType::ZX, vec![0, 1]), // CX(0, 1)
//!     GateData::new(GateType::ZX, vec![1, 0]), // CX(1, 0)
//!     GateData::new(GateType::RZ, vec![0]).with_params(vec![0.25]),
//! ] {
//!     pg.add_op(Op::Gate { data });
//! }
//!
//! let pg = CanonicalFormPass::new().transform(&pg);
//! let pg = RotationMergingPass::new().transform(&pg);
//! let pg = TOptimizationPass::new().transform(&pg);
//! let pg = GroupCommutingOpsPass::new().transform(&pg);
//! let pg = GreedySynthPass::new().transform(&pg);
//! ```
//!
//! [`passes::TOptimizationPass`] uses no ancillas by default and requires rotation
//! angles that are multiples of 0.25 half turns.
//! Measurements, resets, and black boxes are not supported as input to this pass.

#[doc(inline)]
pub use tk_pg_core::{
    BlackBoxData, ConditionalBoxData, GateData, GateType, MeasureData, Op, PGPass, Pauli,
    PauliGraph, PauliGraphError, ResetData, RotationData, TableauData,
};

#[doc(inline)]
pub use tk_pg_core as core;

#[doc(inline)]
pub use tk_pg_bitpacked as bitpacked;
#[doc(inline)]
pub use tk_pg_converter as tk_converter;
#[doc(inline)]
pub use tk_pg_ir_kernels as ir_kernels;
#[doc(inline)]
pub use tk_pg_qm_tableau as qm_tableau;
#[doc(inline)]
pub use tk_pg_utils as utils;

/// Canonical form, optimization, synthesis and rebasing passes for `PauliGraph`.
pub mod passes {
    #[doc(inline)]
    pub use tk_pg_canonical_form::CanonicalFormPass;
    #[doc(inline)]
    pub use tk_pg_core::PGPass;
    #[doc(inline)]
    pub use tk_pg_greedy_synth::{GreedySynthPass, ParallelMode};
    #[doc(inline)]
    pub use tk_pg_optimize::{GroupCommutingOpsPass, RotationMergingPass};
    #[doc(inline)]
    pub use tk_pg_rebase::RebaseTQEToZXPass;
    #[doc(inline)]
    pub use tk_pg_t_optimize::TOptimizationPass;

    #[cfg(feature = "unstable_simd")]
    #[doc(inline)]
    pub use tk_pg_greedy_synth::GreedySynthSimdPass;

    #[doc(inline)]
    pub use tk_pg_canonical_form as canonical_form;
    #[doc(inline)]
    pub use tk_pg_greedy_synth as greedy_synth;
    #[doc(inline)]
    pub use tk_pg_optimize as optimise;
    #[doc(inline)]
    pub use tk_pg_rebase as rebase;
    #[doc(inline)]
    pub use tk_pg_t_optimize as t_optimize;
}
