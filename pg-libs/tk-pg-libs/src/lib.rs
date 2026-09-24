//! An umbrella crate for pg-libs, TKET's Pauli graph libraries. It re-exports
//! core types, transformation and synthesis passes, and supporting utilities.
//!
//! # Quick start
//!
//! This example optimizes and synthesizes a two qubit circuit.
//!
//! ```
//! use tk_pg_libs::{GateData, GateType, Op, PauliGraph};
//! use tk_pg_libs::passes::{
//!     CanonicalFormPass, GreedySynthPass, GroupCommutingOpsPass, PGPass,
//!     RotationMergingPass,
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
//! let pg = GroupCommutingOpsPass::new().transform(&pg);
//! let pg = GreedySynthPass::new().transform(&pg);
//! ```

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
}
