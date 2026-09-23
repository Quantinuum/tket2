#![doc = include_str!("../README.md")]

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
pub use tk_pg_converter as tk;
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
