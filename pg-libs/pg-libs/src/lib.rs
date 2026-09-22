#![doc = include_str!("../README.md")]

#[doc(inline)]
pub use pg_core::{
    BlackBoxData, ConditionalBoxData, GateData, GateType, MeasureData, Op, PGPass, Pauli,
    PauliGraph, PauliGraphError, ResetData, RotationData, TableauData,
};

/// `PauliGraph` IR and the `PGPass` trait.
#[doc(inline)]
pub use pg_core as core;

/// Clifford conjugation of Pauli operators using packed representations.
#[doc(inline)]
pub use pg_bitpacked as bitpacked;
/// Basic rewrite operations for `PauliGraph` and the `PGTableau` trait.
#[doc(inline)]
pub use pg_ir_kernels as ir_kernels;
/// Clifford tableaux using a qubit major memory layout.
#[doc(inline)]
pub use pg_qm_tableau as qm_tableau;
/// Conversions between `PauliGraph` and serialized TKET circuits (JSON).
///
/// Also provides unitary equivalence checks using `pytket`.
#[doc(inline)]
pub use pg_tk as tk;
/// Clifford angle detection and approximate comparison to zero modulo a value.
#[doc(inline)]
pub use pg_utils as utils;

/// Canonical form, optimization, synthesis and rebasing passes for `PauliGraph`.
pub mod passes {
    #[doc(inline)]
    pub use pg_canonical_form::CanonicalFormPass;
    #[doc(inline)]
    pub use pg_core::PGPass;
    #[doc(inline)]
    pub use pg_greedy_synth::{GreedySynthPass, ParallelMode};
    #[doc(inline)]
    pub use pg_optimise::{GroupCommutingOpsPass, RotationMergingPass};
    #[doc(inline)]
    pub use pg_rebase::RebaseTQEToZXPass;

    #[cfg(feature = "simd")]
    #[doc(inline)]
    pub use pg_greedy_synth::GreedySynthSimdPass;

    /// Transforms a `PauliGraph` into canonical form.
    #[doc(inline)]
    pub use pg_canonical_form as canonical_form;
    /// Greedy synthesis of a canonical `PauliGraph` grouped into commuting sets.
    #[doc(inline)]
    pub use pg_greedy_synth as greedy_synth;
    /// Groups commuting operations and merges Pauli rotations.
    #[doc(inline)]
    pub use pg_optimise as optimise;
    /// Rewrites two qubit entangling gates as `ZX` gates and single qubit Clifford gates.
    #[doc(inline)]
    pub use pg_rebase as rebase;
}
