//! Optimisation passes and related utilities for HUGR programs.

pub mod composable;
pub use composable::{ComposablePass, PassScope, WithScope};

// -- tket-defined passes ---------------------------------------------------

/// Elide pairs of return-borrow operations on `BorrowArray`s.
pub mod borrow_squash;
pub use borrow_squash::BorrowSquashPass;

/// Greedy gate commutation on quantum circuits.
pub mod commutation;
pub use commutation::apply_greedy_commutation;

/// Normalize the structure of Guppy-generated programs.
pub mod guppy;
pub use guppy::NormalizeGuppy;

/// Resolve modifier operations (control/dagger/power).
pub mod modifier_resolver;
pub use modifier_resolver::ModifierResolverPass;

// -- Utilities -------------------------------------------------------------

/// Utilities for writing and applying passes on HUGR programs.
pub mod utils;

// -- Re-exports from hugr_passes -------------------------------------------

/// Constant folding pass.
pub use hugr_passes::const_fold;
pub use hugr_passes::const_fold::ConstantFoldPass;

/// Dataflow analysis framework.
pub use hugr_passes::dataflow;

/// Dead code elimination pass.
pub use hugr_passes::dead_code;
pub use hugr_passes::dead_code::DeadCodeElimPass;

/// Force a topological order on nodes.
pub mod force_order {
    pub use hugr_passes::force_order::force_order;
    pub use hugr_passes::force_order::force_order_by_key;
}

/// Inline DFG nodes.
pub use hugr_passes::inline_dfgs;
pub use hugr_passes::inline_dfgs::InlineDFGsPass;

/// Inline function calls.
pub use hugr_passes::inline_funcs;

/// Lower and replace operations.
pub use hugr_passes::lower;

/// Nest SESE regions in CFGs.
pub use hugr_passes::nest_cfgs;

/// Find and localize non-local edges.
pub use hugr_passes::non_local;

/// CFG normalization (merge blocks, simplify control flow).
pub use hugr_passes::normalize_cfgs;
pub use hugr_passes::normalize_cfgs::NormalizeCFGPass;

/// Remove redundant order edges.
pub use hugr_passes::redundant_order_edges;
pub use hugr_passes::redundant_order_edges::RedundantOrderEdgesPass;

/// Replace types, ops, and constants across a HUGR.
pub use hugr_passes::replace_types;
pub use hugr_passes::replace_types::ReplaceTypes;

/// Remove redundant tuple pack/unpack operations.
pub use hugr_passes::untuple;
pub use hugr_passes::untuple::UntuplePass;

/// Dead function removal.
pub mod dead_funcs {
    pub use hugr_passes::RemoveDeadFuncsError;
    pub use hugr_passes::RemoveDeadFuncsPass;
}
pub use dead_funcs::RemoveDeadFuncsPass;

/// Monomorphization pass.
pub mod monomorphize {
    pub use hugr_passes::MonomorphizePass;
    pub use hugr_passes::mangle_name;
}
pub use monomorphize::MonomorphizePass;
