//! Optimisation passes and related utilities for HUGR programs.

/// Compiler passes and utilities for composing them.
pub mod composable;
pub use composable::{ComposablePass, InScope, PassScope, WithScope};

// -- tket-defined passes ---------------------------------------------------

// Elide pairs of return-borrow operations on `BorrowArray`s.
pub mod borrow_squash;
pub use borrow_squash::BorrowSquashPass;

// Greedy gate commutation on quantum circuits.
pub mod commutation;
pub use commutation::apply_greedy_commutation;

// Constant folding pass.
pub mod const_fold;
pub use const_fold::ConstantFoldPass;

// Dataflow analysis framework.
pub mod dataflow;

// Dead code elimination pass.
pub mod dead_code;
pub use dead_code::DeadCodeElimPass;

// Dead function removal.
pub mod dead_funcs;
pub use dead_funcs::{RemoveDeadFuncsError, RemoveDeadFuncsPass};

// Force a topological order on nodes.
pub mod force_order;

// Normalize the structure of Guppy-generated programs.
pub mod guppy;
pub use guppy::NormalizeGuppy;

// Inline DFG nodes.
pub mod inline_dfgs;
pub use inline_dfgs::InlineDFGsPass;

// Inline function calls.
pub mod inline_funcs;
pub use inline_funcs::InlineFunctionsPass;

// Lower and replace operations.
pub mod lower;
pub use lower::{lower_ops, replace_many_ops};

// Resolve modifier operations (control/dagger/power).
pub mod modifier_resolver;
pub use modifier_resolver::ModifierResolverPass;

// Monomorphization pass.
pub mod monomorphize;
pub use monomorphize::{MonomorphizePass, mangle_name};

// Nest SESE regions in CFGs.
pub mod nest_cfgs;

// Find and localize non-local edges.
pub mod non_local;

// CFG normalization (merge blocks, simplify control flow).
pub mod normalize_cfgs;
pub use normalize_cfgs::NormalizeCFGPass;

// Remove redundant order edges.
pub mod redundant_order_edges;
pub use redundant_order_edges::RedundantOrderEdgesPass;

// Replace types, ops, and constants across a HUGR.
pub mod replace_types;
pub use replace_types::ReplaceTypes;

// Remove redundant tuple pack/unpack operations.
pub mod untuple;
pub use untuple::UntuplePass;

// -- Internal modules -------------------------------------------------------

pub(crate) mod half_node;

#[cfg(test)]
pub(crate) mod test_utils;

// -- Utilities -------------------------------------------------------------

/// Utilities for writing and applying passes on HUGR programs.
pub mod utils;
