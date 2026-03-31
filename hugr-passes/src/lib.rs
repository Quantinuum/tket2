//! Compilation passes acting on the HUGR program representation.
//!
//! <div class="warning">This crate is deprecated. Use [`tket::passes`](https://docs.rs/tket/latest/tket/passes/index.html) instead.</div>

pub use tket::passes::composable;
pub use tket::passes::dataflow;
pub use tket::passes::dead_code;
pub use tket::passes::inline_dfgs;
pub use tket::passes::inline_funcs;
pub use tket::passes::lower;
pub use tket::passes::nest_cfgs;
pub use tket::passes::non_local;
pub use tket::passes::normalize_cfgs;
pub use tket::passes::redundant_order_edges;
pub use tket::passes::replace_types;
pub use tket::passes::untuple;
pub use tket::passes::utils::hash;

// Main pass interfaces
pub use tket::passes::composable::{ComposablePass, InScope, PassScope};

// Pass re-exports
pub use tket::passes::dead_code::DeadCodeElimPass;
pub use tket::passes::dead_funcs::{RemoveDeadFuncsError, RemoveDeadFuncsPass};
pub use tket::passes::force_order::{force_order, force_order_by_key};
pub use tket::passes::inline_funcs::inline_acyclic;
pub use tket::passes::lower::{lower_ops, replace_many_ops};
pub use tket::passes::monomorphize::{MonomorphizePass, mangle_name};
#[deprecated(
    note = "Use LocalizeEdgesPass::check_no_nonlocal_edges",
    since = "0.26.0"
)]
#[expect(deprecated)] // Remove at same time
pub use tket::passes::non_local::ensure_no_nonlocal_edges;
pub use tket::passes::replace_types::ReplaceTypes;
pub use tket::passes::untuple::UntuplePass;
