//! Compiler passes and utilities for composing them.
//!
//! Re-exports from [`hugr_passes::composable`].

pub use hugr_passes::composable::{
    ComposablePass, ErrorCombiner, IfThen, InScope, PassScope, Preserve, ValidatePassError,
    ValidatingPass, WithScope,
};
