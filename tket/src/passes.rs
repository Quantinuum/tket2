//! Optimisation passes and related utilities for circuits.

mod commutation;

pub use commutation::{PullForwardError, apply_greedy_commutation};

pub mod borrow_squash;
pub use borrow_squash::BorrowSquashPass;

pub mod chunks;
pub use chunks::CircuitChunks;

pub mod guppy;
pub use guppy::NormalizeGuppy;

pub mod pytket;
pub use pytket::lower_to_pytket;

pub mod tuple_unpack;
#[expect(deprecated)]
pub use tuple_unpack::find_tuple_unpack_rewrites;

pub mod unpack_container;

pub mod rewrite_quantum;
