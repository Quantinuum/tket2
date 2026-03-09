//! Optimisation passes and related utilities for circuits.

mod commutation;

pub use commutation::{PullForwardError, apply_greedy_commutation};

pub mod borrow_squash;
pub use borrow_squash::BorrowSquashPass;

pub mod chunks;
pub use chunks::CircuitChunks;

pub mod guppy;
pub use guppy::NormalizeGuppy;

pub mod unpack_container;
