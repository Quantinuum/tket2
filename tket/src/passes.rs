//! Optimisation passes and related utilities for circuits.

mod commutation;

pub use commutation::{PullForwardError, apply_greedy_commutation};

pub mod borrow_squash;
pub use borrow_squash::BorrowSquashPass;

pub mod chunks;
pub use chunks::CircuitChunks;

// pub mod fast_todd;
// pub use fast_todd::{apply_fast_todd, apply_fast_todd_to_pauli_graph, FastToddResult, FastToddError};

pub mod global_t_resynthesis;
pub use global_t_resynthesis::GlobalTResynthesis;
// pub use global_t_resynthesis::InlineAll;

pub mod guppy;
pub use guppy::NormalizeGuppy;

pub mod pytket;
pub use pytket::lower_to_pytket;

pub mod unpack_container;
