mod common;
mod greedy;
mod greedy_tableau;
mod parallel_mode;

pub(crate) use common::{progress_bar, tqe_op, update_depth};
pub use greedy::GreedySynthPass;
#[cfg(feature = "simd")]
pub use greedy::GreedySynthSimdPass;
pub(crate) use greedy_tableau::synthesise_tableau;
pub use parallel_mode::ParallelMode;

#[cfg(test)]
mod tests;
