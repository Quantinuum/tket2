#![doc = include_str!("../README.md")]
#![cfg_attr(feature = "unstable_simd", feature(portable_simd))]
#![expect(clippy::upper_case_acronyms)]
#![expect(clippy::too_many_arguments)]
mod backend;
mod frontier;
mod packed_pg_slice;
mod reducer;
mod synthesis;
mod tqe;
mod utils;

#[cfg(feature = "unstable_simd")]
pub use synthesis::GreedySynthSimdPass;
pub use synthesis::{GreedySynthPass, ParallelMode};
