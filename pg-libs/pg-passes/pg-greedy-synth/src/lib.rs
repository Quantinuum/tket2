#![doc = include_str!("../README.md")]
#![cfg_attr(feature = "simd", feature(portable_simd))]
#![allow(clippy::upper_case_acronyms)]
mod backend;
mod frontier;
mod packed_pg_slice;
mod reducer;
mod synthesis;
mod tqe;
mod utils;

#[cfg(feature = "simd")]
pub use synthesis::GreedySynthSimdPass;
pub use synthesis::{GreedySynthPass, ParallelMode};
