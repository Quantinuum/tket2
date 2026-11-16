//! Passes for dealing with control-flow graphs

// CFG normalization (merge blocks, simplify control flow).
pub mod normalize_cfgs;

// Dominator trees and CFG cycle removal using that
pub mod dom_tree;
pub mod remove_cycles;

// Nesting SESE regions in CFGs
pub mod nest_cfgs;

// -- Internal modules -------------------------------------------------------
pub(crate) mod half_node;
