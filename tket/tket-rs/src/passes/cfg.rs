//! Passes for dealing with control-flow graphs
//!
//! TODO: move [normalize_cfgs] here.

pub mod dom_tree;
pub mod remove_cycles;

pub mod nest_cfgs;

// -- Internal modules -------------------------------------------------------
pub(crate) mod half_node;
