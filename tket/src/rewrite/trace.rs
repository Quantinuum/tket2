//! Utilities for tracing the rewrites applied to a circuit.
//!
//! This is only tracked if the `rewrite-tracing` feature is enabled.

use crate::{Circuit, metadata};
use hugr::hugr::hugrmut::HugrMut;
use serde::{Deserialize, Serialize};

use super::CircuitRewrite;

/// Global read-only flag for enabling rewrite tracing.
/// Enable it by setting the `rewrite-tracing` feature.
///
/// Note that circuits must be explicitly enabled for rewrite tracing by calling
/// [`Circuit::enable_rewrite_tracing`].
pub const REWRITE_TRACING_ENABLED: bool = cfg!(feature = "rewrite-tracing");

/// The trace of a rewrite applied to a circuit.
///
/// Traces are only enabled if the `rewrite-tracing` feature is enabled and
/// [`Circuit::enable_rewrite_tracing`] is called on the circuit.
#[derive(Deserialize, Serialize, Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct RewriteTrace {
    /// A count of the number of individual patterns matched for this rewrite step.
    ///
    /// This is relevant when using a greedy rewrite strategy.
    individual_matches: u16,
}

impl From<&CircuitRewrite> for RewriteTrace {
    #[inline]
    fn from(_rewrite: &CircuitRewrite) -> Self {
        // NOTE: We don't currently track any actual information about the rewrite.
        Self {
            individual_matches: 1,
        }
    }
}

impl RewriteTrace {
    /// Create a new trace.
    #[inline]
    pub fn new(individual_matches: u16) -> Self {
        Self { individual_matches }
    }
}

/// Implementation for rewrite tracing in circuits.
///
/// This is only tracked if the `rewrite-tracing` feature is enabled and
/// `enable_rewrite_tracing` is called on the circuit before.
impl<T: HugrMut> Circuit<T> {
    /// Enable rewrite tracing for the circuit.
    #[inline]
    pub fn enable_rewrite_tracing(&mut self) {
        if !REWRITE_TRACING_ENABLED {
            return;
        }
        let root = self.parent();
        let hugr = self.hugr_mut();
        let Some(_) = hugr.get_metadata::<metadata::CircuitRewriteTraces>(root) else {
            hugr.set_metadata::<metadata::CircuitRewriteTraces>(root, vec![]);
            return;
        };
    }

    /// Register a rewrite applied to the circuit.
    ///
    /// Returns `true` if the rewrite was successfully registered, or `false` if it was ignored.
    #[inline]
    pub fn add_rewrite_trace(&mut self, rewrite: impl Into<RewriteTrace>) -> bool {
        if !REWRITE_TRACING_ENABLED {
            return false;
        }
        let root = self.parent();
        match self
            .hugr()
            .get_metadata::<metadata::CircuitRewriteTraces>(root)
        {
            Some(mut meta) => {
                meta.push(rewrite.into());
                self.hugr_mut()
                    .set_metadata::<metadata::CircuitRewriteTraces>(root, meta);
                true
            }
            // Tracing was not enabled for this circuit.
            None => false,
        }
    }

    /// Returns the traces of rewrites applied to the circuit.
    ///
    /// Returns `None` if rewrite tracing is not enabled for this circuit.
    #[inline]
    pub fn rewrite_trace(&self) -> Option<impl Iterator<Item = RewriteTrace> + '_> {
        if !REWRITE_TRACING_ENABLED {
            return None;
        }
        let ve = self
            .hugr()
            .get_metadata::<metadata::CircuitRewriteTraces>(self.parent());
        ve.map(Vec::into_iter)
    }
}
