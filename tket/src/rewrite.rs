//! Transform circuits using rewrite rules.

#[cfg(feature = "portmatching")]
pub mod ecc_rewriter;
pub mod strategy;
pub mod trace;

#[cfg(feature = "portmatching")]
pub use ecc_rewriter::ECCRewriter;

use derive_more::{From, Into};
use hugr::hugr::hugrmut::HugrMut;
use hugr::hugr::patch::simple_replace;
use hugr::hugr::views::sibling_subgraph::InvalidReplacement;
use hugr::hugr::Patch;
use hugr::{
    hugr::{views::SiblingSubgraph, SimpleReplacementError},
    SimpleReplacement,
};
use hugr::{Hugr, HugrView, Node};

use crate::circuit::Circuit;

/// A rewrite rule for circuits.
#[derive(Debug, Clone, From, Into)]
pub struct CircuitRewrite<N = Node>(SimpleReplacement<N>);

impl CircuitRewrite {
    /// Create a new rewrite rule.
    pub fn try_new(
        subgraph: &SiblingSubgraph,
        hugr: &impl HugrView<Node = Node>,
        replacement: Circuit<impl HugrView<Node = Node>>,
    ) -> Result<Self, InvalidReplacement> {
        let replacement = replacement
            .extract_dfg()
            .unwrap_or_else(|e| panic!("{}", e))
            .into_hugr();
        Ok(Self(subgraph.create_simple_replacement(hugr, replacement)?))
    }

    /// Number of nodes added or removed by the rewrite.
    ///
    /// The difference between the new number of nodes minus the old. A positive
    /// number is an increase in node count, a negative number is a decrease.
    pub fn node_count_delta(&self) -> isize {
        let new_count = self.replacement().num_operations() as isize;
        let old_count = self.subgraph().node_count() as isize;
        new_count - old_count
    }

    /// The subgraph that is replaced.
    pub fn subgraph(&self) -> &SiblingSubgraph {
        self.0.subgraph()
    }

    /// The replacement subcircuit.
    pub fn replacement(&self) -> Circuit<&Hugr> {
        self.0.replacement().into()
    }

    /// Returns a set of nodes referenced by the rewrite. Modifying any these
    /// nodes will invalidate it.
    ///
    /// Two `CircuitRewrite`s can be composed if their invalidation sets are
    /// disjoint.
    #[inline]
    pub fn invalidation_set(&self) -> impl Iterator<Item = Node> + '_ {
        self.0.invalidation_set()
    }

    /// Apply the rewrite rule to a circuit.
    #[inline]
    pub fn apply(
        self,
        circ: &mut Circuit<impl HugrMut<Node = Node>>,
    ) -> Result<simple_replace::Outcome<Node>, SimpleReplacementError> {
        circ.add_rewrite_trace(&self);
        self.0.apply(circ.hugr_mut())
    }

    /// Apply the rewrite rule to a circuit, without registering it in the rewrite trace.
    #[inline]
    pub fn apply_notrace(
        self,
        circ: &mut Circuit<impl HugrMut<Node = Node>>,
    ) -> Result<simple_replace::Outcome<Node>, SimpleReplacementError> {
        self.0.apply(circ.hugr_mut())
    }
}

/// Generate rewrite rules for circuits.
pub trait Rewriter {
    /// Get the rewrite rules for a circuit.
    fn get_rewrites(&self, circ: &Circuit<impl HugrView<Node = Node>>) -> Vec<CircuitRewrite>;
}
