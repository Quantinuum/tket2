//! Pattern matching for circuits.
//!
//! This module provides a way to define circuit patterns and match
//! them against circuits.
//!
//! # Examples
//! ```
//! use tket::portmatching::{CircuitPattern, PatternMatcher};
//! use tket::TketOp;
//! use hugr::builder::{DFGBuilder, Dataflow, DataflowHugr};
//! use hugr::extension::prelude::qb_t;
//! use hugr::ops::handle::NodeHandle;
//! use hugr::types::Signature;
//!
//! # fn doctest() -> Result<(), Box<dyn std::error::Error>> {
//! // Define a simple pattern that matches a single qubit allocation.
//! let circuit_pattern = {
//!     let mut dfg = DFGBuilder::new(Signature::new(vec![], vec![qb_t()]))?;
//!     let alloc = dfg.add_dataflow_op(TketOp::QAlloc, [])?;
//!     dfg.finish_hugr_with_outputs(alloc.outputs())
//! }?.into();
//! let pattern = CircuitPattern::try_from_circuit(&circuit_pattern)?;
//!
//! // Define a circuit that contains a qubit allocation.
//! //
//! // -----[Z]--x---
//! //           |
//! //  0|--[Z]--o---
//! let (circuit, alloc_node) = {
//!     let mut dfg = DFGBuilder::new(Signature::new(vec![qb_t()], vec![qb_t(), qb_t()]))?;
//!     let [input_wire] = dfg.input_wires_arr();
//!     let alloc = dfg.add_dataflow_op(TketOp::QAlloc, [])?;
//!     let [alloc_wire] = alloc.outputs_arr();
//!
//!     let mut circuit = dfg.as_circuit(vec![input_wire, alloc_wire]);
//!     circuit
//!         .append(TketOp::Z, [1])?
//!         .append(TketOp::Z, [0])?
//!         .append(TketOp::CX, [1, 0])?;
//!     let outputs = circuit.finish();
//!
//!     let circuit = dfg.finish_hugr_with_outputs(outputs)?.into();
//!     (circuit, alloc.node())
//! };
//!
//! // Create a pattern matcher and find matches.
//! let matcher = PatternMatcher::from_patterns(vec![pattern]);
//! let matches = matcher.find_matches(&circuit);
//!
//! assert_eq!(matches.len(), 1);
//! assert_eq!(matches[0].nodes(), [alloc_node]);
//! # Ok(())
//! # }
//! ```

pub mod matcher;
pub mod pattern;

use hugr::types::EdgeKind;
use hugr::{HugrView, OutgoingPort};
use itertools::Itertools;
pub use matcher::{PatternMatch, PatternMatcher, Rule, RuleMatcher};
pub use pattern::CircuitPattern;

use derive_more::{Display, Error};
use hugr::{
    ops::{OpTag, OpTrait},
    Node, Port,
};
use matcher::MatchOp;

use crate::{circuit::Circuit, utils::type_is_linear};

type PNode = MatchOp;

/// An edge property in a circuit pattern.
///
/// Edges are
/// Edges are reversible if the edge type is linear.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize,
)]
enum PEdge {
    /// A "normal" edge between src and dst within a pattern.
    InternalEdge {
        src: Port,
        dst: Port,
        is_reversible: bool,
    },
    /// An edge from a copied input to src.
    ///
    /// Edges from inputs are typically not matched as part of the pattern,
    /// unless a single input is copied multiple times. In this case, an
    /// InputEdge is used to link the source port to the (usually hidden)
    /// copy node.
    ///
    /// Input edges are always irreversible.
    InputEdge { src: Port },
}

#[derive(Debug, Clone, Error, Display)]
#[non_exhaustive]
enum InvalidEdgeProperty {
    /// The port is linked to multiple edges.
    #[display("{port} in {node} is linked to multiple edges")]
    AmbiguousEdge { port: Port, node: Node },
    /// The port is not linked to any edge.
    #[display("{port} in {node} is not linked to any edge")]
    NoLinkedEdge { port: Port, node: Node },
    /// The port does not have a type.
    #[display("{port} in {node} does not have a type")]
    UntypedPort { port: Port, node: Node },
}

impl PEdge {
    fn try_from_port(node: Node, port: Port, circ: &Circuit) -> Result<Self, InvalidEdgeProperty> {
        let hugr = circ.hugr();
        let src = port;
        let (dst_node, dst) = hugr
            .linked_ports(node, src)
            .exactly_one()
            .map_err(|mut e| {
                if e.next().is_some() {
                    InvalidEdgeProperty::AmbiguousEdge { port: src, node }
                } else {
                    InvalidEdgeProperty::NoLinkedEdge { port: src, node }
                }
            })?;
        if hugr.get_optype(dst_node).tag() == OpTag::Input {
            return Ok(Self::InputEdge { src });
        }

        // Get the port type for either value or constant ports.
        let port_type = match hugr.get_optype(node).port_kind(src) {
            Some(EdgeKind::Value(typ)) => typ,
            Some(EdgeKind::Const(typ)) => typ,
            _ => return Err(InvalidEdgeProperty::UntypedPort { node, port: src }),
        };
        let is_reversible = type_is_linear(&port_type);
        Ok(Self::InternalEdge {
            src,
            dst,
            is_reversible,
        })
    }
}

impl portmatching::EdgeProperty for PEdge {
    type OffsetID = Port;

    fn reverse(&self) -> Option<Self> {
        match *self {
            Self::InternalEdge {
                src,
                dst,
                is_reversible,
            } => is_reversible.then_some(Self::InternalEdge {
                src: dst,
                dst: src,
                is_reversible,
            }),
            Self::InputEdge { .. } => None,
        }
    }

    fn offset_id(&self) -> Self::OffsetID {
        match *self {
            Self::InternalEdge { src, .. } => src,
            Self::InputEdge { src, .. } => src,
        }
    }
}

/// A node in a pattern.
///
/// A node is either a real node in the HUGR graph or a hidden copy node
/// that is identified by its node and outgoing port.
///
/// A NodeID::CopyNode can only be found as a target of a PEdge::InputEdge
/// property. Furthermore, a NodeID::CopyNode never has a node property.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize,
)]
pub(super) enum NodeID {
    HugrNode(Node),
    CopyNode(Node, Port),
}

impl NodeID {
    /// Create a new copy NodeID.
    pub fn new_copy(node: Node, port: impl Into<OutgoingPort>) -> Self {
        let port: OutgoingPort = port.into();
        Self::CopyNode(node, port.into())
    }
}

impl From<Node> for NodeID {
    fn from(node: Node) -> Self {
        Self::HugrNode(node)
    }
}

#[cfg(test)]
mod tests {
    use crate::{Circuit, TketOp};
    use hugr::{
        builder::{DFGBuilder, Dataflow, DataflowHugr},
        extension::prelude::qb_t,
        types::Signature,
    };
    use rstest::{fixture, rstest};

    use super::{CircuitPattern, PatternMatcher};

    #[fixture]
    fn lhs() -> Circuit {
        let mut h = DFGBuilder::new(Signature::new(vec![], vec![qb_t()])).unwrap();

        let res = h.add_dataflow_op(TketOp::QAlloc, []).unwrap();
        let q = res.out_wire(0);

        h.finish_hugr_with_outputs([q]).unwrap().into()
    }

    #[fixture]
    pub fn circ() -> Circuit {
        let mut h = DFGBuilder::new(Signature::new(vec![qb_t()], vec![qb_t()])).unwrap();
        let mut inps = h.input_wires();
        let q_in = inps.next().unwrap();

        let res = h.add_dataflow_op(TketOp::QAlloc, []).unwrap();
        let q_out = res.out_wire(0);
        let res = h.add_dataflow_op(TketOp::CZ, [q_in, q_out]).unwrap();
        let q_in = res.out_wire(0);
        let q_out = res.out_wire(1);
        h.add_dataflow_op(TketOp::QFree, [q_in]).unwrap();

        h.finish_hugr_with_outputs([q_out]).unwrap().into()
    }

    #[rstest]
    fn simple_match(circ: Circuit, lhs: Circuit) {
        let p = CircuitPattern::try_from_circuit(&lhs).unwrap();
        let m = PatternMatcher::from_patterns(vec![p]);

        let matches = m.find_matches(&circ);
        assert_eq!(matches.len(), 1);
    }
}
