//! Circuit Patterns for pattern matching

use derive_more::{Display, Error};
use hugr::{HugrView, IncomingPort};
use hugr::{Node, Port};
use itertools::Itertools;
use portmatching::{HashMap, Pattern, SinglePatternMatcher, patterns::NoRootFound};
use std::fmt::Debug;

use super::{
    PEdge, PNode,
    matcher::{validate_circuit_edge, validate_circuit_node},
};
use crate::{circuit::Circuit, portmatching::NodeID};

/// A pattern that match a circuit exactly
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct CircuitPattern {
    pub(super) pattern: Pattern<NodeID, PNode, PEdge>,
    /// The input ports
    pub(super) inputs: Vec<Vec<(Node, Port)>>,
    /// The output ports
    pub(super) outputs: Vec<(Node, Port)>,
}

impl CircuitPattern {
    /// The number of edges in the pattern.
    pub fn n_edges(&self) -> usize {
        self.pattern.n_edges()
    }

    /// Construct a pattern from a circuit.
    pub fn try_from_circuit(circuit: &Circuit) -> Result<Self, InvalidPattern> {
        let hugr = circuit.hugr();
        if circuit.num_operations() == 0 {
            return Err(InvalidPattern::EmptyCircuit);
        }
        let mut pattern = Pattern::new();
        for cmd in circuit.commands() {
            let op = cmd.optype().clone();
            pattern.require(cmd.node().into(), op.into());
            for in_offset in 0..cmd.input_count() {
                let in_offset: IncomingPort = in_offset.into();
                let edge_prop = PEdge::try_from_port(cmd.node(), in_offset.into(), circuit)
                    .unwrap_or_else(|e| panic!("Invalid HUGR, {e}"));
                let (prev_node, prev_port) = hugr
                    .linked_outputs(cmd.node(), in_offset)
                    .exactly_one()
                    .unwrap_or_else(|_| {
                        panic!(
                            "{} input port {in_offset} does not have a single neighbour",
                            cmd.node()
                        )
                    });
                let prev_node = match edge_prop {
                    PEdge::InternalEdge { .. } => NodeID::HugrNode(prev_node),
                    PEdge::InputEdge { .. } => NodeID::new_copy(prev_node, prev_port),
                };
                pattern.add_edge(cmd.node().into(), prev_node, edge_prop);
            }
        }
        pattern.set_any_root()?;
        if !pattern.is_valid() {
            return Err(InvalidPattern::NotConnected);
        }
        let [inp, out] = circuit.io_nodes();
        let inp_ports = hugr.signature(inp).unwrap().output_ports();
        let out_ports = hugr.signature(out).unwrap().input_ports();
        let inputs = inp_ports
            .map(|p| hugr.linked_ports(inp, p).collect())
            .collect_vec();
        let outputs = out_ports
            .map(|p| {
                hugr.linked_ports(out, p)
                    .exactly_one()
                    .ok()
                    .expect("invalid circuit")
            })
            .collect_vec();
        if let Some((to_node, to_port)) = inputs.iter().flatten().find(|&&(n, _)| n == out).copied()
        {
            // An input is connected to an output => empty qubit, not allowed.
            let (from_node, from_port): (Node, Port) =
                hugr.linked_ports(to_node, to_port).next().unwrap();
            return Err(InvalidPattern::EmptyWire {
                from_node,
                from_port,
                to_node,
                to_port,
            });
        }

        // This is a consequence of the test above.
        debug_assert!(outputs.iter().all(|(n, _)| *n != inp));
        Ok(Self {
            pattern,
            inputs,
            outputs,
        })
    }

    /// Compute the map from pattern nodes to circuit nodes in `circ`.
    pub fn get_match_map(
        &self,
        root: Node,
        circ: &Circuit<impl HugrView<Node = Node>>,
    ) -> Option<HashMap<Node, Node>> {
        let single_matcher = SinglePatternMatcher::from_pattern(self.pattern.clone());
        single_matcher
            .get_match_map(
                root.into(),
                validate_circuit_node(circ),
                validate_circuit_edge(circ),
            )
            .map(|m| {
                m.into_iter()
                    .filter_map(|(node_p, node_c)| match (node_p, node_c) {
                        (NodeID::HugrNode(node_p), NodeID::HugrNode(node_c)) => {
                            Some((node_p, node_c))
                        }
                        (NodeID::CopyNode(..), NodeID::CopyNode(..)) => None,
                        _ => panic!("Invalid match map"),
                    })
                    .collect()
            })
    }
}

impl Debug for CircuitPattern {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.pattern.fmt(f)?;
        Ok(())
    }
}

/// Conversion error from circuit to pattern.
#[derive(Display, Debug, Error, PartialEq, Eq)]
#[non_exhaustive]
pub enum InvalidPattern {
    /// An empty circuit cannot be a pattern.
    #[display("Empty circuits are not allowed as patterns")]
    EmptyCircuit,
    /// Patterns must be connected circuits.
    #[display("The pattern is not connected")]
    NotConnected,
    /// Patterns cannot include empty wires.
    #[display(
        "The pattern contains an empty wire between {from_node}, {from_port} and {to_node}, {to_port}"
    )]
    EmptyWire {
        /// The source node
        from_node: Node,
        /// The source port
        from_port: Port,
        /// The target node
        to_node: Node,
        /// The target port
        to_port: Port,
    },
}

impl From<NoRootFound> for InvalidPattern {
    fn from(_: NoRootFound) -> Self {
        InvalidPattern::NotConnected
    }
}

#[cfg(test)]
mod tests {

    use std::collections::HashSet;

    use cool_asserts::assert_matches;
    use hugr::builder::{DFGBuilder, Dataflow, DataflowHugr};
    use hugr::extension::prelude::qb_t;
    use hugr::ops::OpType;
    use hugr::types::Signature;

    use crate::extension::rotation::rotation_type;

    use crate::TketOp;
    use crate::utils::build_simple_circuit;

    use super::*;

    fn h_cx() -> Circuit {
        build_simple_circuit(2, |circ| {
            circ.append(TketOp::CX, [0, 1])?;
            circ.append(TketOp::H, [0])?;
            Ok(())
        })
        .unwrap()
    }

    /// A circuit with two rotation gates in sequence, sharing a param
    fn circ_with_copy() -> Circuit {
        let input_t = vec![qb_t(), rotation_type()];
        let output_t = vec![qb_t()];
        let mut h = DFGBuilder::new(Signature::new(input_t, output_t)).unwrap();

        let mut inps = h.input_wires();
        let qb = inps.next().unwrap();
        let f = inps.next().unwrap();

        let res = h.add_dataflow_op(TketOp::Rx, [qb, f]).unwrap();
        let qb = res.outputs().next().unwrap();
        let res = h.add_dataflow_op(TketOp::Rx, [qb, f]).unwrap();
        let qb = res.outputs().next().unwrap();

        h.finish_hugr_with_outputs([qb]).unwrap().into()
    }

    /// A circuit with two rotation gates in parallel, sharing a param
    fn circ_with_copy_disconnected() -> Circuit {
        let input_t = vec![qb_t(), qb_t(), rotation_type()];
        let output_t = vec![qb_t(), qb_t()];
        let mut h = DFGBuilder::new(Signature::new(input_t, output_t)).unwrap();

        let mut inps = h.input_wires();
        let qb1 = inps.next().unwrap();
        let qb2 = inps.next().unwrap();
        let f = inps.next().unwrap();

        let res = h.add_dataflow_op(TketOp::Rx, [qb1, f]).unwrap();
        let qb1 = res.outputs().next().unwrap();
        let res = h.add_dataflow_op(TketOp::Rx, [qb2, f]).unwrap();
        let qb2 = res.outputs().next().unwrap();

        h.finish_hugr_with_outputs([qb1, qb2]).unwrap().into()
    }

    #[test]
    fn construct_pattern() {
        let circ = h_cx();

        let p = CircuitPattern::try_from_circuit(&circ).unwrap();

        let edges: HashSet<_> = p
            .pattern
            .edges()
            .unwrap()
            .iter()
            .map(|e| (e.source.unwrap(), e.target.unwrap()))
            .collect();
        let inp = circ.input_node();
        let cx_gate = NodeID::HugrNode(get_nodes_by_tket_op(&circ, TketOp::CX)[0]);
        let h_gate = NodeID::HugrNode(get_nodes_by_tket_op(&circ, TketOp::H)[0]);
        assert_eq!(
            edges,
            [
                (cx_gate, h_gate),
                (cx_gate, NodeID::new_copy(inp, 0)),
                (cx_gate, NodeID::new_copy(inp, 1)),
            ]
            .into_iter()
            .collect()
        )
    }

    #[test]
    fn disconnected_pattern() {
        let circ = build_simple_circuit(2, |circ| {
            circ.append(TketOp::X, [0])?;
            circ.append(TketOp::T, [1])?;
            Ok(())
        })
        .unwrap();
        assert_eq!(
            CircuitPattern::try_from_circuit(&circ).unwrap_err(),
            InvalidPattern::NotConnected
        );
    }

    #[test]
    fn pattern_with_empty_qubit() {
        let circ = build_simple_circuit(2, |circ| {
            circ.append(TketOp::X, [0])?;
            Ok(())
        })
        .unwrap();
        assert_matches!(
            CircuitPattern::try_from_circuit(&circ).unwrap_err(),
            InvalidPattern::EmptyWire { .. }
        );
    }

    fn get_nodes_by_tket_op(circ: &Circuit, t2_op: TketOp) -> Vec<Node> {
        let t2_op: OpType = t2_op.into();
        circ.hugr()
            .nodes()
            .filter(|n| circ.hugr().get_optype(*n) == &t2_op)
            .collect()
    }

    #[test]
    fn pattern_with_copy() {
        let circ = circ_with_copy();
        let pattern = CircuitPattern::try_from_circuit(&circ).unwrap();
        let edges = pattern.pattern.edges().unwrap();
        let rx_ns = get_nodes_by_tket_op(&circ, TketOp::Rx);
        let inp = circ.input_node();
        for rx_n in rx_ns {
            assert!(edges.iter().any(|e| {
                e.reverse().is_none()
                    && e.source.unwrap() == rx_n.into()
                    && e.target.unwrap() == NodeID::new_copy(inp, 1)
            }));
        }
    }

    #[test]
    fn pattern_with_copy_disconnected() {
        let circ = circ_with_copy_disconnected();
        assert_eq!(
            CircuitPattern::try_from_circuit(&circ).unwrap_err(),
            InvalidPattern::NotConnected
        );
    }
}
