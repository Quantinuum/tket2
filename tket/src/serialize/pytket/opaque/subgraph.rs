//! Opaque subgraph definition.

use hugr::ops::OpTrait;
use itertools::{Either, Itertools};

use std::collections::BTreeSet;

use crate::serialize::pytket::PytketEncodeError;
use hugr::core::HugrNode;
use hugr::hugr::views::SiblingSubgraph;
use hugr::hugr::views::sibling_subgraph::InvalidSubgraph;
use hugr::types::Signature;
use hugr::{Direction, Hugr, HugrView, IncomingPort, OutgoingPort};

/// A subgraph of nodes in the Hugr that could not be encoded as TKET1
/// operations.
///
/// This is a simpler version of [`SiblingSubgraph`] that
/// - Always maps boundary ports to exactly one node port.
/// - Does not require a convex checker to verify convexity (since we always create regions using a toposort).
/// - Allows non-value edges at its boundaries, which can be left unmodified by the encoder/decoder.
/// - Keeps a flag indicating if it can be represented as a valid [`SiblingSubgraph`].
#[derive(Debug, Clone)]
pub struct OpaqueSubgraph<N> {
    /// The nodes in the subgraph.
    nodes: BTreeSet<N>,
    /// The incoming ports of the subgraph.
    incoming_ports: Vec<(N, IncomingPort)>,
    /// The outgoing ports of the subgraph.
    outgoing_ports: Vec<(N, OutgoingPort)>,
    /// The signature of the subgraph.
    signature: Signature,
    /// The region containing the subgraph.
    region: N,
    /// Whether the subgraph can be represented as a [`SiblingSubgraph`]
    /// with no external edges.
    ///
    /// Some cases where this is not possible are:
    /// - Having non-local edges.
    /// - Having const edges to global definitions.
    /// - Having order edges to nodes outside the subgraph.
    /// - Calling global functions.
    sibling_subgraph_compatible: bool,
}

impl<N: HugrNode> OpaqueSubgraph<N> {
    /// Create a new [`UnsupportedSubgraph`].
    pub(in crate::serialize::pytket) fn try_from_nodes(
        nodes: BTreeSet<N>,
        hugr: &impl HugrView<Node = N>,
    ) -> Result<Self, PytketEncodeError<N>> {
        let region = nodes
            .first()
            .and_then(|n| hugr.get_parent(*n))
            .unwrap_or_else(|| hugr.entrypoint());

        // Traverse the nodes, collecting `EdgeKind::Value` boundary ports that connect to other nodes in the _same region_.
        // Ignores all other ports.
        let mut incoming_ports = Vec::new();
        let mut outgoing_ports = Vec::new();
        let mut input_types = Vec::new();
        let mut output_types = Vec::new();
        let mut sibling_subgraph_compatible = true;

        for &node in &nodes {
            let op = hugr.get_optype(node);
            let Some(signature) = op.dataflow_signature() else {
                continue;
            };
            // Check the value ports for boundary edges.
            let mut has_nonlocal_boundary = false;
            for port in signature
                .ports(Direction::Incoming)
                .chain(signature.ports(Direction::Outgoing))
            {
                let ty = signature.port_type(port).unwrap();
                // If it's a value port to another node in the same region but outside the set, add it to the subgraph.
                let mut is_local_boundary = false;
                for (n, _) in hugr.linked_ports(node, port) {
                    if nodes.contains(&n) {
                        continue;
                    }
                    match hugr.get_parent(n) == Some(region) {
                        true => is_local_boundary = true,
                        false => has_nonlocal_boundary = true,
                    }
                    if is_local_boundary && has_nonlocal_boundary {
                        break;
                    }
                }
                if is_local_boundary {
                    match port.as_directed() {
                        Either::Left(inc) => {
                            incoming_ports.push((node, inc));
                            input_types.push(ty.clone());
                        }
                        Either::Right(out) => {
                            outgoing_ports.push((node, out));
                            output_types.push(ty.clone());
                        }
                    }
                }
            }
            // If the node is a region parent, it cannot be contained in a `SiblingSubgraph`.
            let is_region_parent = hugr.first_child(node).is_some();
            // If the node has static ports or _other_ ports that connect outside the set, it cannot be contained in a `SiblingSubgraph`.
            let non_value_boundary = op
                .static_port(Direction::Incoming)
                .iter()
                .chain(op.static_port(Direction::Outgoing).iter())
                .chain(op.other_port(Direction::Incoming).iter())
                .chain(op.other_port(Direction::Outgoing).iter())
                .any(|&p| hugr.linked_ports(node, p).any(|(n, _)| !nodes.contains(&n)));

            sibling_subgraph_compatible &=
                !has_nonlocal_boundary && !is_region_parent && !non_value_boundary;
        }
        let signature = Signature::new(input_types, output_types);

        Ok(Self {
            nodes,
            incoming_ports,
            outgoing_ports,
            signature,
            region,
            sibling_subgraph_compatible,
        })
    }

    /// Returns the nodes in the subgraph.
    pub fn nodes(&self) -> &BTreeSet<N> {
        &self.nodes
    }

    /// Returns the incoming ports of the subgraph.
    pub fn incoming_ports(&self) -> &[(N, IncomingPort)] {
        &self.incoming_ports
    }

    /// Returns the outgoing ports of the subgraph.
    pub fn outgoing_ports(&self) -> &[(N, OutgoingPort)] {
        &self.outgoing_ports
    }

    /// Returns the signature of the subgraph.
    pub fn signature(&self) -> &Signature {
        &self.signature
    }

    /// Returns the region containing the subgraph.
    pub fn region(&self) -> N {
        self.region
    }

    /// Returns whether the subgraph can be represented as a [`SiblingSubgraph`]
    /// with no external edges.
    ///
    /// Some cases where this is not possible are:
    /// - Having non-local edges.
    /// - Having const edges to global definitions.
    /// - Having order edges to nodes outside the subgraph.
    /// - Calling global functions.
    pub fn is_sibling_subgraph_compatible(&self) -> bool {
        self.sibling_subgraph_compatible
    }

    /// Extract the subgraph as a standalone Hugr.
    ///
    /// Return an error if the subgraph cannot be represented as a
    /// [`SiblingSubgraph`]. See
    /// [`OpaqueSubgraph::is_sibling_subgraph_compatible`] for more details.
    pub fn extract_subgraph(
        &self,
        hugr: &impl HugrView<Node = N>,
    ) -> Result<Hugr, InvalidSubgraph<N>> {
        let nodes = self.nodes().iter().cloned().collect_vec();
        let subgraph = SiblingSubgraph::try_from_nodes(nodes, hugr).unwrap();
        Ok(subgraph.extract_subgraph(hugr, ""))
    }
}
