//! This module provides a utility to split a circuit into chunks, and reassemble them afterwards.
//!
//! See [`CircuitChunks`] for more information.

use std::collections::HashMap;
use std::mem;
use std::ops::{Index, IndexMut};

use derive_more::From;
use hugr::builder::{Container, FunctionBuilder};
use hugr::hugr::hugrmut::HugrMut;
use hugr::hugr::views::SiblingSubgraph;
use hugr::hugr::views::sibling_subgraph::TopoConvexChecker;
use hugr::hugr::{HugrError, NodeMetadataMap};
use hugr::ops::OpType;
use hugr::ops::handle::DataflowParentID;
use hugr::types::Signature;
use hugr::{Hugr, HugrView, IncomingPort, Node, OutgoingPort, PortIndex, Wire};
use hugr_core::hugr::internal::{HugrInternals, HugrMutInternals as _};
use itertools::Itertools;
use rayon::iter::{IntoParallelIterator, IntoParallelRefMutIterator, ParallelIterator};
use rayon::slice::ParallelSliceMut;

use crate::Circuit;

use crate::circuit::cost::{CircuitCost, CostDelta};

/// An identifier for the connection between chunks.
///
/// This is based on the wires of the original circuit.
///
/// When reassembling the circuit, the input/output wires of each chunk are
/// re-linked by matching these identifiers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, From)]
pub struct ChunkConnection(Wire);

/// A chunk of a circuit.
#[derive(Debug, Clone)]
pub struct Chunk {
    /// The extracted circuit.
    pub circ: Circuit,
    /// The original wires connected to the input.
    inputs: Vec<ChunkConnection>,
    /// The original wires connected to the output.
    outputs: Vec<ChunkConnection>,
}

impl Chunk {
    /// Extract a chunk from a circuit.
    ///
    /// The chunk is extracted from the input wires to the output wires.
    pub(self) fn extract(
        circ: &Circuit,
        nodes: impl IntoIterator<Item = Node>,
        checker: &TopoConvexChecker<'_, Hugr>,
    ) -> Self {
        let subgraph = SiblingSubgraph::try_from_nodes_with_checker(
            nodes.into_iter().collect_vec(),
            circ.hugr(),
            checker,
        )
        .expect("Failed to define the chunk subgraph");
        let extracted = subgraph.extract_subgraph(circ.hugr(), "Chunk").into();
        // Transform the subgraph's input/output sets into wires that can be
        // matched between different chunks.
        //
        // This requires finding the `Outgoing` port corresponding to each
        // subgraph input.
        let inputs = subgraph
            .incoming_ports()
            .iter()
            .map(|wires| {
                let (inp_node, inp_port) = wires[0];
                let (out_node, out_port) = circ
                    .hugr()
                    .linked_outputs(inp_node, inp_port)
                    .exactly_one()
                    .ok()
                    .unwrap();
                Wire::new(out_node, out_port).into()
            })
            .collect();
        let outputs = subgraph
            .outgoing_ports()
            .iter()
            .map(|&(node, port)| Wire::new(node, port).into())
            .collect();
        Self {
            circ: extracted,
            inputs,
            outputs,
        }
    }

    /// Insert the chunk back into a circuit.
    pub(self) fn insert(
        &self,
        circ: &mut impl HugrMut<Node = Node>,
        root: Node,
    ) -> ChunkInsertResult {
        let chunk = self.circ.hugr();
        let chunk_root = chunk.entrypoint();
        if chunk.children(self.circ.parent()).nth(2).is_none() {
            // The chunk is empty. We don't need to insert anything.
            return self.empty_chunk_insert_result();
        }

        let [chunk_inp, chunk_out] = chunk.get_io(chunk_root).unwrap();
        // Insert the chunk circuit into the original circuit.
        let subgraph =
            SiblingSubgraph::<Node>::try_new_dataflow_subgraph::<_, DataflowParentID>(&chunk)
                .unwrap_or_else(|e| panic!("The chunk circuit is no longer a dataflow graph: {e}"));
        let node_map = circ.insert_subgraph(root, &chunk, &subgraph);

        let mut input_map = HashMap::with_capacity(self.inputs.len());
        let mut output_map = HashMap::with_capacity(self.outputs.len());

        // Translate each connection from the chunk input into a [`ConnectionTarget`].
        //
        // Connections to an inserted node are translated into a [`ConnectionTarget::InsertedNode`].
        // Connections from the input directly into the output become a [`ConnectionTarget::TransitiveConnection`].
        for (&connection, chunk_inp_port) in self.inputs.iter().zip(chunk.node_outputs(chunk_inp)) {
            let connection_targets: Vec<ConnectionTarget> = chunk
                .linked_inputs(chunk_inp, chunk_inp_port)
                .map(|(node, port)| {
                    if node == chunk_out {
                        // This was a direct wire from the chunk input to the output. Use the output's [`ChunkConnection`].
                        let output_connection = self.outputs[port.index()];
                        ConnectionTarget::TransitiveConnection(output_connection)
                    } else {
                        // Translate the original chunk node into the inserted node.
                        ConnectionTarget::InsertedInput(*node_map.get(&node).unwrap(), port)
                    }
                })
                .collect();
            input_map.insert(connection, connection_targets);
        }

        for (&wire, chunk_out_port) in self.outputs.iter().zip(chunk.node_inputs(chunk_out)) {
            let (node, port) = chunk
                .linked_outputs(chunk_out, chunk_out_port)
                .exactly_one()
                .ok()
                .unwrap();
            let target = if node == chunk_inp {
                // This was a direct wire from the chunk output to the input. Use the input's [`ChunkConnection`].
                let input_connection = self.inputs[port.index()];
                ConnectionTarget::TransitiveConnection(input_connection)
            } else {
                // Translate the original chunk node into the inserted node.
                ConnectionTarget::InsertedOutput(*node_map.get(&node).unwrap(), port)
            };
            output_map.insert(wire, target);
        }

        ChunkInsertResult {
            incoming_connections: input_map,
            outgoing_connections: output_map,
        }
    }

    /// Compute the return value for `insert` when the chunk is empty (Subgraph would throw an error in this case).
    ///
    /// TODO: Support empty Subgraphs in Hugr?
    fn empty_chunk_insert_result(&self) -> ChunkInsertResult {
        let hugr = self.circ.hugr();
        let [chunk_inp, chunk_out] = self.circ.io_nodes();
        let mut input_map = HashMap::with_capacity(self.inputs.len());
        let mut output_map = HashMap::with_capacity(self.outputs.len());

        for (&connection, chunk_inp_port) in self.inputs.iter().zip(hugr.node_outputs(chunk_inp)) {
            let connection_targets: Vec<ConnectionTarget> = hugr
                .linked_ports(chunk_inp, chunk_inp_port)
                .map(|(node, port)| {
                    assert_eq!(node, chunk_out);
                    let output_connection = self.outputs[port.index()];
                    ConnectionTarget::TransitiveConnection(output_connection)
                })
                .collect();
            input_map.insert(connection, connection_targets);
        }

        for (&wire, chunk_out_port) in self.outputs.iter().zip(hugr.node_inputs(chunk_out)) {
            let (node, port) = hugr
                .linked_ports(chunk_out, chunk_out_port)
                .exactly_one()
                .ok()
                .unwrap();
            assert_eq!(node, chunk_inp);
            let input_connection = self.inputs[port.index()];
            output_map.insert(
                wire,
                ConnectionTarget::TransitiveConnection(input_connection),
            );
        }

        ChunkInsertResult {
            incoming_connections: input_map,
            outgoing_connections: output_map,
        }
    }
}

/// A map from the original input/output [`ChunkConnection`]s to an inserted chunk's inputs and outputs.
#[derive(Debug, Clone)]
struct ChunkInsertResult {
    /// A map from incoming connections to a chunk, to the new node and incoming port targets.
    ///
    /// A chunk may specify multiple targets to be connected to a single incoming `ChunkConnection`.
    pub incoming_connections: HashMap<ChunkConnection, Vec<ConnectionTarget>>,
    /// A map from outgoing connections from a chunk, to the new node and outgoing port target.
    pub outgoing_connections: HashMap<ChunkConnection, ConnectionTarget>,
}

/// The target of a chunk connection in a reassembled circuit.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum ConnectionTarget {
    /// The target is a chunk node's input.
    InsertedInput(Node, IncomingPort),
    /// The target is a chunk node's output.
    InsertedOutput(Node, OutgoingPort),
    /// The link goes directly to the opposite boundary, without an intermediary
    /// node.
    TransitiveConnection(ChunkConnection),
}

/// An utility for splitting a circuit into chunks, and reassembling them
/// afterwards.
///
/// Circuits can be split into [`CircuitChunks`] with [`CircuitChunks::split`]
/// or [`CircuitChunks::split_with_cost`], and reassembled with
/// [`CircuitChunks::reassemble`].
#[derive(Debug, Clone)]
pub struct CircuitChunks {
    /// The original circuit's signature.
    signature: Signature,

    /// The original circuit's root metadata.
    root_meta: Option<NodeMetadataMap>,

    /// The original circuit's inputs.
    input_connections: Vec<ChunkConnection>,

    /// The original circuit's outputs.
    output_connections: Vec<ChunkConnection>,

    /// The split circuits.
    chunks: Vec<Chunk>,
}

impl CircuitChunks {
    /// Split a circuit into chunks.
    ///
    /// The circuit is split into chunks of at most `max_size` gates.
    pub fn split(circ: &Circuit, max_size: usize) -> Self {
        Self::split_with_cost(circ, max_size.saturating_sub(1), |_| 1)
    }

    /// Split a circuit into chunks.
    ///
    /// The circuit is split into chunks of at most `max_cost`, using the provided cost function.
    pub fn split_with_cost<C: CircuitCost>(
        circ: &Circuit,
        max_cost: C,
        op_cost: impl Fn(&OpType) -> C,
    ) -> Self {
        let hugr = circ.hugr();
        let root_meta = hugr.node_metadata_map(circ.parent()).clone();
        let signature = circ.circuit_signature().clone();

        let [circ_input, circ_output] = circ.io_nodes();
        let input_connections = hugr
            .node_outputs(circ_input)
            .map(|port| Wire::new(circ_input, port).into())
            .collect();
        let output_connections = hugr
            .node_inputs(circ_output)
            .flat_map(|p| hugr.linked_outputs(circ_output, p))
            .map(|(n, p)| Wire::new(n, p).into())
            .collect();

        let mut chunks = Vec::new();
        let convex_checker = TopoConvexChecker::new(circ.hugr(), circ.parent());
        let mut running_cost = C::default();
        let mut current_group = 0;
        for (_, commands) in &circ.commands().map(|cmd| cmd.node()).chunk_by(|&node| {
            let new_cost = running_cost.clone() + op_cost(hugr.get_optype(node));
            if new_cost.sub_cost(&max_cost).as_isize() > 0 {
                running_cost = C::default();
                current_group += 1;
            } else {
                running_cost = new_cost;
            }
            current_group
        }) {
            chunks.push(Chunk::extract(circ, commands, &convex_checker));
        }
        Self {
            signature: signature.into_owned(),
            root_meta: Some(root_meta),
            input_connections,
            output_connections,
            chunks,
        }
    }

    /// Reassemble the chunks into a circuit.
    pub fn reassemble(self) -> Result<Circuit, HugrError> {
        let name = self
            .root_meta
            .as_ref()
            .and_then(|map| map.get("name"))
            .and_then(|s| s.as_str())
            .unwrap_or("");

        let mut builder = FunctionBuilder::new(name, self.signature).unwrap();
        // Take the unfinished Hugr from the builder, to avoid unnecessary
        // validation checks that require connecting the inputs an outputs.
        let mut reassembled = mem::take(builder.hugr_mut());
        let root = reassembled.entrypoint();
        let [reassembled_input, reassembled_output] = reassembled.get_io(root).unwrap();

        // The chunks input and outputs are each identified with a
        // [`ChunkConnection`]. We collect both sides first, and rewire them
        // after the chunks have been inserted.
        let mut sources: HashMap<ChunkConnection, (Node, OutgoingPort)> = HashMap::new();
        let mut targets: HashMap<ChunkConnection, Vec<(Node, IncomingPort)>> = HashMap::new();

        // A map for `ChunkConnection`s that have been merged into another (due
        // to identity wires in the updated chunks).
        //
        // Maps each `ChunkConnection` to the `ChunkConnection` it has been
        // merged into.
        //
        // This is a poor man's Union Find. Since we traverse the chunks in
        // order, we can assume that already seen connections will not be merged
        // again.
        let mut transitive_connections: HashMap<ChunkConnection, ChunkConnection> = HashMap::new();
        let get_merged_connection = |transitive_connections: &HashMap<_, _>, connection| {
            transitive_connections
                .get(&connection)
                .copied()
                .unwrap_or(connection)
        };

        // Register the source ports for the `ChunkConnections` in the circuit input.
        for (&connection, port) in self
            .input_connections
            .iter()
            .zip(reassembled.node_outputs(reassembled_input))
        {
            sources.insert(connection, (reassembled_input, port));
        }

        for chunk in self.chunks {
            // Insert the chunk circuit without its input/output nodes.
            let ChunkInsertResult {
                incoming_connections,
                outgoing_connections,
            } = chunk.insert(&mut reassembled, root);
            // Associate the chunk's inserted inputs and outputs to the
            // `ChunkConnection` identifiers, so we can re-connect everything
            // afterwards.
            //
            // The chunk may return `ConnectionTarget::TransitiveConnection`s to
            // indicate that a `ChunkConnection` has been merged into another
            // (due to an identity wire).
            for (connection, conn_target) in outgoing_connections {
                match conn_target {
                    ConnectionTarget::InsertedOutput(node, port) => {
                        // The output of a chunk always has fresh `ChunkConnection`s.
                        sources.insert(connection, (node, port));
                    }
                    ConnectionTarget::TransitiveConnection(merged_connection) => {
                        // The output's `ChunkConnection` has been merged into one of the input's.
                        let merged_connection =
                            get_merged_connection(&transitive_connections, merged_connection);
                        transitive_connections.insert(connection, merged_connection);
                    }
                    _ => panic!("Unexpected connection target"),
                }
            }
            for (connection, conn_targets) in incoming_connections {
                // The connection in the chunk's input may have been merged into a earlier one.
                let connection = get_merged_connection(&transitive_connections, connection);
                for tgt in conn_targets {
                    match tgt {
                        ConnectionTarget::InsertedInput(node, port) => {
                            targets.entry(connection).or_default().push((node, port));
                        }
                        ConnectionTarget::TransitiveConnection(_merged_connection) => {
                            // The merge has been registered when scanning the
                            // outgoing_connections, so we don't need to do
                            // anything here.
                        }
                        _ => panic!("Unexpected connection target"),
                    }
                }
            }
        }

        // Register the target ports for the `ChunkConnections` into the circuit output.
        for (&connection, port) in self
            .output_connections
            .iter()
            .zip(reassembled.node_inputs(reassembled_output))
        {
            // The connection in the chunk's input may have been merged into a earlier one.
            let connection = get_merged_connection(&transitive_connections, connection);
            targets
                .entry(connection)
                .or_default()
                .push((reassembled_output, port));
        }

        // Reconnect the different chunks.
        for (connection, (source, source_port)) in sources {
            let Some(tgts) = targets.remove(&connection) else {
                continue;
            };
            for (target, target_port) in tgts {
                reassembled.connect(source, source_port, target, target_port);
            }
        }

        *reassembled.node_metadata_map_mut(root) = self.root_meta.unwrap_or_default();
        Ok(reassembled.into())
    }

    /// Returns a list of references to the split circuits.
    pub fn iter(&self) -> impl Iterator<Item = &Circuit> {
        self.chunks.iter().map(|chunk| &chunk.circ)
    }

    /// Returns a list of references to the split circuits.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut Circuit> {
        self.chunks.iter_mut().map(|chunk| &mut chunk.circ)
    }

    /// Returns the number of chunks.
    pub fn len(&self) -> usize {
        self.chunks.len()
    }

    /// Returns `true` if there are no chunks.
    pub fn is_empty(&self) -> bool {
        self.chunks.is_empty()
    }

    /// Supports implementation of rayon::iter::IntoParallelRefMutIterator
    fn par_iter_mut(
        &mut self,
    ) -> rayon::iter::Map<
        rayon::slice::IterMut<'_, Chunk>,
        for<'a> fn(&'a mut Chunk) -> &'a mut Circuit,
    > {
        self.chunks
            .as_parallel_slice_mut()
            .into_par_iter()
            .map(|chunk| &mut chunk.circ)
    }
}

impl Index<usize> for CircuitChunks {
    type Output = Circuit;

    fn index(&self, index: usize) -> &Self::Output {
        &self.chunks[index].circ
    }
}

impl IndexMut<usize> for CircuitChunks {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.chunks[index].circ
    }
}

impl<'data> IntoParallelRefMutIterator<'data> for CircuitChunks {
    type Item = &'data mut Circuit;
    type Iter = rayon::iter::Map<
        rayon::slice::IterMut<'data, Chunk>,
        for<'a> fn(&'a mut Chunk) -> &'a mut Circuit,
    >;

    fn par_iter_mut(&'data mut self) -> Self::Iter {
        self.par_iter_mut()
    }
}

#[cfg(test)]
mod test {
    use crate::circuit::CircuitHash;

    use crate::TketOp;
    use crate::utils::build_simple_circuit;

    use super::*;

    #[test]
    fn split_reassemble() {
        let circ = build_simple_circuit(2, |circ| {
            circ.append(TketOp::H, [0])?;
            circ.append(TketOp::CX, [0, 1])?;
            circ.append(TketOp::T, [1])?;
            circ.append(TketOp::H, [0])?;
            circ.append(TketOp::CX, [0, 1])?;
            circ.append(TketOp::H, [0])?;
            circ.append(TketOp::CX, [0, 1])?;
            Ok(())
        })
        .unwrap();

        let chunks = CircuitChunks::split(&circ, 3);

        assert_eq!(chunks.len(), 3);

        let mut reassembled = chunks.reassemble().unwrap();

        reassembled.hugr_mut().validate().unwrap();
        assert_eq!(
            circ.circuit_hash(circ.parent()),
            reassembled.circuit_hash(reassembled.parent())
        );
    }

    #[test]
    fn reassemble_empty() {
        let circ = build_simple_circuit(3, |circ| {
            circ.append(TketOp::CX, [0, 1])?;
            circ.append(TketOp::H, [0])?;
            circ.append(TketOp::H, [1])?;
            Ok(())
        })
        .unwrap();

        let circ_1q_id = build_simple_circuit(1, |_| Ok(())).unwrap();
        let circ_2q_id_h = build_simple_circuit(2, |circ| {
            circ.append(TketOp::H, [0])?;
            Ok(())
        })
        .unwrap();

        let mut chunks = CircuitChunks::split(&circ, 1);

        // Replace the Hs with identities, and the CX with an identity and an H gate.
        chunks[0] = circ_2q_id_h.clone();
        chunks[1] = circ_1q_id.clone();
        chunks[2] = circ_1q_id.clone();

        let mut reassembled = chunks.reassemble().unwrap();

        reassembled.hugr_mut().validate().unwrap();

        assert_eq!(reassembled.commands().count(), 1);
        let h = reassembled.commands().next().unwrap().node();

        let [inp, out] = reassembled.io_nodes();
        assert_eq!(
            &reassembled.hugr().output_neighbours(inp).collect_vec(),
            &[h, out, out]
        );
    }
}
