//! Temporary structure linking an encoded pytket circuit and subcircuits, with their originating HUGR.

use std::collections::{HashMap, VecDeque};
use std::ops::{Index, IndexMut};
use std::sync::Arc;

use hugr::core::{HugrNode, IncomingPort, OutgoingPort};
use hugr::hugr::hugrmut::HugrMut;
use hugr::ops::handle::NodeHandle;
use hugr::ops::{OpParent, OpTag, OpTrait};
use hugr::types::EdgeKind;
use hugr::{Hugr, HugrView, Node};
use hugr_core::hugr::internal::HugrMutInternals;
use itertools::Itertools;
use rayon::iter::{IntoParallelRefIterator, IntoParallelRefMutIterator, ParallelIterator};
use tket_json_rs::circuit_json::{Command as PytketCommand, SerialCircuit};

use crate::serialize::pytket::decoder::PytketDecoderContext;
use crate::serialize::pytket::opaque::SubgraphId;
use crate::serialize::pytket::{
    default_decoder_config, default_encoder_config, DecodeInsertionTarget, DecodeOptions,
    EncodeOptions, PytketDecodeError, PytketDecodeErrorInner, PytketDecoderConfig,
    PytketEncodeError, PytketEncoderContext,
};
use crate::Circuit;

use super::opaque::OpaqueSubgraphs;

/// An encoded pytket circuit that may be linked to an existing HUGR.
///
/// Tracks correspondences between references to the HUGR in the encoded
/// circuit, so we can reconstruct the HUGR if needed.
///
/// Serial circuits in this structure are intended to be transient, only alive
/// while this structure is in memory. To obtain a fully standalone pytket
/// circuit that can be used independently, and stored permanently, use
/// [`EncodedCircuit::new_standalone`] or call
/// [`EncodedCircuit::ensure_standalone`].
#[derive(Debug, Clone)]
pub struct EncodedCircuit<Node: HugrNode> {
    /// Circuits encoded from independent dataflow regions in the HUGR.
    ///
    /// These correspond to sections of the HUGR that can be optimized
    /// independently.
    circuits: HashMap<Node, EncodedCircuitInfo>,
    /// Sets of subgraphs in the HUGR that have been encoded as opaque barriers
    /// in the pytket circuit.
    ///
    /// Subcircuits are identified in the barrier metadata by their ID in this
    /// vector. See [`SubgraphId`].
    opaque_subgraphs: OpaqueSubgraphs<Node>,
}

/// Information stored about a pytket circuit encoded from a HUGR region.
#[derive(Debug, Clone)]
pub(super) struct EncodedCircuitInfo {
    /// The serial circuit encoded from the region.
    pub serial_circuit: SerialCircuit,
    /// Information about any unsupported nodes in the region that could not be encoded as a pytket command.
    pub additional_nodes_and_wires: AdditionalNodesAndWires,
    /// List of parameters in the pytket circuit in the order they appear in the
    /// hugr input.
    ///
    /// We require this to correctly reconstruct the input order in the reassembled hugr,
    /// since parameters in pytket are unordered.
    pub input_params: Vec<String>,
    /// List of output parameter expressions found at the end of the encoded region.
    //
    // TODO: The decoder does not currently connect these, everything that
    // _produces_ a parameter gets included in unsupported subgraphs instead.
    pub output_params: Vec<String>,
}

/// Nodes and edges from the original region that could not be encoded into the
/// pytket circuit, as they cannot be attached to a pytket command.
#[derive(Debug, Clone)]
pub(super) struct AdditionalNodesAndWires {
    /// Subgraphs of the region that could not be encoded as a pytket commands,
    /// and have no qubit/bits in their boundary that could be used to emit an
    /// opaque barrier command in the [`serial_circuit`].
    pub additional_subgraphs: Vec<AdditionalSubgraph>,
    /// List of wires that directly connected the input node to the output node in the encoded region,
    /// and were not encoded in [`serial_circuit`].
    ///
    /// We just store the input nodes's output port and output node's input port here.
    pub straight_through_wires: Vec<StraightThroughWire>,
}

/// A subgraph of the encoded circuit that could not be associated to any qubit or bit register in the pytket circuit.
#[derive(Debug, Clone)]
pub(super) struct AdditionalSubgraph {
    /// The subgraph of the region that could not be encoded as a pytket command,
    /// and has no qubit/bits in its boundary that could be used to emit an opaque
    /// barrier command in the [`serial_circuit`].
    pub id: SubgraphId,
    /// Parameter expression inputs to the `subgraph`.
    pub params: Vec<String>,
}

/// A wire stored in the [`EncodedCircuitInfo`] that directly connected the
/// input node to the output node in the encoded region, and was not encoded in
/// the pytket circuit.
#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq)]
pub(super) struct StraightThroughWire {
    /// Source port of the wire in the input node.
    pub input_source: OutgoingPort,
    /// Target port of the wire in the output node.
    pub output_target: IncomingPort,
}

impl EncodedCircuit<Node> {
    /// Encode a HugrView into a [`EncodedCircuit`].
    ///
    /// The HUGR's entrypoint must be a dataflow region that will be encoded as
    /// the main circuit. Additional circuits may be encoded if
    /// [`EncodeOptions::encode_subcircuits`] is set.
    ///
    /// The circuit may contain opaque barriers referencing subgraphs in the
    /// original HUGR. To obtain a fully standalone pytket circuit that can be
    /// used independently, and stored permanently, use
    /// [`EncodedCircuit::new_standalone`] or call
    /// [`EncodedCircuit::ensure_standalone`].
    ///
    /// See [`EncodeOptions`] for the options used by the encoder.
    pub fn new<H: AsRef<Hugr> + AsMut<Hugr> + HugrView<Node = Node>>(
        circuit: &Circuit<H>,
        options: EncodeOptions<H>,
    ) -> Result<Self, PytketEncodeError<H::Node>> {
        let mut enc = Self {
            circuits: HashMap::new(),
            opaque_subgraphs: OpaqueSubgraphs::new(0),
        };

        enc.encode_circuits(circuit, options)?;

        Ok(enc)
    }

    /// Reassemble the encoded circuits into the original [`Hugr`], replacing
    /// the existing regions that were encoded in `self` as subcircuits.
    ///
    ///
    ///
    /// # Arguments
    ///
    /// - `hugr`: The [`Hugr`] to reassemble the circuits in. This should
    ///   contain all the original subgraphs referenced as external opaque
    ///   barriers in the pytket circuit.
    /// - `config`: The set of extension decoders used to convert the pytket
    ///   commands into HUGR operations.
    ///
    /// # Returns
    ///
    /// A list of region parents whose contents were replaced by the updated
    /// circuits.
    ///
    /// # Errors
    ///
    /// Returns a [`PytketDecodeErrorInner::IncompatibleTargetRegion`] error if
    /// the source region of an encoded circuit does not match the circuit
    /// signature. This is likely caused by the original hugr being modified
    /// since the circuit was encoded.
    ///
    /// Returns an error if a circuit being decoded is invalid. See
    /// [`PytketDecodeErrorInner`][super::error::PytketDecodeErrorInner] for
    /// more details.
    pub fn reassemble_inplace(
        &self,
        hugr: &mut Hugr,
        config: Option<Arc<PytketDecoderConfig>>,
    ) -> Result<Vec<hugr::Node>, PytketDecodeError> {
        let options = DecodeOptions::new().with_config(
            config
                .clone()
                .unwrap_or_else(|| Arc::new(default_decoder_config())),
        );

        for (&original_region, encoded) in &self.circuits {
            // Decode the circuit into a temporary function node.
            let Some(signature) = hugr.get_optype(original_region).inner_function_type() else {
                return Err(PytketDecodeErrorInner::IncompatibleTargetRegion {
                    region: original_region,
                    new_optype: hugr.get_optype(original_region).clone(),
                }
                .wrap());
            };
            let options = options
                .clone()
                .with_signature(signature.into_owned())
                .with_input_params(encoded.input_params.iter().cloned());

            // Run the decoder, generating a new function with the extracted definition.
            //
            // Unsupported subgraphs of the original region will be transplanted here.
            let mut decoder = PytketDecoderContext::new(
                &encoded.serial_circuit,
                hugr,
                DecodeInsertionTarget::Function { fn_name: None },
                options,
                Some(&self.opaque_subgraphs),
            )?;
            decoder.run_decoder(
                &encoded.serial_circuit.commands,
                Some(&encoded.additional_nodes_and_wires),
            )?;
            let decoded_node = decoder.finish(&encoded.output_params)?.node();

            // Move any non-local edges from originating from the old input node.
            let old_input = hugr.get_io(original_region).unwrap()[0];
            let input_optype = hugr.get_optype(old_input).clone();
            let new_input = hugr.get_io(decoded_node).unwrap()[0];
            for src_port in hugr.node_outputs(old_input).collect_vec() {
                for (tgt_node, tgt_port) in hugr.linked_inputs(old_input, src_port).collect_vec() {
                    let tgt_parent = hugr.get_parent(tgt_node);
                    let is_local_wire = tgt_parent == Some(original_region);
                    let is_value_wire =
                        matches!(input_optype.port_kind(src_port), Some(EdgeKind::Value(_)));
                    let wire_to_decoded_region = tgt_parent == Some(decoded_node);
                    // Ignore local wires, as all nodes will be deleted.
                    // Also ignore value wires to the newly decoded region,
                    // as they come from transplanted opaque subgraphs that already
                    // re-connected their inputs.
                    if !(is_local_wire || (is_value_wire && wire_to_decoded_region)) {
                        hugr.connect(new_input, src_port, tgt_node, tgt_port);
                    }
                }
            }

            // Replace the region with the decoded function.
            //
            // All descendant nodes that were re-used by the decoded circuit got
            // re-parented at this point, so we can just do a full clear here.
            while let Some(child) = hugr.first_child(original_region) {
                hugr.remove_subtree(child);
            }
            while let Some(child) = hugr.first_child(decoded_node) {
                hugr.set_parent(child, original_region);
            }
            hugr.remove_node(decoded_node);
        }
        Ok(self.circuits.keys().copied().collect_vec())
    }
}

impl<Node: HugrNode> EncodedCircuit<Node> {
    /// Encode a HugrView into a [`EncodedCircuit`].
    ///
    /// The HUGR's entrypoint must be a dataflow region that will be encoded as
    /// the main circuit. Additional circuits may be encoded if
    /// [`EncodeOptions::encode_subcircuits`] is set.
    ///
    /// The circuit may contain opaque barriers encoding opaque subgraphs in the
    /// original HUGR. These are encoded completely as Hugr envelopes in the
    /// barrier operations' metadata.
    ///
    /// When encoding a `Hugr`, prefer using [`EncodedCircuit::new`] instead to
    /// avoid unnecessary copying of the opaque subgraphs and preserve non-local
    /// edges (like function references).
    ///
    /// See [`EncodeOptions`] for the options used by the encoder.
    pub fn new_standalone<H: HugrView<Node = Node>>(
        circuit: &Circuit<H>,
        options: EncodeOptions<H>,
    ) -> Result<Self, PytketEncodeError<H::Node>> {
        let mut enc = Self {
            circuits: HashMap::new(),
            opaque_subgraphs: OpaqueSubgraphs::new(0),
        };

        enc.encode_circuits(circuit, options)?;
        enc.ensure_standalone(circuit.hugr())?;

        Ok(enc)
    }

    /// Encode the circuits for the entrypoint region to the hugr, and if [`EncodeOptions::encode_subcircuits`] is set,
    /// for the descendants of any unsupported node in the main circuit.
    ///
    /// Auxiliary method for [`Self::new`] and [`Self::new_standalone`].
    ///
    // TODO: Add an option in [EncodeOptions] to run the subcircuit encoders in parallel.
    fn encode_circuits<H: HugrView<Node = Node>>(
        &mut self,
        circuit: &Circuit<H>,
        mut options: EncodeOptions<H>,
    ) -> Result<(), PytketEncodeError<H::Node>> {
        // List of nodes to check for subcircuits.
        //
        // These may be either dataflow region parents that we can encode, or
        // any node with children that we should traverse recursively until we
        // find a dataflow region.
        let mut candidate_nodes = VecDeque::from([circuit.parent()]);
        let config = options
            .config
            .take()
            .unwrap_or_else(|| Arc::new(default_encoder_config()));

        // Add a node to the list of candidates if it's a region parent.
        let add_candidate = |node: H::Node, queue: &mut VecDeque<H::Node>| {
            if circuit.hugr().first_child(node).is_some() {
                queue.push_back(node);
            }
        };

        // Add all container nodes from the new opaque subgraphs to the list of
        // candidates.
        let mut encoder_count = 0;
        while let Some(node) = candidate_nodes.pop_front() {
            let node_op = circuit.hugr().get_optype(node);
            if !OpTag::DataflowParent.is_superset(node_op.tag()) {
                for child in circuit.hugr().children(node) {
                    add_candidate(child, &mut candidate_nodes);
                }
                continue;
            }
            encoder_count += 1;
            let opaque_subgraphs = OpaqueSubgraphs::new(encoder_count);
            let mut encoder: PytketEncoderContext<H> =
                PytketEncoderContext::new(circuit, node, opaque_subgraphs, config.clone())?;
            encoder.run_encoder(circuit, node)?;
            let (encoded, opaque_subgraphs) = encoder.finish(circuit, node)?;

            if options.encode_subcircuits {
                for subgraph_id in opaque_subgraphs.ids() {
                    for &node in opaque_subgraphs[subgraph_id].nodes() {
                        add_candidate(node, &mut candidate_nodes);
                    }
                }
            }

            self.circuits.insert(node, encoded);
            self.opaque_subgraphs.merge(opaque_subgraphs);
        }

        Ok(())
    }

    /// Reassemble the encoded circuits into a new [`Hugr`], containing a
    /// function with the decoded circuit originally corresponding to `region`.
    ///
    /// # Arguments
    ///
    /// - `fn_name`: The name of the function to create. If `None`, we will use
    ///   the name of the circuit, or "main" if the circuit has no name.
    /// - `options`: The options for the decoder.
    ///
    /// # Errors
    ///
    /// Returns a [`PytketDecodeErrorInner::NotAnEncodedRegion`] error if
    /// there is no encoded circuit for `region`.
    pub fn reassemble(
        &self,
        region: Node,
        fn_name: Option<String>,
        options: DecodeOptions,
    ) -> Result<Hugr, PytketDecodeError> {
        if !self.contains_circuit(region) {
            return Err(PytketDecodeErrorInner::NotAnEncodedRegion {
                region: region.to_string(),
            }
            .wrap());
        }
        let serial_circuit = &self[region];

        if self.len() > 1 {
            unimplemented!(
                "Reassembling an `EncodedCircuit` with nested subcircuits is not yet implemented."
            );
        };

        let mut hugr = Hugr::new();
        let target = DecodeInsertionTarget::Function { fn_name };

        let mut decoder =
            PytketDecoderContext::new(serial_circuit, &mut hugr, target, options, None)?;
        decoder.run_decoder(&serial_circuit.commands, None)?;
        decoder.finish(&[])?;
        Ok(hugr)
    }

    /// Ensure that none of the encoded circuits contain references to opaque subgraphs in the original HUGR.
    ///
    /// Traverses the commands in the encoded circuits and replaces
    /// [`OpaqueSubgraphPayload::External`][super::opaque::OpaqueSubgraphPayload::External]
    /// payloads in opaque barriers with inline payloads.
    ///
    /// Barrier operation with unrecognised payloads will be ignored.
    pub fn ensure_standalone(
        &mut self,
        hugr: &impl HugrView<Node = Node>,
    ) -> Result<(), PytketEncodeError<Node>> {
        /// Replace references to the `EncodedCircuit` context from the circuit commands.
        ///
        /// Replaces [`OpaqueSubgraphPayloadType::External`][super::opaque::OpaqueSubgraphPayloadType::External]
        /// pointers in opaque barriers with inline payloads.
        fn make_commands_standalone<N: HugrNode>(
            commands: &mut [PytketCommand],
            subgraphs: &OpaqueSubgraphs<N>,
            hugr: &impl HugrView<Node = N>,
        ) -> Result<(), PytketEncodeError<N>> {
            for command in commands.iter_mut() {
                subgraphs.inline_if_payload(command, hugr)?;

                if let Some(tket_json_rs::opbox::OpBox::CircBox { circuit, .. }) =
                    &mut command.op.op_box
                {
                    make_commands_standalone(&mut circuit.commands, subgraphs, hugr)?;
                }
            }
            Ok(())
        }

        for encoded in self.circuits.values_mut() {
            make_commands_standalone(
                &mut encoded.serial_circuit.commands,
                &self.opaque_subgraphs,
                hugr,
            )?;
        }
        Ok(())
    }

    /// Returns `true` if there is an encoded pytket circuit for the given region.
    pub fn contains_circuit(&self, region: Node) -> bool {
        self.circuits.contains_key(&region)
    }

    /// Returns the number of encoded pytket circuits.
    pub fn len(&self) -> usize {
        self.circuits.len()
    }

    /// Returns whether the encoded circuit is empty.
    pub fn is_empty(&self) -> bool {
        self.circuits.is_empty()
    }

    /// Returns an iterator over the encoded pytket circuits.
    pub fn iter(&self) -> impl Iterator<Item = (Node, &SerialCircuit)> {
        self.circuits
            .iter()
            .map(|(&n, circ)| (n, &circ.serial_circuit))
    }

    /// Returns a mutable iterator over the encoded pytket circuits.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (Node, &mut SerialCircuit)> {
        self.circuits
            .iter_mut()
            .map(|(&n, circ)| (n, &mut circ.serial_circuit))
    }
}

impl<Node: HugrNode + Send + Sync> EncodedCircuit<Node> {
    /// Returns a parallel iterator over the encoded pytket circuits.
    pub fn par_iter(&self) -> impl ParallelIterator<Item = (Node, &SerialCircuit)> {
        self.circuits
            .par_iter()
            .map(|(&n, circ)| (n, &circ.serial_circuit))
    }

    /// Returns a parallel mutable iterator over the encoded pytket circuits.
    pub fn par_iter_mut(&mut self) -> impl ParallelIterator<Item = (Node, &mut SerialCircuit)> {
        self.circuits
            .par_iter_mut()
            .map(|(&n, circ)| (n, &mut circ.serial_circuit))
    }
}

impl<Node: HugrNode> Index<Node> for EncodedCircuit<Node> {
    type Output = SerialCircuit;

    fn index(&self, index: Node) -> &Self::Output {
        &self.circuits[&index].serial_circuit
    }
}

impl<Node: HugrNode> IndexMut<Node> for EncodedCircuit<Node> {
    fn index_mut(&mut self, index: Node) -> &mut Self::Output {
        &mut self
            .circuits
            .get_mut(&index)
            .unwrap_or_else(|| panic!("Indexing into a circuit that was not encoded: {index}"))
            .serial_circuit
    }
}
