//! Temporary structure linking an encoded pytket circuit and subcircuits, with their originating HUGR.

use std::collections::VecDeque;
use std::ops::{Index, IndexMut};
use std::sync::Arc;

use hugr::core::{HugrNode, IncomingPort, OutgoingPort};
use hugr::hugr::hugrmut::HugrMut;
use hugr::ops::handle::NodeHandle;
use hugr::ops::{OpParent, OpTag, OpTrait};
use hugr::types::EdgeKind;
use hugr::{Hugr, HugrView, Node};
use hugr_core::hugr::internal::HugrMutInternals;
use indexmap::IndexMap;
use itertools::Itertools;
use rayon::iter::{IntoParallelRefIterator, IntoParallelRefMutIterator, ParallelIterator};
use tket_json_rs::circuit_json::{Command as PytketCommand, SerialCircuit};

use crate::serialize::pytket::decoder::PytketDecoderContext;
use crate::serialize::pytket::opaque::SubgraphId;
use crate::serialize::pytket::{
    DecodeInsertionTarget, DecodeOptions, EncodeOptions, PytketDecodeError, PytketDecodeErrorInner,
    PytketDecoderConfig, PytketEncodeError, PytketEncoderContext, default_decoder_config,
    default_encoder_config,
};

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
    /// Circuit segments grouped by their originating dataflow region.
    ///
    /// These correspond to sections of the HUGR that can be optimized
    /// independently.
    ///
    /// Regions are ordered as a depth-first pre-order; segments retain their
    /// execution order within each region.
    circuits: IndexMap<Node, EncodedCircuitInfo>,
    /// Sets of subgraphs in the HUGR that have been encoded as opaque barriers
    /// in the pytket circuit.
    ///
    /// Subcircuits are identified in the barrier metadata by their ID in this
    /// vector. See [`SubgraphId`].
    opaque_subgraphs: OpaqueSubgraphs<Node>,
}

/// Identifies one pytket circuit segment within an encoded HUGR region.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct EncodedCircuitId<Node> {
    /// The HUGR dataflow region from which the segment was encoded.
    pub region: Node,
    /// The segment's zero-based position within the region.
    pub segment: usize,
}

/// Information stored about a pytket circuit encoded from a HUGR region.
#[derive(Debug, Default, Clone)]
pub(super) struct EncodedCircuitInfo {
    /// Runs of pytket commands encoded from the region.
    pub serial_circuits: Vec<SerialCircuit>,
    /// Opaque subgraphs that must be restored before, between, or after the
    /// serial circuits.
    ///
    /// This always contains one more entry than [`Self::serial_circuits`].
    pub boundaries: Vec<EncodedCircuitBoundary>,
    /// List of wires that directly connected the input node to the output node in the encoded region,
    /// and were not encoded in [`serial_circuits`].
    ///
    /// We just store the input nodes's output port and output node's input port here.
    pub straight_through_wires: Vec<StraightThroughWire>,
    /// List of parameters in the pytket circuit in the order they appear in the
    /// hugr input.
    ///
    /// We require this to correctly reconstruct the input order in the reassembled hugr,
    /// since parameters in pytket are unordered.
    pub input_params: Vec<String>,
    /// List of output parameter expressions found at the end of the encoded region.
    pub output_params: Vec<String>,
    /// List of qubit registers seen at the output of the encoded region.
    pub output_qubits: Vec<tket_json_rs::register::ElementId>,
    /// List of bit registers seen at the output of the encoded region.
    pub output_bits: Vec<tket_json_rs::register::ElementId>,
}

impl EncodedCircuitInfo {
    /// Initialize a pytket circuit for a decoder
    /// for every segment in this region.
    ///
    /// Callers may add registers to an individual segment, so the initialization
    /// circuit contains the deterministic union of registers in first-use order.
    fn decoder_initialization_circuit(&self) -> SerialCircuit {
        let mut initialization_circuit = self
            .serial_circuits
            .first()
            .cloned()
            .expect("encoded regions always contain a circuit segment");
        for circuit in self.serial_circuits.iter().skip(1) {
            for qubit in &circuit.qubits {
                if !initialization_circuit.qubits.contains(qubit) {
                    initialization_circuit.qubits.push(qubit.clone());
                }
            }
            for bit in &circuit.bits {
                if !initialization_circuit.bits.contains(bit) {
                    initialization_circuit.bits.push(bit.clone());
                }
            }
        }
        initialization_circuit.commands.clear();
        initialization_circuit.phase = "0".to_owned();
        initialization_circuit.implicit_permutation.clear();
        initialization_circuit
    }

    /// Whether the region requires its original HUGR context for decoding.
    fn has_external_boundaries(&self) -> bool {
        self.boundaries
            .iter()
            .any(|boundary| !boundary.external_subgraphs.is_empty())
    }
}

/// Unsupported subgraphs positioned at one boundary between serial circuits.
#[derive(Debug, Default, Clone)]
pub(super) struct EncodedCircuitBoundary {
    /// Subgraphs of the region that could not be encoded as pytket commands,
    /// and have no qubit/bits in their boundary that could be used to emit an
    /// opaque barrier command in an adjacent serial circuit.
    pub external_subgraphs: Vec<AdditionalSubgraph>,
}

/// A subgraph of the encoded circuit that could not be associated to any qubit or bit register in the pytket circuit.
#[derive(Debug, Clone)]
pub(super) struct AdditionalSubgraph {
    /// The subgraph of the region that could not be encoded as a pytket command,
    /// and has no qubit/bits in its boundary that could be used to emit an opaque
    /// barrier command in an adjacent circuit segment.
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
        hugr: &H,
        options: EncodeOptions<H>,
    ) -> Result<Self, PytketEncodeError<H::Node>> {
        Self::new_with_entrypoint(hugr, hugr.entrypoint(), options)
    }

    /// Encode a HugrView into a [`EncodedCircuit`].
    ///
    /// Encodes the dataflow regions under the given entrypoint. If
    /// [`EncodeOptions::encode_subcircuits`] is set, the descendants of any found
    /// dataflow regions will be encoded as well.
    ///
    /// The circuit may contain opaque barriers referencing subgraphs in the
    /// original HUGR. To obtain a fully standalone pytket circuit that can be
    /// used independently, and stored permanently, use
    /// [`EncodedCircuit::new_standalone`] or call
    /// [`EncodedCircuit::ensure_standalone`].
    ///
    /// See [`EncodeOptions`] for the options used by the encoder.
    pub fn new_with_entrypoint<H: AsRef<Hugr> + AsMut<Hugr> + HugrView<Node = Node>>(
        hugr: &H,
        entrypoint: H::Node,
        options: EncodeOptions<H>,
    ) -> Result<Self, PytketEncodeError<H::Node>> {
        let mut enc = Self {
            circuits: IndexMap::new(),
            opaque_subgraphs: OpaqueSubgraphs::new(0),
        };

        enc.encode_circuits(hugr, entrypoint, options)?;

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

        // Reassemble each circuit, processing the inner regions before their
        // ancestors to ensure any external edge is kept valid.
        for (&original_region, encoded) in self.circuits.iter().rev() {
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
            let initialization_circuit = encoded.decoder_initialization_circuit();
            let mut decoder = PytketDecoderContext::new(
                &initialization_circuit,
                hugr,
                DecodeInsertionTarget::Function { fn_name: None },
                options,
                Some(&self.opaque_subgraphs),
            )?;
            decoder.reserve_boundary_parameters(&encoded.boundaries);
            decoder.connect_straight_through_wires(&encoded.straight_through_wires);
            for (segment, boundary) in encoded.serial_circuits.iter().zip(&encoded.boundaries) {
                decoder.insert_boundary(boundary)?;
                decoder.add_serial_circuit_phase(segment)?;
                decoder.run_commands(&segment.commands)?;
                decoder.apply_implicit_permutation(segment)?;
            }
            decoder.insert_boundary(
                encoded
                    .boundaries
                    .last()
                    .expect("encoded region has a trailing boundary"),
            )?;
            let decoded_node = decoder.finish(Some(encoded))?.node();

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
        hugr: &H,
        options: EncodeOptions<H>,
    ) -> Result<Self, PytketEncodeError<H::Node>> {
        Self::new_standalone_with_entrypoint(hugr, hugr.entrypoint(), options)
    }

    /// Encode a HugrView into a [`EncodedCircuit`].
    ///
    /// Encodes the dataflow region under the given entrypoint.
    ///
    /// The circuit may contain opaque barriers referencing subgraphs in the
    /// original HUGR. To obtain a fully standalone pytket circuit that can be
    /// used independently, and stored permanently, use
    /// [`EncodedCircuit::new_standalone`] or call
    /// [`EncodedCircuit::ensure_standalone`].
    ///
    /// See [`EncodeOptions`] for the options used by the encoder.
    pub fn new_standalone_with_entrypoint<H: HugrView<Node = Node>>(
        hugr: &H,
        entrypoint: H::Node,
        options: EncodeOptions<H>,
    ) -> Result<Self, PytketEncodeError<H::Node>> {
        let mut enc = Self {
            circuits: IndexMap::new(),
            opaque_subgraphs: OpaqueSubgraphs::new(0),
        };
        enc.encode_circuits(hugr, entrypoint, options)?;
        enc.ensure_standalone(hugr)?;
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
        hugr: &H,
        entrypoint: H::Node,
        mut options: EncodeOptions<H>,
    ) -> Result<(), PytketEncodeError<H::Node>> {
        // List of nodes to check for subcircuits.
        //
        // These may be either dataflow region parents that we can encode, or
        // any node with children that we should traverse recursively until we
        // find a dataflow region.
        let mut candidate_nodes = VecDeque::from([entrypoint]);
        let config = options
            .config
            .take()
            .unwrap_or_else(|| Arc::new(default_encoder_config()));

        // Add a node to the list of candidates if it's a region parent.
        let add_candidate = |node: H::Node, queue: &mut VecDeque<H::Node>| {
            if hugr.first_child(node).is_some() {
                queue.push_back(node);
            }
        };

        // Add all container nodes from the new opaque subgraphs to the list of
        // candidates.
        let mut encoder_count = 0;
        while let Some(node) = candidate_nodes.pop_front() {
            let node_op = hugr.get_optype(node);
            if !OpTag::DataflowParent.is_superset(node_op.tag()) {
                for child in hugr.children(node) {
                    add_candidate(child, &mut candidate_nodes);
                }
                continue;
            }
            encoder_count += 1;
            let opaque_subgraphs = OpaqueSubgraphs::new(encoder_count);
            let mut encoder: PytketEncoderContext<H> =
                PytketEncoderContext::new(hugr, node, opaque_subgraphs, config.clone())?;
            encoder.run_encoder(hugr, node)?;
            let (encoded, opaque_subgraphs) = encoder.finish(hugr, node)?;

            if options.encode_subcircuits {
                for subgraph_id in opaque_subgraphs.ids() {
                    for &node in opaque_subgraphs[subgraph_id].nodes() {
                        add_candidate(node, &mut candidate_nodes);
                    }
                }
            }

            // Ignore empty circuits, for regions with no supported operation.
            //
            // A circuit is empty if it only contains barriers representing unsupported HUGR subgraphs, or plain
            // pytket barriers (as they are effectively no-ops).
            let is_empty_circuit = |encoded: &SerialCircuit| {
                encoded
                    .commands
                    .iter()
                    .all(|cmd| cmd.op.op_type == tket_json_rs::OpType::Barrier)
            };
            if !options.keep_empty_circuits && encoded.serial_circuits.iter().all(is_empty_circuit)
            {
                continue;
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
        if !self.contains_region(region) {
            return Err(PytketDecodeErrorInner::NotAnEncodedRegion {
                region: region.to_string(),
            }
            .wrap());
        }
        let encoded_info = &self.circuits[&region];
        let serial_circuit = encoded_info
            .serial_circuits
            .first()
            .expect("encoded regions always contain a circuit segment");

        if self.circuits.len() > 1 {
            return Err(PytketDecodeError::custom(
                "Reassembling an `EncodedCircuit` with nested subcircuits is not yet implemented.",
            ));
        }
        if encoded_info.serial_circuits.len() != 1 || encoded_info.has_external_boundaries() {
            return Err(PytketDecodeError::custom(
                "A segmented encoded region requires its original HUGR context; use `reassemble_inplace`.",
            ));
        }

        let mut hugr = Hugr::new();
        let target = DecodeInsertionTarget::Function { fn_name };

        let mut decoder =
            PytketDecoderContext::new(serial_circuit, &mut hugr, target, options, None)?;
        decoder.run_decoder(&serial_circuit.commands)?;
        decoder.finish(Some(encoded_info))?;
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

        if self
            .circuits
            .values()
            .any(EncodedCircuitInfo::has_external_boundaries)
        {
            return Err(PytketEncodeError::custom(
                "A region split by register-free opaque subgraphs cannot be represented as a standalone `SerialCircuit`.",
            ));
        }

        for encoded in self.circuits.values_mut() {
            for circuit in &mut encoded.serial_circuits {
                make_commands_standalone(&mut circuit.commands, &self.opaque_subgraphs, hugr)?;
            }
        }
        Ok(())
    }

    /// Returns `true` if the given region has any encoded circuit segments.
    pub fn contains_region(&self, region: Node) -> bool {
        self.circuits.contains_key(&region)
    }

    /// Returns the circuit segments encoded for a region, in execution order.
    ///
    /// Returns an empty iterator when the region was not encoded.
    pub fn get_circuits(
        &self,
        region: Node,
    ) -> impl Iterator<Item = (EncodedCircuitId<Node>, &SerialCircuit)> {
        self.circuits
            .get(&region)
            .into_iter()
            .flat_map(move |info| {
                info.serial_circuits
                    .iter()
                    .enumerate()
                    .map(move |(segment, circuit)| (EncodedCircuitId { region, segment }, circuit))
            })
    }

    /// Returns mutable circuit segments encoded for a region, in execution
    /// order.
    ///
    /// Returns an empty iterator when the region was not encoded.
    pub fn get_circuits_mut(
        &mut self,
        region: Node,
    ) -> impl Iterator<Item = (EncodedCircuitId<Node>, &mut SerialCircuit)> {
        self.circuits
            .get_mut(&region)
            .into_iter()
            .flat_map(move |info| {
                info.serial_circuits
                    .iter_mut()
                    .enumerate()
                    .map(move |(segment, circuit)| (EncodedCircuitId { region, segment }, circuit))
            })
    }

    /// Returns a circuit segment by its region and position.
    pub fn get_segment(&self, id: EncodedCircuitId<Node>) -> Option<&SerialCircuit> {
        self.circuits
            .get(&id.region)?
            .serial_circuits
            .get(id.segment)
    }

    /// Returns a mutable circuit segment by its region and position.
    pub fn get_segment_mut(&mut self, id: EncodedCircuitId<Node>) -> Option<&mut SerialCircuit> {
        self.circuits
            .get_mut(&id.region)?
            .serial_circuits
            .get_mut(id.segment)
    }

    /// Returns the total number of encoded pytket circuit segments.
    pub fn len(&self) -> usize {
        self.circuits
            .values()
            .map(|region| region.serial_circuits.len())
            .sum()
    }

    /// Returns whether the encoded circuit is empty.
    pub fn is_empty(&self) -> bool {
        self.circuits.is_empty()
    }

    /// Returns an iterator over every encoded pytket circuit segment and its region.
    ///
    /// A region containing multiple segments appears once for each segment.
    // TODO: Update signature to return `(EncodedCircuitId<Node>, &SerialCircuit)` in a breaking release.
    // See https://github.com/Quantinuum/tket2/issues/1901
    pub fn iter(&self) -> impl Iterator<Item = (Node, &SerialCircuit)> {
        self.circuits.iter().flat_map(|(&node, region)| {
            region
                .serial_circuits
                .iter()
                .map(move |circuit| (node, circuit))
        })
    }

    /// Returns a mutable iterator over every circuit segment and its region.
    ///
    /// A region containing multiple segments appears once for each segment.
    // TODO: Update signature to return `(EncodedCircuitId<Node>, &mut SerialCircuit)` in a breaking release.
    // See https://github.com/Quantinuum/tket2/issues/1901
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (Node, &mut SerialCircuit)> {
        self.circuits.iter_mut().flat_map(|(&node, region)| {
            region
                .serial_circuits
                .iter_mut()
                .map(move |circuit| (node, circuit))
        })
    }
}

impl<Node: HugrNode + Send + Sync> EncodedCircuit<Node> {
    /// Returns a parallel iterator over every circuit segment and its region.
    ///
    /// A region containing multiple segments appears once for each segment.
    // TODO: Update signature to return `(EncodedCircuitId<Node>, &SerialCircuit)` in a breaking release.
    // See https://github.com/Quantinuum/tket2/issues/1901
    pub fn par_iter(&self) -> impl ParallelIterator<Item = (Node, &SerialCircuit)> {
        self.circuits.par_iter().flat_map_iter(|(&node, region)| {
            region
                .serial_circuits
                .iter()
                .map(move |circuit| (node, circuit))
        })
    }

    /// Returns a parallel mutable iterator over every circuit segment and its region.
    ///
    /// A region containing multiple segments appears once for each segment.
    // TODO: Update signature to return `(EncodedCircuitId<Node>, &mut SerialCircuit)` in a breaking release.
    // See https://github.com/Quantinuum/tket2/issues/1901
    pub fn par_iter_mut(&mut self) -> impl ParallelIterator<Item = (Node, &mut SerialCircuit)> {
        self.circuits
            .par_iter_mut()
            .flat_map_iter(|(&node, region)| {
                region
                    .serial_circuits
                    .iter_mut()
                    .map(move |circuit| (node, circuit))
            })
    }
}

/// Backwards-compatible region indexing that ignores all but the first segment.
/// This implementation will be removed in a breaking release.
impl<Node: HugrNode> Index<Node> for EncodedCircuit<Node> {
    type Output = SerialCircuit;

    fn index(&self, index: Node) -> &Self::Output {
        self.circuits
            .get(&index)
            .and_then(|region| region.serial_circuits.first())
            .unwrap_or_else(|| panic!("Indexing into a circuit that was not encoded: {index}"))
    }
}

/// Backwards-compatible mutable region indexing that ignores all but the first segment.
/// This implementation will be removed in a breaking release.
impl<Node: HugrNode> IndexMut<Node> for EncodedCircuit<Node> {
    fn index_mut(&mut self, index: Node) -> &mut Self::Output {
        self.circuits
            .get_mut(&index)
            .and_then(|region| region.serial_circuits.first_mut())
            .unwrap_or_else(|| panic!("Indexing into a circuit that was not encoded: {index}"))
    }
}

impl<Node: HugrNode> Index<EncodedCircuitId<Node>> for EncodedCircuit<Node> {
    type Output = SerialCircuit;

    fn index(&self, index: EncodedCircuitId<Node>) -> &Self::Output {
        self.get_segment(index).unwrap_or_else(|| {
            panic!(
                "Indexing an encoded circuit segment that does not exist: {}:{}",
                index.region, index.segment
            )
        })
    }
}

impl<Node: HugrNode> IndexMut<EncodedCircuitId<Node>> for EncodedCircuit<Node> {
    fn index_mut(&mut self, index: EncodedCircuitId<Node>) -> &mut Self::Output {
        self.get_segment_mut(index).unwrap_or_else(|| {
            panic!(
                "Indexing an encoded circuit segment that does not exist: {}:{}",
                index.region, index.segment
            )
        })
    }
}
