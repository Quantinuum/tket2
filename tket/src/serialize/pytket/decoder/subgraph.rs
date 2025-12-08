//! Methods to decode opaque subgraphs from a pytket barrier operation.

use std::ops::RangeTo;
use std::sync::Arc;

use hugr::builder::Container;
use hugr::hugr::hugrmut::{HugrMut, InsertedForest};
use hugr::ops::{OpTag, OpTrait};
use hugr::types::Type;
use hugr::{Hugr, HugrView, Node, OutgoingPort, PortIndex, Wire};
use hugr_core::hugr::internal::HugrMutInternals;
use itertools::Itertools;

use crate::extension::rotation::rotation_type;
use crate::serialize::pytket::decoder::{
    DecodeStatus, FoundWire, LoadedParameter, PytketDecoderContext, TrackedBit, TrackedQubit,
};
use crate::serialize::pytket::extension::RegisterCount;
use crate::serialize::pytket::opaque::{
    EncodedEdgeID, OpaqueSubgraph, OpaqueSubgraphPayload, SubgraphId,
};
use crate::serialize::pytket::{
    PytketDecodeError, PytketDecodeErrorInner, PytketDecoderConfig, PARAMETER_TYPES,
};

impl<'h> PytketDecoderContext<'h> {
    /// Insert a subgraph encoded in the payload of a pytket barrier operation into
    /// the Hugr being decoded.
    pub(in crate::serialize::pytket) fn insert_subgraph_from_payload(
        &mut self,
        qubits: &[TrackedQubit],
        bits: &[TrackedBit],
        params: &[LoadedParameter],
        payload: &OpaqueSubgraphPayload,
    ) -> Result<DecodeStatus, PytketDecodeError> {
        let status = match payload {
            OpaqueSubgraphPayload::External { id } => {
                self.insert_external_subgraph(*id, qubits, bits, params)
            }
            OpaqueSubgraphPayload::Inline {
                hugr_envelope,
                inputs,
                outputs,
            } => self.insert_inline_subgraph(hugr_envelope, inputs, outputs, qubits, bits, params),
        }?;

        // Mark the used qubits and bits as outdated.
        qubits.iter().for_each(|q| {
            self.wire_tracker.mark_qubit_outdated(q.clone());
        });
        bits.iter().for_each(|b| {
            self.wire_tracker.mark_bit_outdated(b.clone());
        });

        Ok(status)
    }

    /// Move the subgraph nodes referenced by an
    /// [`OpaqueSubgraphPayload::External`] into the region being decoded.
    pub(in crate::serialize::pytket) fn insert_external_subgraph(
        &mut self,
        id: SubgraphId,
        qubits: &[TrackedQubit],
        bits: &[TrackedBit],
        params: &[LoadedParameter],
    ) -> Result<DecodeStatus, PytketDecodeError> {
        // TODO: Remove the graphs as we extract them, and give a nice error if subgraph id are reused.
        let Some(subgraph) = self.opaque_subgraphs.and_then(|s| s.get(id)) else {
            return Err(PytketDecodeErrorInner::OpaqueSubgraphNotFound { id }.wrap());
        };

        let old_parent = subgraph.region();
        if !OpTag::DataflowParent.is_superset(self.builder.hugr().get_optype(old_parent).tag()) {
            return Err(PytketDecodeErrorInner::ExternalSubgraphWasModified { id }.wrap());
        }
        let new_parent = self.builder.container_node();

        // Re-parent the nodes in the subgraph.
        for &node in subgraph.nodes() {
            self.builder.hugr_mut().set_parent(node, new_parent);
        }

        self.rewire_external_subgraph_inputs(
            subgraph, qubits, bits, params, old_parent, new_parent,
        )?;

        self.rewire_external_subgraph_outputs(id, subgraph, qubits, bits, old_parent, new_parent)?;

        Ok(DecodeStatus::Success)
    }

    /// Rewire the inputs of an external subgraph moved to the new region.
    ///
    /// Helper for [`Self::insert_external_subgraph`].
    fn rewire_external_subgraph_inputs(
        &mut self,
        subgraph: &OpaqueSubgraph<Node>,
        mut input_qubits: &[TrackedQubit],
        mut input_bits: &[TrackedBit],
        mut input_params: &[LoadedParameter],
        old_parent: Node,
        new_parent: Node,
    ) -> Result<(), PytketDecodeError> {
        let old_input = self.builder.hugr().get_io(old_parent).unwrap()[0];
        let new_input = self.builder.hugr().get_io(new_parent).unwrap()[0];

        // Reconnect input wires from parts of/nodes in the region that have been encoded into pytket.
        for (ty, (tgt_node, tgt_port)) in subgraph
            .signature()
            .input()
            .iter()
            .zip_eq(subgraph.incoming_ports())
        {
            let found_wire = self.wire_tracker.find_typed_wire(
                &self.config,
                &mut self.builder,
                ty,
                &mut input_qubits,
                &mut input_bits,
                &mut input_params,
                Some(EncodedEdgeID::default()),
            )?;

            let wire = match found_wire {
                FoundWire::Register(wire_data) => wire_data.wire(),
                FoundWire::Parameter(param) => param.wire(),
                FoundWire::Unsupported { .. } => {
                    // Input port with an unsupported type.
                    let Some((neigh, neigh_port)) = self
                        .builder
                        .hugr()
                        .single_linked_output(*tgt_node, *tgt_port)
                    else {
                        // The input was disconnected. We just skip it.
                        // (This is the case for unused other-ports)
                        continue;
                    };
                    if neigh != old_input {
                        // If it was linked to some node, that wasn't the old
                        // region's input, we can keep the link.
                        continue;
                    }
                    // If it was linked to the old circuit's input, we need to
                    // re-wire it to the new region's input.
                    Wire::new(new_input, neigh_port)
                }
            };

            self.builder
                .hugr_mut()
                .connect(wire.node(), wire.source(), *tgt_node, *tgt_port);
        }

        Ok(())
    }

    /// Rewire the outputs of an external subgraph moved to the new region.
    ///
    /// Registers the output wires that should be connected to nodes in the new region.
    /// Re-wires edges to old region's output node to the new one's.
    ///
    /// Helper for [`Self::insert_external_subgraph`].
    fn rewire_external_subgraph_outputs(
        &mut self,
        id: SubgraphId,
        subgraph: &OpaqueSubgraph<Node>,
        qubits: &[TrackedQubit],
        bits: &[TrackedBit],
        old_parent: Node,
        new_parent: Node,
    ) -> Result<(), PytketDecodeError> {
        let old_output = self.builder.hugr().get_io(old_parent).unwrap()[1];
        let new_output = self.builder.hugr().get_io(new_parent).unwrap()[1];

        let mut output_qubits = qubits;
        let mut output_bits = bits;

        for ((out_idx, ty), (src, src_port)) in subgraph
            .signature()
            .output()
            .iter()
            .enumerate()
            .zip_eq(subgraph.outgoing_ports())
        {
            // Output wire from the subgraph. Depending on the type, we may need
            // to track new qubits and bits, re-connect it to some output, or
            // leave it untouched.
            let wire = Wire::new(*src, *src_port);
            if let Some(counts) = self.config().type_to_pytket(ty).filter(|c| c.params == 0) {
                // This port declares new bit/qubit outputs to be tracked by the decoder.

                // Make sure to disconnect the old wire.
                self.builder.hugr_mut().disconnect(*src, *src_port);

                let wire_qubits = split_off(&mut output_qubits, ..counts.qubits);
                let wire_bits = split_off(&mut output_bits, ..counts.bits);
                if wire_qubits.is_none() || wire_bits.is_none() {
                    return Err(make_unexpected_node_out_error(
                        self.config(),
                        subgraph.signature().output().iter(),
                        qubits.len(),
                        bits.len(),
                    ));
                }
                self.wire_tracker.track_wire(
                    wire,
                    Arc::new(ty.clone()),
                    wire_qubits.unwrap().iter().cloned(),
                    wire_bits.unwrap().iter().cloned(),
                )?;
            } else if PARAMETER_TYPES.contains(ty) {
                let param_name = id.output_parameter(out_idx);
                let param = if ty == &rotation_type() {
                    LoadedParameter::rotation(wire)
                } else {
                    LoadedParameter::float_half_turns(wire)
                };
                self.wire_tracker
                    .register_input_parameter(param, param_name)?;
            } else {
                // This is an unsupported wire. If it was connected to the old
                // region's output, rewire it to the new region's output.
                // Otherwise leave it connected.
                for (tgt, tgt_port) in self
                    .builder
                    .hugr()
                    .linked_inputs(*src, *src_port)
                    .collect_vec()
                {
                    if tgt == old_output {
                        // We only need to disconnect the specific edge here,
                        // but there should only be one incoming value edge.
                        self.builder.hugr_mut().disconnect(old_output, tgt_port);
                        self.builder
                            .hugr_mut()
                            .connect(*src, *src_port, new_output, tgt_port);
                    }
                }
            }
        }

        Ok(())
    }

    /// Insert an [`OpaqueSubgraphPayload::Inline`] into the Hugr being decoded.
    fn insert_inline_subgraph(
        &mut self,
        hugr_envelope: &str,
        payload_inputs: &[(Type, EncodedEdgeID)],
        payload_outputs: &[(Type, EncodedEdgeID)],
        qubits: &[TrackedQubit],
        bits: &[TrackedBit],
        params: &[LoadedParameter],
    ) -> Result<DecodeStatus, PytketDecodeError> {
        let to_insert_hugr = Hugr::load_str(hugr_envelope, Some(self.extension_registry()))
            .map_err(|e| PytketDecodeErrorInner::UnsupportedSubgraphInlinePayload { source: e })?;
        let to_insert_signature = to_insert_hugr.inner_function_type().unwrap();

        let module = self.builder.hugr().module_root();
        let region = self.builder.container_node();

        // Collect the non-IO nodes in the hugr we plan to insert.
        let Some([to_insert_input, to_insert_output]) =
            to_insert_hugr.get_io(to_insert_hugr.entrypoint())
        else {
            return Err(PytketDecodeError::custom(
                "Opaque subgraph payload has a non-dataflow parent as entrypoint",
            ));
        };
        let entrypoint_children = to_insert_hugr
            .children(to_insert_hugr.entrypoint())
            // Ignore the input and output nodes.
            .skip(2)
            .map(|c| (c, region));

        // Compute the inputs and output ports of the subgraph.
        // Since we insert the nodes inside the region directly (without the I/O nodes),
        // we need to do some bookkeeping to match the ports.
        //
        // `to_insert_inputs` is a vector of vectors, where each first-level entry corresponds to
        // an input to the subgraph / element in `payload.inputs`, and the second-level
        // list is all the target ports that connect to that input.
        //
        // `to_insert_outputs` is just a vector of node+outgoing ports.
        let to_insert_inputs: hugr::hugr::views::sibling_subgraph::IncomingPorts<Node> =
            to_insert_signature
                .input_ports()
                .map(|p| {
                    to_insert_hugr
                        .linked_inputs(to_insert_input, p.index())
                        .collect_vec()
                })
                .collect_vec();
        let to_insert_outputs: Vec<(Node, OutgoingPort)> = to_insert_signature
            .output_ports()
            .map(|p| {
                to_insert_hugr
                    .single_linked_output(to_insert_output, p.index())
                    .unwrap()
            })
            .collect_vec();

        // Collect any module child that does not contain the entrypoint function.
        //
        // These are global functions or constant definitions.
        let entrypoint_function =
            std::iter::successors(Some(to_insert_hugr.entrypoint()), |&node| {
                let parent = to_insert_hugr.get_parent(node)?;
                if parent == module {
                    None
                } else {
                    Some(parent)
                }
            })
            .last()
            .unwrap();
        let module_children = to_insert_hugr
            .children(to_insert_hugr.module_root())
            .filter(|c| *c != entrypoint_function)
            .map(|c| (c, module));

        // Insert the hugr's entrypoint region directly into the region being built,
        // and any other function in the HUGR module into the module being built.
        let insertion_roots = entrypoint_children.chain(module_children).collect_vec();
        let insertion_result = self
            .builder
            .hugr_mut()
            .insert_forest(to_insert_hugr, insertion_roots)
            .unwrap_or_else(|e| panic!("Invalid `insertion_roots`. {e}"));

        self.wire_inline_subgraph_inputs(
            qubits,
            bits,
            params,
            payload_inputs,
            to_insert_inputs,
            &insertion_result,
        )?;

        self.wire_inline_subgraph_outputs(
            qubits,
            bits,
            payload_outputs,
            to_insert_outputs,
            &insertion_result,
        )?;

        Ok(DecodeStatus::Success)
    }

    /// Wire the inputs of a newly inserted inline subgraph.
    ///
    /// Helper for [`Self::insert_inline_subgraph`].
    fn wire_inline_subgraph_inputs(
        &mut self,
        mut input_qubits: &[TrackedQubit],
        mut input_bits: &[TrackedBit],
        mut input_params: &[LoadedParameter],
        payload_inputs: &[(Type, EncodedEdgeID)],
        to_insert_inputs: hugr::hugr::views::sibling_subgraph::IncomingPorts<Node>,
        insertion_result: &InsertedForest,
    ) -> Result<(), PytketDecodeError> {
        // A list of incoming ports corresponding to [`EncodedEdgeID`]s that must be
        // connected once the outgoing port is created.
        //
        // This handles the case where unsupported subgraphs in opaque barriers on
        // the pytket circuit get reordered and input ports are seen before their
        // outputs.
        for ((ty, edge_id), targets) in payload_inputs.iter().zip_eq(to_insert_inputs) {
            let found_wire = self.wire_tracker.find_typed_wire(
                &self.config,
                &mut self.builder,
                ty,
                &mut input_qubits,
                &mut input_bits,
                &mut input_params,
                Some(*edge_id),
            )?;

            let wire = match found_wire {
                FoundWire::Register(wire_data) => wire_data.wire(),
                FoundWire::Parameter(param) => param.wire(),
                FoundWire::Unsupported { id } => {
                    self.wire_tracker.connect_unsupported_wire_targets(
                        id,
                        targets,
                        self.builder.hugr_mut(),
                    );
                    continue;
                }
            };

            for (to_insert_node, port) in targets {
                let node = *insertion_result.node_map.get(&to_insert_node).unwrap();
                self.builder
                    .hugr_mut()
                    .connect(wire.node(), wire.source(), node, port);
            }
        }

        Ok(())
    }

    /// Wire the outputs of a newly inserted inline subgraph.
    ///
    /// Helper for [`Self::insert_inline_subgraph`].
    fn wire_inline_subgraph_outputs(
        &mut self,
        qubits: &[TrackedQubit],
        bits: &[TrackedBit],
        payload_outputs: &[(Type, EncodedEdgeID)],
        to_insert_outputs: Vec<(Node, OutgoingPort)>,
        insertion_result: &InsertedForest,
    ) -> Result<(), PytketDecodeError> {
        // Register the subgraph outputs in the wire tracker.
        let mut output_qubits = qubits;
        let mut output_bits = bits;
        for ((ty, edge_id), (to_insert_node, port)) in
            payload_outputs.iter().zip_eq(to_insert_outputs)
        {
            let node = *insertion_result.node_map.get(&to_insert_node).unwrap();
            let wire = Wire::new(node, port);
            match self.config().type_to_pytket(ty).filter(|c| c.params == 0) {
                Some(counts) => {
                    // Track the registers in the subgraph output wires.
                    // Output parameters from a subgraph are always marked as not supported (they don't map to any pytket argument variable).
                    // We only track qubit/bit wires here.
                    let wire_qubits = split_off(&mut output_qubits, ..counts.qubits);
                    let wire_bits = split_off(&mut output_bits, ..counts.bits);
                    if wire_qubits.is_none() || wire_bits.is_none() {
                        return Err(make_unexpected_node_out_error(
                            self.config(),
                            payload_outputs.iter().map(|(ty, _)| ty),
                            qubits.len(),
                            bits.len(),
                        ));
                    }
                    self.wire_tracker.track_wire(
                        wire,
                        Arc::new(ty.clone()),
                        wire_qubits.unwrap().iter().cloned(),
                        wire_bits.unwrap().iter().cloned(),
                    )?;
                }
                None => {
                    // This is an unsupported wire, so we let the wire tracker manage the connections.
                    self.wire_tracker.connect_unsupported_wire_source(
                        *edge_id,
                        wire,
                        self.builder.hugr_mut(),
                    );
                }
            }
        }
        Ok(())
    }
}

// TODO: Replace with array's `split_off` method once MSRV is ≥1.87
fn split_off<'a, T>(slice: &mut &'a [T], range: RangeTo<usize>) -> Option<&'a [T]> {
    let split_index = range.end;
    if split_index > slice.len() {
        return None;
    }
    let (front, back) = slice.split_at(split_index);
    *slice = back;
    Some(front)
}

/// Helper to compute the expected register counts before generating a
/// [`PytketDecodeErrorInner::UnexpectedNodeOutput`] error when registering the
/// outputs of an unsupported subgraph.
///
/// Processes all the output types to compute the number of qubits and bits we
/// required to have available.
fn make_unexpected_node_out_error<'ty>(
    config: &PytketDecoderConfig,
    output_types: impl IntoIterator<Item = &'ty Type>,
    available_qubits: usize,
    available_bits: usize,
) -> PytketDecodeError {
    let mut expected_count = RegisterCount::default();
    for ty in output_types {
        expected_count += config.type_to_pytket(ty).unwrap_or_default();
    }
    PytketDecodeErrorInner::UnexpectedNodeOutput {
        expected_qubits: expected_count.qubits,
        expected_bits: expected_count.bits,
        circ_qubits: available_qubits,
        circ_bits: available_bits,
    }
    .wrap()
}
