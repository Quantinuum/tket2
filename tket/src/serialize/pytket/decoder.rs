//! Intermediate structure for decoding [`SerialCircuit`]s into [`Hugr`]s.

mod param;
mod subgraph;
mod tracked_elem;
mod wires;

use hugr::extension::ExtensionRegistry;
use hugr::hugr::hugrmut::HugrMut;
use hugr::std_extensions::arithmetic::float_types::float64_type;
use indexmap::IndexSet;
pub use param::{LoadedParameter, ParameterType};
pub use tracked_elem::{TrackedBit, TrackedQubit};
pub use wires::TrackedWires;

pub(super) use wires::FoundWire;

use std::sync::Arc;

use hugr::builder::{BuildHandle, Container, DFGBuilder, Dataflow, FunctionBuilder, SubContainer};
use hugr::extension::prelude::{bool_t, qb_t};
use hugr::ops::handle::{DataflowOpID, NodeHandle};
use hugr::ops::{DFG, OpParent, OpTrait, OpType};
use hugr::types::{Signature, Type, TypeRow};
use hugr::{Hugr, HugrView, Node, OutgoingPort, Wire};
use tracked_elem::{TrackedBitId, TrackedQubitId};

use itertools::Itertools;
use tket_json_rs::circuit_json;
use tket_json_rs::circuit_json::SerialCircuit;

use super::PytketDecodeError;
use crate::TketOp;
use crate::extension::rotation::rotation_type;
use crate::metadata;
use crate::serialize::pytket::circuit::{
    AdditionalNodesAndWires, EncodedCircuitInfo, StraightThroughWire,
};
use crate::serialize::pytket::config::PytketDecoderConfig;
use crate::serialize::pytket::decoder::wires::WireTracker;
use crate::serialize::pytket::extension::{RegisterCount, build_opaque_tket_op};
use crate::serialize::pytket::opaque::{EncodedEdgeID, OpaqueSubgraphs};
use crate::serialize::pytket::{
    DecodeInsertionTarget, DecodeOptions, PytketDecodeErrorInner, default_decoder_config,
};

/// State of the tket circuit being decoded.
///
/// The state of a DFG being built from a [`SerialCircuit`] into a Hugr.
///
/// The lifetime parameter `'h` is the lifetime of the target Hugr, as well
/// as the lifetime of the external subgraphs referenced by
/// [`OpaqueSubgraphPayload`][super::opaque::OpaqueSubgraphPayload]s in the
/// pytket circuit.
#[derive(Debug)]
pub struct PytketDecoderContext<'h> {
    /// The Hugr being built.
    pub builder: DFGBuilder<&'h mut Hugr>,
    /// A tracker keeping track of the generated wires and their corresponding types.
    pub(super) wire_tracker: Box<WireTracker>,
    // A registry containing custom operation decoders.
    ///
    /// This is a copy of the configuration in `options`, if present, or
    /// [`default_decoder_config`][super::default_decoder_config] if not.
    config: Arc<PytketDecoderConfig>,
    /// The extensions to use when loading the HUGR envelope.
    ///
    /// When `None`, we will use a default registry that includes the prelude,
    /// std, TKET1, and TketOps extensions.
    pub extensions: Option<ExtensionRegistry>,
    /// A registry of opaque subgraphs from the original Hugr, that may be referenced by opaque barriers in the pytket circuit
    /// via their [`SubgraphId`].
    pub(super) opaque_subgraphs: Option<&'h OpaqueSubgraphs<Node>>,
}

impl<'h> PytketDecoderContext<'h> {
    /// Initialize a new [`PytketDecoderContext`], using the metadata from a
    /// [`SerialCircuit`].
    ///
    /// The circuit will be defined as a new function in the HUGR.
    ///
    /// # Arguments
    ///
    /// - `serialcirc`: The serialised circuit to decode.
    /// - `hugr`: The [`Hugr`] to define the new function in.
    /// - `target`: The target to insert the decoded circuit into.
    /// - `options`: The options for the decoder.
    /// - `opaque_subgraphs`: A registry of opaque subgraphs from `hugr`, that
    ///   are referenced by opaque barriers in the pytket circuit via their
    ///   [`SubgraphId`].
    ///
    /// # Defining the function signature
    ///
    /// If `options` do not define a `signature`, we default to a sequence of
    /// qubit types followed by bool types, according to the qubit and bit
    /// counts in the circuit.
    ///
    /// If provided, we produce a hugr with the given signature instead.
    /// Leftover qubits will be `QAlloc`d, and `QFree`d, as required. Bit values
    /// not in the input will be initialized to `false`.
    ///
    /// The signature may include bare parameter wires (e.g. `float64` or
    /// `rotation`) mixed between the value types. These will be associated with
    /// the [`DecodeOptions::input_params`] names, if possible. Any remaining
    /// parameters will be added as additional inputs with type
    /// [`rotation_type`]. Additional parameter inputs may be added during
    /// runtime, as new free variables are found in the command arguments.
    pub(super) fn new(
        serialcirc: &SerialCircuit,
        hugr: &'h mut Hugr,
        target: DecodeInsertionTarget,
        options: DecodeOptions,
        opaque_subgraphs: Option<&'h OpaqueSubgraphs<Node>>,
    ) -> Result<Self, PytketDecodeError> {
        // Ensure that the set of decoders is present, use a default one if not.
        let config = options
            .config
            .unwrap_or_else(|| Arc::new(default_decoder_config()));

        // Compute the signature of the decoded region, if not provided, and
        // initialize the DFG builder.
        let signature = options.signature.clone().unwrap_or_else(|| {
            let num_qubits = serialcirc.qubits.len();
            let num_bits = serialcirc.bits.len();
            let types: TypeRow = [vec![qb_t(); num_qubits], vec![bool_t(); num_bits]]
                .concat()
                .into();
            Signature::new(types.clone(), types)
        });
        let mut dfg: DFGBuilder<&mut Hugr> = match target {
            DecodeInsertionTarget::Function { fn_name } => {
                let name = fn_name
                    .or_else(|| serialcirc.name.clone())
                    .unwrap_or_default();
                FunctionBuilder::with_hugr(hugr, name, signature.clone())
                    .unwrap()
                    .into_dfg_builder()
            }
            DecodeInsertionTarget::Region { parent } => {
                let op = DFG {
                    signature: signature.clone(),
                };
                let dfg = hugr.add_node_with_parent(parent, op);
                DFGBuilder::create_with_io(hugr, dfg, signature.clone()).unwrap()
            }
        };

        Self::init_metadata(&mut dfg, serialcirc);
        let wire_tracker = Self::init_wire_tracker(
            serialcirc,
            &mut dfg,
            &signature.input,
            options.input_params.iter().cloned(),
            &config,
        )?;

        if !serialcirc.phase.is_empty() {
            // TODO - add a phase gate
            // <https://github.com/quantinuum/tket2/issues/598>
            // let phase = Param::new(serialcirc.phase);
            // decoder.add_phase(phase);
        }

        Ok(PytketDecoderContext {
            builder: dfg,
            wire_tracker: Box::new(wire_tracker),
            config,
            extensions: options.extensions,
            opaque_subgraphs,
        })
    }

    /// Store the serialised circuit information as HUGR metadata,
    /// so it can be reused later when re-encoding the circuit.
    fn init_metadata(dfg: &mut DFGBuilder<&mut Hugr>, serialcirc: &SerialCircuit) {
        // Metadata. The circuit requires "name", and we store other things that
        // should pass through the serialization roundtrip.

        let node = dfg.container_node();
        dfg.hugr_mut()
            .set_metadata::<metadata::Phase>(node, &serialcirc.phase);
        dfg.hugr_mut()
            .set_metadata::<metadata::QubitRegisters>(node, serialcirc.qubits.clone()); // TODO Do we find something better than clone?
        dfg.hugr_mut()
            .set_metadata::<metadata::BitRegisters>(node, serialcirc.bits.clone());
    }

    /// Initialize the wire tracker with the input wires.
    ///
    /// Checks that the signature matches the expected number of qubits, bits,
    /// and parameters. See
    /// [`DecodeOptions::signature`][super::DecodeOptions::signature] for more
    /// details.
    ///
    /// Utility method for [`PytketDecoderContext::new`].
    ///
    /// # Panics
    ///
    /// If the dfg builder does not support adding input wires.
    /// (That is, we're not building a FuncDefn or a DFG).
    fn init_wire_tracker(
        serialcirc: &SerialCircuit,
        dfg: &mut DFGBuilder<&mut Hugr>,
        input_types: &TypeRow,
        input_params: impl IntoIterator<Item = String>,
        config: &PytketDecoderConfig,
    ) -> Result<WireTracker, PytketDecodeError> {
        let num_qubits = serialcirc.qubits.len();
        let num_bits = serialcirc.bits.len();
        let mut input_params = input_params.into_iter();

        let mut wire_tracker = WireTracker::with_capacity(num_qubits, num_bits);

        let mut qubits = serialcirc
            .qubits
            .iter()
            .map(|qb| TrackedQubit::new(TrackedQubitId(0), Arc::new(qb.id.clone())));
        let mut bits = serialcirc
            .bits
            .iter()
            .map(|bit| TrackedBit::new(TrackedBitId(0), Arc::new(bit.id.clone())));
        let mut added_inputs_count = RegisterCount::default();
        for (wire, ty) in dfg.input_wires().zip(input_types.iter()) {
            let elem_counts = config.type_to_pytket(ty).unwrap_or_default();

            if elem_counts.is_empty() {
                // Input is ignored.
                continue;
            }

            let wire_qubits = qubits.by_ref().take(elem_counts.qubits).collect_vec();
            let wire_bits = bits.by_ref().take(elem_counts.bits).collect_vec();
            // Error out if the signature inputs has more qubits than the ones defined in the circuit.
            // We ignore additional bits, and leave them disconnected.
            if wire_qubits.len() != elem_counts.qubits {
                let expected_count: RegisterCount = input_types
                    .iter()
                    .map(|t| config.type_to_pytket(t).unwrap_or_default())
                    .sum();
                return Err(PytketDecodeErrorInner::InvalidInputSignature {
                    input_types: input_types.iter().map(|t| t.to_string()).collect(),
                    expected_qubits: expected_count.qubits,
                    expected_bits: expected_count.bits,
                    circ_qubits: serialcirc.qubits.len(),
                    circ_bits: serialcirc.bits.len(),
                }
                .wrap());
            }
            wire_tracker.track_wire(wire, Arc::new(ty.clone()), wire_qubits, wire_bits)?;

            if elem_counts.params > 0 {
                if elem_counts != RegisterCount::only_params(1) {
                    return Err(PytketDecodeErrorInner::UnsupportedParametersInInput {
                        ty: ty.to_string(),
                        num_params: elem_counts.params,
                    }
                    .wrap());
                }
                let loaded = if ty == &rotation_type() {
                    LoadedParameter::rotation(wire)
                } else {
                    LoadedParameter::float_half_turns(wire)
                };
                match input_params.next() {
                    Some(param) => wire_tracker.register_input_parameter(loaded, param)?,
                    None => wire_tracker.register_unused_parameter_input(loaded),
                };
            }

            added_inputs_count += elem_counts;
        }

        // Insert any remaining named parameters as new inputs
        for param in input_params {
            let wire = dfg
                .add_input(rotation_type())
                .expect("Must be building a FuncDefn or a DFG");
            wire_tracker.register_input_parameter(LoadedParameter::rotation(wire), param)?;
        }

        // Any additional qubits or bits required by the circuit are registered
        // in the tracker without a wire being created.
        //
        // We'll lazily initialize them with a QAlloc or a LoadConstant
        // operation if necessary.
        for q in qubits {
            wire_tracker.track_qubit(q.pytket_register_arc(), Some(q.reg_hash()))?;
        }
        for b in bits {
            wire_tracker.track_bit(b.pytket_register_arc(), Some(b.reg_hash()))?;
        }

        wire_tracker.compute_output_permutation(&serialcirc.implicit_permutation);

        Ok(wire_tracker)
    }

    /// Finish building the function definition for the legacy tket circuit.
    ///
    /// After this call, the HUGR passed to the [`PytketDecoderContext::new`]
    /// constructor will contain the fully defined function.
    ///
    /// The original Hugr entrypoint is _not_ modified, it must be set by the
    /// caller if required.
    ///
    /// # Arguments
    ///
    /// - `encoded_info`: Information stored while encoding the circuit, used to
    ///   recover its original structure.
    pub(super) fn finish(
        mut self,
        encoded_info: Option<&EncodedCircuitInfo>,
    ) -> Result<Node, PytketDecodeError> {
        // Order the final wires according to the serial circuit register order.
        let known_qubits = self
            .wire_tracker
            .known_pytket_qubits()
            .cloned()
            .collect_vec();
        let known_bits = self.wire_tracker.known_pytket_bits().cloned().collect_vec();

        // Qubits and bits appearing at the output.
        let mut qubits: IndexSet<TrackedQubit> = IndexSet::new();
        let mut bits: IndexSet<TrackedBit> = IndexSet::new();

        if let Some(encoded_info) = encoded_info {
            for qubit in encoded_info.output_qubits.iter() {
                let id = self.wire_tracker.tracked_qubit_for_register(qubit)?;
                qubits.insert(id.clone());
            }
            for bit in encoded_info.output_bits.iter() {
                let id = self.wire_tracker.tracked_bit_for_register(bit)?;
                bits.insert(id.clone());
            }
        }
        // Add any additional qubits or bits we have seen, without modifying the
        // order of the qubits already there.
        qubits.extend(known_qubits);
        bits.extend(known_bits);
        let qubits: Vec<TrackedQubit> = Vec::from_iter(qubits);
        let bits: Vec<TrackedBit> = Vec::from_iter(bits);
        let mut qubits_slice: &[TrackedQubit] = &qubits;
        let mut bits_slice: &[TrackedBit] = &bits;

        // Load the output parameter expressions.
        let output_params = encoded_info
            .iter()
            .flat_map(|info| info.output_params.iter())
            .map(|p| self.load_half_turns(p))
            .collect_vec();
        let mut params: &[LoadedParameter] = &output_params;

        let function_type = self
            .builder
            .hugr()
            .get_optype(self.builder.container_node())
            .inner_function_type()
            .unwrap();
        let expected_output_types = function_type.output_types().iter().cloned().collect_vec();
        let [_, output_node] = self.builder.io();

        for (ty, port) in expected_output_types
            .iter()
            .zip(self.builder.hugr().node_inputs(output_node).collect_vec())
        {
            // If the region's output is already connected, leave it alone.
            // (It's a wire from an unsupported operation, or was a connected
            // straight through wire)
            if self.builder.hugr().is_linked(output_node, port) {
                continue;
            }

            // Otherwise, get the tracked wire.
            let found_wire = self
                .wire_tracker
                .find_typed_wire(
                    &self.config,
                    &mut self.builder,
                    ty,
                    &mut qubits_slice,
                    &mut bits_slice,
                    &mut params,
                    Some(EncodedEdgeID::default()),
                )
                .map_err(|mut e| {
                    if matches!(
                        e,
                        PytketDecodeError {
                            inner: PytketDecodeErrorInner::NoMatchingWire { .. },
                            ..
                        }
                    ) {
                        e.inner = PytketDecodeErrorInner::InvalidOutputSignature {
                            expected_types: expected_output_types
                                .iter()
                                .map(ToString::to_string)
                                .collect(),
                        };
                    }
                    e.hugr_op("Output")
                })?;

            let wire = match found_wire {
                FoundWire::Register(wire) => wire.wire(),

                FoundWire::Parameter(param) => {
                    let param_ty = if ty == &float64_type() {
                        ParameterType::FloatHalfTurns
                    } else {
                        ParameterType::Rotation
                    };
                    param.with_type(param_ty, &mut self.builder).wire()
                }
                FoundWire::Unsupported { .. } => {
                    // Disconnected port with an unsupported type. We just skip
                    // it, since it must have been disconnected in the original
                    // hugr too.
                    debug_assert!(
                        self.builder
                            .hugr()
                            .get_optype(output_node)
                            .port_kind(port)
                            .is_none_or(|kind| !kind.is_value())
                    );
                    continue;
                }
            };
            self.builder
                .hugr_mut()
                .connect(wire.node(), wire.source(), output_node, port);
        }

        // Qubits not in the output need to be freed.
        self.add_implicit_qfree_operations(qubits_slice);

        // Store the name for the input parameter wires
        let input_params = self.wire_tracker.finish();
        if !input_params.is_empty() {
            let node = self.builder.container_node();
            self.builder
                .hugr_mut()
                .set_metadata::<metadata::InputParameters>(
                    node,
                    input_params.into_iter().collect_vec(),
                );
        }

        Ok(self
            .builder
            .finish_sub_container()
            .map_err(PytketDecodeError::custom)?
            .node())
    }

    /// Add the implicit QFree operations for a list of qubits that are not in the hugr output.
    ///
    /// We only do this if there's a wire with type `qb_t` containing the qubit.
    fn add_implicit_qfree_operations(&mut self, qubits: &[TrackedQubit]) {
        let qb_type = qb_t();
        let mut bit_args: &[TrackedBit] = &[];
        let mut params: &[LoadedParameter] = &[];
        for q in qubits.iter() {
            // Ignore qubits that didn't get initialized
            if !self.wire_tracker.qubit_is_initialized(q) {
                continue;
            }

            let mut qubit_args: &[TrackedQubit] = std::slice::from_ref(q);
            let Ok(FoundWire::Register(wire)) = self.wire_tracker.find_typed_wire(
                &self.config,
                &mut self.builder,
                &qb_type,
                &mut qubit_args,
                &mut bit_args,
                &mut params,
                None,
            ) else {
                continue;
            };

            self.builder
                .add_dataflow_op(TketOp::QFree, [wire.wire()])
                .unwrap()
                .out_wire(0);
        }
    }

    /// Decode a list of pytket commands.
    ///
    /// # Arguments
    ///
    /// - `commands`: The list of pytket commands to decode.
    /// - `extra_subgraph`: An additional subgraph of the original Hugr that was
    ///   not encoded as a pytket command, and must be decoded independently.
    /// - `straight_through_wires`: A list of wires that directly connected the
    ///   input node to the output node in the original region, and were not
    ///   encoded in the pytket circuit or unsupported graphs.
    ///   (They cannot be encoded in `extra_subgraph`).
    pub(super) fn run_decoder(
        &mut self,
        commands: &[circuit_json::Command],
        extra_nodes_and_wires: Option<&AdditionalNodesAndWires>,
    ) -> Result<(), PytketDecodeError> {
        // Add additional subgraphs and wires not encoded in commands.
        let [input_node, output_node] = self.builder.io();
        if let Some(extras) = extra_nodes_and_wires {
            for extra_subgraph in &extras.additional_subgraphs {
                let params = extra_subgraph
                    .params
                    .iter()
                    .map(|p| self.load_half_turns(p))
                    .collect_vec();

                self.insert_external_subgraph(extra_subgraph.id, &[], &[], &params)
                    .map_err(|e| e.hugr_op("External subgraph"))?;
            }

            // Add wires from the input node to the output node that didn't get encoded in commands.
            for StraightThroughWire {
                input_source,
                output_target,
            } in &extras.straight_through_wires
            {
                self.builder.hugr_mut().connect(
                    input_node,
                    *input_source,
                    output_node,
                    *output_target,
                );
            }
        }

        // Decode the pytket commands.
        let config = self.config().clone();
        for com in commands {
            let op_type = com.op.op_type;
            self.process_command(com, config.as_ref())
                .map_err(|e| e.pytket_op(&op_type))?;
        }

        Ok(())
    }

    /// Add a tket1 [`circuit_json::Command`] from the serial circuit to the
    /// decoder.
    pub(super) fn process_command(
        &mut self,
        command: &circuit_json::Command,
        config: &PytketDecoderConfig,
    ) -> Result<(), PytketDecodeError> {
        let circuit_json::Command { op, args, opgroup } = command;

        // Find the latest [`TrackedQubit`] and [`TrackedBit`] for the command registers.
        let (qubits, bits) = self.wire_tracker.pytket_args_to_tracked_elems(args)?;

        // Collect the parameters used in the command.
        let params: Vec<LoadedParameter> = match &op.params {
            Some(params) => params
                .iter()
                .map(|v| self.load_half_turns(v.as_str()))
                .collect_vec(),
            None => Vec::new(),
        };

        // Try to decode the command with the configured decoders.
        match config.op_to_hugr(op, &qubits, &bits, &params, opgroup, self)? {
            DecodeStatus::Success => {}
            DecodeStatus::Unsupported => {
                // The command couldn't be translated into a native HUGR counterpart, so
                // we generate an opaque `Tk1Op` instead.
                build_opaque_tket_op(op, &qubits, &bits, &params, opgroup, self)?;
            }
        }
        Ok(())
    }
}

/// Public API, used by the [`PytketDecoder`][super::extension::PytketDecoder] implementers.
impl<'h> PytketDecoderContext<'h> {
    /// Returns a new set of [TrackedWires] for a list of [`TrackedQubit`]s,
    /// [`TrackedBit`]s, and [`LoadedParameter`]s following the required types.
    ///
    /// Returns an error if a valid set of wires with the given types cannot be
    /// found.
    ///
    /// The qubit and bit arguments are only consumed as required by the types.
    /// Some registers may be left unused.
    ///
    /// # Arguments
    ///
    /// * `config` - The configuration for the decoder, used to count the qubits and bits required by each type.
    /// * `hugr` - The hugr to load the parameters to.
    /// * `types` - The types of the arguments we require in the wires.
    /// * `qubit_args` - The list of tracked qubits we require in the wires.
    /// * `bit_args` - The list of tracked bits we require in the wire.
    /// * `params` - The list of parameters to load to wires. See
    ///   [`PytketDecoderContext::load_half_turns`] for more details.
    ///
    /// # Errors
    ///
    /// - [`PytketDecodeErrorInner::OutdatedQubit`] if a qubit in `qubit_args` was marked as outdated.
    /// - [`PytketDecodeErrorInner::OutdatedBit`] if a bit in `bit_args` was marked as outdated.
    /// - [`PytketDecodeErrorInner::UnexpectedInputType`] if a type in `types` cannot be mapped to a [`RegisterCount`]
    /// - [`PytketDecodeErrorInner::NoMatchingWire`] if there is no wire with the requested type for the given qubit/bit arguments.
    pub fn find_typed_wires(
        &mut self,
        types: &[Type],
        qubit_args: &[TrackedQubit],
        bit_args: &[TrackedBit],
        params: &[LoadedParameter],
    ) -> Result<TrackedWires, PytketDecodeError> {
        self.wire_tracker.find_typed_wires(
            &self.config,
            &mut self.builder,
            types,
            qubit_args,
            bit_args,
            params,
        )
    }

    /// Connects the input ports of a node using a list of input qubits, bits,
    /// and pytket parameters. Registers the node's output wires in the wire
    /// tracker.
    ///
    /// The qubits registers in `wires` are reused between the operation inputs
    /// and outputs. Bit registers, on the other hand, are not reused. We use
    /// the first registers in `wires` for the bit inputs and the remaining
    /// registers for the outputs.
    ///
    /// The input wire types must match the operation's input signature, no type
    /// conversion is performed.
    ///
    /// The caller must take care of converting the parameter wires to the
    /// required types and units expected by the operation. An error will be
    /// returned if the parameter does not match the expected wire type, but the
    /// unit (radians or half-turns) cannot be checked automatically.
    ///
    /// Use [`Self::add_node_with_wires`] to insert a new node before wiring it
    /// up.
    ///
    /// # Arguments
    ///
    /// - `node`: The node to wire up.
    /// - `input_qubits`: The qubits to use as input.
    ///   This list must match exactly the number of input qubits required by the operation.
    /// - `output_qubits`: The qubits to use as output.
    ///   This list must match exactly the number of output qubits required by the operation.
    /// - `input_bits`: The bits to use as input.
    ///   These should match exactly the number of input bits required by the operation.
    /// - `output_bits`: The bits to use as output.
    ///   These should match exactly the number of output bits required by the operation.
    /// - `params`: The parameters to use for the operation inputs.
    ///   This should match exactly the number of input parameters required by the operation.
    ///
    /// # Errors
    ///
    /// - [`PytketDecodeErrorInner::NotEnoughInputRegisters`] if the register
    ///   count required to encode the node inputs does not match the ones provided.
    /// - [`PytketDecodeErrorInner::NotEnoughOutputRegisters`] if the register
    ///   count required to encode the node outputs does not match the ones provided.
    /// - [`PytketDecodeErrorInner::OutdatedQubit`] if a qubit in `qubits` was marked as outdated.
    /// - [`PytketDecodeErrorInner::OutdatedBit`] if a bit in `bits` was marked as outdated.
    /// - [`PytketDecodeErrorInner::UnexpectedInputType`] if a type in the node's signature cannot be mapped to a [`RegisterCount`]
    pub fn wire_up_node(
        &mut self,
        node: Node,
        input_qubits: &[TrackedQubit],
        output_qubits: &[TrackedQubit],
        input_bits: &[TrackedBit],
        output_bits: &[TrackedBit],
        params: &[LoadedParameter],
    ) -> Result<(), PytketDecodeError> {
        let Some(sig) = self.builder.hugr().signature(node) else {
            return Err(PytketDecodeError::custom(
                "Cannot wire up non-dataflow operation",
            ));
        };

        // Compute the amount of elements required by the operation,
        // and the amount of elements in the input wires.
        let op_input_count: RegisterCount = sig
            .input_types()
            .iter()
            .map(|ty| self.config().type_to_pytket(ty).unwrap_or_default())
            .sum();
        let op_output_count: RegisterCount = sig
            .output_types()
            .iter()
            .map(|ty| self.config().type_to_pytket(ty).unwrap_or_default())
            .sum();

        // Validate input counts
        if op_input_count.qubits != input_qubits.len()
            || op_input_count.bits != input_bits.len()
            || op_input_count.params != params.len()
        {
            let expected_types = sig
                .input_types()
                .iter()
                .map(ToString::to_string)
                .collect_vec();
            return Err(PytketDecodeErrorInner::NotEnoughInputRegisters {
                expected_types,
                expected_count: op_input_count,
                actual_count: RegisterCount::new(
                    input_qubits.len(),
                    input_bits.len(),
                    params.len(),
                ),
            }
            .wrap());
        }

        // Validate output counts
        // We currently don't allow output parameters
        if op_output_count.qubits != output_qubits.len()
            || op_output_count.bits != output_bits.len()
            || op_output_count.params != 0
        {
            let expected_types = sig
                .output_types()
                .iter()
                .map(ToString::to_string)
                .collect_vec();
            return Err(PytketDecodeErrorInner::NotEnoughOutputRegisters {
                expected_types,
                expected_count: op_output_count,
                actual_count: RegisterCount::new(output_qubits.len(), output_bits.len(), 0),
            }
            .wrap());
        }

        // Gather the input wires, with the types needed by the operation.
        let input_types = sig.input_types().to_vec();
        let input_wires = self.find_typed_wires(&input_types, input_qubits, input_bits, params)?;
        debug_assert_eq!(op_input_count, input_wires.register_count());

        for (input_idx, wire) in input_wires.wires().enumerate() {
            self.builder
                .hugr_mut()
                .connect(wire.node(), wire.source(), node, input_idx);
        }
        input_bits.iter().take(op_input_count.bits).for_each(|b| {
            self.wire_tracker.mark_bit_outdated(b.clone());
        });

        // Register the output wires.
        let output_qubits = output_qubits.iter().take(op_output_count.qubits).cloned();
        let output_bits = output_bits.iter().cloned();
        self.register_node_outputs(node.node(), output_qubits, output_bits)?;

        Ok(())
    }

    /// Add a new node to the HUGR and wire it up using the provided wire set as
    /// input and output.
    ///
    /// Inserts the new node into the HUGR, connects its input ports and
    /// registers the node's output wires in the wire tracker.
    ///
    /// See [`PytketDecoderContext::wire_up_node`] for more details on the
    /// wiring process.
    ///
    /// # Arguments
    ///
    /// - `op`: The operation to add.
    /// - `input_qubits`: The qubits to use as input.
    ///   This list must match exactly the number of input qubits required by the operation.
    /// - `output_qubits`: The qubits to use as output.
    ///   This list must match exactly the number of output qubits required by the operation.
    /// - `input_bits`: The bits to use as input.
    ///   These should match exactly the number of input bits required by the operation.
    /// - `output_bits`: The bits to use as output.
    ///   These should match exactly the number of output bits required by the operation.
    /// - `params`: The parameters to use for the operation inputs.
    ///   This should match exactly the number of input parameters required by the operation.
    ///
    /// # Errors
    ///
    /// See [`PytketDecoderContext::wire_up_node`] for error details.
    pub fn add_node_with_wires(
        &mut self,
        op: impl Into<OpType>,
        input_qubits: &[TrackedQubit],
        output_qubits: &[TrackedQubit],
        input_bits: &[TrackedBit],
        output_bits: &[TrackedBit],
        params: &[LoadedParameter],
    ) -> Result<BuildHandle<DataflowOpID>, PytketDecodeError> {
        let op: OpType = op.into();
        let op_name = op.to_string();
        let num_outputs = op
            .dataflow_signature()
            .map(|s| s.output_count())
            .unwrap_or_default();

        // Add the node to the HUGR.
        let node = self.builder.add_child_node(op);

        self.wire_up_node(
            node.node(),
            input_qubits,
            output_qubits,
            input_bits,
            output_bits,
            params,
        )
        .map_err(|e| e.hugr_op(&op_name))?;

        Ok((node, num_outputs).into())
    }

    /// Given a new node in the HUGR, register all of its output wires in the
    /// tracker.
    ///
    /// Consumes the bits and qubits in order. Any unused bits and qubits are
    /// marked as outdated, as they are assumed to have been consumed in the
    /// inputs.
    pub fn register_node_outputs(
        &mut self,
        node: Node,
        qubits: impl IntoIterator<Item = TrackedQubit>,
        bits: impl IntoIterator<Item = TrackedBit>,
    ) -> Result<(), PytketDecodeError> {
        let mut qubits = qubits.into_iter();
        let mut bits = bits.into_iter();
        let Some(sig) = self.builder.hugr().signature(node) else {
            return Err(PytketDecodeErrorInner::UnexpectedNodeOutput {
                expected_qubits: qubits.count(),
                expected_bits: bits.count(),
                circ_qubits: 0,
                circ_bits: 0,
            }
            .wrap());
        };

        let mut reg_count = RegisterCount::default();
        let mut port_types = sig.output_ports().zip(sig.output_types().iter());
        while let Some((port, ty)) = port_types.next() {
            let wire = Wire::new(node, port);
            let counts = self.config().type_to_pytket(ty).unwrap_or_default();
            reg_count += counts;

            // Get the qubits and bits for this wire.
            let wire_qubits = qubits.by_ref().take(counts.qubits).collect_vec();
            let wire_bits = bits.by_ref().take(counts.bits).collect_vec();
            if wire_qubits.len() != counts.qubits || wire_bits.len() != counts.bits {
                let expected_qubits = reg_count.qubits - counts.qubits + wire_qubits.len();
                let expected_bits = reg_count.bits - counts.bits + wire_bits.len();
                return Err(make_unexpected_node_out_error(
                    self.config(),
                    port_types,
                    reg_count,
                    expected_qubits,
                    expected_bits,
                ));
            }

            self.wire_tracker
                .track_wire(wire, Arc::new(ty.clone()), wire_qubits, wire_bits)?;
        }

        // Mark any unused qubits and bits as outdated.
        qubits.for_each(|q| {
            self.wire_tracker.mark_qubit_outdated(q);
        });
        bits.for_each(|b| {
            self.wire_tracker.mark_bit_outdated(b);
        });

        Ok(())
    }

    /// Loads a half-turns expression as a [`LoadedParameter`] in the hugr.
    ///
    /// - If the parameter is a known algebraic operation, adds the required op and recurses on its inputs.
    /// - If the parameter is a constant, a constant definition is added to the Hugr.
    /// - If the parameter is a variable, adds a new `rotation` input to the region.
    /// - If the parameter is a sympy expressions, adds it as a [`SympyOpDef`][crate::extension::sympy::SympyOpDef] black box.
    pub fn load_half_turns(&mut self, param: &str) -> LoadedParameter {
        self.wire_tracker
            .load_half_turns_parameter(&mut self.builder, param, None)
    }

    /// Loads a half-turns expression as a [`LoadedParameter`] in the hugr, and
    /// converts it to the required parameter type.
    ///
    /// See [`PytketDecoderContext::load_half_turns`] for more details.
    pub fn load_half_turns_with_type(
        &mut self,
        param: &str,
        typ: ParameterType,
    ) -> LoadedParameter {
        self.wire_tracker
            .load_half_turns_parameter(&mut self.builder, param, Some(typ))
            .with_type(typ, &mut self.builder)
    }

    /// Returns the configuration used by the decoder.
    pub fn config(&self) -> &Arc<PytketDecoderConfig> {
        &self.config
    }

    /// Returns the extensions to use when decoding HUGR envelopes.
    ///
    /// If the option is `None`, we will use a default registry that includes
    /// the prelude, std, TKET1, and TketOps extensions.
    pub fn extension_registry(&self) -> &ExtensionRegistry {
        self.extensions
            .as_ref()
            .unwrap_or(&crate::extension::REGISTRY)
    }
}

/// Result of trying to decode pytket operation into a HUGR definition.
///
/// This flag indicates that either
/// - The operation was successful encoded and is now registered on the
///   [`PytketDecoderContext`]
/// - The operation cannot be encoded, and the context was left unchanged.
///
/// The latter is a recoverable error, as it promises that the context wasn't
/// modified. For non-recoverable errors, see [`PytketDecodeError`].
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash, derive_more::Display)]
pub enum DecodeStatus {
    /// The pytket operation was successfully encoded and registered in the
    /// context.
    Success,
    /// The pytket operation could not be encoded, and the context was left
    /// unchanged.
    Unsupported,
}

/// Helper to continue exhausting the iterators in
/// [`PytketDecoderContext::register_node_outputs`] until we have the total
/// number of elements to report.
///
/// Processes remaining port types and adds them to the partial count of the
/// number of qubits and bits we expected to have available.
fn make_unexpected_node_out_error<'ty>(
    config: &PytketDecoderConfig,
    port_types: impl IntoIterator<Item = (OutgoingPort, &'ty Type)>,
    mut partial_count: RegisterCount,
    expected_qubits: usize,
    expected_bits: usize,
) -> PytketDecodeError {
    for (_, ty) in port_types {
        partial_count += config.type_to_pytket(ty).unwrap_or_default();
    }
    PytketDecodeErrorInner::UnexpectedNodeOutput {
        expected_qubits,
        expected_bits,
        circ_qubits: partial_count.qubits,
        circ_bits: partial_count.bits,
    }
    .wrap()
}
