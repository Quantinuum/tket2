//! Encoder for pytket operations that cannot be represented naturally in tket.

use crate::Circuit;
use crate::extension::rotation::rotation_type;
use crate::extension::{TKET1_EXTENSION, TKET1_EXTENSION_ID, TKET1_OP_NAME};
use crate::serialize::pytket::decoder::{
    LoadedParameter, PytketDecoderContext, TrackedBit, TrackedQubit,
};
use crate::serialize::pytket::encoder::{EmitCommandOptions, EncodeStatus};
use crate::serialize::pytket::{PytketDecodeError, PytketEncodeError, PytketEncoderContext};

use super::PytketEmitter;
use hugr::builder::Dataflow;
use hugr::extension::ExtensionId;
use hugr::extension::prelude::{bool_t, qb_t};
use hugr::ops::{ExtensionOp, OpTrait, OpType};
use hugr::types::{Signature, TypeArg};
use hugr::{HugrView, IncomingPort};
use itertools::Itertools;
use tket_json_rs::circuit_json;

/// Encoder for [TKET1_EXTENSION] operations.
///
/// That is, operations originating from a pytket circuit that did not have a
/// native HUGR representation and were instead serialized as opaque black-box
/// operations.
#[derive(Debug, Clone, Default)]
pub struct Tk1Emitter;

impl<H: HugrView> PytketEmitter<H> for Tk1Emitter {
    fn extensions(&self) -> Option<Vec<ExtensionId>> {
        Some(vec![TKET1_EXTENSION_ID])
    }

    fn op_to_pytket(
        &self,
        node: H::Node,
        op: &ExtensionOp,
        circ: &Circuit<H>,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<EncodeStatus, PytketEncodeError<H::Node>> {
        if op.qualified_id() != format!("{TKET1_EXTENSION_ID}.{TKET1_OP_NAME}") {
            return Ok(EncodeStatus::Unsupported);
        }
        let Some(TypeArg::String(arg)) = op.args().first() else {
            return Err(PytketEncodeError::custom(
                "Opaque TKET1 operation did not have a json-encoded type argument.",
            ));
        };
        let op: OpaqueTk1Op = serde_json::from_str(arg).map_err(PytketEncodeError::custom)?;

        // Most operations map directly to a pytket one.
        encoder.emit_node_command(
            node,
            circ,
            EmitCommandOptions::new(),
            // Emit the pre-defined pytket operation stored in the metadata.
            move |_| op.serialised_op,
        )?;

        Ok(EncodeStatus::Success)
    }
}

/// Add an [`OpaqueTk1Op`] to the Hugr, representing a pytket operation that could not
/// be decoded by the configured decoders.
///
/// We don't implement [`PytketDecoder`][super::PytketDecoder] for [`Tk1Emitter`] so it doesn't get added
/// to the [`PytketDecoderConfig`][crate::serialize::pytket::PytketDecoderConfig] by mistake.
///
/// This function is used internally by the [`PytketDecoderContext`] as a fallback.
///
/// TODO: This only accepts input/outputs composed of bare qubit, bit, and parameter wires.
/// We should accept arbitrary wires, but the opaque extension op needs to be modified (or replaced with a new one)
/// since it currently has a limited signature definition.
pub(crate) fn build_opaque_tket_op<'h>(
    op: &tket_json_rs::circuit_json::Operation,
    qubits: &[TrackedQubit],
    bits: &[TrackedBit],
    params: &[LoadedParameter],
    _opgroup: &Option<String>,
    decoder: &mut PytketDecoderContext<'h>,
) -> Result<(), PytketDecodeError> {
    let tk1op: OpType = OpaqueTk1Op::new_from_op(op, qubits.len(), bits.len())
        .as_extension_op()
        .into();
    let op_name = tk1op.to_string();

    // Gather the input wires.
    // The wires all must have raw qubit/bit/parameter types.
    let op_sig = tk1op.dataflow_signature().unwrap();

    let wires = decoder
        .find_typed_wires(op_sig.input_types(), qubits, bits, params)
        .map_err(|e| e.hugr_op(&op_name))?;

    // Ensure all parameter inputs have rotation types rather than float.
    let param_wires = wires
        .iter_parameters()
        .map(|p| p.as_rotation(&mut decoder.builder).wire())
        .collect_vec();

    let opaque_op = decoder
        .builder
        .add_dataflow_op(tk1op, wires.value_wires().chain(param_wires))
        .map_err(|e| PytketDecodeError::custom(e).hugr_op(&op_name))?;

    // Associate the output wires to the corresponding register.
    let mut outputs = opaque_op.outputs();

    for qubit in qubits {
        let wire = outputs.next().expect("Qubit should have an output wire");
        decoder
            .wire_tracker
            .track_wire(wire, qubit.ty(), [qubit.clone()], [])
            .map_err(|e| e.hugr_op(&op_name))?;
    }
    for bit in bits {
        let wire = outputs.next().expect("Bit should have an output wire");
        decoder
            .wire_tracker
            .track_wire(wire, bit.ty(), [], [bit.clone()])
            .map_err(|e| e.hugr_op(&op_name))?;
    }

    Ok(())
}

/// A serialized operation, containing the operation type and all its attributes.
///
/// This value is only used if the operation does not have a native TKET
/// counterpart that can be represented as a [`NativeOp`].
///
/// Wrapper around [`tket_json_rs::circuit_json::Operation`] with cached number of qubits and bits.
///
/// The `Operation` contained by this struct is guaranteed to have a signature.
///
///   [`NativeOp`]: super::native::NativeOp
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct OpaqueTk1Op {
    /// Internal operation data.
    #[serde(rename = "op")]
    pub serialised_op: circuit_json::Operation,
    /// Number of qubits declared by the operation.
    pub num_qubits: usize,
    /// Number of bits declared by the operation.
    pub num_bits: usize,
    /// Node input for each parameter in `op.params`.
    ///
    /// If the input is `None`, the parameter does not use a Hugr port and is
    /// instead stored purely as metadata for the `Operation`.
    pub param_inputs: Vec<Option<IncomingPort>>,
    /// The number of non-None inputs in `param_inputs`, corresponding to the
    /// rotation_type() inputs to the Hugr operation.
    pub num_params: usize,
}

impl OpaqueTk1Op {
    /// Create a new `OpaqueTk1Op` from a `circuit_json::Operation`, with the number
    /// of qubits and bits explicitly specified.
    ///
    /// If the operation does not define a signature, one is generated with the
    /// given amounts.
    pub fn new_from_op(op: &circuit_json::Operation, num_qubits: usize, num_bits: usize) -> Self {
        let mut op = op.clone();

        if op.signature.is_none() {
            op.signature =
                Some([vec!["Q".into(); num_qubits], vec!["B".into(); num_bits]].concat());
        }
        let mut op = Self {
            serialised_op: op,
            num_qubits,
            num_bits,
            param_inputs: Vec::new(),
            num_params: 0,
        };
        op.compute_param_fields();
        op
    }

    /// Compute the signature of the operation.
    ///
    /// The signature returned has `num_qubits` qubit inputs, followed by
    /// `num_bits` bit inputs, followed by `num_params` `f64` inputs. It has
    /// `num_qubits` qubit outputs followed by `num_bits` bit outputs.
    #[inline]
    pub fn signature(&self) -> Signature {
        let linear = [
            vec![qb_t(); self.num_qubits],
            vec![bool_t().clone(); self.num_bits],
        ]
        .concat();
        let params = vec![rotation_type(); self.num_params];
        Signature::new([linear.clone(), params].concat(), linear)
    }

    /// Wraps the op into a [`TKET1_OP_NAME`] opaque operation.
    pub fn as_extension_op(&self) -> ExtensionOp {
        let payload = TypeArg::String(serde_json::to_string(self).unwrap());
        let op_def = TKET1_EXTENSION.get_op(&TKET1_OP_NAME).unwrap();
        ExtensionOp::new(op_def.clone(), vec![payload]).unwrap_or_else(|e| panic!("{e}"))
    }

    /// Compute the `parameter_input` and `num_params` fields by looking for
    /// parameters in `op.params` that can be mapped to input wires in the Hugr.
    ///
    /// Updates the internal `num_params` and `param_inputs` fields.
    fn compute_param_fields(&mut self) {
        let Some(params) = self.serialised_op.params.as_ref() else {
            self.param_inputs = vec![];
            self.num_params = 0;
            return;
        };

        self.num_params = params.len();
        self.param_inputs = (0..params.len()).map(|i| Some(i.into())).collect();
    }
}
