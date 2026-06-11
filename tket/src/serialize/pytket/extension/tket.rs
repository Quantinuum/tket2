//! Encoder and decoder for tket operations with native pytket counterparts.

use std::borrow::Cow;

use super::PytketEmitter;
use crate::extension::TKET_EXTENSION_ID;
use crate::extension::measurement::MeasurementOp;
use crate::extension::sympy::SympyOp;
use crate::serialize::pytket::decoder::{
    DecodeStatus, LoadedParameter, PytketDecoderContext, TrackedBit, TrackedQubit,
};
use crate::serialize::pytket::encoder::{
    EmitCommandOptions, EncodeStatus, MakeOperationArgs, PytketEncoderContext, TrackedValue,
    TrackedValues, make_tk1_operation,
};
use crate::serialize::pytket::extension::PytketDecoder;
use crate::serialize::pytket::{PytketDecodeError, PytketEncodeError};
use crate::{TketOp, metadata};
use hugr::extension::ExtensionId;
use hugr::extension::simple_op::MakeExtensionOp;
use hugr::ops::ExtensionOp;
use hugr::{HugrView, Wire};
use itertools::Itertools as _;
use tket_json_rs::optype::OpType as PytketOptype;

/// Encoder for [TketOp] operations.
#[derive(Debug, Clone, Default)]
pub struct TketOpEmitter;

impl<H: HugrView> PytketEmitter<H> for TketOpEmitter {
    fn extensions(&self) -> Option<Vec<ExtensionId>> {
        Some(vec![TKET_EXTENSION_ID])
    }

    fn op_to_pytket(
        &self,
        node: H::Node,
        op: &ExtensionOp,
        hugr: &H,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<EncodeStatus, PytketEncodeError<H::Node>> {
        if let Ok(tket_op) = TketOp::from_extension_op(op) {
            self.encode_tket_op(node, tket_op, hugr, encoder)
        } else if let Ok(sympy_op) = SympyOp::from_extension_op(op) {
            self.encode_sympy_op(node, sympy_op, hugr, encoder)
        } else {
            Ok(EncodeStatus::Unsupported)
        }
    }
}

impl TketOpEmitter {
    /// Encode a tket operation into a pytket operation.
    fn encode_tket_op<H: HugrView>(
        &self,
        node: H::Node,
        tket_op: TketOp,
        hugr: &H,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<EncodeStatus, PytketEncodeError<H::Node>> {
        let serial_op = match tket_op {
            TketOp::H => PytketOptype::H,
            TketOp::CX => PytketOptype::CX,
            TketOp::CY => PytketOptype::CY,
            TketOp::CZ => PytketOptype::CZ,
            TketOp::CRz => PytketOptype::CRz,
            TketOp::T => PytketOptype::T,
            TketOp::Tdg => PytketOptype::Tdg,
            TketOp::S => PytketOptype::S,
            TketOp::Sdg => PytketOptype::Sdg,
            TketOp::V => PytketOptype::V,
            TketOp::Vdg => PytketOptype::Vdg,
            TketOp::X => PytketOptype::X,
            TketOp::Y => PytketOptype::Y,
            TketOp::Z => PytketOptype::Z,
            TketOp::Rx => PytketOptype::Rx,
            TketOp::Rz => PytketOptype::Rz,
            TketOp::Ry => PytketOptype::Ry,
            TketOp::Toffoli => PytketOptype::CCX,
            TketOp::Reset => PytketOptype::Reset,
            // Because `Measure` returns a bool, we can translate the op directly.
            TketOp::Measure => PytketOptype::Measure,
            // We translate `MeasureFree` the same way as a `Measure` operation.
            // Since the node does not have outputs the qubit/bit will simply be
            // ignored, but will appear when collecting the final pytket registers.
            // However, as `MeasureFree` returns a `Measurement` type, we need to
            // check whether it is directly followed by a `Read` turning it into a
            // bool before we can translate it.
            TketOp::MeasureFree => {
                let is_read = hugr
                    .node_outputs(node)
                    .next()
                    .and_then(|out_port| hugr.single_linked_input(node, out_port))
                    .and_then(|(next_node, _)| hugr.get_optype(next_node).as_extension_op())
                    .and_then(|op| MeasurementOp::from_extension_op(op).ok())
                    .is_some_and(|op| op == MeasurementOp::Read);
                if !is_read {
                    // `MeasureFree` not immediately followed by a `Read` is not
                    // supported as we cannot directly translate the `Measurement` type.
                    return Ok(EncodeStatus::Unsupported);
                }
                self.emit_measure_free(node, hugr, encoder)?;
                return Ok(EncodeStatus::Success);
            }
            // These operations are implicitly supported by the encoding,
            // they do not create a new command but just modify the value trackers.
            TketOp::QAlloc => {
                let out_port = hugr.node_outputs(node).next().unwrap();
                let wire = Wire::new(node, out_port);
                let qb = encoder.values.new_qubit();
                encoder.values.register_wire(wire, [qb], hugr)?;
                return Ok(EncodeStatus::Success);
            }
            // Since the qubit still gets connected at the end of the circuit,
            // `QFree` is a no-op.
            TketOp::QFree => {
                return Ok(EncodeStatus::Success);
            }
            // Unsupported
            TketOp::TryQAlloc => {
                return Ok(EncodeStatus::Unsupported);
            }
        };

        // Most operations map directly to a pytket one.
        encoder.emit_node(serial_op, node, hugr, EmitCommandOptions::new())?;

        Ok(EncodeStatus::Success)
    }

    fn emit_measure_free<H: HugrView>(
        &self,
        node: H::Node,
        hugr: &H,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<(), PytketEncodeError<H::Node>> {
        let pytket_op = PytketOptype::Measure;
        // We cannot use `emit_node` because we need to manually handle the wire of
        // type `Measurement` and associate it with a bit value.
        let TrackedValues {
            qubits,
            mut bits,
            params,
        } = encoder.get_input_values(node, hugr)?;

        // Qubits stay the same, but we need to associate the measurement wire to a new
        // bit. We assume there is only one output wire, as `MeasureFree` should have
        // exactly one output of type `Measurement`.
        let msmt_outport = hugr.node_outputs(node).next().unwrap();
        let msmt_wire = hugr::Wire::new(node, msmt_outport);
        let bit = encoder.values.new_bit();
        bits.push(bit);
        encoder
            .values
            .register_wire(msmt_wire, vec![TrackedValue::Bit(bit)], hugr)?;

        // The rest follows `emit_node_command` (as if called by `emit_node`).
        let params: Vec<String> = params
            .into_iter()
            .map(|p| encoder.values.param_expression(p).to_owned())
            .collect();

        let opgroup: Option<String> = hugr
            .get_metadata::<metadata::PytketOpGroup>(node)
            .map(ToString::to_string);

        let args = MakeOperationArgs {
            num_qubits: qubits.len(),
            num_bits: bits.len(),
            params: Cow::Borrowed(&params),
        };
        let op = make_tk1_operation(pytket_op, args);
        encoder.emit_command(op, &qubits, &bits, opgroup);
        Ok(())
    }

    /// Encode a tket sympy operation into a pytket operation.
    fn encode_sympy_op<H: HugrView>(
        &self,
        node: H::Node,
        sympy_op: SympyOp,
        hugr: &H,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<EncodeStatus, PytketEncodeError<H::Node>> {
        encoder.emit_transparent_node(node, hugr, |_| vec![sympy_op.expr.clone()])?;
        Ok(EncodeStatus::Success)
    }
}

impl PytketDecoder for TketOpEmitter {
    fn op_types(&self) -> Vec<PytketOptype> {
        vec![
            PytketOptype::H,
            PytketOptype::CX,
            PytketOptype::CY,
            PytketOptype::CZ,
            PytketOptype::CRz,
            PytketOptype::T,
            PytketOptype::Tdg,
            PytketOptype::S,
            PytketOptype::Sdg,
            PytketOptype::V,
            PytketOptype::Vdg,
            PytketOptype::X,
            PytketOptype::Y,
            PytketOptype::Z,
            PytketOptype::Rx,
            PytketOptype::Rz,
            PytketOptype::Ry,
            PytketOptype::CCX,
            PytketOptype::Reset,
            PytketOptype::Measure,
        ]
    }

    fn op_to_hugr<'h>(
        &self,
        op: &tket_json_rs::circuit_json::Operation,
        qubits: &[TrackedQubit],
        bits: &[TrackedBit],
        params: &[LoadedParameter],
        _opgroup: Option<&str>,
        decoder: &mut PytketDecoderContext<'h>,
    ) -> Result<DecodeStatus, PytketDecodeError> {
        let op = match op.op_type {
            PytketOptype::H => TketOp::H,
            PytketOptype::CX => TketOp::CX,
            PytketOptype::CY => TketOp::CY,
            PytketOptype::CZ => TketOp::CZ,
            PytketOptype::CRz => TketOp::CRz,
            PytketOptype::T => TketOp::T,
            PytketOptype::Tdg => TketOp::Tdg,
            PytketOptype::S => TketOp::S,
            PytketOptype::Sdg => TketOp::Sdg,
            PytketOptype::V => TketOp::V,
            PytketOptype::Vdg => TketOp::Vdg,
            PytketOptype::X => TketOp::X,
            PytketOptype::Y => TketOp::Y,
            PytketOptype::Z => TketOp::Z,
            PytketOptype::Rx => TketOp::Rx,
            PytketOptype::Ry => TketOp::Ry,
            PytketOptype::Rz => TketOp::Rz,
            PytketOptype::CCX => TketOp::Toffoli,
            PytketOptype::Reset => TketOp::Reset,
            // In a roundtrip both `Measure` and `MeasureFree` end up as a `Measure`.
            // Given a `MeasureFree` immediately followed by a `Read` is equivalent to
            //  a `Measure` (which returns a bool directly) (followed by `QFree` which
            // is automatically inserted), and as that immediate `Read` is required to
            // support the encoding, we can do this safely.
            PytketOptype::Measure => TketOp::Measure,
            _ => {
                return Ok(DecodeStatus::Unsupported);
            }
        };

        // We expect all parameters to be rotations in half-turns.
        let params = params
            .iter()
            .map(|p| p.as_rotation(&mut decoder.builder))
            .collect_vec();

        let num_input_bits = 0;
        let input_bits = &bits[..num_input_bits];
        let output_bits = &bits[num_input_bits..];
        decoder.add_node_with_wires(op, qubits, qubits, input_bits, output_bits, &params)?;

        Ok(DecodeStatus::Success)
    }
}
