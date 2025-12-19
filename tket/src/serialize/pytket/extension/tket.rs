//! Encoder and decoder for tket operations with native pytket counterparts.

use super::PytketEmitter;
use crate::extension::TKET_EXTENSION_ID;
use crate::extension::sympy::SympyOp;
use crate::serialize::pytket::decoder::{
    DecodeStatus, LoadedParameter, PytketDecoderContext, TrackedBit, TrackedQubit,
};
use crate::serialize::pytket::encoder::{EmitCommandOptions, EncodeStatus, PytketEncoderContext};
use crate::serialize::pytket::extension::PytketDecoder;
use crate::serialize::pytket::{PytketDecodeError, PytketEncodeError};
use crate::{Circuit, TketOp};
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
        circ: &Circuit<H>,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<EncodeStatus, PytketEncodeError<H::Node>> {
        if let Ok(tket_op) = TketOp::from_extension_op(op) {
            self.encode_tket_op(node, tket_op, circ, encoder)
        } else if let Ok(sympy_op) = SympyOp::from_extension_op(op) {
            self.encode_sympy_op(node, sympy_op, circ, encoder)
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
        circ: &Circuit<H>,
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
            TketOp::Measure => PytketOptype::Measure,
            // We translate `MeasureFree` the same way as a `Measure` operation.
            // Since the node does not have outputs the qubit/bit will simply be ignored,
            // but will appear when collecting the final pytket registers.
            TketOp::MeasureFree => PytketOptype::Measure,
            // These operations are implicitly supported by the encoding,
            // they do not create a new command but just modify the value trackers.
            TketOp::QAlloc => {
                let out_port = circ.hugr().node_outputs(node).next().unwrap();
                let wire = Wire::new(node, out_port);
                let qb = encoder.values.new_qubit();
                encoder.values.register_wire(wire, [qb], circ)?;
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
        encoder.emit_node(serial_op, node, circ, EmitCommandOptions::new())?;

        Ok(EncodeStatus::Success)
    }

    /// Encode a tket sympy operation into a pytket operation.
    fn encode_sympy_op<H: HugrView>(
        &self,
        node: H::Node,
        sympy_op: SympyOp,
        circ: &Circuit<H>,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<EncodeStatus, PytketEncodeError<H::Node>> {
        encoder.emit_transparent_node(node, circ, |_| vec![sympy_op.expr.clone()])?;
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
        let mut num_input_bits = 0;
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
            PytketOptype::Measure => {
                num_input_bits = 0;
                TketOp::Measure
            }
            _ => {
                return Ok(DecodeStatus::Unsupported);
            }
        };

        // We expect all parameters to be rotations in half-turns.
        let params = params
            .iter()
            .map(|p| p.as_rotation(&mut decoder.builder))
            .collect_vec();

        let input_bits = &bits[..num_input_bits];
        let output_bits = &bits[num_input_bits..];
        decoder.add_node_with_wires(op, qubits, qubits, input_bits, output_bits, &params)?;

        Ok(DecodeStatus::Success)
    }
}
