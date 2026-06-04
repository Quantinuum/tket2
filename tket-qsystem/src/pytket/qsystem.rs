//! Encoder/decoder for [qsystem::EXTENSION][use crate::extension::qsystem::EXTENSION] operations.

use hugr::HugrView;
use hugr::extension::ExtensionId;
use hugr::extension::simple_op::MakeExtensionOp;
use hugr::ops::ExtensionOp;
use itertools::Itertools as _;
use tket::serialize::pytket::decoder::{
    DecodeStatus, LoadedParameter, ParameterType, PytketDecoderContext, TrackedBit, TrackedQubit,
};
use tket::serialize::pytket::encoder::{EmitCommandOptions, EncodeStatus, make_tk1_operation};
use tket::serialize::pytket::extension::PytketDecoder;
use tket::serialize::pytket::{
    PytketDecodeError, PytketEmitter, PytketEncodeError, PytketEncoderContext,
};
use tket_json_rs::optype::OpType as PytketOptype;

use crate::extension;
use crate::extension::qsystem::{
    QSystemPlatform, SharedOp,
    helios::{HeliosOp, RuntimeBarrierDef as HeliosRuntimeBarrierDef},
    sol::{RuntimeBarrierDef as SolRuntimeBarrierDef, SolOp},
};

/// Encoder/decoder for the native qsystem operations, parametrised by platform.
#[derive(Debug, Clone)]
pub struct QSystemEmitter(pub QSystemPlatform);

impl Default for QSystemEmitter {
    fn default() -> Self {
        Self(QSystemPlatform::Helios)
    }
}

impl<H: HugrView> PytketEmitter<H> for QSystemEmitter {
    fn extensions(&self) -> Option<Vec<ExtensionId>> {
        Some(match self.0 {
            QSystemPlatform::Helios => vec![extension::qsystem::helios::EXTENSION_ID],
            QSystemPlatform::Sol => vec![extension::qsystem::sol::EXTENSION_ID],
        })
    }

    fn op_to_pytket(
        &self,
        node: H::Node,
        op: &ExtensionOp,
        hugr: &H,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<EncodeStatus, PytketEncodeError<H::Node>> {
        match self.0 {
            QSystemPlatform::Helios => {
                if let Ok(helios_op) = HeliosOp::from_extension_op(op) {
                    self.encode_helios_op(node, helios_op, hugr, encoder)
                } else if let Ok(barrier) = HeliosRuntimeBarrierDef::from_extension_op(op) {
                    self.encode_runtime_barrier_op(node, barrier, hugr, encoder)
                } else {
                    Ok(EncodeStatus::Unsupported)
                }
            }
            QSystemPlatform::Sol => {
                if let Ok(sol_op) = SolOp::from_extension_op(op) {
                    self.encode_sol_op(node, sol_op, hugr, encoder)
                } else if let Ok(barrier) = SolRuntimeBarrierDef::from_extension_op(op) {
                    self.encode_runtime_barrier_op(node, barrier, hugr, encoder)
                } else {
                    Ok(EncodeStatus::Unsupported)
                }
            }
        }
    }
}

impl QSystemEmitter {
    /// Encode a Helios operation into a pytket operation.
    fn encode_helios_op<H: HugrView>(
        &self,
        node: H::Node,
        op: HeliosOp,
        hugr: &H,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<EncodeStatus, PytketEncodeError<H::Node>> {
        if let Ok(shared) = SharedOp::try_from(op) {
            return self.encode_shared_op(node, shared, hugr, encoder);
        }
        // Helios-specific ops.
        let serial_op = match op {
            HeliosOp::ZZPhase => PytketOptype::ZZPhase,
            _ => return Ok(EncodeStatus::Unsupported),
        };
        self.emit_radian_op(node, serial_op, hugr, encoder)
    }

    /// Encode a Sol operation into a pytket operation.
    fn encode_sol_op<H: HugrView>(
        &self,
        node: H::Node,
        op: SolOp,
        hugr: &H,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<EncodeStatus, PytketEncodeError<H::Node>> {
        if let Ok(shared) = SharedOp::try_from(op) {
            return self.encode_shared_op(node, shared, hugr, encoder);
        }
        // Sol-specific ops.
        let serial_op = match op {
            SolOp::PhasedXX => PytketOptype::PhasedXX,
            _ => return Ok(EncodeStatus::Unsupported),
        };
        self.emit_radian_op(node, serial_op, hugr, encoder)
    }

    /// Encode a shared operation into a pytket operation.
    fn encode_shared_op<H: HugrView>(
        &self,
        node: H::Node,
        op: SharedOp,
        hugr: &H,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<EncodeStatus, PytketEncodeError<H::Node>> {
        let serial_op = match op {
            // "Lazy" operations are translated as eager measurements in pytket,
            // as there is no `Future<T>` type there.
            SharedOp::Measure | SharedOp::LazyMeasure => PytketOptype::Measure,
            SharedOp::Rz => PytketOptype::Rz,
            SharedOp::PhasedX => PytketOptype::PhasedX,
            SharedOp::Reset => PytketOptype::Reset,
            SharedOp::QFree => {
                // Mark the qubit inputs as explored and forget about them.
                encoder.get_input_values(node, hugr)?;
                return Ok(EncodeStatus::Success);
            }
            SharedOp::LazyMeasureReset | SharedOp::MeasureReset => {
                // These may require a pytket measurement followed by a reset.
                return Ok(EncodeStatus::Unsupported);
            }
            SharedOp::LazyMeasureLeaked => {
                // No equivalent pytket operation.
                return Ok(EncodeStatus::Unsupported);
            }
            SharedOp::TryQAlloc => {
                // Pytket circuits don't support the optional type returned by `TryQAlloc`.
                return Ok(EncodeStatus::Unsupported);
            }
        };
        self.emit_radian_op(node, serial_op, hugr, encoder)
    }

    /// Emit a node command, converting parameter radians to half-turns.
    fn emit_radian_op<H: HugrView>(
        &self,
        node: H::Node,
        serial_op: PytketOptype,
        hugr: &H,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<EncodeStatus, PytketEncodeError<H::Node>> {
        // pytket parameters are always in half-turns.
        // Since the `tket.qsystem` op inputs are in radians, we have to convert them here.
        encoder.emit_node_command(node, hugr, EmitCommandOptions::new(), move |mut inputs| {
            for param in inputs.params.to_mut() {
                *param = match param.strip_suffix(") * (pi)") {
                    Some(s) if s.starts_with("(") => s[1..].to_string(),
                    _ => format!("{param} / (pi)"),
                };
            }
            make_tk1_operation(serial_op, inputs)
        })?;
        Ok(EncodeStatus::Success)
    }

    fn encode_runtime_barrier_op<H: HugrView, B>(
        &self,
        node: H::Node,
        _runtime_barrier_op: B,
        hugr: &H,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<EncodeStatus, PytketEncodeError<H::Node>> {
        encoder.emit_node(
            PytketOptype::Barrier,
            node,
            hugr,
            EmitCommandOptions::new().reuse_all_bits(),
        )?;
        Ok(EncodeStatus::Success)
    }
}

impl PytketDecoder for QSystemEmitter {
    fn op_types(&self) -> Vec<PytketOptype> {
        // Process native optypes with direct qsystem counterparts.
        //
        // Some of these overlap with what the `TketOp` emitter can decode. The
        // decoder used for those cases will be the first one registered in the
        // [`PytketDecoderConfig`].
        let mut ops = vec![PytketOptype::Rz, PytketOptype::PhasedX];
        match self.0 {
            QSystemPlatform::Helios => {
                ops.extend([PytketOptype::ZZPhase, PytketOptype::ZZMax]);
            }
            QSystemPlatform::Sol => {
                ops.extend([PytketOptype::PhasedXX]);
            }
        }
        ops
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
        // Converts a SharedOp to the platform-appropriate OpType.
        let platform_op = |s: SharedOp| self.0.shared_op_type(s);
        // Collects all parameters as floats in radians.
        let float_params = |decoder: &mut PytketDecoderContext<'h>| {
            params
                .iter()
                .map(|p| p.as_float_radians(&mut decoder.builder))
                .collect_vec()
        };

        let (hugr_op, p): (hugr::ops::OpType, Vec<LoadedParameter>) = match (self.0, op.op_type) {
            (_, PytketOptype::Rz) => (platform_op(SharedOp::Rz), float_params(decoder)),
            (_, PytketOptype::PhasedX) => (platform_op(SharedOp::PhasedX), float_params(decoder)),
            (QSystemPlatform::Helios, PytketOptype::ZZPhase) => {
                (HeliosOp::ZZPhase.into(), float_params(decoder))
            }
            (QSystemPlatform::Helios, PytketOptype::ZZMax) => {
                // ZZPhase with a 1/2 angle.
                let param = decoder.load_half_turns_with_type("0.5", ParameterType::FloatRadians);
                (HeliosOp::ZZPhase.into(), vec![param])
            }
            (QSystemPlatform::Sol, PytketOptype::PhasedXX) => {
                (SolOp::PhasedXX.into(), float_params(decoder))
            }
            _ => return Ok(DecodeStatus::Unsupported),
        };

        decoder.add_node_with_wires(hugr_op, qubits, qubits, bits, &[], &p)?;
        Ok(DecodeStatus::Success)
    }
}
