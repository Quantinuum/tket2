//! Encoder and decoder for pytket global phase operations.

use super::{PytketDecoder, PytketEmitter};
use crate::extension::global_phase::{GLOBAL_PHASE_EXTENSION_ID, GlobalPhase};
use crate::serialize::pytket::decoder::{
    DecodeStatus, LoadedParameter, PytketDecoderContext, TrackedBit, TrackedQubit,
};
use crate::serialize::pytket::encoder::{EmitCommandOptions, EncodeStatus, PytketEncoderContext};
use crate::serialize::pytket::extension::RegisterCount;
use crate::serialize::pytket::{PytketDecodeError, PytketDecodeErrorInner, PytketEncodeError};
use hugr::HugrView;
use hugr::extension::ExtensionId;
use hugr::extension::simple_op::MakeExtensionOp;
use hugr::ops::ExtensionOp;
use tket_json_rs::optype::OpType as PytketOptype;

/// Decoder for pytket global phase operations.
#[derive(Debug, Clone, Default)]
pub struct GlobalPhaseEmitter;

impl PytketDecoder for GlobalPhaseEmitter {
    fn op_types(&self) -> Vec<PytketOptype> {
        vec![PytketOptype::Phase]
    }

    fn op_to_hugr<'h>(
        &self,
        _op: &tket_json_rs::circuit_json::Operation,
        qubits: &[TrackedQubit],
        bits: &[TrackedBit],
        params: &[LoadedParameter],
        _opgroup: Option<&str>,
        decoder: &mut PytketDecoderContext<'h>,
    ) -> Result<DecodeStatus, PytketDecodeError> {
        let count = RegisterCount::new(qubits.len(), bits.len(), params.len());
        if count != RegisterCount::only_params(1) {
            return Err(PytketDecodeErrorInner::NotEnoughInputRegisters {
                expected_types: vec!["rotation".to_string()],
                expected_count: RegisterCount::only_params(1),
                actual_count: count,
            }
            .wrap());
        }

        decoder.add_global_phase(params[0])?;
        Ok(DecodeStatus::Success)
    }
}

impl<H: HugrView> PytketEmitter<H> for GlobalPhaseEmitter {
    fn extensions(&self) -> Option<Vec<ExtensionId>> {
        Some(vec![GLOBAL_PHASE_EXTENSION_ID])
    }

    fn op_to_pytket(
        &self,
        node: H::Node,
        op: &ExtensionOp,
        hugr: &H,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<EncodeStatus, PytketEncodeError<H::Node>> {
        let Ok(GlobalPhase) = GlobalPhase::from_extension_op(op) else {
            return Ok(EncodeStatus::Unsupported);
        };

        encoder.emit_node(PytketOptype::Phase, node, hugr, EmitCommandOptions::new())?;
        Ok(EncodeStatus::Success)
    }
}
