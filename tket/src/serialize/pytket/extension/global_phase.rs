//! Decoder for pytket global phase operations.

use super::PytketDecoder;
use crate::serialize::pytket::decoder::{
    DecodeStatus, LoadedParameter, PytketDecoderContext, TrackedBit, TrackedQubit,
};
use crate::serialize::pytket::extension::RegisterCount;
use crate::serialize::pytket::{PytketDecodeError, PytketDecodeErrorInner};
use tket_json_rs::optype::OpType as PytketOptype;

/// Decoder for pytket global phase operations.
#[derive(Debug, Clone, Default)]
pub struct GlobalPhaseDecoder;

impl PytketDecoder for GlobalPhaseDecoder {
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
