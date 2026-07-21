//! Encoder and decoder for tket operations with native pytket counterparts.

use super::PytketEmitter;
use crate::serialize::pytket::config::TypeTranslatorSet;
use crate::serialize::pytket::decoder::{
    DecodeStatus, LoadedParameter, PytketDecoderContext, TrackedBit, TrackedQubit,
};
use crate::serialize::pytket::encoder::{EmitCommandOptions, EncodeStatus, PytketEncoderContext};
use crate::serialize::pytket::extension::{PytketDecoder, PytketTypeTranslator, RegisterCount};
use crate::serialize::pytket::opaque::OpaqueSubgraphPayload;
use crate::serialize::pytket::{PytketDecodeError, PytketEncodeError};
use hugr::HugrView;
use hugr::extension::ExtensionId;
use hugr::extension::prelude::{BarrierDef, Noop, PRELUDE_ID, bool_t, qb_t};
use hugr::extension::simple_op::MakeExtensionOp;
use hugr::ops::{ExtensionOp, OpType};
use tket_json_rs::optype::OpType as PytketOptype;

/// Encoder for [prelude](hugr::extension::prelude) operations.
#[derive(Debug, Clone, Default)]
pub struct PreludeEmitter;

impl<H: HugrView> PytketEmitter<H> for PreludeEmitter {
    fn extensions(&self) -> Option<Vec<ExtensionId>> {
        Some(vec![PRELUDE_ID])
    }

    fn op_to_pytket(
        &self,
        node: H::Node,
        op: &ExtensionOp,
        hugr: &H,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<EncodeStatus, PytketEncodeError<H::Node>> {
        if let Ok(_barrier) = BarrierDef::from_extension_op(op) {
            // Check if the barrier has encodable types in its signature.
            // If not, fallback to marking it as unsupported.
            if hugr.signature(node).is_none_or(|sig| {
                sig.input()
                    .iter()
                    .chain(sig.output().iter())
                    .any(|ty| encoder.config().type_to_pytket(ty).is_none())
            }) {
                return Ok(EncodeStatus::Unsupported);
            }

            encoder.emit_node(
                PytketOptype::Barrier,
                node,
                hugr,
                EmitCommandOptions::new().reuse_all_bits(),
            )?;
            return Ok(EncodeStatus::Success);
        };
        Ok(EncodeStatus::Unsupported)
    }
}

impl PytketTypeTranslator for PreludeEmitter {
    fn extensions(&self) -> Vec<ExtensionId> {
        vec![PRELUDE_ID]
    }

    fn type_to_pytket(
        &self,
        typ: &hugr::types::CustomType,
        _set: &TypeTranslatorSet,
    ) -> Option<RegisterCount> {
        match typ.name().as_str() {
            "qubit" => Some(RegisterCount::only_qubits(1)),
            // We don't translate `usize`s currently, as none of the operations
            // that use them are translated to pytket.
            _ => None,
        }
    }
}

impl PytketDecoder for PreludeEmitter {
    fn op_types(&self) -> Vec<PytketOptype> {
        vec![PytketOptype::noop, PytketOptype::Barrier]
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
        let op: OpType = match op.op_type {
            PytketOptype::noop => Noop::new(qb_t()).into(),
            PytketOptype::Barrier => {
                // We use tket1 barriers as part of the encoder/decoder to
                // represent regions of the hugr that could not be encoded.
                //
                // Those are handled in in the `core.rs` decoder, so we should
                // ignore them here.
                if op
                    .data
                    .as_ref()
                    .is_some_and(|payload| OpaqueSubgraphPayload::is_valid_payload(payload))
                {
                    return Ok(DecodeStatus::Unsupported);
                }

                // For all other barrier commands, we emit a hugr Barrier.
                let types = [vec![qb_t(); qubits.len()], vec![bool_t(); bits.len()]].concat();
                hugr::extension::prelude::Barrier::new(types).into()
            }
            _ => return Ok(DecodeStatus::Unsupported),
        };
        if !params.is_empty() {
            return Ok(DecodeStatus::Unsupported);
        }
        decoder.add_node_with_wires(op, qubits, qubits, bits, &[], &[])?;

        Ok(DecodeStatus::Success)
    }
}
