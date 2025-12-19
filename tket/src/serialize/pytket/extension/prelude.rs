//! Encoder and decoder for tket operations with native pytket counterparts.

use super::PytketEmitter;
use crate::Circuit;
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
use hugr::extension::prelude::{BarrierDef, Noop, PRELUDE_ID, TupleOpDef, bool_t, qb_t};
use hugr::extension::simple_op::MakeExtensionOp;
use hugr::ops::{ExtensionOp, OpType};
use hugr::types::TypeArg;
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
        circ: &Circuit<H>,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<EncodeStatus, PytketEncodeError<H::Node>> {
        if let Ok(tuple_op) = TupleOpDef::from_extension_op(op) {
            return self.tuple_op_to_pytket(node, op, &tuple_op, circ, encoder);
        };
        if let Ok(_barrier) = BarrierDef::from_extension_op(op) {
            encoder.emit_node(
                PytketOptype::Barrier,
                node,
                circ,
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

impl PreludeEmitter {
    /// Encode a prelude tuple operation.
    ///
    /// These just bundle/unbundle the values of the inputs/outputs. Since
    /// pytket types are already flattened, the translation of these is just a
    /// no-op.
    fn tuple_op_to_pytket<H: HugrView>(
        &self,
        node: H::Node,
        op: &ExtensionOp,
        tuple_op: &TupleOpDef,
        circ: &Circuit<H>,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<EncodeStatus, PytketEncodeError<H::Node>> {
        if !matches!(tuple_op, TupleOpDef::MakeTuple | TupleOpDef::UnpackTuple) {
            // Unknown operation
            return Ok(EncodeStatus::Unsupported);
        };

        // First, check if we are working with supported types.
        //
        // If any of the types cannot be translated to a pytket type, we return
        // false so the operation is marked as unsupported as a whole.
        let args = op.args().first();
        match args {
            Some(TypeArg::Tuple(elems)) | Some(TypeArg::List(elems)) => {
                if elems.is_empty() {
                    return Ok(EncodeStatus::Unsupported);
                }

                for arg in elems {
                    let TypeArg::Runtime(ty) = arg else {
                        return Ok(EncodeStatus::Unsupported);
                    };
                    let count = encoder.config().type_to_pytket(ty);
                    if count.is_none_or(|c| c.params > 0) {
                        return Ok(EncodeStatus::Unsupported);
                    }
                }
            }
            _ => return Ok(EncodeStatus::Unsupported),
        };

        // Now we can gather all inputs and assign them to the node outputs transparently.
        encoder.emit_transparent_node(node, circ, |ps| ps.input_params.to_owned())?;

        Ok(EncodeStatus::Success)
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
