//! Decoder for native HUGR structures generated from pytket operations.
//!
//! These core structures are handled natively by the pytket encoder, so we
//! don't need to implement a
//! [`PytketEmitter`][crate::serialize::pytket::extension::PytketEmitter] for
//! them.

use crate::extension::rotation::rotation_type;
use crate::serialize::pytket::decoder::{
    DecodeStatus, LoadedParameter, PytketDecoderContext, TrackedBit, TrackedQubit,
};
use crate::serialize::pytket::extension::{PytketDecoder, RegisterCount};
use crate::serialize::pytket::opaque::OpaqueSubgraphPayload;
use crate::serialize::pytket::{DecodeInsertionTarget, DecodeOptions, PytketDecodeError};
use hugr::builder::{Container, Dataflow};
use hugr::extension::prelude::{bool_t, qb_t};
use hugr::ops::Value;
use hugr::ops::handle::NodeHandle;
use hugr::types::{Signature, Type};
use itertools::Itertools;
use tket_json_rs::circuit_json::Operation as PytketOperation;
use tket_json_rs::opbox::OpBox;
use tket_json_rs::optype::OpType as PytketOptype;

/// Decoder for native HUGR structures.
#[derive(Debug, Clone, Default)]
pub struct CoreDecoder;

impl PytketDecoder for CoreDecoder {
    fn op_types(&self) -> Vec<PytketOptype> {
        vec![
            PytketOptype::Barrier,
            PytketOptype::CircBox,
            PytketOptype::SetBits,
        ]
    }

    fn op_to_hugr<'h>(
        &self,
        op: &PytketOperation,
        qubits: &[TrackedQubit],
        bits: &[TrackedBit],
        params: &[LoadedParameter],
        _opgroup: Option<&str>,
        decoder: &mut PytketDecoderContext<'h>,
    ) -> Result<DecodeStatus, PytketDecodeError> {
        let count = RegisterCount {
            qubits: qubits.len(),
            bits: bits.len(),
            params: params.len(),
        };
        match &op {
            PytketOperation {
                op_type: PytketOptype::Barrier,
                data: Some(payload),
                ..
            } =>
            // Load an opaque subgraph from a barrier.
            //
            // Note that pytket can delete parameters to a barrier operation, so
            // we encode them in the payload instead of reading `params`.
            {
                match OpaqueSubgraphPayload::load_str(payload, decoder.extension_registry()) {
                    Ok(payload) => decoder.insert_subgraph_from_payload(qubits, bits, &payload),
                    _ => Ok(DecodeStatus::Unsupported),
                }
            }
            PytketOperation {
                op_type: PytketOptype::CircBox,
                op_box:
                    Some(OpBox::CircBox {
                        id: _id,
                        circuit: serial_circuit,
                    }),
                ..
            } => {
                // We have no way to distinguish between input and output bits
                // in the circuit box, so we assume all bits are both inputs and
                // outputs here.
                let circ_inputs: Vec<Type> = itertools::repeat_n(qb_t(), qubits.len())
                    .chain(itertools::repeat_n(bool_t(), bits.len()))
                    .chain(itertools::repeat_n(rotation_type(), params.len()))
                    .collect_vec();
                let circ_outputs: Vec<Type> = itertools::repeat_n(qb_t(), qubits.len())
                    .chain(itertools::repeat_n(bool_t(), bits.len()))
                    .collect_vec();
                let circ_signature = Signature::new(circ_inputs, circ_outputs);

                // Decode the boxed circuit into a DFG node in the Hugr.
                let options = DecodeOptions::new()
                    .with_config(decoder.config().clone())
                    .with_signature(circ_signature);
                let target = DecodeInsertionTarget::Region {
                    parent: decoder.builder.container_node(),
                };

                // Decode the circuit box into a DFG node in the region.
                let mut nested_decoder = PytketDecoderContext::new(
                    serial_circuit,
                    decoder.builder.hugr_mut(),
                    target,
                    options,
                    decoder.opaque_subgraphs,
                )?;
                nested_decoder.run_decoder(&serial_circuit.commands, None)?;
                let internal = nested_decoder.finish(None)?.node();

                decoder
                    .wire_up_node(internal, qubits, qubits, bits, bits, params)
                    .map_err(|e| e.hugr_op("DFG"))?;

                Ok(DecodeStatus::Success)
            }
            PytketOperation {
                op_type: PytketOptype::SetBits,
                classical: Some(classical),
                ..
            } => {
                let tket_json_rs::circuit_json::Classical::SetBits { values } = classical.as_ref()
                else {
                    return Ok(DecodeStatus::Unsupported);
                };
                if count != RegisterCount::only_bits(values.len()) {
                    return Ok(DecodeStatus::Unsupported);
                }

                for (bit, &value) in bits.iter().zip(values) {
                    let wire = decoder.builder.add_load_const(Value::from_bool(value));
                    decoder.register_node_outputs(wire.node(), [], [bit.clone()])?;
                }

                Ok(DecodeStatus::Success)
            }
            _ => Ok(DecodeStatus::Unsupported),
        }
    }
}
