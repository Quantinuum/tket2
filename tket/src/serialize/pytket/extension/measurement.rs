//! Encoder for `tket.measurement` extension operations.

use super::PytketEmitter;
use crate::TketOp;
use crate::extension::measurement::{MEASUREMENT_EXTENSION_ID, MeasurementOp};
use crate::serialize::pytket::PytketEncodeError;
use crate::serialize::pytket::encoder::{EncodeStatus, PytketEncoderContext, TrackedValue};
use hugr::extension::ExtensionId;
use hugr::extension::prelude::bool_t;
use hugr::extension::simple_op::MakeExtensionOp;
use hugr::ops::ExtensionOp;
use hugr::ops::OpTrait;
use hugr::{HugrView, Wire};

/// Emitter for `tket.measurement` operations.
///
/// Only `MeasurementOp::Read` is supported and emitted as a no-op by reusing the
/// tracked bit from the input measurement wire.
#[derive(Debug, Clone, Default)]
pub struct MeasurementEmitter;

impl<H: HugrView> PytketEmitter<H> for MeasurementEmitter {
    fn extensions(&self) -> Option<Vec<ExtensionId>> {
        Some(vec![MEASUREMENT_EXTENSION_ID])
    }

    fn op_to_pytket(
        &self,
        node: H::Node,
        op: &ExtensionOp,
        hugr: &H,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<EncodeStatus, PytketEncodeError<H::Node>> {
        let Ok(op) = MeasurementOp::from_extension_op(op) else {
            return Ok(EncodeStatus::Unsupported);
        };

        match op {
            MeasurementOp::Read => self.encode_read(node, hugr, encoder),
        }
    }
}

impl MeasurementEmitter {
    fn encode_read<H: HugrView>(
        &self,
        node: H::Node,
        hugr: &H,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<EncodeStatus, PytketEncodeError<H::Node>> {
        // Find the `Read` input wire that comes specifically from a `MeasureFree` op.
        let Some((measure, measure_port)) = hugr.node_inputs(node).find_map(|input_port| {
            let (pred_node, pred_port) = hugr.single_linked_output(node, input_port)?;
            let pred_op = hugr.get_optype(pred_node).as_extension_op()?;
            let pred_tket_op = TketOp::from_extension_op(pred_op).ok()?;
            (pred_tket_op == TketOp::MeasureFree).then_some((pred_node, pred_port))
        }) else {
            return Ok(EncodeStatus::Unsupported);
        };

        // Find the output wire that is of type `bool`.
        let op = hugr.get_optype(node);
        let Some(signature) = op.dataflow_signature() else {
            return Ok(EncodeStatus::Unsupported);
        };
        let Some(output_port) = hugr.node_outputs(node).find(|&out_port| {
            signature
                .out_port_type(out_port)
                .is_some_and(|ty| ty == &bool_t())
        }) else {
            return Ok(EncodeStatus::Unsupported);
        };

        let wire = Wire::new(measure, measure_port);

        // Then check that the input wire is associated with a tracked bit value.
        let Some([TrackedValue::Bit(_)]) = encoder.peek_wire_values(wire) else {
            return Ok(EncodeStatus::Unsupported);
        };

        // If it is, we can emit the `Read` op as a no-op by associating the output wire
        // to the same tracked bit value.
        let input_values = encoder.get_wire_values(wire, hugr)?;
        let &[TrackedValue::Bit(bit)] = input_values.as_ref() else {
            // As we checked with `peek_wire_values``, this should not happen.
            return Ok(EncodeStatus::Unsupported);
        };

        let output_wire = Wire::new(node, output_port);
        encoder.values.register_wire(output_wire, [bit], hugr)?;
        Ok(EncodeStatus::Success)
    }
}
