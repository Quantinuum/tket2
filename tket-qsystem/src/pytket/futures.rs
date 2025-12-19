//! Encoder and decoder for floating point operations.

use hugr::HugrView;
use hugr::extension::ExtensionId;
use hugr::extension::simple_op::MakeExtensionOp;
use hugr::ops::ExtensionOp;
use hugr::types::Term;
use itertools::Itertools;
use tket::Circuit;
use tket::serialize::pytket::encoder::EncodeStatus;
use tket::serialize::pytket::extension::{PytketTypeTranslator, RegisterCount};
use tket::serialize::pytket::{
    PytketEmitter, PytketEncodeError, PytketEncoderContext, TypeTranslatorSet,
};

use crate::extension::futures::{self, FutureOpDef};

/// Emitter for [futures](crate::extension::futures) operations and types.
///
/// The `Future<T>` type is treated as a transparent wrapper when translated to
/// pytket circuits, and operations dealing with futures do not produce pytket
/// commands.
#[derive(Debug, Clone, Default)]
pub struct FutureEmitter;

impl<H: HugrView> PytketEmitter<H> for FutureEmitter {
    fn extensions(&self) -> Option<Vec<ExtensionId>> {
        Some(vec![futures::EXTENSION_ID])
    }

    fn op_to_pytket(
        &self,
        node: H::Node,
        op: &ExtensionOp,
        circ: &Circuit<H>,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<EncodeStatus, PytketEncodeError<H::Node>> {
        let Ok(rot_op) = FutureOpDef::from_extension_op(op) else {
            return Ok(EncodeStatus::Unsupported);
        };

        match rot_op {
            FutureOpDef::Read => {
                // Transparent map
                encoder.emit_transparent_node(node, circ, |ps| ps.input_params.to_vec())?;
                Ok(EncodeStatus::Success)
            }
            FutureOpDef::Dup => {
                // Register the same input values for each output.
                let values = encoder.get_input_values(node, circ)?;
                let outputs = circ.hugr().node_outputs(node).collect_vec();
                let out0 = hugr::Wire::new(node, outputs[0]);
                let out1 = hugr::Wire::new(node, outputs[1]);

                encoder.values.register_wire(out0, values.clone(), circ)?;
                encoder.values.register_wire(out1, values, circ)?;

                Ok(EncodeStatus::Success)
            }
            FutureOpDef::Free => Ok(EncodeStatus::Success),
        }
    }
}

impl PytketTypeTranslator for FutureEmitter {
    fn extensions(&self) -> Vec<ExtensionId> {
        vec![futures::EXTENSION_ID]
    }

    fn type_to_pytket(
        &self,
        typ: &hugr::types::CustomType,
        type_translators: &TypeTranslatorSet,
    ) -> Option<RegisterCount> {
        if typ.name() != futures::FUTURE_TYPE_NAME.as_str() {
            return None;
        }
        let Some(Term::Runtime(inner_ty)) = typ.args().first() else {
            return None;
        };

        type_translators.type_to_pytket(inner_ty)
    }
}
