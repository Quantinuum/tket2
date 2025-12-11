//! Encoder and decoder for rotation operations.

use super::PytketEmitter;
use crate::Circuit;
use crate::extension::rotation::{
    ConstRotation, ROTATION_EXTENSION_ID, ROTATION_TYPE_ID, RotationOp,
};
use crate::serialize::pytket::PytketEncodeError;
use crate::serialize::pytket::config::TypeTranslatorSet;
use crate::serialize::pytket::encoder::{EncodeStatus, PytketEncoderContext, TrackedValues};
use crate::serialize::pytket::extension::{PytketTypeTranslator, RegisterCount};
use hugr::HugrView;
use hugr::extension::ExtensionId;
use hugr::extension::simple_op::MakeExtensionOp;
use hugr::ops::ExtensionOp;
use hugr::ops::constant::OpaqueValue;
use itertools::Itertools;

/// Encoder for [prelude](hugr::extension::prelude) operations.
#[derive(Debug, Clone, Default)]
pub struct RotationEmitter;

impl<H: HugrView> PytketEmitter<H> for RotationEmitter {
    fn extensions(&self) -> Option<Vec<ExtensionId>> {
        Some(vec![ROTATION_EXTENSION_ID])
    }

    fn op_to_pytket(
        &self,
        node: H::Node,
        op: &ExtensionOp,
        circ: &Circuit<H>,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<EncodeStatus, PytketEncodeError<H::Node>> {
        let Ok(rot_op) = RotationOp::from_extension_op(op) else {
            return Ok(EncodeStatus::Unsupported);
        };

        match rot_op {
            RotationOp::from_halfturns_unchecked | RotationOp::to_halfturns => {
                encoder.emit_transparent_node(node, circ, |ps| vec![ps.input_params[0].clone()])?;
                Ok(EncodeStatus::Success)
            }
            RotationOp::from_halfturns => {
                // Unsupported due to having an Option as output.
                Ok(EncodeStatus::Unsupported)
            }
            _ => {
                encoder.emit_transparent_node(node, circ, |ps| {
                    RotationEmitter::encode_rotation_op(&rot_op, ps.input_params)
                        .into_iter()
                        .collect_vec()
                })?;
                Ok(EncodeStatus::Success)
            }
        }
    }

    fn const_to_pytket(
        &self,
        value: &OpaqueValue,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<Option<TrackedValues>, PytketEncodeError<H::Node>> {
        use std::f64::consts::PI;

        let Some(const_f) = value.value().downcast_ref::<ConstRotation>() else {
            return Ok(None);
        };

        // Special cases to print 'pi' multiples nicely
        let approx_eq = |a: f64, b: f64| (a - b).abs() < 1e-10;
        const VALS: [(f64, &str); 7] = [
            (PI, "pi"),
            (PI / 2., "pi/2"),
            (-PI / 2., "-pi/2"),
            (PI / 4., "pi/4"),
            (3. * PI / 4., "3pi/4"),
            (-PI / 4., "-pi/4"),
            (-3. * PI / 4., "-3pi/4"),
        ];
        for (val, name) in VALS.iter() {
            if approx_eq(const_f.half_turns(), *val) {
                let param = encoder.values.new_param(name.to_string());
                return Ok(Some(TrackedValues::new_params([param])));
            }
        }

        let param = encoder.values.new_param(const_f.half_turns());
        Ok(Some(TrackedValues::new_params([param])))
    }
}

impl PytketTypeTranslator for RotationEmitter {
    fn extensions(&self) -> Vec<ExtensionId> {
        vec![ROTATION_EXTENSION_ID]
    }

    fn type_to_pytket(
        &self,
        typ: &hugr::types::CustomType,
        _set: &TypeTranslatorSet,
    ) -> Option<RegisterCount> {
        match typ.name() == &ROTATION_TYPE_ID {
            true => Some(RegisterCount::only_params(1)),
            false => None,
        }
    }
}

impl RotationEmitter {
    /// Encode a rotation operation into a pytket param expression.
    fn encode_rotation_op(op: &RotationOp, inputs: &[String]) -> Option<String> {
        let s = match op {
            RotationOp::radd => format!("({}) + ({})", inputs[0], inputs[1]),
            RotationOp::to_halfturns
            | RotationOp::from_halfturns_unchecked
            | RotationOp::from_halfturns => return None,
        };
        Some(s)
    }
}
