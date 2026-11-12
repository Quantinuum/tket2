//! Reconnection of non-local edges to decoded values.

use hugr::Node;
use hugr::builder::Container;
use hugr::hugr::hugrmut::HugrMut;
use hugr::std_extensions::arithmetic::float_types::float64_type;
use itertools::Itertools;

use super::{FoundWire, ParameterType, PytketDecoderContext};
use crate::serialize::pytket::PytketDecodeError;
use crate::serialize::pytket::circuit::NonlocalOutput;
use crate::serialize::pytket::opaque::EncodedEdgeID;

impl PytketDecoderContext<'_> {
    /// Reconnect non-local edges to the decoded source values.
    ///
    /// Call this after decoding all circuit segments and before removing the
    /// old region's contents. Existing edges at the target ports are replaced,
    /// since their sources may belong to the old region.
    pub(in crate::serialize::pytket) fn reconnect_nonlocal_outputs(
        &mut self,
        outputs: &[NonlocalOutput<Node>],
    ) -> Result<(), PytketDecodeError> {
        for output in outputs {
            let qubits = output
                .qubits
                .iter()
                .map(|q| self.wire_tracker.tracked_qubit_for_register(q).cloned())
                .collect::<Result<Vec<_>, _>>()?;
            let bits = output
                .bits
                .iter()
                .map(|b| self.wire_tracker.tracked_bit_for_register(b).cloned())
                .collect::<Result<Vec<_>, _>>()?;
            let params = output
                .params
                .iter()
                .map(|p| self.load_half_turns(p))
                .collect_vec();
            let found = self.wire_tracker.find_typed_wire(
                &self.config,
                &mut self.builder,
                &output.ty,
                &mut qubits.as_slice(),
                &mut bits.as_slice(),
                &mut params.as_slice(),
                Some(EncodedEdgeID::default()),
            )?;
            let wire = match found {
                FoundWire::Register(wire) => wire.wire(),
                FoundWire::Parameter(param) => {
                    let ty = if output.ty == float64_type() {
                        ParameterType::FloatHalfTurns
                    } else {
                        ParameterType::Rotation
                    };
                    param.with_type(ty, &mut self.builder).wire()
                }
                FoundWire::Unsupported { .. } => {
                    return Err(PytketDecodeError::custom(
                        "Cannot reconstruct an encoded non-local output.",
                    ));
                }
            };
            for &(target, port) in &output.targets {
                self.builder.hugr_mut().disconnect(target, port);
                self.builder
                    .hugr_mut()
                    .connect(wire.node(), wire.source(), target, port);
            }
        }
        Ok(())
    }
}
