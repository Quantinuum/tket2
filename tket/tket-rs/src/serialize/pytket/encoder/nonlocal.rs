//! Encoding of values used in nested dataflow regions.

use hugr::{Direction, HugrView, Wire};
use itertools::Itertools;

use super::{PytketEncoderContext, TrackedValue};
use crate::serialize::pytket::circuit::NonlocalOutput;

impl<H: HugrView> PytketEncoderContext<H> {
    /// Collect the pytket registers and parameter expressions for non-local outputs.
    ///
    /// The encoder only visits nodes in `region`. Non-local consumers therefore
    /// keep their source values in the tracker after all local uses have been
    /// processed.
    pub(super) fn collect_nonlocal_outputs(
        &self,
        hugr: &H,
        region: H::Node,
    ) -> Vec<NonlocalOutput<H::Node>> {
        let mut outputs = Vec::new();
        for node in hugr.children(region) {
            for (port, ty) in hugr.value_types(node, Direction::Outgoing) {
                let port = port.as_outgoing().expect("outgoing value port");
                let targets = hugr
                    .linked_inputs(node, port)
                    .filter(|(target, _)| hugr.get_parent(*target) != Some(region))
                    .collect_vec();
                if targets.is_empty() {
                    continue;
                }
                let Some(values) = self.values.peek_wire_values(Wire::new(node, port)) else {
                    // Untracked edges are handled when transplanting opaque
                    // subgraphs and replacing the region's input node.
                    continue;
                };
                let mut output = NonlocalOutput {
                    targets,
                    ty,
                    qubits: Vec::new(),
                    bits: Vec::new(),
                    params: Vec::new(),
                };
                for value in values {
                    match value {
                        TrackedValue::Qubit(q) => {
                            output.qubits.push(self.values.qubit_register(*q).clone());
                        }
                        TrackedValue::Bit(b) => {
                            output.bits.push(self.values.bit_register(*b).clone());
                        }
                        TrackedValue::Param(p) => {
                            output
                                .params
                                .push(self.values.param_expression(*p).to_owned());
                        }
                    }
                }
                outputs.push(output);
            }
        }
        outputs
    }
}
