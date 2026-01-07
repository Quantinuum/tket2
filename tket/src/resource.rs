//! Resource tracking within HUGR DFG subgraphs.
//!
//! This module implements the resource tracking system. It provides
//! facilities for tracking linear resources (such as qubits) through quantum
//! circuits represented as HUGR subgraphs.
//!
//! # Overview
//!
//! As far as HUGR is concerned, a linear value (or any value, for that matter)
//! is created at an op's output and destroyed at the next op's input. TKET
//! introduces the notion of "Resource" to extend the lifetime of a linear value
//! over multiple ops.
//!
//! Every linear value is associated with a resource. If a linear value passed
//! as input to an op is also returned by the op as an output, then both input
//! and output values are associated with the same resource. We say that the op
//! "preserves" the resource. Using [`ResourceFlow`], we can track resources as
//! they "flow" through multiple operations. The chains of resource-preserving
//! ops acting on a same resource form a so-called resource path.
//!
//! # Resources and Copyable Values
//!
//! Resource tracking distinguishes between two types of values:
//!
//! - **Linear resources**: Non-copyable values that form resource paths through
//!   the circuit. Each resource has a unique [`ResourceId`] and operations on
//!   the same resource are ordered by a [`Position`]. The [`ResourceFlow`]
//!   trait determines how resources are passed through, discarded or created by
//!   an op at its linear ports.
//! - **Copyable values**: Regular values that can be copied and discarded
//!   freely. Each is identified uniquely by a [`hugr::Wire`], i.e. the outgoing
//!   port defining the value. These are not tracked across ops and do not form
//!   resources.
//!
//! # Resource Scope
//!
//! Tracking resources is not free: there is a one-off linear cost to compute
//! the resource paths, plus a linear memory cost to store them.
//!
//! Use a [`SiblingSubgraph`] to define a region of a `HUGR`, within which
//! resources should be tracked. You can then construct a resource-tracked scope
//! using [`ResourceScope::new`].
//!
//! [`SiblingSubgraph`]: hugr::hugr::views::SiblingSubgraph

// Public API exports
pub use flow::{DefaultResourceFlow, ResourceFlow, UnsupportedOp};
pub use scope::{ResourceScope, ResourceScopeConfig};
pub use types::{CircuitUnit, Position, ResourceAllocator, ResourceId};

// Internal modules
mod flow;
mod scope;
mod types;

#[cfg(test)]
pub(crate) mod tests {
    use hugr::{
        CircuitUnit, Hugr,
        builder::{DFGBuilder, Dataflow, DataflowHugr},
        extension::prelude::qb_t,
        hugr::views::SiblingSubgraph,
        ops::handle::DataflowParentID,
        types::Signature,
    };

    use itertools::Itertools;
    use rstest::rstest;

    use crate::{
        TketOp,
        extension::rotation::{ConstRotation, rotation_type},
        resource::scope::tests::ResourceScopeReport,
        utils::build_simple_circuit,
    };

    use super::ResourceScope;

    /// A two-qubit circuit with `n_cx` CNOTs.
    pub fn cx_circuit(n_cx: usize) -> Hugr {
        build_simple_circuit(2, |circ| {
            for _ in 0..n_cx {
                circ.append(TketOp::CX, [0, 1])?;
            }
            Ok(())
        })
        .unwrap()
        .into_hugr()
    }

    // Gate being commuted has a non-linear input
    fn circ(n_qubits: usize, add_rz: bool, add_const_rz: bool) -> Hugr {
        let build = || {
            let out_qb_row = vec![qb_t(); n_qubits];
            let mut inp_qb_row = out_qb_row.clone();
            if add_rz {
                inp_qb_row.push(rotation_type());
            };
            let mut dfg = DFGBuilder::new(Signature::new(inp_qb_row, out_qb_row))?;

            let (qubits, f) = if add_rz {
                let mut inputs = dfg.input_wires().collect_vec();
                let f = inputs.pop().unwrap();
                (inputs, Some(f))
            } else {
                (dfg.input_wires().collect_vec(), None)
            };

            let mut circ = dfg.as_circuit(qubits);

            for i in 0..n_qubits {
                circ.append(TketOp::H, [i])?;
            }
            for i in (0..n_qubits).step_by(2) {
                if i + 1 < n_qubits {
                    circ.append(TketOp::CX, [i, i + 1])?;
                }
            }
            if let Some(f) = f {
                for i in 0..n_qubits {
                    circ.append_and_consume(
                        TketOp::Rz,
                        [CircuitUnit::Linear(i), CircuitUnit::Wire(f)],
                    )?;
                }
            }
            if add_const_rz {
                let const_angle = circ.add_constant(ConstRotation::PI_2);
                for i in 0..n_qubits {
                    circ.append_and_consume(
                        TketOp::Rz,
                        [CircuitUnit::Linear(i), CircuitUnit::Wire(const_angle)],
                    )?;
                }
            }
            let qbs = circ.finish();
            dfg.finish_hugr_with_outputs(qbs)
        };
        build().unwrap()
    }

    #[rstest]
    #[case(2, false, false)]
    #[case(2, true, false)]
    #[case(2, false, true)]
    #[case(2, true, true)]
    #[case(4, false, false)]
    #[case(4, true, false)]
    #[case(4, false, true)]
    #[case(4, true, true)]
    #[cfg_attr(miri, ignore)] // Opening files is not supported in (isolated) miri
    fn test_resource_scope_creation(
        #[case] n_qubits: usize,
        #[case] add_rz: bool,
        #[case] add_const_rz: bool,
    ) {
        use hugr::hugr::views::RootChecked;

        let circ = circ(n_qubits, add_rz, add_const_rz);
        let subgraph = SiblingSubgraph::try_new_dataflow_subgraph::<_, DataflowParentID>(
            RootChecked::try_new(&circ).expect("Entrypoint should be a dataflow parent."),
        )
        .unwrap();
        let scope = ResourceScope::new(&circ, subgraph);
        let info = ResourceScopeReport::from(&scope);

        let mut name = format!("{n_qubits}_qubits");
        if add_rz {
            name.push('_');
            name.push_str("add_rz");
        }
        if add_const_rz {
            name.push('_');
            name.push_str("add_const_rz");
        }

        assert_eq!(info.resource_paths.len(), n_qubits);
        assert_eq!(info.n_copyable, add_const_rz as usize + add_rz as usize);
        insta::assert_snapshot!(name, info);
    }
}
