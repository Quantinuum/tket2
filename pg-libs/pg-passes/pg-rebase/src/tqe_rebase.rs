use std::collections::HashSet;

use pg_core::{GateData, GateType, Op, PGPass, PauliGraph};

fn add_rebased_gate(
    output: &mut PauliGraph,
    source: &GateData,
    gate_type: GateType,
    argument_order: &[usize],
) {
    output.add_op(Op::Gate {
        data: GateData::new(
            gate_type,
            argument_order
                .iter()
                .map(|&index| source.get_args()[index])
                .collect(),
        )
        .with_conditional(
            source.get_conditional_bits().clone(),
            source.get_conditional_values().clone(),
        ),
    });
}

fn rebase_tqe_to_zx(pg: &PauliGraph, allowed_tqes: &HashSet<GateType>) -> PauliGraph {
    let mut output = PauliGraph::new(pg.get_n_qubits());
    for op in pg.get_ops() {
        if let Op::Gate { data } = op {
            if allowed_tqes.contains(data.get_gate_type()) {
                output.add_op(op.clone());
                continue;
            }
            match data.get_gate_type() {
                GateType::XX => {
                    add_rebased_gate(&mut output, data, GateType::H, &[0]);
                    add_rebased_gate(&mut output, data, GateType::ZX, &[0, 1]);
                    add_rebased_gate(&mut output, data, GateType::H, &[0]);
                }
                GateType::XY => {
                    if allowed_tqes.contains(&GateType::YX) {
                        add_rebased_gate(&mut output, data, GateType::YX, &[1, 0]);
                    } else {
                        add_rebased_gate(&mut output, data, GateType::V, &[1]);
                        add_rebased_gate(&mut output, data, GateType::ZX, &[1, 0]);
                        add_rebased_gate(&mut output, data, GateType::Vdg, &[1]);
                    }
                }
                GateType::XZ => {
                    add_rebased_gate(&mut output, data, GateType::ZX, &[1, 0]);
                }
                GateType::YX => {
                    if allowed_tqes.contains(&GateType::XY) {
                        add_rebased_gate(&mut output, data, GateType::XY, &[1, 0]);
                    } else {
                        add_rebased_gate(&mut output, data, GateType::V, &[0]);
                        add_rebased_gate(&mut output, data, GateType::ZX, &[0, 1]);
                        add_rebased_gate(&mut output, data, GateType::Vdg, &[0]);
                    }
                }
                GateType::YY => {
                    add_rebased_gate(&mut output, data, GateType::V, &[0]);
                    add_rebased_gate(&mut output, data, GateType::Sdg, &[1]);
                    add_rebased_gate(&mut output, data, GateType::ZX, &[0, 1]);
                    add_rebased_gate(&mut output, data, GateType::Vdg, &[0]);
                    add_rebased_gate(&mut output, data, GateType::S, &[1]);
                }
                GateType::YZ => {
                    if allowed_tqes.contains(&GateType::ZY) {
                        add_rebased_gate(&mut output, data, GateType::ZY, &[1, 0]);
                    } else {
                        add_rebased_gate(&mut output, data, GateType::Sdg, &[0]);
                        add_rebased_gate(&mut output, data, GateType::ZX, &[1, 0]);
                        add_rebased_gate(&mut output, data, GateType::S, &[0]);
                    }
                }
                GateType::ZY => {
                    if allowed_tqes.contains(&GateType::YZ) {
                        add_rebased_gate(&mut output, data, GateType::YZ, &[1, 0]);
                    } else {
                        add_rebased_gate(&mut output, data, GateType::Sdg, &[1]);
                        add_rebased_gate(&mut output, data, GateType::ZX, &[0, 1]);
                        add_rebased_gate(&mut output, data, GateType::S, &[1]);
                    }
                }
                GateType::ZZ => {
                    add_rebased_gate(&mut output, data, GateType::H, &[1]);
                    add_rebased_gate(&mut output, data, GateType::ZX, &[0, 1]);
                    add_rebased_gate(&mut output, data, GateType::H, &[1]);
                }
                _ => output.add_op(op.clone()),
            }
        } else {
            output.add_op(op.clone());
        }
    }
    output
}

/// Rebases two-qubit entangling gates to `ZX` and single-qubit Cliffords.
///
/// By default, `ZX` is the only retained TQE gate. Reversed `XY`/`YX` and
/// `YZ`/`ZY` orientations are used directly when their counterpart is allowed.
pub struct RebaseTQEToZXPass {
    allowed_tqes: HashSet<GateType>,
}

impl Default for RebaseTQEToZXPass {
    fn default() -> Self {
        Self::new()
    }
}

impl RebaseTQEToZXPass {
    /// Creates a pass that rebases every TQE gate except `ZX`.
    pub fn new() -> Self {
        Self {
            allowed_tqes: [GateType::ZX].into_iter().collect(),
        }
    }

    /// Sets TQE gate types that should remain unchanged during rebasing.
    ///
    /// `ZX` is always retained because it is the target gate of the rebase.
    pub fn with_allowed_tqes(mut self, allowed_tqes: Vec<GateType>) -> Self {
        self.allowed_tqes = allowed_tqes.into_iter().collect();
        self.allowed_tqes.insert(GateType::ZX);
        self
    }
}

impl PGPass for RebaseTQEToZXPass {
    /// Rebases every disallowed TQE gate and leaves all other operations intact.
    ///
    fn transform(&self, pg: &PauliGraph) -> PauliGraph {
        rebase_tqe_to_zx(pg, &self.allowed_tqes)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use pg_core::{BlackBoxData, GateData, GateType, Op, PGPass, Pauli, PauliGraph, RotationData};
    use pg_tk::compare_unitaries_via_tk;
    use rstest::rstest;

    fn graph(ops: Vec<Op>) -> PauliGraph {
        PauliGraph::new(2).with_ops(ops)
    }

    fn gate(gate_type: GateType, args: Vec<usize>) -> Op {
        Op::Gate {
            data: GateData::new(gate_type, args),
        }
    }

    fn is_tqe(gate_type: &GateType) -> bool {
        matches!(
            gate_type,
            GateType::XX
                | GateType::XY
                | GateType::XZ
                | GateType::YX
                | GateType::YY
                | GateType::YZ
                | GateType::ZX
                | GateType::ZY
                | GateType::ZZ
        )
    }

    #[rstest]
    #[case::xx(GateType::XX)]
    #[case::xy(GateType::XY)]
    #[case::xz(GateType::XZ)]
    #[case::yx(GateType::YX)]
    #[case::yy(GateType::YY)]
    #[case::yz(GateType::YZ)]
    #[case::zx(GateType::ZX)]
    #[case::zy(GateType::ZY)]
    #[case::zz(GateType::ZZ)]
    fn rebase_preserves_unitary(#[case] gate_type: GateType) {
        let input = graph(vec![gate(gate_type, vec![0, 1])]);
        let output = RebaseTQEToZXPass::new().transform(&input);
        assert!(compare_unitaries_via_tk(&input, &output));
        // Check that only ZX gates remain
        assert!(output.get_ops().iter().all(|op| {
            let Op::Gate { data } = op else {
                return false;
            };
            !is_tqe(data.get_gate_type()) || data.get_gate_type() == &GateType::ZX
        }));
    }

    #[test]
    fn explicitly_allowed_tqe_is_unchanged() {
        let input = graph(vec![gate(GateType::YY, vec![0, 1])]);
        let output = RebaseTQEToZXPass::new()
            .with_allowed_tqes(vec![GateType::YY])
            .transform(&input);
        assert_eq!(output, input);
    }

    #[rstest]
    #[case::xy_to_yx(GateType::XY, GateType::YX)]
    #[case::yx_to_xy(GateType::YX, GateType::XY)]
    #[case::yz_to_zy(GateType::YZ, GateType::ZY)]
    #[case::zy_to_yz(GateType::ZY, GateType::YZ)]
    fn reversed_allowed_orientation_is_used(#[case] source: GateType, #[case] allowed: GateType) {
        let input = graph(vec![gate(source, vec![0, 1])]);
        let output = RebaseTQEToZXPass::new()
            .with_allowed_tqes(vec![allowed.clone()])
            .transform(&input);

        assert_eq!(output, graph(vec![gate(allowed, vec![1, 0])]),);
        assert!(compare_unitaries_via_tk(&input, &output));
    }

    #[test]
    fn zx_remains_allowed_when_the_caller_omits_it() {
        let input = graph(vec![gate(GateType::ZX, vec![0, 1])]);
        let output = RebaseTQEToZXPass::new()
            .with_allowed_tqes(vec![GateType::YY])
            .transform(&input);

        assert_eq!(output, input);
    }

    #[test]
    fn rebase_preserves_conditions() {
        let conditional = Op::Gate {
            data: GateData::new(GateType::XX, vec![0, 1])
                .with_conditional(vec![1, 3], vec![true, false]),
        };
        let input = graph(vec![conditional]);
        let output = RebaseTQEToZXPass::new().transform(&input);

        assert!(output.get_ops().len() > 1);
        assert!(output.get_ops().iter().all(|op| {
            matches!(
                op,
                Op::Gate { data }
                    if data.get_conditional_bits() == &[1, 3]
                        && data.get_conditional_values() == &[true, false]
            )
        }));

        // Check equivalence after removing the conditions
        let input_no_condition = graph(vec![gate(GateType::XX, vec![0, 1])]);
        let output_no_condition = graph(
            output
                .get_ops()
                .iter()
                .map(|op| {
                    let Op::Gate { data } = op else {
                        unreachable!()
                    };
                    Op::Gate {
                        data: data.clone().with_conditional(Vec::new(), Vec::new()),
                    }
                })
                .collect(),
        );
        assert!(compare_unitaries_via_tk(
            &input_no_condition,
            &output_no_condition
        ));
    }

    #[test]
    fn rebase_preserves_non_tqe_ops() {
        let op1 = Op::SetBoundary;
        let op2 = Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::I], 0.37),
        };
        let op3 = Op::SetBoundary;
        let op4 = gate(GateType::XX, vec![0, 1]);
        let op5 = Op::BlackBox {
            data: BlackBoxData::new(vec![0], "bb".into()),
        };
        let input = graph(vec![
            op1.clone(),
            op2.clone(),
            op3.clone(),
            op4,
            op5.clone(),
        ]);
        let output = RebaseTQEToZXPass::new().transform(&input);
        let expected = graph(vec![
            op1,
            op2,
            op3,
            gate(GateType::H, vec![0]),
            gate(GateType::ZX, vec![0, 1]),
            gate(GateType::H, vec![0]),
            op5,
        ]);
        assert_eq!(output, expected);
    }
}
