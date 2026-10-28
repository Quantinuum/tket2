use tk_pg_canonical_form::CanonicalFormPass;
use tk_pg_core::{
    ConditionalBoxData, GateData, GateType, Op, PGPass, Pauli, PauliGraph, RotationData,
    TableauData,
};
use tk_pg_ir_kernels::{PGTableau, get_dagger};
use tk_pg_qm_tableau::Tableau;

/// Append a Clifford gate to the tableau.
pub fn append_gate(frame: &mut Tableau, gate: &GateData) {
    frame.postcompose_op(&Op::Gate { data: gate.clone() });
}

/// Collect Cliffords without moving them past conditional boxes, measurements, or resets.
pub fn collect_cliffords(graph: PauliGraph) -> PauliGraph {
    let width = graph.get_n_qubits();
    let mut out = PauliGraph::new(width);
    let mut segment = PauliGraph::new(width);
    for op in graph.get_ops() {
        if matches!(op, Op::ConditionalBox { .. })
            || matches!(op,
            Op::Gate { data } if matches!(data.get_gate_type(), GateType::Measure | GateType::Reset))
        {
            out.extend(CanonicalFormPass::new().transform(&segment));
            out.add_op(op.clone());
            segment = PauliGraph::new(width);
        } else {
            segment.add_op(op.clone());
        }
    }
    out.extend(CanonicalFormPass::new().transform(&segment));
    out
}

/// Wrap a tableau as a graph operation.
pub fn tableau_op(tableau: Tableau) -> Op {
    Op::Tableau {
        data: TableauData::from(tableau),
    }
}

/// We push the classically controlled X gate separately here as CanonicalFormPass
/// does not currently handle this.
pub fn push_conditional_x(graph: PauliGraph) -> (PauliGraph, PauliGraph) {
    let width = graph.get_n_qubits();
    let mut core = Vec::new();
    let mut suffix = Vec::new();

    for op in graph.get_ops().iter().rev() {
        match op {
            Op::ConditionalBox { data } => {
                let [Op::Gate { data: x }] = data.get_ops().as_slice() else {
                    panic!("expected a conditional X gate");
                };

                let mut correction = Tableau::eye(width);
                append_gate(&mut correction, x);
                let mut string = vec![Pauli::I; width];
                string[x.get_args()[0]] = Pauli::X;
                let mut rotations = vec![Op::Rotation {
                    data: RotationData::new(string, 1.0),
                }];
                for later in core.iter().rev() {
                    push_through(&mut correction, &mut rotations, later);
                }
                suffix.push(Op::ConditionalBox {
                    data: ConditionalBoxData::new(
                        rotations,
                        data.get_conditional_bits().clone(),
                        data.get_conditional_values().clone(),
                    ),
                });
            }
            Op::Gate { data }
                if matches!(
                    data.get_gate_type(),
                    GateType::H | GateType::Measure | GateType::Reset
                ) =>
            {
                suffix.push(op.clone())
            }
            _ => core.push(op.clone()),
        }
    }
    core.reverse();
    suffix.reverse();
    (
        PauliGraph::new(width).with_ops(core),
        PauliGraph::new(width).with_ops(suffix),
    )
}

// Keep an explicit rotation decomposition for greedy synthesis, alongside the
// tableau used to calculate the corrections when crossing T rotations.
fn push_through(correction: &mut Tableau, rotations: &mut Vec<Op>, op: &Op) {
    match op {
        Op::Gate { .. } => {
            correction.precompose_op(&get_dagger::<Tableau>(op));
            correction.postcompose_op(op);
            let mut gate = Tableau::eye(correction.get_n_qubits());
            gate.postcompose_op(op);
            *rotations = rotations.iter().flat_map(|r| gate.conjugate(r)).collect();
        }
        Op::Rotation { data } => {
            let (p, sign_bit) = correction.conjugate_string(data.get_string());
            if sign_bit {
                let half_pis = (4.0 * data.get_angle()).round().rem_euclid(4.0) as u8;
                let rotation = Op::Rotation {
                    data: RotationData::new(p, f64::from(half_pis) / 2.0),
                };
                correction.postcompose_op(&rotation);
                rotations.push(rotation);
            }
        }
        _ => panic!("expected an H-free Clifford+T region"),
    }
}

/// Check the input and return its rotations and final Clifford tableau.
pub fn normalize(pg: &PauliGraph) -> (PauliGraph, Tableau) {
    let mut input = PauliGraph::new(pg.get_n_qubits());
    for op in pg.get_ops() {
        match op {
            Op::Rotation { data } => {
                let turns = 4.0 * data.get_angle();
                assert!(
                    turns.is_finite() && (turns - turns.round()).abs() <= 1e-10,
                    "TODD requires multiples of 0.25 half turns"
                );
                if data.get_string().iter().all(|p| *p == Pauli::I) {
                    continue;
                }
            }
            Op::Tableau { .. } => (),
            Op::Gate { data }
                if *data.get_gate_type() == GateType::H
                    && data.get_conditional_bits().is_empty() => {}
            _ => panic!("input must contain rotations, tableaux, or H gates"),
        }
        input.add_op(op.clone());
    }
    input.try_validate().expect("invalid T-optimization input");
    let canonical = CanonicalFormPass::new().transform(&input);
    let mut ops = canonical.get_ops().clone();
    let Op::Tableau { data } = ops.pop().unwrap() else {
        unreachable!()
    };
    (
        PauliGraph::new(pg.get_n_qubits()).with_ops(ops),
        Tableau::from(data),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn clifford_tableau(rotations: &[Op], nb_qubits: usize) -> Tableau {
        let mut tableau = Tableau::eye(nb_qubits);
        for rotation in rotations {
            let Op::Rotation { data } = rotation else {
                panic!("The correction should contain only rotations");
            };
            assert_eq!(data.get_angle() % 0.5, 0.0);
            tableau.postcompose_op(rotation);
        }
        tableau
    }

    #[test]
    fn test_correction_rotations_match_tableau() {
        for angle in [0.25, -0.25] {
            let mut correction = Tableau::eye(2);
            append_gate(&mut correction, &GateData::new(GateType::X, vec![0]));
            let mut rotations = vec![Op::Rotation {
                data: RotationData::new(vec![Pauli::X, Pauli::I], 1.0),
            }];
            let operations = [
                Op::Rotation {
                    data: RotationData::new(vec![Pauli::Z, Pauli::I], angle),
                },
                Op::Gate {
                    data: GateData::new(GateType::S, vec![0]),
                },
                Op::Gate {
                    data: GateData::new(GateType::ZZ, vec![0, 1]),
                },
                Op::Gate {
                    data: GateData::new(GateType::SWAP, vec![0, 1]),
                },
                Op::Rotation {
                    data: RotationData::new(vec![Pauli::I, Pauli::Z], -angle),
                },
            ];
            for op in operations {
                push_through(&mut correction, &mut rotations, &op);

                assert_eq!(clifford_tableau(&rotations, 2), correction);
            }
        }
    }

    #[test]
    fn test_push_through_gate() {
        let gates = [
            (GateType::S, vec![0]),
            (GateType::Sdg, vec![0]),
            (GateType::ZX, vec![0, 1]),
            (GateType::ZZ, vec![0, 1]),
            (GateType::SWAP, vec![0, 1]),
        ];

        for (gate_type, args) in gates {
            let op = Op::Gate {
                data: GateData::new(gate_type, args),
            };
            let mut correction = Tableau::eye(2);
            append_gate(&mut correction, &GateData::new(GateType::V, vec![0]));
            let mut rotations = vec![Op::Rotation {
                data: RotationData::new(vec![Pauli::X, Pauli::I], 0.5),
            }];

            let mut before = correction.clone();
            before.postcompose_op(&op);

            push_through(&mut correction, &mut rotations, &op);

            let mut after = Tableau::eye(2);
            after.postcompose_op(&op);
            after.compose(&correction);

            assert_eq!(clifford_tableau(&rotations, 2), correction);
            assert_eq!(after, before);
        }
    }
}
