use crate::clifford_propagation::{append_gate, collect_cliffords, push_conditional_x, tableau_op};
use tk_pg_core::{GateData, GateType, Op, Pauli, PauliGraph, RotationData};
use tk_pg_ir_kernels::PGTableau;
use tk_pg_qm_tableau::Tableau;

/// The circuit produced by Hadamard synthesis.
pub struct Synthesis {
    /// Initial Clifford basis change.
    pub prefix: Vec<GateData>,
    /// I/Z rotations interleaved with Clifford gates.
    pub body: Vec<Op>,
    /// Clifford correction to apply after the body.
    pub correction: Tableau,
}

fn diagonalize(frame: &mut Tableau, string: &[Pauli]) -> Vec<GateData> {
    let (mut p, _) = frame.conjugate_string(string);
    let mut gates = Vec::new();
    if let Some(pivot) = p.iter().position(|p| matches!(p, Pauli::X | Pauli::Y)) {
        for (q, pauli) in p.iter().enumerate() {
            if q != pivot && matches!(pauli, Pauli::X | Pauli::Y) {
                let gate = GateData::new(GateType::ZX, vec![pivot, q]);
                append_gate(frame, &gate);
                gates.push(gate);
            }
        }
        p = frame.conjugate_string(string).0;
        if p[pivot] == Pauli::Y {
            let gate = GateData::new(GateType::S, vec![pivot]);
            append_gate(frame, &gate);
            gates.push(gate);
        }
        let gate = GateData::new(GateType::H, vec![pivot]);
        append_gate(frame, &gate);
        gates.push(gate);
    }
    gates
}

/// Synthesize rotations on `n` qubits while reducing Hadamards in the body.
pub fn synthesize(rotations: &[RotationData], n: usize) -> Synthesis {
    let mut reverse = Tableau::eye(n);
    for rotation in rotations.iter().rev() {
        diagonalize(&mut reverse, rotation.get_string());
    }
    let inverse = reverse.invert();
    let mut frame = Tableau::eye(n);
    let mut prefix = Vec::new();
    for q in 0..n {
        prefix.extend(diagonalize(&mut frame, &inverse.z_image(q).0));
    }
    let mut body = Vec::new();
    for rotation in rotations {
        for gate in diagonalize(&mut frame, rotation.get_string()) {
            body.push(Op::Gate { data: gate });
        }
        body.extend(frame.conjugate(&Op::Rotation {
            data: rotation.clone(),
        }));
    }
    Synthesis {
        prefix,
        body,
        correction: frame.invert(),
    }
}

/// Replace Hadamards in the body with ancilla gadgets.
/// Returns the prefix, I/Z rotations, and a suffix with measurements and corrections.
pub fn gadgetize(
    synthesis: Synthesis,
    budget: usize,
    next_bit: &mut usize,
) -> (PauliGraph, PauliGraph, PauliGraph) {
    let width = synthesis.correction.get_n_qubits();
    let mut prefix = PauliGraph::new(width);
    let mut core = PauliGraph::new(width);
    let mut ancilla = width - budget;

    for data in synthesis.prefix {
        prefix.add_op(Op::Gate { data });
    }
    for op in synthesis.body {
        match op {
            Op::Gate { data } if *data.get_gate_type() == GateType::H => {
                let q = data.get_args()[0];
                let a = ancilla;
                for gate in [GateType::ZZ, GateType::SWAP] {
                    core.add_op(gate_op(gate, vec![q, a]));
                }
                core.add_op(gate_op(GateType::H, vec![a]));
                core.add_op(gate_op(GateType::Measure, vec![a, *next_bit]));
                core.add_conditional_op(gate_op(GateType::X, vec![q]), vec![*next_bit], vec![true]);
                core.add_op(gate_op(GateType::Reset, vec![a]));
                core.add_op(gate_op(GateType::H, vec![a]));
                ancilla += 1;
                *next_bit += 1;
            }
            op => core.add_op(op),
        }
    }

    let (core, readouts) = push_conditional_x(core);
    let mut diagonal = collect_cliffords(core).get_ops().clone();
    let tail = diagonal
        .pop()
        .expect("canonical region must end with a tableau");

    let mut suffix = PauliGraph::new(width).with_ops(vec![tail]);
    suffix.extend(readouts);
    suffix.add_op(tableau_op(synthesis.correction));

    (prefix, PauliGraph::new(width).with_ops(diagonal), suffix)
}

fn gate_op(kind: GateType, args: Vec<usize>) -> Op {
    Op::Gate {
        data: GateData::new(kind, args),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn corrections_stay_outside_todd_input() {
        let rotations = vec![
            RotationData::new(vec![Pauli::Z, Pauli::I], 0.25),
            RotationData::new(vec![Pauli::X, Pauli::I], -0.25),
        ];
        let mut next_bit = 3;
        let (_, diagonal, suffix) = gadgetize(synthesize(&rotations, 2), 1, &mut next_bit);
        assert_eq!(next_bit, 4);
        for op in diagonal.get_ops() {
            let Op::Rotation { data } = op else {
                panic!("non-rotation in TODD input")
            };
            assert_eq!(data.get_angle().abs(), 0.25);
            assert!(
                data.get_string()
                    .iter()
                    .all(|p| matches!(p, Pauli::I | Pauli::Z))
            );
        }
        let conditions: Vec<_> = suffix
            .get_ops()
            .iter()
            .filter_map(|op| match op {
                Op::ConditionalBox { data } => Some(data),
                _ => None,
            })
            .collect();
        assert_eq!(conditions.len(), 1);
        assert_eq!(conditions[0].get_conditional_bits(), &vec![3]);
        assert_eq!(conditions[0].get_conditional_values(), &vec![true]);
        for op in conditions[0].get_ops() {
            let Op::Rotation { data } = op else {
                panic!("correction was not lowered")
            };
            assert_eq!((data.get_angle() * 2.0).fract(), 0.0);
        }
    }

    #[test]
    fn test_synthesize_rotation_sign() {
        for (pauli, angle) in [(Pauli::X, 0.25), (Pauli::Y, -0.25), (Pauli::Z, 0.25)] {
            let synthesis = synthesize(&[RotationData::new(vec![pauli], 0.25)], 1);
            assert_eq!(
                synthesis.body,
                vec![Op::Rotation {
                    data: RotationData::new(vec![Pauli::Z], angle),
                }]
            );
        }
    }
}
