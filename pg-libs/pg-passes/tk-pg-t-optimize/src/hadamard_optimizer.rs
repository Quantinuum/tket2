//! Applies algorithm 2 from https://arxiv.org/pdf/2302.07040 to reduce
//! the number of internal Hadamard gates, then gadgetizes them to produce
//! a diagonal region of rotations in the circuit.
use crate::clifford_propagation::{append_gate, collect_cliffords, push_conditional_x, tableau_op};
use tk_pg_core::{GateData, GateType, Op, Pauli, PauliGraph, RotationData};
use tk_pg_ir_kernels::PGTableau;
use tk_pg_qm_tableau::Tableau;

/// The circuit produced after Hadamard optimization and 
/// rotation diagonalization.
pub struct DiagonalizedCircuit {
    /// Initial Clifford basis change.
    pub prefix: Vec<GateData>,
    /// I/Z rotations interleaved with Clifford gates.
    pub rotations: Vec<Op>,
    /// Clifford correction to apply after the internal rotations.
    pub corrections: Tableau,
}

/// Updates the tableau by conjugating the given string to only I/Z letters.
/// Returns the Clifford gates added, or no gates if the string is already diagonal
/// under the current tableau.
fn diagonalize(tableau: &mut Tableau, string: &[Pauli]) -> Vec<GateData> {
    let (mut p, _) = tableau.conjugate_string(string);
    let mut gates = Vec::new();
    if let Some(pivot) = p.iter().position(|p| matches!(p, Pauli::X | Pauli::Y)) {
        for (q, pauli) in p.iter().enumerate() {
            if q != pivot && matches!(pauli, Pauli::X | Pauli::Y) {
                let gate = GateData::new(GateType::ZX, vec![pivot, q]);
                append_gate(tableau, &gate);
                gates.push(gate);
            }
        }
        p = tableau.conjugate_string(string).0;
        if p[pivot] == Pauli::Y {
            let gate = GateData::new(GateType::S, vec![pivot]);
            append_gate(tableau, &gate);
            gates.push(gate);
        }
        let gate = GateData::new(GateType::H, vec![pivot]);
        append_gate(tableau, &gate);
        gates.push(gate);
    }
    gates
}

/// Synthesize rotations on `n` qubits while reducing internal Hadamards.
pub fn synthesize(rotations: &[RotationData], n: usize) -> DiagonalizedCircuit {
    let mut reverse = Tableau::eye(n);
    for rotation in rotations.iter().rev() {
        diagonalize(&mut reverse, rotation.get_string());
    }
    
    let inverse = reverse.invert();
    let mut tableau = Tableau::eye(n);
    let mut prefix = Vec::new();
    for q in 0..n {
        prefix.extend(diagonalize(&mut tableau, &inverse.z_image(q).0));
    }

    let mut internal_rotations = Vec::new();
    for rotation in rotations {
        for gate in diagonalize(&mut tableau, rotation.get_string()) {
            internal_rotations.push(Op::Gate { data: gate });
        }
        internal_rotations.extend(tableau.conjugate(&Op::Rotation {
            data: rotation.clone(),
        }));
    }

    DiagonalizedCircuit {
        prefix,
        rotations: internal_rotations,
        corrections: tableau.invert(),
    }
}

/// Replace Hadamards in the body with ancilla gadgets.
/// Returns the prefix, I/Z rotations, and a suffix with measurements and corrections.
pub fn gadgetize(
    synthesis: DiagonalizedCircuit,
    budget: usize,
    next_bit: &mut usize,
) -> (PauliGraph, PauliGraph, PauliGraph) {
    let width = synthesis.corrections.get_n_qubits();
    let mut prefix = PauliGraph::new(width);
    let mut core = PauliGraph::new(width);
    let mut ancilla = width - budget;

    for data in synthesis.prefix {
        prefix.add_op(Op::Gate { data });
    }
    for op in synthesis.rotations {
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
    suffix.add_op(tableau_op(synthesis.corrections));

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

    fn gadgetize_two_rotations(next_bit: &mut usize) -> (PauliGraph, PauliGraph, PauliGraph) {
        let rotations = vec![
            RotationData::new(vec![Pauli::Z, Pauli::I], 0.25),
            RotationData::new(vec![Pauli::X, Pauli::I], -0.25),
        ];
        let synthesis = synthesize(&rotations, 2);
        gadgetize(synthesis, 1, next_bit)
    }

    #[test]
    fn test_gadgetize_produces_diagonal_t_rotations() {
        let mut next_bit = 0;
        let (_, diagonal, _) = gadgetize_two_rotations(&mut next_bit);

        assert!(!diagonal.get_ops().is_empty());
        for op in diagonal.get_ops() {
            let Op::Rotation { data } = op else {
                panic!("TODD input should contain only rotations");
            };

            assert_eq!(data.get_angle().abs(), 0.25);
            for pauli in data.get_string() {
                assert!(matches!(pauli, Pauli::I | Pauli::Z));
            }
        }
    }

    #[test]
    fn test_gadgetize_adds_conditional_clifford_correction() {
        let mut next_bit = 3;
        let (_, _, suffix) = gadgetize_two_rotations(&mut next_bit);

        let mut corrections = Vec::new();
        for op in suffix.get_ops() {
            if let Op::ConditionalBox { data } = op {
                corrections.push(data);
            }
        }

        assert_eq!(next_bit, 4);
        assert_eq!(corrections.len(), 1);
        let correction = corrections[0];
        assert_eq!(correction.get_conditional_bits(), &vec![3]);
        assert_eq!(correction.get_conditional_values(), &vec![true]);
        assert!(!correction.get_ops().is_empty());

        for op in correction.get_ops() {
            let Op::Rotation { data } = op else {
                panic!("The correction should contain only rotations");
            };
            assert_eq!(data.get_angle() % 0.5, 0.0);
        }
    }

    #[test]
    fn test_synthesize_rotation_sign() {
        let cases = [(Pauli::X, 0.25), (Pauli::Y, -0.25), (Pauli::Z, 0.25)];

        for (pauli, expected_angle) in cases {
            let rotations = vec![RotationData::new(vec![pauli], 0.25)];
            let synthesis = synthesize(&rotations, 1);

            let expected = vec![Op::Rotation {
                data: RotationData::new(vec![Pauli::Z], expected_angle),
            }];
            assert_eq!(synthesis.rotations, expected, "input Pauli: {pauli:?}");
        }
    }
}
