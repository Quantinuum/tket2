use crate::clifford_propagation::append_gate;
use tk_pg_core::{GateData, GateType, Op, Pauli, RotationData};
use tk_pg_ir_kernels::PGTableau;
use tk_pg_qm_tableau::Tableau;

pub struct Synthesis {
    pub prefix: Vec<GateData>,
    pub body: Vec<Op>,
    pub correction: Tableau,
}

fn diagonalize(frame: &mut Tableau, string: &[Pauli]) -> Vec<GateData> {
    let (mut p, _) = frame.conjugate_string(string);
    let mut gates = Vec::new();
    if let Some(pivot) = p.iter().position(|p| matches!(p, Pauli::X | Pauli::Y)) {
        for q in 0..p.len() {
            if q != pivot && matches!(p[q], Pauli::X | Pauli::Y) {
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

#[cfg(test)]
mod tests {
    use super::*;

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
