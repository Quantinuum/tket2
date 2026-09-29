use tk_pg_canonical_form::CanonicalFormPass;
use tk_pg_core::{
    ConditionalBoxData, GateData, GateType, Op, PGPass, Pauli, PauliGraph, RotationData,
    TableauData,
};
use tk_pg_ir_kernels::PGTableau;
use tk_pg_qm_tableau::Tableau;

pub fn append_gate(frame: &mut Tableau, gate: &GateData) {
    frame.postcompose_op(&Op::Gate { data: gate.clone() });
}

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
                for later in core.iter().rev() {
                    push_through(&mut correction, later);
                }
                suffix.push(Op::ConditionalBox {
                    data: ConditionalBoxData::new(
                        vec![tableau_op(correction)],
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

fn push_through(correction: &mut Tableau, op: &Op) {
    match op {
        Op::Gate { data } => {
            let mut gate = Tableau::eye(correction.get_n_qubits());
            append_gate(&mut gate, data);
            let mut conjugated = gate.invert();
            conjugated.compose(correction);
            conjugated.compose(&gate);
            *correction = conjugated;
        }
        Op::Rotation { data } => {
            let (p, sign_bit) = correction.conjugate_string(data.get_string());
            if sign_bit {
                let half_pis = (4.0 * data.get_angle()).round().rem_euclid(4.0) as u8;
                correction.postcompose_op(&Op::Rotation {
                    data: RotationData::new(p, f64::from(half_pis) / 2.0),
                });
            }
        }
        _ => panic!("expected an H-free Clifford+T region"),
    }
}

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
                    && data.get_conditional_bits().is_empty() =>
            {
                ()
            }
            _ => panic!("input must contain rotations, tableaux, or H preparations"),
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
