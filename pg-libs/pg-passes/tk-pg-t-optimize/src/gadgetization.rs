use crate::clifford_propagation::{collect_cliffords, push_conditional_x, tableau_op};
use crate::hadamard_optimizer::Synthesis;
use tk_pg_core::{GateData, GateType, Op, PauliGraph};

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
