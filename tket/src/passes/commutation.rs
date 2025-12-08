use std::{collections::HashMap, rc::Rc};

use derive_more::{Display, Error, From};
use hugr::hugr::patch::PatchVerification;
use hugr::hugr::Patch;
use hugr::hugr::{hugrmut::HugrMut, HugrError};
use hugr::{CircuitUnit, Direction, HugrView, Node, Port, PortIndex};
use itertools::Itertools;
use portgraph::PortOffset;

use crate::Circuit;
use crate::{
    circuit::command::Command,
    ops::{Pauli, TketOp},
};

type Qb = crate::circuit::units::LinearUnit;

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
// remove once https://github.com/quantinuum-DEV/tket2/issues/126 is resolved
struct ComCommand {
    /// The operation node.
    node: Node,
    /// An assignment of linear units to the node's ports.
    //
    // We'll need something more complex if `follow_linear_port` stops being a
    // direct map from input to output.
    inputs: Vec<CircuitUnit>,
}

impl<'c, T: HugrView<Node = Node>> From<Command<'c, T>> for ComCommand {
    fn from(com: Command<'c, T>) -> Self {
        ComCommand {
            node: com.node(),
            inputs: com.inputs().map(|(c, _, _)| c).collect(),
        }
    }
}
impl ComCommand {
    fn node(&self) -> Node {
        self.node
    }
    fn qubits(&self) -> impl Iterator<Item = Qb> + '_ {
        self.inputs.iter().filter_map(|u| {
            let CircuitUnit::Linear(i) = u else {
                return None;
            };
            Some(Qb::new(*i))
        })
    }
    fn port_of_qb(&self, qb: Qb, direction: Direction) -> Option<Port> {
        self.inputs
            .iter()
            .position(|cu| {
                let q_cu: CircuitUnit = qb.into();
                cu == &q_cu
            })
            .map(|i| PortOffset::new(direction, i).into())
    }
}

type Slice = Vec<Option<Rc<ComCommand>>>;
type SliceVec = Vec<Slice>;

fn add_to_slice(slice: &mut Slice, com: Rc<ComCommand>) {
    for q in com.qubits() {
        slice[q.index()] = Some(com.clone());
    }
}

fn load_slices(circ: &Circuit<impl HugrView<Node = Node>>) -> SliceVec {
    let mut slices = vec![];

    let n_qbs = circ.qubit_count();
    let mut qubit_free_slice = vec![0; n_qbs];

    for command in circ
        .commands()
        .filter(|c| is_slice_op(circ.hugr(), c.node()))
    {
        let command: ComCommand = command.into();
        let free_slice = command
            .qubits()
            .map(|qb| qubit_free_slice[qb.index()])
            .max()
            .unwrap();

        for q in command.qubits() {
            qubit_free_slice[q.index()] = free_slice + 1;
        }
        if free_slice >= slices.len() {
            debug_assert!(free_slice == slices.len());
            slices.push(vec![None; n_qbs]);
        }
        let command = Rc::new(command);
        add_to_slice(&mut slices[free_slice], command);
    }

    slices
}

/// check if node is one we want to put in to a slice.
fn is_slice_op<H: HugrView>(h: &H, node: H::Node) -> bool {
    h.get_optype(node).cast::<TketOp>().is_some()
}

/// Starting from starting_index, work back along slices to check for the
/// earliest slice that can accommodate this command, if any.
fn available_slice(
    circ: &Circuit,
    slice_vec: &[Slice],
    starting_index: usize,
    command: &Rc<ComCommand>,
) -> Option<(usize, HashMap<Qb, Rc<ComCommand>>)> {
    let mut available = None;
    let mut prev_nodes: HashMap<Qb, Rc<ComCommand>> = HashMap::new();
    for slice_index in (0..=starting_index).rev() {
        // if all qubit slots are empty here the command can be moved here
        if command
            .qubits()
            .all(|q| slice_vec[slice_index][q.index()].is_none())
        {
            available = Some((slice_index, prev_nodes.clone()));
        } else if slice_index == 0 {
            break;
        } else {
            // if command commutes with all ports here it can be moved past,
            // otherwise stop
            if let Some(new_prev_nodes) = commutes_at_slice(command, &slice_vec[slice_index], circ)
            {
                prev_nodes.extend(new_prev_nodes);
            } else {
                break;
            }
        }
    }

    available
}

// If a command commutes back through this slice return a map from the qubits of
// the command to the commands in this slice acting on those qubits.
fn commutes_at_slice(
    command: &Rc<ComCommand>,
    slice: &Slice,
    circ: &Circuit,
) -> Option<HashMap<Qb, Rc<ComCommand>>> {
    // map from qubit to node it is connected to immediately after the free slice.
    let mut prev_nodes: HashMap<Qb, Rc<ComCommand>> = HashMap::new();

    for q in command.qubits() {
        // if slot is empty, continue checking.
        let Some(other_com) = &slice[q.index()] else {
            continue;
        };

        let port = command.port_of_qb(q, Direction::Incoming)?;

        let op: TketOp = circ.hugr().get_optype(command.node()).cast()?;
        // TODO: if not tket_op, might still have serialized commutation data we
        // can use.
        let pauli = commutation_on_port(&op.qubit_commutation(), port)?;

        let other_op: TketOp = circ.hugr().get_optype(other_com.node()).cast()?;
        let other_pauli = commutation_on_port(
            &other_op.qubit_commutation(),
            other_com.port_of_qb(q, Direction::Outgoing)?,
        )?;

        if pauli.commutes_with(other_pauli) {
            prev_nodes.insert(q, other_com.clone());
        } else {
            return None;
        }
    }

    Some(prev_nodes)
}

fn commutation_on_port(comms: &[(usize, Pauli)], port: Port) -> Option<Pauli> {
    comms
        .iter()
        .find_map(|(i, p)| (*i == port.index()).then_some(*p))
}

/// Error from a `PullForward` operation.
#[derive(Debug, Display, Clone, Error, PartialEq, Eq, From)]
#[non_exhaustive]
pub enum PullForwardError {
    /// Error in hugr mutation.
    #[display("Hugr mutation error: {_0}")]
    #[from]
    HugrError(HugrError),
    /// Qubit not found in command.
    #[display("Qubit {qubit} not found in command.")]
    NoQbInCommand {
        /// The qubit index
        qubit: usize,
    },
    /// No command for qubit
    #[display("No subsequent command found for qubit {qubit}")]
    NoCommandForQb {
        /// The qubit index
        qubit: usize,
    },
}

struct PullForward {
    command: Rc<ComCommand>,
    new_nexts: HashMap<Qb, Rc<ComCommand>>,
}

impl PatchVerification for PullForward {
    type Node = Node;
    type Error = PullForwardError;

    fn verify(&self, _h: &impl HugrView) -> Result<(), Self::Error> {
        unimplemented!()
    }

    fn invalidation_set(&self) -> impl Iterator<Item = Node> {
        let cmd_node = std::iter::once(self.command.node());
        let next_nodes = self.new_nexts.values().map(|c| c.node());
        cmd_node.chain(next_nodes)
    }
}

impl<H: HugrMut<Node = Node>> Patch<H> for PullForward {
    type Outcome = ();

    const UNCHANGED_ON_FAILURE: bool = false;

    fn apply(self, h: &mut H) -> Result<Self::Outcome, Self::Error> {
        let Self { command, new_nexts } = self;

        let qb_port = |command: &ComCommand, qb, direction| {
            command
                .port_of_qb(qb, direction)
                .ok_or(PullForwardError::NoQbInCommand { qubit: qb.index() })
        };
        // for each qubit, disconnect node and reconnect at destination.
        for qb in command.qubits() {
            let out_port = qb_port(&command, qb, Direction::Outgoing)?;
            let in_port = qb_port(&command, qb, Direction::Incoming)?;

            let (src, src_port) = h
                .linked_ports(command.node(), in_port)
                .exactly_one()
                .ok() // PortLinks don't implement Debug
                .unwrap();
            let (dst, dst_port) = h
                .linked_ports(command.node(), out_port)
                .exactly_one()
                .ok()
                .unwrap();

            let Some(new_neighbour_com) = new_nexts.get(&qb) else {
                return Err(PullForwardError::NoCommandForQb { qubit: qb.index() });
            };
            if new_neighbour_com == &command {
                // do not need to commute along this qubit.
                continue;
            }
            h.disconnect(command.node(), in_port);
            h.disconnect(command.node(), out_port);
            // connect old source and destination - identity operation.
            h.connect(src, src_port.index(), dst, dst_port.index());

            let new_dst_port = qb_port(new_neighbour_com, qb, Direction::Incoming)?;
            let (new_src, new_src_port) = h
                .linked_ports(new_neighbour_com.node(), new_dst_port)
                .exactly_one()
                .ok()
                .unwrap();
            // disconnect link which we will insert in to.
            h.disconnect(new_neighbour_com.node(), new_dst_port);

            h.connect(
                new_src,
                new_src_port.index(),
                command.node(),
                in_port.index(),
            );
            h.connect(
                command.node(),
                out_port.index(),
                new_neighbour_com.node(),
                new_dst_port.index(),
            );
        }
        Ok(())
    }
}

/// Pass which greedily commutes operations forwards in order to reduce depth.
pub fn apply_greedy_commutation(circ: &mut Circuit) -> Result<u32, PullForwardError> {
    let mut count = 0;
    let mut slice_vec = load_slices(circ);

    for slice_index in 0..slice_vec.len() {
        let slice_commands: Vec<_> = slice_vec[slice_index]
            .iter()
            .flatten()
            .unique()
            .cloned()
            .collect();

        for command in slice_commands {
            let Some((destination, new_nexts)) =
                available_slice(circ, &slice_vec, slice_index, &command)
            else {
                continue;
            };

            debug_assert!(
                destination < slice_index,
                "Avoid mutating slices we haven't got to yet."
            );
            for q in command.qubits() {
                let com = slice_vec[slice_index][q.index()].take();
                slice_vec[destination][q.index()] = com;
            }
            let rewrite = PullForward { command, new_nexts };
            circ.hugr_mut().apply_patch(rewrite)?;
            count += 1;
        }
    }
    Ok(count)
}

#[cfg(test)]
mod test {

    use crate::{
        extension::rotation::rotation_type, ops::test::t2_bell_circuit, utils::build_simple_circuit,
    };
    use hugr::{
        builder::{DFGBuilder, Dataflow, DataflowHugr},
        extension::prelude::{bool_t, qb_t},
        types::Signature,
    };
    use rstest::{fixture, rstest};

    use super::*;

    #[fixture]
    // example circuit from original task
    fn example_cx() -> Circuit {
        build_simple_circuit(4, |circ| {
            circ.append(TketOp::CX, [0, 2])?;
            circ.append(TketOp::CX, [1, 2])?;
            circ.append(TketOp::CX, [1, 3])?;
            Ok(())
        })
        .unwrap()
    }

    #[fixture]
    // example circuit from original task with lower depth
    fn example_cx_better() -> Circuit {
        build_simple_circuit(4, |circ| {
            circ.append(TketOp::CX, [0, 2])?;
            circ.append(TketOp::CX, [1, 3])?;
            circ.append(TketOp::CX, [1, 2])?;
            Ok(())
        })
        .unwrap()
    }

    #[fixture]
    // can't commute anything here
    fn cant_commute() -> Circuit {
        build_simple_circuit(4, |circ| {
            circ.append(TketOp::Z, [1])?;
            circ.append(TketOp::CX, [0, 1])?;
            circ.append(TketOp::CX, [2, 1])?;
            Ok(())
        })
        .unwrap()
    }

    #[fixture]
    fn big_example() -> Circuit {
        build_simple_circuit(4, |circ| {
            circ.append(TketOp::CX, [0, 3])?;
            circ.append(TketOp::CX, [1, 2])?;
            circ.append(TketOp::H, [0])?;
            circ.append(TketOp::H, [3])?;
            circ.append(TketOp::CX, [0, 1])?;
            circ.append(TketOp::CX, [2, 3])?;
            circ.append(TketOp::CX, [0, 1])?;
            circ.append(TketOp::CX, [2, 3])?;
            circ.append(TketOp::CX, [2, 1])?;
            circ.append(TketOp::H, [1])?;
            Ok(())
        })
        .unwrap()
    }

    #[fixture]
    // commute a single qubit gate
    fn single_qb_commute() -> Circuit {
        build_simple_circuit(3, |circ| {
            circ.append(TketOp::H, [1])?;
            circ.append(TketOp::CX, [0, 1])?;
            circ.append(TketOp::Z, [0])?;
            Ok(())
        })
        .unwrap()
    }
    #[fixture]

    // commute 2 single qubit gates
    fn single_qb_commute_2() -> Circuit {
        build_simple_circuit(4, |circ| {
            circ.append(TketOp::CX, [1, 2])?;
            circ.append(TketOp::CX, [1, 0])?;
            circ.append(TketOp::CX, [3, 2])?;
            circ.append(TketOp::X, [0])?;
            circ.append(TketOp::Z, [3])?;
            Ok(())
        })
        .unwrap()
    }

    #[fixture]
    // A commutation forward exists but depth doesn't change
    fn commutes_but_same_depth() -> Circuit {
        build_simple_circuit(3, |circ| {
            circ.append(TketOp::H, [1])?;
            circ.append(TketOp::CX, [0, 1])?;
            circ.append(TketOp::Z, [0])?;
            circ.append(TketOp::X, [1])?;
            Ok(())
        })
        .unwrap()
    }

    #[fixture]
    // Gate being commuted has a non-linear input
    fn non_linear_inputs() -> Circuit {
        let build = || {
            let mut dfg = DFGBuilder::new(Signature::new(
                vec![qb_t(), qb_t(), rotation_type()],
                vec![qb_t(), qb_t()],
            ))?;

            let [q0, q1, f] = dfg.input_wires_arr();

            let mut circ = dfg.as_circuit(vec![q0, q1]);

            circ.append(TketOp::H, [1])?;
            circ.append(TketOp::CX, [0, 1])?;
            circ.append_and_consume(TketOp::Rz, [CircuitUnit::Linear(0), CircuitUnit::Wire(f)])?;
            let qbs = circ.finish();
            dfg.finish_hugr_with_outputs(qbs)
        };
        build().unwrap().into()
    }

    #[fixture]
    // Gates being commuted have non-linear outputs
    fn non_linear_outputs() -> Circuit {
        let build = || {
            let mut dfg = DFGBuilder::new(Signature::new(
                vec![qb_t(), qb_t()],
                vec![qb_t(), qb_t(), bool_t()],
            ))?;

            let [q0, q1] = dfg.input_wires_arr();

            let mut circ = dfg.as_circuit(vec![q0, q1]);

            circ.append(TketOp::H, [1])?;
            circ.append(TketOp::CX, [0, 1])?;
            let measured = circ.append_with_outputs(TketOp::Measure, [0])?;
            let mut outs = circ.finish();
            outs.extend(measured);
            dfg.finish_hugr_with_outputs(outs)
        };
        build().unwrap().into()
    }

    // bug https://github.com/quantinuum/tket2/issues/253
    fn cx_commute_bug() -> Circuit {
        build_simple_circuit(3, |circ| {
            circ.append(TketOp::H, [2])?;
            circ.append(TketOp::CX, [2, 1])?;
            circ.append(TketOp::CX, [0, 2])?;
            circ.append(TketOp::CX, [0, 1])?;
            Ok(())
        })
        .unwrap()
    }
    fn slice_from_command(
        commands: &[ComCommand],
        n_qbs: usize,
        slice_arr: &[&[usize]],
    ) -> SliceVec {
        slice_arr
            .iter()
            .map(|command_indices| {
                let mut slice = vec![None; n_qbs];
                for ind in command_indices.iter() {
                    let com = commands[*ind].clone();
                    add_to_slice(&mut slice, Rc::new(com))
                }

                slice
            })
            .collect()
    }

    #[rstest]
    fn test_load_slices_cx(example_cx: Circuit) {
        let circ = example_cx;
        let commands: Vec<ComCommand> = circ.commands().map_into().collect();
        let slices = load_slices(&circ);
        let correct = slice_from_command(&commands, 4, &[&[0], &[1], &[2]]);

        assert_eq!(slices, correct);
    }

    #[rstest]
    fn test_load_slices_cx_better(example_cx_better: Circuit) {
        let circ = example_cx_better;
        let commands: Vec<ComCommand> = circ.commands().map_into().collect();

        let slices = load_slices(&circ);
        let correct = slice_from_command(&commands, 4, &[&[0, 1], &[2]]);

        assert_eq!(slices, correct);
    }

    #[rstest]
    fn test_load_slices_bell(t2_bell_circuit: Circuit) {
        let circ = t2_bell_circuit;
        let commands: Vec<ComCommand> = circ.commands().map_into().collect();

        let slices = load_slices(&circ);
        let correct = slice_from_command(&commands, 2, &[&[0], &[1]]);

        assert_eq!(slices, correct);
    }

    #[rstest]
    fn test_available_slice(example_cx: Circuit) {
        let circ = example_cx;
        let slices = load_slices(&circ);
        let (found, prev_nodes) =
            available_slice(&circ, &slices, 1, slices[2][1].as_ref().unwrap()).unwrap();
        assert_eq!(found, 0);

        assert_eq!(
            *prev_nodes.get(&Qb::new(1)).unwrap(),
            slices[1][1].as_ref().unwrap().clone()
        );

        assert!(!prev_nodes.contains_key(&Qb::new(3)));
    }

    #[rstest]
    fn big_test(big_example: Circuit) {
        let circ = big_example;
        let slices = load_slices(&circ);
        assert_eq!(slices.len(), 6);
        // can commute final cx to front
        let (found, prev_nodes) =
            available_slice(&circ, &slices, 3, slices[4][1].as_ref().unwrap()).unwrap();
        assert_eq!(found, 1);
        assert_eq!(
            *prev_nodes.get(&Qb::new(1)).unwrap(),
            slices[2][1].as_ref().unwrap().clone()
        );

        assert_eq!(
            *prev_nodes.get(&Qb::new(2)).unwrap(),
            slices[2][2].as_ref().unwrap().clone()
        );
        // hadamard can't commute past anything
        assert!(available_slice(&circ, &slices, 4, slices[5][1].as_ref().unwrap()).is_none());
    }

    /// Calculate depth by placing commands in slices.
    fn depth(h: &Circuit) -> usize {
        load_slices(h).len()
    }
    #[rstest]
    #[case(example_cx(), true, 1)]
    #[case(example_cx_better(), false, 0)]
    #[case(big_example(), true, 1)]
    #[case(cant_commute(), false, 0)]
    #[case(t2_bell_circuit(), false, 0)]
    #[case(single_qb_commute(), true, 1)]
    #[case(single_qb_commute_2(), true, 2)]
    #[case(commutes_but_same_depth(), false, 1)]
    #[case(non_linear_inputs(), true, 1)]
    #[case(non_linear_outputs(), true, 1)]
    #[case(cx_commute_bug(), true, 1)]
    fn commutation_example(
        #[case] mut case: Circuit,
        #[case] should_reduce: bool,
        #[case] expected_moves: u32,
    ) {
        let node_count = case.hugr().num_nodes();
        let depth_before = depth(&case);
        let move_count = apply_greedy_commutation(&mut case).unwrap();
        case.hugr_mut().validate().unwrap();

        assert_eq!(
            move_count, expected_moves,
            "Number of commutations did not match expected."
        );
        let depth_after = depth(&case);

        if should_reduce {
            assert!(depth_after < depth_before, "Depth should have decreased..");
        } else {
            assert_eq!(
                depth_before, depth_after,
                "Depth should not have changed for this case."
            );
        }

        assert_eq!(
            case.hugr().num_nodes(),
            node_count,
            "depth optimisation should not change the number of nodes."
        )
    }
}
