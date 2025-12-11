//! Circuit commands.
//!
//! A [`Command`] is an operation applied to an specific wires, possibly identified by their index in the circuit's input vector.

use std::collections::hash_map::Entry;
use std::collections::{HashMap, HashSet};
use std::iter::FusedIterator;

use hugr::hugr::NodeMetadata;
use hugr::ops::{OpTag, OpTrait};
use hugr::{HugrView, IncomingPort, OutgoingPort};
use hugr_core::hugr::internal::{HugrInternals, PortgraphNodeMap};
use itertools::Either::{self, Left, Right};
use itertools::{EitherOrBoth, Itertools};
use petgraph::visit as pv;
use portgraph::PortView;

use super::Circuit;
use super::units::{DefaultUnitLabeller, LinearUnit, UnitLabeller, Units, filter};

pub use hugr::ops::OpType;
pub use hugr::types::{EdgeKind, Type, TypeRow};
pub use hugr::{CircuitUnit, Direction, Node, Port, PortIndex, Wire};

/// An operation applied to specific wires.
pub struct Command<'circ, T: HugrView> {
    /// The circuit.
    circ: &'circ Circuit<T>,
    /// The operation node.
    node: Node,
    /// An assignment of linear units to the node's input ports.
    input_linear_units: Vec<LinearUnit>,
    /// An assignment of linear units to the node's output ports.
    output_linear_units: Vec<LinearUnit>,
}

impl<'circ, T: HugrView<Node = Node>> Command<'circ, T> {
    /// Returns the node corresponding to this command.
    #[inline]
    pub fn node(&self) -> Node {
        self.node
    }

    /// Returns the [`OpType`] of the command.
    #[inline]
    pub fn optype(&self) -> &'circ OpType {
        self.circ.hugr().get_optype(self.node)
    }

    /// Returns the units of this command in a given direction.
    #[inline]
    pub fn units(
        &self,
        direction: Direction,
    ) -> impl Iterator<Item = (CircuitUnit, Port, Type)> + '_ {
        match direction {
            Direction::Incoming => Either::Left(self.inputs().map(|(u, p, t)| (u, p.into(), t))),
            Direction::Outgoing => Either::Right(self.outputs().map(|(u, p, t)| (u, p.into(), t))),
        }
    }

    /// Returns the linear units of this command in a given direction.
    #[inline]
    pub fn linear_units(
        &self,
        direction: Direction,
    ) -> impl Iterator<Item = (LinearUnit, Port, Type)> + '_ {
        match direction {
            Direction::Incoming => {
                Either::Left(self.linear_inputs().map(|(u, p, t)| (u, p.into(), t)))
            }
            Direction::Outgoing => {
                Either::Right(self.linear_outputs().map(|(u, p, t)| (u, p.into(), t)))
            }
        }
    }

    /// Returns the linear units of this command in a given direction.
    #[inline]
    pub fn input_qubits(&self) -> impl Iterator<Item = (LinearUnit, IncomingPort, Type)> + '_ {
        self.inputs().filter_map(filter::filter_qubit)
    }

    /// Returns the linear units of this command in a given direction.
    #[inline]
    pub fn output_qubits(&self) -> impl Iterator<Item = (LinearUnit, OutgoingPort, Type)> + '_ {
        self.outputs().filter_map(filter::filter_qubit)
    }

    /// Returns the output units of this command. See [`Command::units`].
    #[inline]
    pub fn outputs(&self) -> Units<OutgoingPort, Node, &'_ Self> {
        Units::new_outgoing(self.circ, self.node, self)
    }

    /// Returns the linear output units of this command. See [`Command::linear_units`].
    #[inline]
    pub fn linear_outputs(&self) -> impl Iterator<Item = (LinearUnit, OutgoingPort, Type)> + '_ {
        self.outputs().filter_map(filter::filter_linear)
    }

    /// Returns the output units and wires of this command.
    #[inline]
    pub fn output_wires(&self) -> impl Iterator<Item = (CircuitUnit, Wire)> + '_ {
        self.outputs().filter_map(move |(unit, port, _typ)| {
            let w = self.assign_wire(self.node, port.into())?;
            Some((unit, w))
        })
    }

    /// Returns the output units of this command.
    #[inline]
    pub fn inputs(&self) -> Units<IncomingPort, Node, &'_ Self> {
        Units::new_incoming(self.circ, self.node, self)
    }

    /// Returns the linear input units of this command. See [`Command::linear_units`].
    #[inline]
    pub fn linear_inputs(&self) -> impl Iterator<Item = (LinearUnit, IncomingPort, Type)> + '_ {
        self.inputs().filter_map(filter::filter_linear)
    }

    /// Returns the input units and wires of this command.
    #[inline]
    pub fn input_wires(&self) -> impl IntoIterator<Item = (CircuitUnit, Wire)> + '_ {
        self.inputs().filter_map(move |(unit, port, _typ)| {
            let w = self.assign_wire(self.node, port.into())?;
            Some((unit, w))
        })
    }

    /// Returns the number of inputs of this command.
    #[inline]
    pub fn input_count(&self) -> usize {
        self.optype().value_input_count() + self.optype().static_input_port().is_some() as usize
    }

    /// Returns the number of outputs of this command.
    #[inline]
    pub fn output_count(&self) -> usize {
        self.optype().value_output_count() + self.optype().static_output_port().is_some() as usize
    }

    /// Returns the port in the command given a linear unit.
    #[inline]
    pub fn linear_unit_port(&self, unit: LinearUnit, direction: Direction) -> Option<Port> {
        self.linear_units(direction)
            .find(|(cu, _, _)| *cu == unit)
            .map(|(_, port, _)| port)
    }

    /// Returns whether the port is a linear port.
    #[inline]
    pub fn is_linear_port(&self, port: Port) -> bool {
        self.optype()
            .port_kind(port)
            .is_some_and(|kind| kind.is_linear())
    }

    /// Returns a metadata value associated with the command's node.
    #[inline]
    pub fn metadata(&self, key: impl AsRef<str>) -> Option<&NodeMetadata> {
        self.circ.hugr().get_metadata(self.node, key)
    }
}

impl<T: HugrView<Node = Node>> UnitLabeller<Node> for &Command<'_, T> {
    #[inline]
    fn assign_linear(&self, _: Node, port: Port, _linear_count: usize) -> LinearUnit {
        let units = match port.direction() {
            Direction::Incoming => &self.input_linear_units,
            Direction::Outgoing => &self.output_linear_units,
        };
        *units.get(port.index()).unwrap_or_else(|| {
            panic!(
                "Could not assign a linear unit to port {port:?} of node {:?}",
                self.node
            )
        })
    }

    #[inline]
    fn assign_wire(&self, node: Node, port: Port) -> Option<Wire> {
        match port.as_directed() {
            Left(to_port) => {
                let (from, from_port) = self.circ.hugr().linked_outputs(node, to_port).next()?;
                Some(Wire::new(from, from_port))
            }
            Right(from_port) => Some(Wire::new(node, from_port)),
        }
    }
}

impl<T: HugrView<Node = Node>> std::fmt::Debug for Command<'_, T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Command")
            .field("circuit name", &self.circ.name())
            .field("node", &self.node)
            .field("input_linear_units", &self.input_linear_units)
            .field("output_linear_units", &self.output_linear_units)
            .finish()
    }
}

impl<T: HugrView> PartialEq for Command<'_, T> {
    fn eq(&self, other: &Self) -> bool {
        self.node == other.node
            && self.input_linear_units == other.input_linear_units
            && self.output_linear_units == other.output_linear_units
    }
}

impl<T: HugrView> Eq for Command<'_, T> {}

impl<T: HugrView> Clone for Command<'_, T> {
    fn clone(&self) -> Self {
        Self {
            circ: self.circ,
            node: self.node,
            input_linear_units: self.input_linear_units.clone(),
            output_linear_units: self.output_linear_units.clone(),
        }
    }
}

impl<T: HugrView> std::hash::Hash for Command<'_, T> {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.node.hash(state);
        self.input_linear_units.hash(state);
        self.output_linear_units.hash(state);
    }
}

/// A non-borrowing topological walker over the nodes of a circuit.
type NodeWalker<'circ, T> = pv::Topo<
    portgraph::NodeIndex,
    <portgraph::view::FlatRegion<'circ, <T as HugrInternals>::RegionPortgraph<'circ>> as petgraph::visit::Visitable>::Map,
>;

/// An iterator over the commands of a circuit.
// TODO: this can only be made generic over node type once `SiblingGraph` is
// generic over node type. See https://github.com/quantinuum/hugr/issues/1926
#[derive(Clone)]
pub struct CommandIterator<'circ, T: HugrView> {
    /// The circuit.
    circ: &'circ Circuit<T>,
    /// A view of the top-level region of the circuit.
    region: portgraph::view::FlatRegion<'circ, T::RegionPortgraph<'circ>>,
    /// A map between portgraph nodes in [`CommandIterator::region`] and circuit nodes.
    region_node_map: T::RegionPortgraphNodes,
    /// Toposorted nodes.
    nodes: NodeWalker<'circ, T>,
    /// Last wire for each [`LinearUnit`] in the circuit.
    wire_unit: HashMap<Wire, usize>,
    /// Maximum number of remaining commands, not counting I/O nodes nor root nodes.
    max_remaining: usize,
    /// Delayed output of constant and load const nodes. Contains nodes that
    /// haven't been yielded yet.
    ///
    /// We only yield them as Commands when their consumers require them.
    delayed_consts: HashSet<Node>,
    /// Nodes with delayed predecessors.
    ///
    /// Each node is associated with the number of predecessors that are present
    /// in `delayed_consts`.
    ///
    /// This map is used for performance, to avoid checking the neighbours vs
    /// the `delayed_consts` set for each processed node.
    delayed_consumers: HashMap<Node, usize>,
    /// The next node to be processed.
    ///
    /// This node was produced by the last call to `nodes.next()`, but we had to
    /// yield some delayed const nodes before it.
    delayed_node: Option<Node>,
}

impl<'circ, T: HugrView<Node = Node>> CommandIterator<'circ, T> {
    /// Create a new iterator over the commands of a circuit.
    pub(super) fn new(circ: &'circ Circuit<T>) -> Self {
        // Initialize the map assigning linear units to the input's linear
        // ports.
        //
        // TODO: `with_wires` combinator for `Units`?
        let wire_unit = circ
            .linear_units()
            .map(|(linear_unit, port, _)| (Wire::new(circ.input_node(), port), linear_unit.index()))
            .collect();

        let (region, region_node_map) = circ.hugr().region_portgraph(circ.parent());
        let node_count = region.node_count();
        let nodes = pv::Topo::new(&region);
        Self {
            circ,
            region,
            region_node_map,
            nodes,
            wire_unit,
            // Ignore the input and output nodes, and the root.
            max_remaining: node_count - 2,
            delayed_consts: HashSet::new(),
            delayed_consumers: HashMap::new(),
            delayed_node: None,
        }
    }

    /// Returns the next node to be processed.
    ///
    /// If the next node in the topological order is a constant or load const node,
    /// delay it until its consumers are processed.
    fn next_node(&mut self) -> Option<Node> {
        let node = self.delayed_node.take().or_else(|| {
            let pg_node = self.nodes.next(&self.region)?;
            Some(self.region_node_map.from_portgraph(pg_node))
        })?;
        if node == self.circ.parent() {
            // Ignore the root of the circuit.
            // This will only happen once.
            return self.next_node();
        }

        // If this node is a constant or load const node, delay it.
        let tag = self.circ.hugr().get_optype(node).tag();
        if tag == OpTag::Const || tag == OpTag::LoadConst {
            self.delayed_consts.insert(node);
            for consumer in self.circ.hugr().output_neighbours(node) {
                *self.delayed_consumers.entry(consumer).or_default() += 1;
            }
            return self.next_node();
        }

        // Check if we have any delayed const nodes that are consumed by this node.
        match self.delayed_consumers.contains_key(&node) {
            true => {
                let delayed = self.next_delayed_node(node);
                self.delayed_consts.remove(&delayed);
                for consumer in self.circ.hugr().output_neighbours(delayed) {
                    let Entry::Occupied(mut entry) = self.delayed_consumers.entry(consumer) else {
                        panic!(
                            "Delayed node consumer was not in delayed_consumers. Delayed node: {delayed:?}, consumer: {consumer:?}."
                        );
                    };
                    *entry.get_mut() -= 1;
                    if *entry.get() == 0 {
                        entry.remove();
                    }
                }
                self.delayed_node = Some(node);
                Some(delayed)
            }
            false => Some(node),
        }
    }

    /// Given a node with delayed predecessors, returns one of those predecessors.
    fn next_delayed_node(&mut self, consumer: Node) -> Node {
        let Some(delayed_pred) = self
            .circ
            .hugr()
            .input_neighbours(consumer)
            .find(|k| self.delayed_consts.contains(k))
        else {
            panic!("Could not find a delayed predecessor for node {consumer:?}.");
        };

        // Only output this node if it doesn't require any other delayed predecessors.
        match self.delayed_consumers.contains_key(&delayed_pred) {
            true => self.next_delayed_node(delayed_pred),
            false => delayed_pred,
        }
    }

    /// Process a new node, updating wires in `unit_wires`.
    ///
    /// Returns the an option with the `input_linear_units` and
    /// `output_linear_units` needed to construct a [`Command`], if the node is
    /// not an input or output.
    ///
    /// We don't return the command directly to avoid lifetime issues due to the
    /// mutable borrow here.
    fn process_node(&mut self, node: Node) -> Option<(Vec<LinearUnit>, Vec<LinearUnit>)> {
        // The root node is ignored.
        if node == self.circ.parent() {
            return None;
        }
        // Inputs and outputs are also ignored.
        // The input wire ids are already set in the `wire_unit` map during initialization.
        let tag = self.circ.hugr().get_optype(node).tag();
        if tag == OpTag::Input || tag == OpTag::Output {
            return None;
        }

        // Collect the linear units passing through this command into the maps
        // required to construct a `Command`.
        //
        // Linear input ports are matched sequentially against the linear output
        // ports, ignoring any non-linear ports when assigning unit ids. That
        // is, the nth linear input is matched against the nth linear output,
        // independently of whether there are any other ports mixed in.
        //
        // Updates the map tracking the last wire of linear units.
        let mut input_linear_units = Vec::new();
        let mut output_linear_units = Vec::new();

        let input_units = Units::new_incoming(self.circ, node, DefaultUnitLabeller)
            .filter_map(filter::filter_linear);
        let output_units = Units::new_outgoing(self.circ, node, DefaultUnitLabeller)
            .filter_map(filter::filter_linear);
        for ports in input_units.zip_longest(output_units) {
            // Terminate the input linear unit.
            // Returns the linear id of the terminated unit.
            let mut terminate_input =
                |port: IncomingPort, wire_unit: &mut HashMap<Wire, usize>| -> Option<usize> {
                    let linear_id = self.circ.hugr().single_linked_output(node, port).and_then(
                        |(wire_node, wire_port)| wire_unit.remove(&Wire::new(wire_node, wire_port)),
                    )?;
                    input_linear_units.push(LinearUnit::new(linear_id));
                    Some(linear_id)
                };

            // Add a new linear unit for this output port.
            let mut register_output =
                |unit: usize, port: OutgoingPort, wire_unit: &mut HashMap<Wire, usize>| {
                    let wire = Wire::new(node, port);
                    wire_unit.insert(wire, unit);
                    output_linear_units.push(LinearUnit::new(unit));
                };

            match ports {
                EitherOrBoth::Right((_, out_port, _)) => {
                    let new_id = self.wire_unit.len();
                    register_output(new_id, out_port, &mut self.wire_unit);
                }
                EitherOrBoth::Left((_, in_port, _)) => {
                    terminate_input(in_port, &mut self.wire_unit);
                }
                EitherOrBoth::Both((_, in_port, _), (_, out_port, _)) => {
                    if let Some(linear_id) = terminate_input(in_port, &mut self.wire_unit) {
                        register_output(linear_id, out_port, &mut self.wire_unit);
                    }
                }
            }
        }

        Some((input_linear_units, output_linear_units))
    }
}

impl<'circ, T: HugrView<Node = Node>> Iterator for CommandIterator<'circ, T> {
    type Item = Command<'circ, T>;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let node = self.next_node()?;
            // Process the node, returning a command if it's not an input or output.
            if let Some((input_linear_units, output_linear_units)) = self.process_node(node) {
                self.max_remaining -= 1;
                return Some(Command {
                    circ: self.circ,
                    node,
                    input_linear_units,
                    output_linear_units,
                });
            }
        }
    }

    #[inline]
    fn size_hint(&self) -> (usize, Option<usize>) {
        (0, Some(self.max_remaining))
    }
}

impl<T: HugrView<Node = Node>> FusedIterator for CommandIterator<'_, T> {}

impl<T: HugrView<Node = Node>> std::fmt::Debug for CommandIterator<'_, T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CommandIterator")
            .field("circuit name", &self.circ.name())
            .field("wire_unit", &self.wire_unit)
            .field("max_remaining", &self.max_remaining)
            .finish()
    }
}

#[cfg(test)]
mod test {
    use hugr::builder::{Container, DFGBuilder, Dataflow, DataflowHugr};
    use hugr::extension::prelude::qb_t;
    use hugr::hugr::hugrmut::HugrMut;
    use hugr::ops::Value;
    use hugr::ops::handle::NodeHandle;
    use hugr::types::Signature;
    use itertools::Itertools;
    use rstest::{fixture, rstest};
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};

    use crate::extension::rotation::ConstRotation;

    use crate::TketOp;
    use crate::utils::build_simple_circuit;

    use super::*;

    // We use a macro instead of a function to get the failing line numbers right.
    macro_rules! assert_eq_iter {
        ($iterable:expr, $expected:expr $(,)?) => {
            assert_eq!($iterable.collect_vec(), $expected.into_iter().collect_vec());
        };
    }

    /// 2-qubit circuit with a Hadamard, a CNOT, and a T gate.
    #[fixture]
    fn simple_circuit() -> Circuit {
        build_simple_circuit(2, |circ| {
            circ.append(TketOp::H, [0])?;
            circ.append(TketOp::CX, [0, 1])?;
            circ.append(TketOp::T, [1])?;
            Ok(())
        })
        .unwrap()
    }

    /// 2-qubit circuit with a Hadamard, a CNOT, and a T gate,
    /// defined inside a module.
    #[fixture]
    fn simple_module() -> Circuit {
        build_simple_circuit(2, |circ| {
            circ.append(TketOp::H, [0])?;
            circ.append(TketOp::CX, [0, 1])?;
            circ.append(TketOp::T, [1])?;
            Ok(())
        })
        .unwrap()
    }

    /// 2-qubit circuit with a Hadamard, a CNOT, and a T gate,
    /// defined inside a module containing other circuits.
    #[fixture]
    fn module_with_circuits() -> Circuit {
        let mut module = simple_module();
        let other_circ = simple_circuit();
        let hugr = module.hugr_mut();
        hugr.insert_hugr(hugr.module_root(), other_circ.into_hugr());
        return module;
    }

    #[rstest]
    #[case::dfg_rooted(simple_circuit())]
    #[case::module_rooted(simple_module())]
    #[case::complex_module_rooted(module_with_circuits())]
    fn iterate_commands_simple(#[case] circ: Circuit) {
        assert_eq!(CommandIterator::new(&circ).count(), 3);

        let tket_op_name = |op: TketOp| op.exposed_name();

        let mut commands = CommandIterator::new(&circ);
        assert_eq!(commands.size_hint(), (0, Some(3)));

        let hadamard = commands.next().unwrap();
        assert_eq!(hadamard.optype().to_string(), tket_op_name(TketOp::H));
        assert_eq_iter!(
            hadamard.inputs().map(|(u, _, _)| u),
            [CircuitUnit::Linear(0)],
        );
        assert_eq_iter!(
            hadamard.outputs().map(|(u, _, _)| u),
            [CircuitUnit::Linear(0)],
        );

        let cx = commands.next().unwrap();
        assert_eq!(cx.optype().to_string(), tket_op_name(TketOp::CX));
        assert_eq_iter!(
            cx.inputs().map(|(unit, _, _)| unit),
            [CircuitUnit::Linear(0), CircuitUnit::Linear(1)],
        );
        assert_eq_iter!(
            cx.outputs().map(|(unit, _, _)| unit),
            [CircuitUnit::Linear(0), CircuitUnit::Linear(1)],
        );

        let t = commands.next().unwrap();
        assert_eq!(t.optype().to_string(), tket_op_name(TketOp::T));
        assert_eq_iter!(
            t.inputs().map(|(unit, _, _)| unit),
            [CircuitUnit::Linear(1)],
        );
        assert_eq_iter!(
            t.outputs().map(|(unit, _, _)| unit),
            [CircuitUnit::Linear(1)],
        );

        assert_eq!(commands.next(), None);
    }

    /// Commands iterator with non-linear wires.
    #[test]
    fn commands_nonlinear() {
        let qb_row = vec![qb_t(); 1];
        let mut h = DFGBuilder::new(Signature::new(qb_row.clone(), qb_row)).unwrap();
        let [q_in] = h.input_wires_arr();

        let constant = h.add_constant(Value::extension(ConstRotation::PI_2));
        let loaded_const = h.load_const(&constant);
        let rz = h.add_dataflow_op(TketOp::Rz, [q_in, loaded_const]).unwrap();

        let circ: Circuit = h.finish_hugr_with_outputs(rz.outputs()).unwrap().into();

        assert_eq!(CommandIterator::new(&circ).count(), 3);
        let mut commands = CommandIterator::new(&circ);

        // First command is the constant definition.
        // It has a single output.
        let const_cmd = commands.next().unwrap();
        assert_eq!(const_cmd.optype().to_string(), "const:custom:a(π*0.5)");
        assert_eq_iter!(const_cmd.inputs().map(|(u, _, _)| u), [],);
        assert_eq_iter!(
            const_cmd.outputs().map(|(u, _, _)| u),
            [CircuitUnit::Wire(Wire::new(constant.node(), 0))],
        );

        // Next, the load constant command.
        // It has a single input and a single output.
        let load_const_cmd = commands.next().unwrap();
        let load_const_node = load_const_cmd.node();
        assert!(load_const_cmd.optype().is_load_constant());
        assert_eq_iter!(
            load_const_cmd.inputs().map(|(u, _, _)| u),
            [CircuitUnit::Wire(Wire::new(constant.node(), 0))],
        );
        assert_eq_iter!(
            load_const_cmd.outputs().map(|(u, _, _)| u),
            [CircuitUnit::Wire(Wire::new(load_const_node, 0))],
        );

        // Finally, the rz command.
        // It has the qubit and loaded constant as input and a single output.
        let rz_cmd = commands.next().unwrap();
        assert_eq!(rz_cmd.optype().cast(), Some(TketOp::Rz));
        assert_eq_iter!(
            rz_cmd.inputs().map(|(u, _, _)| u),
            [
                CircuitUnit::Linear(0),
                CircuitUnit::Wire(Wire::new(load_const_node, 0))
            ],
        );
        assert_eq_iter!(
            rz_cmd.outputs().map(|(u, _, _)| u),
            [CircuitUnit::Linear(0)],
        );
    }

    /// Commands that allocate and free linear units.
    ///
    /// Creates the following circuit:
    /// ```plaintext
    /// -------------[  ]---[QFree]
    ///              [CX]
    ///   [QAlloc]---[  ]-------------
    /// ```
    /// and checks that every command is correctly generated, and correctly
    /// computes input/output units.
    #[test]
    fn alloc_free() -> Result<(), Box<dyn std::error::Error>> {
        let qb_row = vec![qb_t(); 1];
        let mut h = DFGBuilder::new(Signature::new(qb_row.clone(), qb_row))?;

        let [q_in] = h.input_wires_arr();

        let alloc = h.add_dataflow_op(TketOp::QAlloc, [])?;
        let [q_new] = alloc.outputs_arr();

        let cx = h.add_dataflow_op(TketOp::CX, [q_in, q_new])?;
        let [q_in, q_new] = cx.outputs_arr();

        let free = h.add_dataflow_op(TketOp::QFree, [q_in])?;

        let circ: Circuit = h.finish_hugr_with_outputs([q_new])?.into();

        let mut cmds = circ.commands();

        let alloc_cmd = cmds.next().unwrap();
        assert_eq!(alloc_cmd.node(), alloc.node());
        assert_eq!(
            alloc_cmd.inputs().map(|(unit, _, _)| unit).collect_vec(),
            []
        );
        assert_eq!(
            alloc_cmd.outputs().map(|(unit, _, _)| unit).collect_vec(),
            [CircuitUnit::Linear(1)]
        );

        let cx_cmd = cmds.next().unwrap();
        assert_eq!(cx_cmd.node(), cx.node());
        assert_eq!(
            cx_cmd.inputs().map(|(unit, _, _)| unit).collect_vec(),
            [CircuitUnit::Linear(0), CircuitUnit::Linear(1)]
        );
        assert_eq!(
            cx_cmd.outputs().map(|(unit, _, _)| unit).collect_vec(),
            [CircuitUnit::Linear(0), CircuitUnit::Linear(1)]
        );

        let free_cmd = cmds.next().unwrap();
        assert_eq!(free_cmd.node(), free.node());
        assert_eq!(
            free_cmd.inputs().map(|(unit, _, _)| unit).collect_vec(),
            [CircuitUnit::Linear(0)]
        );
        assert_eq!(
            free_cmd.outputs().map(|(unit, _, _)| unit).collect_vec(),
            []
        );

        Ok(())
    }

    /// Test the manual trait implementations of `Command`.
    #[test]
    fn test_impls() -> Result<(), Box<dyn std::error::Error>> {
        let qb_row = vec![qb_t(); 1];
        let mut h = DFGBuilder::new(Signature::new(qb_row.clone(), vec![]))?;
        let [q_in] = h.input_wires_arr();
        h.add_dataflow_op(TketOp::QFree, [q_in])?;
        let circ: Circuit = h.finish_hugr_with_outputs([])?.into();

        let cmd1 = circ.commands().next().unwrap();
        let cmd2 = circ.commands().next().unwrap();

        assert_eq!(cmd1, cmd2);

        let mut hasher1 = DefaultHasher::new();
        cmd1.hash(&mut hasher1);
        let mut hasher2 = DefaultHasher::new();
        cmd2.hash(&mut hasher2);
        assert_eq!(hasher1.finish(), hasher2.finish());

        Ok(())
    }
}
