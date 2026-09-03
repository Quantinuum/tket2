//! Iterators over the units of a circuit.
//!
//! A [`CircuitUnit`] can either be a unique identifier for a linear unit (a
//! [`LinearUnit`]), or a wire between two HUGR nodes (a [`Wire`]).
//!
//! Linear units are tracked along the circuit, so values like qubits that are
//! used as input to a gate can continue to be tracked after the gate is
//! applied.
//!
//! The [`Units`] iterator defined in this module yields all the input or output
//! units of a node. See [`Circuit::units`] and [`Command`] for more details.
//!
//! [`Command`]: super::command::Command

pub mod filter;

use std::iter::FusedIterator;
use std::marker::PhantomData;

use hugr::core::HugrNode;
use hugr::types::{EdgeKind, Type, TypeRow};
use hugr::{CircuitUnit, HugrView, IncomingPort, OutgoingPort};
use hugr::{Direction, Node, Port, Wire};

use crate::utils::type_is_linear;

use super::Circuit;

/// A linear unit id, used in [`CircuitUnit::Linear`].
// TODO: Add this to hugr?
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct LinearUnit(usize);

impl LinearUnit {
    /// Creates a new [`LinearUnit`].
    pub fn new(index: usize) -> Self {
        Self(index)
    }
    /// Returns the index of this [`LinearUnit`].
    pub fn index(&self) -> usize {
        self.0
    }
}

impl From<LinearUnit> for CircuitUnit {
    fn from(lu: LinearUnit) -> Self {
        CircuitUnit::Linear(lu.index())
    }
}

impl TryFrom<CircuitUnit> for LinearUnit {
    type Error = ();

    fn try_from(cu: CircuitUnit) -> Result<Self, Self::Error> {
        match cu {
            CircuitUnit::Wire(_) => Err(()),
            CircuitUnit::Linear(i) => Ok(LinearUnit(i)),
        }
    }
}
/// An iterator over the units in the input or output boundary of a [Node].
#[derive(Clone, Debug)]
pub struct Units<P, N = Node, UL = DefaultUnitLabeller> {
    /// The node of the circuit.
    node: N,
    /// The types of the boundary.
    types: TypeRow,
    /// The current index in the type row.
    pos: usize,
    /// The number of linear units yielded so far.
    linear_count: usize,
    /// A pre-set assignment of units that maps linear ports to
    /// [`CircuitUnit::Linear`] ids.
    ///
    /// The default type is `()`, which assigns new linear ids sequentially.
    unit_labeller: UL,
    /// The type of port yield by the iterator.
    _port: PhantomData<P>,
}

impl<N: HugrNode> Units<OutgoingPort, N, DefaultUnitLabeller> {
    /// Create a new iterator over the input units of a circuit.
    ///
    /// This iterator will yield all units originating from the circuit's input
    /// node.
    #[inline]
    pub(super) fn new_circ_input<T: HugrView<Node = N>>(circuit: &Circuit<T>) -> Self {
        Self::new_outgoing(circuit, circuit.input_node(), DefaultUnitLabeller)
    }
}

impl<N: HugrNode, UL> Units<OutgoingPort, N, UL>
where
    UL: UnitLabeller<N>,
{
    /// Create a new iterator over the units originating from node.
    #[inline]
    pub(super) fn new_outgoing<T: HugrView<Node = N>>(
        circuit: &Circuit<T>,
        node: N,
        unit_labeller: UL,
    ) -> Self {
        Self::new_with_dir(circuit, node, Direction::Outgoing, unit_labeller)
    }
}

impl<N: HugrNode, UL> Units<IncomingPort, N, UL>
where
    UL: UnitLabeller<N>,
{
    /// Create a new iterator over the units terminating on the node.
    #[inline]
    pub(super) fn new_incoming<T: HugrView<Node = N>>(
        circuit: &Circuit<T>,
        node: N,
        unit_labeller: UL,
    ) -> Self {
        Self::new_with_dir(circuit, node, Direction::Incoming, unit_labeller)
    }
}

impl<P, N: HugrNode, UL> Units<P, N, UL>
where
    P: Into<Port> + Copy,
    UL: UnitLabeller<N>,
{
    /// Create a new iterator over the units of a node.
    #[inline]
    fn new_with_dir<T: HugrView<Node = N>>(
        circuit: &Circuit<T>,
        node: N,
        direction: Direction,
        unit_labeller: UL,
    ) -> Self {
        Self {
            node,
            types: Self::init_types(circuit, node, direction),
            pos: 0,
            linear_count: 0,
            unit_labeller,
            _port: PhantomData,
        }
    }

    /// Initialize the boundary types.
    ///
    /// We use a [`TypeRow`] to avoid allocating for simple boundaries, but if
    /// any static port is present we create a new owned [`TypeRow`] with them included.
    //
    // TODO: This is quite hacky, but we need it to accept Const static inputs.
    // We should revisit it once this is reworked on the HUGR side.
    //
    // TODO: EdgeKind::Function is not currently supported.
    fn init_types<T: HugrView>(
        circuit: &Circuit<T>,
        node: T::Node,
        direction: Direction,
    ) -> TypeRow {
        let hugr = circuit.hugr();
        let optype = hugr.get_optype(node);
        let mut types: TypeRow = hugr
            .value_types(node, direction)
            .map(|(_, ty)| ty)
            .collect();
        if let Some(EdgeKind::Const(static_type)) = optype.static_port_kind(direction) {
            types.to_mut().push(static_type);
        };
        if let Some(EdgeKind::Const(other)) = optype.other_port_kind(direction) {
            types.to_mut().push(other);
        }
        types
    }

    /// Construct an output value to yield.
    ///
    /// Calls [`UnitLabeller::assign_linear`] to assign a linear unit id to the linear ports.
    /// Non-linear ports are assigned [`CircuitUnit::Wire`]s via [`UnitLabeller::assign_wire`].
    #[inline]
    fn make_value(&self, typ: &Type, port: P) -> Option<(CircuitUnit<N>, P, Type)> {
        let unit = if type_is_linear(typ) {
            let linear_unit =
                self.unit_labeller
                    .assign_linear(self.node, port.into(), self.linear_count - 1);
            CircuitUnit::Linear(linear_unit.index())
        } else {
            let wire = self.unit_labeller.assign_wire(self.node, port.into())?;
            CircuitUnit::Wire(wire)
        };
        Some((unit, port, typ.clone()))
    }

    /// Advances the iterator and returns the next value.
    fn next_generic(&mut self) -> Option<(CircuitUnit<N>, P, Type)>
    where
        P: From<usize>,
    {
        loop {
            let typ = self.types.get(self.pos)?;
            let port = P::from(self.pos);
            self.pos += 1;
            if type_is_linear(typ) {
                self.linear_count += 1;
            }
            if let Some(val) = self.make_value(typ, port) {
                return Some(val);
            }
        }
    }
}

impl<N: HugrNode, UL> Iterator for Units<OutgoingPort, N, UL>
where
    UL: UnitLabeller<N>,
{
    type Item = (CircuitUnit<N>, OutgoingPort, Type);

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        self.next_generic()
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let len = self.types.len() - self.pos;
        (len, Some(len))
    }
}

impl<N: HugrNode, UL> Iterator for Units<IncomingPort, N, UL>
where
    UL: UnitLabeller<N>,
{
    type Item = (CircuitUnit<N>, IncomingPort, Type);

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        self.next_generic()
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let len = self.types.len() - self.pos;
        // Even when yielding every unit, a disconnected input non-linear
        // port cannot be assigned a `CircuitUnit::Wire` and so it will be
        // skipped.
        (0, Some(len))
    }
}

impl<N: HugrNode, UL> FusedIterator for Units<OutgoingPort, N, UL> where UL: UnitLabeller<N> {}
impl<N: HugrNode, UL> FusedIterator for Units<IncomingPort, N, UL> where UL: UnitLabeller<N> {}

/// A trait for assigning linear unit ids and wires to ports of a node.
pub trait UnitLabeller<N> {
    /// Assign a linear unit id to an port.
    ///
    /// # Parameters
    /// - node: The node in the circuit.
    /// - port: The node's port in the node.
    /// - linear_count: The number of linear units yielded so far.
    fn assign_linear(&self, node: N, port: Port, linear_count: usize) -> LinearUnit;

    /// Assign a wire to a port, if possible.
    ///
    /// # Parameters
    /// - node: The node in the circuit.
    /// - port: The node's port in the node.
    fn assign_wire(&self, node: N, port: Port) -> Option<Wire<N>>;
}

/// The default [`UnitLabeller`] that assigns new linear unit ids
/// sequentially, and only assigns wires to an outgoing ports (as input ports
/// require querying the HUGR for their neighbours).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DefaultUnitLabeller;

impl<N: HugrNode> UnitLabeller<N> for DefaultUnitLabeller {
    #[inline]
    fn assign_linear(&self, _: N, _: Port, linear_count: usize) -> LinearUnit {
        LinearUnit(linear_count)
    }

    #[inline]
    fn assign_wire(&self, node: N, port: Port) -> Option<Wire<N>> {
        let port = port.as_outgoing().ok()?;
        Some(Wire::new(node, port))
    }
}
