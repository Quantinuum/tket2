//! Tracking resources in HUGR subgraphs using [ResourceScope].
//!
//! This module provides the ResourceScope struct which manages resource
//! tracking within a specific region of a HUGR, computing resource paths and
//! providing efficient lookup of circuit units associated with ports.

use std::collections::{BTreeSet, VecDeque};
use std::{cmp, iter};

use crate::Circuit;
use crate::resource::flow::{DefaultResourceFlow, ResourceFlow};
use crate::resource::types::{CircuitUnit, PortMap};
use crate::utils::type_is_linear;
use hugr::core::HugrNode;
use hugr::hugr::views::SiblingSubgraph;
use hugr::hugr::views::sibling_subgraph::InvalidSubgraph;
use hugr::ops::OpTrait;
use hugr::types::Signature;
use hugr::{Direction, HugrView, IncomingPort, OutgoingPort, Port, PortIndex, Wire};
use hugr_core::hugr::internal::PortgraphNodeMap;
use indexmap::IndexMap;
use indexmap::map::Entry;
use itertools::Itertools;
use portgraph::algorithms::{TopoSort, toposort};
use portgraph::view::{FilteredGraph, NodeFilter, NodeFiltered};

use super::{Position, ResourceAllocator, ResourceId};

/// ResourceScope tracks resources within a HUGR subgraph.
///
/// This struct computes and caches resource paths through a given subgraph,
/// allowing efficient lookup of [`CircuitUnit`]s for any port of any operation
/// within the scope.
#[derive(Debug, Clone)]
pub struct ResourceScope<H: HugrView = hugr::Hugr> {
    /// The HUGR containing the operations.
    hugr: H,
    /// The subgraph within which resources are tracked, or `None` if the
    /// circuit is empty.
    subgraph: Option<SiblingSubgraph<H::Node>>,
    /// Mapping from nodes and ports to their [`CircuitUnit`]s.
    circuit_units: IndexMap<H::Node, NodeCircuitUnits<H::Node>>,
}

#[derive(Debug, Clone)]
struct NodeCircuitUnits<N: HugrNode> {
    /// Mapping from ports to their [`CircuitUnit`]s.
    port_map: PortMap<CircuitUnit<N>>,
    /// The position of the node.
    position: Position,
}

impl<N: HugrNode> NodeCircuitUnits<N> {
    fn with_default(default: CircuitUnit<N>, signature: &Signature) -> Self {
        Self {
            port_map: PortMap::with_default(default, signature),
            position: Position::default(),
        }
    }
}

/// Configuration for a ResourceScope.
pub struct ResourceScopeConfig<'a, H: HugrView> {
    /// The objects implementing the [`ResourceFlow`] trait, used to determine
    /// how resources are preserved ("flow") through operations.
    ///
    /// For each operation `op` in the circuit, the resource flows will be tried
    /// in order until the first one that succeeds on `op`. If none succeed,
    /// the [ResourceScope] construction will panic, so it is recommended to
    /// add [`DefaultResourceFlow`] to the end of flows (never fails).
    flows: Vec<Box<dyn 'a + ResourceFlow<H>>>,
}

impl<H: HugrView> Default for ResourceScopeConfig<'_, H> {
    fn default() -> Self {
        Self {
            flows: vec![Box::new(DefaultResourceFlow)],
        }
    }
}

impl<'a, H: HugrView, RF: ResourceFlow<H> + 'a> FromIterator<RF> for ResourceScopeConfig<'a, H> {
    fn from_iter<T: IntoIterator<Item = RF>>(iter: T) -> Self {
        Self {
            flows: iter
                .into_iter()
                .map(|rf| Box::new(rf) as Box<dyn ResourceFlow<H>>)
                .collect(),
        }
    }
}

impl<H: HugrView> ResourceScope<H> {
    /// Create a new ResourceScope from a SiblingSubgraph using the default
    /// resource flow.
    pub fn new(hugr: H, subgraph: SiblingSubgraph<H::Node>) -> Self {
        Self::with_config(hugr, subgraph, &Default::default())
    }

    /// Create a new ResourceScope with a custom resource flow implementation.
    ///
    /// The resource flows passed in `config` will be tried in order for every
    /// `op` until the first one that succeeds on `op`. If none succeed,
    /// this will panic, so it is recommended to add [`DefaultResourceFlow`] to
    /// the end of flows (never fails).
    pub fn with_config(
        hugr: H,
        subgraph: SiblingSubgraph<H::Node>,
        config: &ResourceScopeConfig<H>,
    ) -> Self {
        let mut scope = Self {
            hugr,
            subgraph: Some(subgraph),
            circuit_units: IndexMap::new(),
        };
        scope.compute_circuit_units(&config.flows);
        scope
    }

    /// Create a new ResourceScope from a HUGR that is an empty DFG.
    ///
    /// Panics if the HUGR is not an empty DFG.
    pub fn new_empty(hugr: H) -> Self {
        assert_eq!(
            hugr.children(hugr.entrypoint()).count(),
            2,
            "HUGR is not empty"
        );
        Self {
            hugr,
            subgraph: None,
            circuit_units: IndexMap::new(),
        }
    }

    /// Get the nodes within the scope.
    pub fn nodes(&self) -> &[H::Node] {
        self.subgraph
            .as_ref()
            .map_or(&[], |subgraph| subgraph.nodes())
    }

    /// Get the underlying HUGR.
    pub fn hugr(&self) -> &H {
        &self.hugr
    }

    /// Get the underlying subgraph, or `None` if the circuit is empty.
    pub fn subgraph(&self) -> Option<&SiblingSubgraph<H::Node>> {
        self.subgraph.as_ref()
    }

    /// Get the [`CircuitUnit`] for a given port.
    pub fn get_circuit_unit(
        &self,
        node: H::Node,
        port: impl Into<Port>,
    ) -> Option<CircuitUnit<H::Node>> {
        let port_map = self.port_map(node)?;
        Some(*port_map.get(port))
    }

    /// Get the [`ResourceId`] for a given port.
    ///
    /// Return None if the port is not a resource port.
    pub fn get_resource_id(&self, node: H::Node, port: impl Into<Port>) -> Option<ResourceId> {
        let unit = self.get_circuit_unit(node, port)?;
        unit.as_resource()
    }

    /// Get all [`CircuitUnit`]s for either the incoming or outgoing ports of a
    /// node.
    pub fn get_circuit_units_slice(
        &self,
        node: H::Node,
        direction: Direction,
    ) -> Option<&[CircuitUnit<H::Node>]> {
        let port_map = self.port_map(node)?;
        Some(port_map.get_slice(direction))
    }

    /// Get the port of node on the given resource path.
    ///
    /// The returned port will have the direction `dir`.
    pub fn get_port(&self, node: H::Node, resource_id: ResourceId, dir: Direction) -> Option<Port> {
        let units = self.get_circuit_units_slice(node, dir)?;
        let offset = units
            .iter()
            .position(|unit| unit.as_resource() == Some(resource_id))?;
        Some(Port::new(dir, offset))
    }

    /// Get the position of the given node.
    pub fn get_position(&self, node: H::Node) -> Option<Position> {
        self.circuit_units
            .get(&node)
            .map(|node_circuit_units| node_circuit_units.position)
    }

    /// Whether the scope is an empty DFG.
    pub fn is_empty(&self) -> bool {
        self.subgraph.is_none()
    }

    /// All resource IDs on the ports of `node` in the given direction.
    pub fn get_resources(
        &self,
        node: H::Node,
        dir: Direction,
    ) -> impl Iterator<Item = ResourceId> + '_ {
        let units = self.get_circuit_units_slice(node, dir);
        units
            .into_iter()
            .flatten()
            .filter_map(|unit| unit.as_resource())
    }

    /// All resource IDs on the ports of `node`, in both directions.
    pub fn get_all_resources(&self, node: H::Node) -> Vec<ResourceId> {
        let in_resources = self.get_resources(node, Direction::Incoming);
        let out_resources = self.get_resources(node, Direction::Outgoing);
        let mut all_resources = in_resources.chain(out_resources).collect_vec();
        all_resources.sort_unstable();
        all_resources.dedup();
        all_resources.shrink_to_fit();
        all_resources
    }

    /// Whether the given node is the first node on the path of the given
    /// resource.
    pub fn is_resource_start(&self, node: H::Node, resource_id: ResourceId) -> bool {
        self.get_port(node, resource_id, Direction::Outgoing)
            .is_some()
            && self
                .get_port(node, resource_id, Direction::Incoming)
                .is_none()
    }

    /// Iterate over all distinct resources in the scope.
    pub fn resources_iter(&self) -> impl Iterator<Item = ResourceId> + '_ {
        self.nodes()
            .iter()
            .map(|&n| self.get_all_resources(n))
            .kmerge()
            .dedup()
    }

    /// All copyable wires on the ports of `node` in the given direction.
    pub fn get_copyable_wires(
        &self,
        node: H::Node,
        dir: Direction,
    ) -> impl Iterator<Item = Wire<H::Node>> + '_ {
        let units = self.get_circuit_units_slice(node, dir);
        units.into_iter().flatten().filter_map(|unit| match unit {
            &CircuitUnit::Copyable(wire) => Some(wire),
            _ => None,
        })
    }

    /// Iterate over the nodes on the resource path starting from the given
    /// node in the given direction.
    pub fn resource_path_iter(
        &self,
        resource_id: ResourceId,
        start_node: H::Node,
        direction: Direction,
    ) -> impl Iterator<Item = H::Node> + '_ {
        iter::successors(Some(start_node), move |&curr_node| {
            let port = self.get_port(curr_node, resource_id, direction)?;
            let (next_node, _) = self
                .hugr()
                .single_linked_port(curr_node, port)
                .expect("linear resource");
            self.nodes().contains(&next_node).then_some(next_node)
        })
    }

    /// Whether any of the ends are reachable from any of the starts, within
    /// `self`.
    ///
    /// Any nodes outside of `self` are ignored.
    pub fn any_reachable_from(
        &self,
        starts: impl IntoIterator<Item = (H::Node, OutgoingPort)>,
        ends: impl IntoIterator<Item = (H::Node, IncomingPort)>,
    ) -> bool {
        let end_nodes = BTreeSet::from_iter(ends.into_iter().map(|(n, _)| n));
        let Some(max_end) = end_nodes.iter().filter_map(|&n| self.get_position(n)).max() else {
            return false;
        };
        let mut visited_nodes = BTreeSet::new();

        let mut curr_nodes = VecDeque::from_iter(
            starts
                .into_iter()
                .flat_map(|(n, p)| self.hugr().linked_inputs(n, p))
                .map(|(n, _)| n),
        );

        while let Some(node) = curr_nodes.pop_front() {
            if self.get_position(node).is_none_or(|p| p > max_end) || !visited_nodes.insert(node) {
                continue;
            }
            if end_nodes.contains(&node) {
                return true;
            }
            curr_nodes.extend(self.hugr().output_neighbours(node));
        }

        false
    }
}

impl<H: Clone + HugrView<Node = hugr::Node>> ResourceScope<H> {
    /// Create a new ResourceScope from a reference to a circuit.
    ///
    /// This will panic if the subgraph given by the sibling DFG graph of the
    /// circuit is invalid, e.g. if there are any non-local edges or static
    /// edges at the boundary.
    ///
    /// Use [`ResourceScope::try_from_circuit`] instead for a version that
    /// returns an error.
    pub fn from_circuit(circuit: Circuit<H>) -> Self {
        Self::try_from_circuit(circuit).unwrap_or_else(|e| panic!("Invalid circuit: {e}"))
    }

    /// Create a new ResourceScope from a circuit.
    ///
    /// This will return an error if the subgraph given by the sibling DFG graph
    /// of the circuit is invalid, e.g. if there are any non-local edges or
    /// static edges at the boundary.
    pub fn try_from_circuit(circuit: Circuit<H>) -> Result<Self, InvalidSubgraph> {
        match circuit.subgraph() {
            Ok(subgraph) => Ok(Self::new(circuit.into_hugr(), subgraph)),
            Err(InvalidSubgraph::EmptySubgraph) => Ok(Self::new_empty(circuit.into_hugr())),
            Err(err) => Err(err),
        }
    }
}

impl<'h, H: Clone + HugrView<Node = hugr::Node>> ResourceScope<&'h H> {
    /// Create a new ResourceScope from a reference to a circuit.
    ///
    /// This will panic if the subgraph given by the sibling DFG graph of the
    /// circuit is invalid, e.g. if there are any non-local edges or static
    /// edges at the boundary.
    ///
    /// Use [`ResourceScope::try_from_circuit_ref`] instead for a version that
    /// returns an error.
    pub fn from_circuit_ref(circuit: &'h Circuit<H>) -> Self {
        Self::try_from_circuit_ref(circuit).unwrap_or_else(|e| panic!("Invalid circuit: {e}"))
    }

    /// Create a new ResourceScope from a reference to a circuit.
    ///
    /// This will return an error if the subgraph given by the sibling DFG graph
    /// of the circuit is invalid, e.g. if there are any non-local edges or
    /// static edges at the boundary.
    pub fn try_from_circuit_ref(circuit: &'h Circuit<H>) -> Result<Self, InvalidSubgraph> {
        match circuit.subgraph() {
            Ok(subgraph) => Ok(Self::new(circuit.hugr(), subgraph)),
            Err(InvalidSubgraph::EmptySubgraph) => Ok(Self::new_empty(circuit.hugr())),
            Err(err) => Err(err),
        }
    }
}

impl<H: HugrView<Node = hugr::Node> + Clone> From<Circuit<H>> for ResourceScope<H> {
    fn from(value: Circuit<H>) -> Self {
        Self::from_circuit(value)
    }
}

impl<'h, H: HugrView<Node = hugr::Node> + Clone> From<&'h Circuit<H>> for ResourceScope<&'h H> {
    fn from(value: &'h Circuit<H>) -> Self {
        Self::from_circuit_ref(value)
    }
}

// Private methods to construct the circuit_units map.
impl<H: HugrView> ResourceScope<H> {
    fn port_map(&self, node: H::Node) -> Option<&PortMap<CircuitUnit<H::Node>>> {
        Some(&self.circuit_units.get(&node)?.port_map)
    }

    /// Compute circuit units for all nodes in the subgraph.
    fn compute_circuit_units(&mut self, flows: &[Box<dyn '_ + ResourceFlow<H>>]) {
        let Some(subgraph) = self.subgraph.as_ref() else {
            // Nothing to compute on an empty circuit
            return;
        };

        let mut allocator = CircuitUnitAllocator::default();

        // First, assign circuit units to the inputs to the subgraph.
        let all_inputs = subgraph
            .incoming_ports()
            .iter()
            .flatten()
            .copied()
            .collect_vec();
        self.assign_circuit_units(all_inputs, &mut allocator);

        let subgraph = self.subgraph.as_ref().unwrap(); // re-borrow

        // Proceed to propagating the circuit units through the subgraph, in topological
        // order.
        for node in toposort_subgraph(&self.hugr, subgraph, self.find_sources()) {
            self.assign_missing_circuit_units(node, &mut allocator);
            self.propagate_to_outputs(node, flows, &mut allocator);
            self.propagate_to_next_inputs(node);
        }
    }

    /// Assign circuit units to the given ports in order, ignoring non-dataflow
    /// ports.
    fn assign_circuit_units(
        &mut self,
        incoming_ports: impl IntoIterator<Item = (H::Node, IncomingPort)>,
        allocator: &mut CircuitUnitAllocator,
    ) {
        for (node, port) in incoming_ports {
            let unit = allocator.allocate_circuit_unit(node, port, &self.hugr);
            let Some(node_units) =
                node_circuit_units_mut(&mut self.circuit_units, node, &self.hugr)
            else {
                continue;
            };
            node_units.port_map.set(port, unit);
        }
    }

    /// Ensure all input dataflow ports of `node` have assigned circuit units.
    fn assign_missing_circuit_units(
        &mut self,
        node: H::Node,
        allocator: &mut CircuitUnitAllocator,
    ) {
        let Some(signature) = self.hugr.get_optype(node).dataflow_signature() else {
            return;
        };

        let mut incoming_ports = signature.input_ports().collect_vec();
        if let Some(node_units) = self.circuit_units.get(&node) {
            // Only assign circuit units to input ports without assigned units
            incoming_ports.retain(|&p| node_units.port_map.get(p).is_sentinel());
        }

        self.assign_circuit_units(incoming_ports.into_iter().map(|p| (node, p)), allocator);
    }

    /// Find source nodes (nodes with no predecessors in the subgraph).
    fn find_sources(&self) -> impl Iterator<Item = H::Node> + '_ {
        let has_pred_in_subgraph = |node: H::Node| {
            self.hugr
                .all_linked_outputs(node)
                .any(|(n, _)| self.nodes().contains(&n))
        };

        self.nodes()
            .iter()
            .copied()
            .filter(move |&n| !has_pred_in_subgraph(n))
    }

    /// Propagate circuit units at input ports to output ports.
    fn propagate_to_outputs(
        &mut self,
        node: H::Node,
        flows: &[Box<dyn '_ + ResourceFlow<H>>],
        allocator: &mut CircuitUnitAllocator,
    ) {
        let Some(port_map) = node_circuit_units_mut(&mut self.circuit_units, node, &self.hugr)
            .map(|units| &mut units.port_map)
        else {
            return;
        };

        let inp_resources = port_map
            .get_slice(Direction::Incoming)
            .iter()
            .map(CircuitUnit::as_resource)
            .collect_vec();

        let out_resources = flows
            .iter()
            .find_map(|f| f.map_resources(node, &self.hugr, &inp_resources).ok())
            .expect("no flow found");

        let signature = self
            .hugr
            .get_optype(node)
            .dataflow_signature()
            .expect("op has dataflow inputs");

        // Set out resources to output, create new circuit units where required
        for p in signature.output_ports() {
            let unit = match out_resources.get(p.index()).copied().flatten() {
                Some(resource_id) => {
                    let index = inp_resources
                        .iter()
                        .position(|&res| res == Some(resource_id))
                        .expect("invalid resource ID returned by flow");
                    *port_map.get(IncomingPort::from(index))
                }
                None => allocator.allocate_circuit_unit(node, p, &self.hugr),
            };
            port_map.set(p, unit);
        }
    }

    /// Propagate circuit units at output ports across wires to connected
    /// inputs.
    fn propagate_to_next_inputs(&mut self, node: H::Node) {
        let Some(signature) = self.hugr.get_optype(node).dataflow_signature() else {
            return;
        };
        let pos = self.get_position(node).expect("dataflow node has position");

        for p in signature.output_ports() {
            let unit = self
                .get_circuit_unit(node, p)
                .expect("dataflow node has circuit unit");

            for (in_node, in_port) in self.hugr.linked_inputs(node, p) {
                if !self.nodes().contains(&in_node) {
                    continue;
                }
                let Some(next_node_units) =
                    node_circuit_units_mut(&mut self.circuit_units, in_node, &self.hugr)
                else {
                    continue;
                };
                next_node_units.port_map.set(in_port, unit);
                next_node_units.position = cmp::max(next_node_units.position, pos.increment());
            }
        }
    }
}

/// Get the circuit units for the given node, creating them if they don't exist.
///
/// Return `None` if the node is not a dataflow op.
fn node_circuit_units_mut<H: HugrView>(
    all_circuit_units: &mut IndexMap<H::Node, NodeCircuitUnits<H::Node>>,
    node: H::Node,
    hugr: H,
) -> Option<&mut NodeCircuitUnits<H::Node>> {
    match all_circuit_units.entry(node) {
        Entry::Occupied(occupied_entry) => Some(occupied_entry.into_mut()),
        Entry::Vacant(vacant_entry) => {
            let signature = hugr.get_optype(node).dataflow_signature()?;
            Some(vacant_entry.insert(NodeCircuitUnits::with_default(
                CircuitUnit::sentinel(),
                &signature,
            )))
        }
    }
}

#[derive(Debug, Clone, Default)]
struct CircuitUnitAllocator(ResourceAllocator);

impl CircuitUnitAllocator {
    fn allocate_resource<N: HugrNode>(&mut self) -> CircuitUnit<N> {
        let resource_id = self.0.allocate();
        CircuitUnit::Resource(resource_id)
    }

    fn allocate_circuit_unit<H: HugrView>(
        &mut self,
        node: H::Node,
        port: impl Into<Port>,
        hugr: &H,
    ) -> CircuitUnit<H::Node> {
        let op = hugr.get_optype(node);
        let signature = op.dataflow_signature().expect("dataflow op");
        let port = port.into();
        let ty = signature.port_type(port).expect("valid dataflow port");
        if type_is_linear(ty) {
            self.allocate_resource()
        } else {
            let w = Wire::from_connected_port(node, port, hugr);
            CircuitUnit::Copyable(w)
        }
    }
}

fn toposort_subgraph<'h, H: HugrView>(
    hugr: &'h H,
    subgraph: &'h SiblingSubgraph<H::Node>,
    sources: impl IntoIterator<Item = H::Node>,
) -> Vec<H::Node> {
    fn contains_node(node: portgraph::NodeIndex, nodes: &&BTreeSet<portgraph::NodeIndex>) -> bool {
        nodes.contains(&node)
    }

    let (pg, pg_map) = hugr.region_portgraph(subgraph.get_parent(hugr));
    let subgraph_nodes: BTreeSet<_> = subgraph
        .nodes()
        .iter()
        .map(|&n| pg_map.to_portgraph(n))
        .collect();

    let pg: NodeFiltered<_, NodeFilter<_>, _> =
        FilteredGraph::new(&pg, contains_node, |_, _| true, &subgraph_nodes);
    let topo: TopoSort<_> = toposort(
        pg,
        sources.into_iter().map(|n| pg_map.to_portgraph(n)),
        Direction::Outgoing,
    );

    topo.map(|n| pg_map.from_portgraph(n)).collect()
}

#[cfg(test)]
pub(crate) mod tests {
    //! Implementation of [`ResourceScopeReport`], used for testing.
    //!
    //! Tests are found in the parent module.

    use super::*;
    use std::collections::{BTreeMap, HashSet};

    use hugr::HugrView;

    use super::{CircuitUnit, ResourceScope};
    use crate::{
        TketOp,
        resource::{Position, ResourceId},
        utils::build_simple_circuit,
    };

    pub type PathEl<N> = (Position, N, Port);

    impl<H: HugrView> ResourceScope<H> {
        /// A test helper to create scopes with modified positions.
        ///
        /// The transformation must preserve the order of positions, i.e. if
        /// pos[n] < pos[m], then map_pos(pos[n]) < map_pos(pos[m]).
        pub(crate) fn map_positions(&mut self, map_pos: impl Fn(Position) -> Position) {
            for node_units in self.circuit_units.values_mut() {
                node_units.position = map_pos(node_units.position);
            }
        }
    }

    /// Statistics about a ResourceScope.
    #[derive(Debug, Clone)]
    pub struct ResourceScopeReport<H: HugrView> {
        pub hugr: H,
        pub resource_paths: BTreeMap<ResourceId, Vec<PathEl<H::Node>>>,
        pub n_copyable: usize,
    }

    impl<'a, H: HugrView> From<&'a ResourceScope<H>> for ResourceScopeReport<&'a H> {
        fn from(scope: &'a ResourceScope<H>) -> Self {
            let mut resource_paths: BTreeMap<ResourceId, Vec<PathEl<H::Node>>> = BTreeMap::new();
            let mut copyable_wires = HashSet::new();

            for (&node, units) in &scope.circuit_units {
                let pos = units.position;
                for (port, &unit) in units.port_map.iter() {
                    match unit {
                        CircuitUnit::Resource(res) => resource_paths
                            .entry(res)
                            .or_default()
                            .push((pos, node, port)),
                        CircuitUnit::Copyable(id) => {
                            copyable_wires.insert(id);
                        }
                    }
                }
            }

            for path in resource_paths.values_mut() {
                path.sort_unstable();
            }

            Self {
                hugr: &scope.hugr,
                resource_paths,
                n_copyable: copyable_wires.len(),
            }
        }
    }

    impl<H: HugrView> std::fmt::Display for ResourceScopeReport<H> {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            writeln!(f, "Found {} copyable wires", self.n_copyable)?;
            writeln!(f, "Found {} resource paths:", self.resource_paths.len())?;
            for (res, path) in &self.resource_paths {
                writeln!(f, "  - {res:?}:")?;
                let mut path = path.iter().peekable();
                while let Some(&(pos, node, port)) = path.next() {
                    if let Some(&&(next_pos, next_node, next_port)) = path.peek()
                        && next_node == node
                    {
                        debug_assert_eq!(pos, next_pos);
                        path.next();
                        let in_port = port.as_incoming().unwrap();
                        let out_port = next_port.as_outgoing().unwrap();
                        let op_desc = self.hugr.get_optype(node).description();
                        writeln!(f, "    * {op_desc}({node:?}) [{in_port} -> {out_port}]",)?;
                        continue;
                    }
                    writeln!(f, "    * {node:?}@{} [{port}]", pos.to_f64(2))?;
                }
                writeln!(f)?;
            }
            Ok(())
        }
    }

    #[test]
    fn test_position_monotonic() {
        const N_HADAMARDS: [usize; 3] = [4, 10, 1];
        // A circuit with 3 qubits and a certain number of H on each qubit
        let circ = build_simple_circuit(3, |circ| {
            for (qb, n_hadamards) in N_HADAMARDS.iter().enumerate() {
                for _ in 0..*n_hadamards {
                    circ.append(TketOp::H, [qb])?;
                }
            }
            Ok(())
        })
        .unwrap();

        let scope = ResourceScope::from(&circ);
        let first_hadamards = circ
            .hugr()
            .all_linked_inputs(circ.input_node())
            .map(|(n, _)| n);

        for h in first_hadamards {
            let res = scope
                .get_all_resources(h)
                .into_iter()
                .exactly_one()
                .unwrap();
            let nodes_on_path = scope.resource_path_iter(res, h, Direction::Outgoing);
            let pos_on_path = nodes_on_path.map(|n| scope.get_position(n).unwrap());

            assert!(
                pos_on_path.collect_vec().windows(2).all(|w| w[0] <= w[1]),
                "position is not monotonically increasing on path {res:?}"
            );
        }
    }

    #[test]
    fn test_empty_scope() {
        let circ = build_simple_circuit(3, |_| Ok(())).unwrap();

        let scope = ResourceScope::from(&circ);
        assert!(scope.is_empty());
    }
}
