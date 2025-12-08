//! Subcircuits of circuits.
//!
//! Subcircuits are subgraphs of [`hugr::Hugr`] that use a pre-computed
//! [`ResourceScope`] to express subgraphs in terms of intervals on resource
//! paths and (purely classical) copyable expressions.

use std::collections::BTreeSet;
use std::iter;

use derive_more::derive::{Display, Error};
use hugr::core::HugrNode;
use hugr::hugr::patch::simple_replace::InvalidReplacement;
use hugr::hugr::views::sibling_subgraph::{IncomingPorts, InvalidSubgraph, OutgoingPorts};
use hugr::hugr::views::SiblingSubgraph;
use hugr::ops::{constant, OpTrait};
use hugr::types::Signature;
use hugr::{Direction, HugrView, IncomingPort, OutgoingPort, Port, Wire};
use indexmap::{IndexMap, IndexSet};
use itertools::{Either, Itertools};

use crate::circuit::Circuit;
use crate::resource::{ResourceId, ResourceScope};
use crate::rewrite::CircuitRewrite;
use crate::subcircuit::expression::is_pure_copyable;

mod interval;
pub use interval::{Interval, InvalidInterval};

mod expression;
pub use expression::CopyableExpr;

/// A subgraph within a [`ResourceScope`].
///
/// Just like [`SiblingSubgraph`], [`Subcircuit`] represents connected subgraphs
/// within hugr dataflow regions. Unlike [`SiblingSubgraph`], the convexity
/// check is not performed at construction time, but instead defered until
/// a [`SiblingSubgraph`] needs to be constructed (see
/// [`Subcircuit::try_to_subgraph`] and [`Subcircuit::validate_subgraph`]).
///
/// [`Subcircuit`] distinguishes between "pure copyable" nodes, which has
/// exclusively copyable inputs and outputs, and "resource" nodes, which have at
/// least one linear input or output.
///
/// ## Subcircuit representation: resource nodes
///
/// The subgraph composed of resource nodes is represented as a vector of
/// intervals on the resource paths of the circuit (see below and
/// [`ResourceScope`]). Convex subgraphs can always be represented by intervals;
/// some non-convex subgraphs can also be expressed, as long as for each
/// resource path within the subgraph, the nodes on that path are connected.
///
/// ## Subcircuit representation: copyable values
///
/// Subgraphs within the subcircuit that are made solely of copyable values
/// are represented as [`CopyableExpr`]s. For any copyable input to a
/// resource node, its value as a copyable expression can be retrieved using
/// [`Subcircuit::get_copyable_expression`]. These expressions are constructed
/// on the fly from the set of copyable inputs of the subcircuit.
///
/// ## Differences with [`SiblingSubgraph`]
///
/// There are some performance and feature distinctions between [`Subcircuit`]s
/// and [`SiblingSubgraph`] to be aware of:
///  - Subcircuit are typically more memory efficient. The size of a
///    [`Subcircuit`] instance is not linear in the number of nodes in the
///    subgraph, but linear in the number of resources (~qubits) in the
///    subcircuit.
///  - Subcircuits may store some non-convex subgraphs, so conversion to
///    [`SiblingSubgraph`] may fail.
///  - Subcircuits can be updated by extending the intervals without having to
///    recompute the subgraph from scratch or rechecking convexity.
///  - Just like [`SiblingSubgraph`], subcircuits store their inputs and outputs
///    as ordered lists. However, subcircuits impose stricter limitations on the
///    ordering of inputs and outputs: all resource inputs/outputs must come
///    before any copyable inputs/outputs; furthermore, the order of resources
///    at the outputs must match the inputs (i.e. if the i-th resource input
///    comes before the j-th resource input, then the i-th resource output must
///    also come before the j-th resource output).
#[derive(Debug, Clone, PartialEq)]
pub struct Subcircuit<N: HugrNode = hugr::Node> {
    /// The resource intervals making up the resource part of the subcircuit
    intervals: Vec<Interval<N>>,
    /// The subcircuit inputs that are copyable values.
    ///
    /// The copyable expressions within the subcircuit can be computed on the
    /// fly based on these inputs.
    input_copyable: IncomingPorts<N>,
    /// The subcircuit outputs that are copyable values.
    ///
    /// These determine the subset of copyable values in the subcircuit that
    /// are exposed as outputs.
    output_copyable: OutgoingPorts<N>,
}

impl<N: HugrNode> Default for Subcircuit<N> {
    fn default() -> Self {
        Self {
            intervals: Vec::new(),
            input_copyable: Vec::new(),
            output_copyable: Vec::new(),
        }
    }
}

/// Errors that can occur when creating a [`Subcircuit`].
#[derive(Debug, Clone, PartialEq, Display, Error)]
pub enum InvalidSubcircuit<N> {
    /// Copyable values at the output are currently not supported.
    #[display("unsupported copyable output values in {_0:?}")]
    OutputCopyableValues(N),
    /// The [`Subcircuit::try_from_resource_nodes`] constructor does not support pure
    /// copyable nodes. Use [`Subcircuit::try_from_subgraph`] instead.
    #[display("unsupported pure copyable node {_0:?}")]
    PureCopyableNode(N),
    /// The node is not contiguous with the subcircuit.
    #[display("node {_0:?} is not contiguous with the subcircuit")]
    NotContiguous(N),
    /// The node is not contiguous with the subcircuit.
    #[display("unsupported subcircuit boundary: {_0}")]
    UnsupportedBoundary(#[error(not(source))] String),
}

impl<N: HugrNode> Subcircuit<N> {
    /// Create a new subcircuit induced from a single node.
    #[inline(always)]
    pub fn from_node(node: N, circuit: &ResourceScope<impl HugrView<Node = N>>) -> Self {
        Self::try_from_resource_nodes([node], circuit).expect("single node is a valid subcircuit")
    }

    /// Create a new subcircuit induced from a set of nodes.
    ///
    /// All nodes in `nodes` must be resource nodes (i.e. not pure copyable).
    /// To create more general [`Subcircuit`]s, use
    /// [`Subcircuit::try_from_subgraph`].
    pub fn try_from_resource_nodes(
        nodes: impl IntoIterator<Item = N>,
        circuit: &ResourceScope<impl HugrView<Node = N>>,
    ) -> Result<Self, InvalidSubcircuit<N>> {
        // For each resource, track the largest interval that contains all nodes,
        // as well as the number of nodes in the interval.
        let mut intervals: IndexMap<ResourceId, (Interval<N>, usize)> = IndexMap::new();
        let mut input_copyable = Vec::new();
        let mut output_copyable = Vec::new();

        for node in nodes {
            if is_pure_copyable(node, circuit.hugr()) {
                // We do not support pure copyable nodes. Use
                // [`Subcircuit::try_from_subgraph`] instead.
                return Err(InvalidSubcircuit::PureCopyableNode(node));
            }

            extend_intervals(&mut intervals, node, circuit);

            // Collect copyable input and output boundary values
            for p in circuit.get_copyable_ports(node, Direction::Incoming) {
                input_copyable.push(vec![(node, p.as_incoming().expect("incoming port"))]);
            }
            for p in circuit.get_copyable_ports(node, Direction::Outgoing) {
                output_copyable.push((node, p.as_outgoing().expect("outgoing port")));
            }
        }

        // Check that all intervals are full, i.e. all expected nodes are present
        for &(interval, num_nodes) in intervals.values() {
            let exp_num_nodes = circuit.nodes_in_interval(interval).count();
            if num_nodes != exp_num_nodes {
                return Err(InvalidSubcircuit::NotContiguous(interval.start_node()));
            }
        }

        let intervals = intervals
            .into_values()
            .map(|(interval, _)| interval)
            .collect_vec();

        Ok(Self {
            intervals,
            input_copyable,
            output_copyable,
        })
    }

    /// Create a new empty subcircuit.
    pub fn new_empty() -> Self {
        Self::default()
    }

    /// Create a new subcircuit from a [`SiblingSubgraph`].
    ///
    /// The returned subcircuit will match the boundary of the subgraph. If
    /// the boundary cannot be matched, an error is returned.
    pub fn try_from_subgraph(
        subgraph: &SiblingSubgraph<N>,
        circuit: &ResourceScope<impl HugrView<Node = N>>,
    ) -> Result<Self, InvalidSubcircuit<N>> {
        let resource_nodes = subgraph
            .nodes()
            .iter()
            .filter(|&&n| !is_pure_copyable(n, circuit.hugr()))
            .copied();
        let mut subcircuit = Self::try_from_resource_nodes(resource_nodes, circuit)?;

        // Now adjust the boundary of subcircuit to match the subgraph
        let (resource_inputs, copyable_inputs) = parse_input_boundary(subgraph, circuit)?;
        let (resource_outputs, copyable_outputs) = parse_output_boundary(subgraph, circuit)?;

        // Reorder intervals to match resource inputs/outputs
        subcircuit.reorder_intervals(&resource_inputs, &resource_outputs, circuit)?;

        // Ensure all copyable inputs/outputs of the subgraph are included
        let missing_inputs = copyable_inputs
            .iter()
            .flatten()
            .filter(|np| !subcircuit.input_copyable.iter().flatten().contains(np))
            .copied()
            .collect_vec();
        let missing_outputs = copyable_outputs
            .iter()
            .filter(|np| !subcircuit.output_copyable.contains(np))
            .copied()
            .collect_vec();
        subcircuit
            .input_copyable
            .extend(missing_inputs.into_iter().map(|np| vec![np]));
        subcircuit.output_copyable.extend(missing_outputs);

        // Remove all copyable inputs not in the subgraph boundary
        let remove_inputs = subcircuit
            .input_copyable
            .iter()
            .flatten()
            .filter(|&np| !copyable_inputs.iter().flatten().contains(np))
            .copied()
            .collect_vec();
        for (node, port) in remove_inputs {
            subcircuit
                .try_remove_copyable_input(node, port, circuit)
                .map_err(|err| {
                    InvalidSubcircuit::UnsupportedBoundary(format!(
                        "copyable input {:?} is not in subgraph boundary but cannot be removed: {err}",
                        (node, port)
                    ))
                })?;
        }

        // It is now safe to set the copyable inputs/outputs to match subgraph
        // (basically just a reordering + grouping + discarding unused outputs)
        subcircuit.input_copyable = copyable_inputs;
        subcircuit.output_copyable = copyable_outputs;

        Ok(subcircuit)
    }

    /// Extend the subcircuit to include the given node.
    ///
    /// Return whether the subcircuit whether the extension was successful, i.e.
    /// return `true` if the subcircuit was modified and `false` if it is left
    /// unchanged (because the node was already in the subcircuit).
    ///
    /// An error will be returned if the subcircuit cannot be extended to
    /// include the node. Currently, this also fails if the node has
    /// copyable values at its outputs.
    pub fn try_add_node(
        &mut self,
        node: N,
        circuit: &ResourceScope<impl HugrView<Node = N>>,
    ) -> Result<bool, InvalidSubcircuit<N>> {
        // Do not support copyable values at node outputs
        let output_copyable_values = circuit.all_copyable_wires(node, Direction::Outgoing);
        if output_copyable_values.count() > 0 {
            return Err(InvalidSubcircuit::OutputCopyableValues(node));
        }

        let backup = self.to_owned();
        let mut was_changed = false;

        // Extend the subcircuit resource intervals to include the node
        match self.try_extend_resources(node, circuit) {
            Ok(new_change) => was_changed |= new_change,
            Err(err) => {
                *self = backup;
                return Err(err);
            }
        };

        // Add copyable inputs/outputs to the subcircuit where required
        was_changed |= self.extend_copyable_io(node, circuit);

        Ok(was_changed)
    }

    /// Extend the subcircuit by including the intervals of `other`.
    ///
    /// The two subcircuits may not have any resources in common. If they do,
    /// false is returned and `self` is left unchanged. Otherwise return `true`.
    pub fn try_extend(
        &mut self,
        other: Self,
        circuit: &ResourceScope<impl HugrView<Node = N>>,
    ) -> bool {
        let curr: BTreeSet<_> = self.resources(circuit).collect();
        if other
            .resources(circuit)
            .any(|resource| curr.contains(&resource))
        {
            return false;
        }

        self.intervals.extend(other.intervals.iter().copied());
        extend_unique(&mut self.input_copyable, other.input_copyable);
        extend_unique(&mut self.output_copyable, other.output_copyable);

        true
    }

    /// Remove a copyable input from the subcircuit.
    ///
    /// This is possible when the input can be expressed as a function of other
    /// inputs or computations within the subcircuit.
    pub fn try_remove_copyable_input(
        &mut self,
        node: N,
        port: IncomingPort,
        circuit: &ResourceScope<impl HugrView<Node = N>>,
    ) -> Result<(), RemoveCopyableInputError<N>> {
        if !self.input_copyable.iter().flatten().contains(&(node, port)) {
            return Err(RemoveCopyableInputError::InputNotFound(node, port));
        }

        let value = circuit
            .hugr()
            .single_linked_output(node, port)
            .expect("valid dataflow wire");
        let Ok(value_ast) = CopyableExpr::try_new(
            value,
            self.copyable_inputs(circuit).collect(),
            iter::empty().collect(),
            circuit,
        ) else {
            return Err(RemoveCopyableInputError::NonConvexAST(value.0, value.1));
        };
        let CopyableExpr::Composite { subgraph } = value_ast else {
            return Err(RemoveCopyableInputError::TrivialExpression(
                value.0, value.1,
            ));
        };

        let known_inputs: BTreeSet<_> = self
            .copyable_inputs(circuit)
            .filter(|&np| np != (node, port))
            .collect();
        let mut subgraph_inputs = subgraph.incoming_ports().iter().flatten();
        let invalid_inp = |(node, port)| {
            if known_inputs.contains(&(node, port)) {
                return false;
            }
            let (prev_node, _) = circuit
                .hugr()
                .single_linked_output(node, port)
                .expect("valid dataflow wire");
            !self.nodes_on_resource_paths(circuit).contains(&prev_node)
        };
        if let Some(&missing_input) = subgraph_inputs.find(|&&np| invalid_inp(np)) {
            return Err(RemoveCopyableInputError::MissingInputs(
                missing_input.0,
                missing_input.1,
            ));
        }

        // It is safe to remove the input
        self.input_copyable.retain_mut(|all_uses| {
            all_uses.retain(|&np| np != (node, port));
            !all_uses.is_empty()
        });

        Ok(())
    }

    /// Iterate over the resources in the subcircuit.
    pub fn resources<'a>(
        &'a self,
        circuit: &'a ResourceScope<impl HugrView<Node = N>>,
    ) -> impl Iterator<Item = ResourceId> + 'a {
        self.intervals
            .iter()
            .map(|interval| interval.resource_id(circuit))
    }

    /// Nodes in the subcircuit.
    pub fn nodes_on_resource_paths<'a>(
        &'a self,
        circuit: &'a ResourceScope<impl HugrView<Node = N>>,
    ) -> impl Iterator<Item = N> + 'a {
        self.intervals
            .iter()
            .map(|interval| circuit.nodes_in_interval(*interval))
            .kmerge_by(|&n1, &n2| {
                let pos1 = circuit.get_position(n1).expect("valid node");
                let pos2 = circuit.get_position(n2).expect("valid node");
                (pos1, n1) < (pos2, n2)
            })
            .dedup()
    }

    /// All nodes in the subcircuit.
    ///
    /// This includes both nodes on resource paths and within copyable
    /// expressions.
    pub fn nodes<'a>(&'a self, circuit: &'a ResourceScope<impl HugrView<Node = N>>) -> IndexSet<N> {
        let mut nodes: IndexSet<N> = self.nodes_on_resource_paths(circuit).collect();

        // Add any nodes and function calls that are part of copyable expressions
        for n in self.nodes_on_resource_paths(circuit) {
            for p in circuit
                .get_copyable_ports(n, Direction::Incoming)
                .map(|p| p.as_incoming().expect("incoming port"))
            {
                let Some(expr) = self.get_copyable_expression(n, p, circuit) else {
                    continue;
                };
                let Some(subgraph) = expr.as_subgraph() else {
                    continue;
                };
                nodes.extend(subgraph.nodes());
            }
        }

        nodes
    }

    /// All function calls in the subcircuit.
    ///
    /// Currently, this only handles function calls within copyable
    /// expressions.
    pub fn function_calls<'a>(
        &'a self,
        circuit: &'a ResourceScope<impl HugrView<Node = N>>,
    ) -> IndexSet<Vec<(N, IncomingPort)>> {
        let mut func_calls = IndexSet::<Vec<(N, IncomingPort)>>::new();

        // Add any nodes and function calls that are part of copyable expressions
        for n in self.nodes_on_resource_paths(circuit) {
            for p in circuit
                .get_copyable_ports(n, Direction::Incoming)
                .map(|p| p.as_incoming().expect("incoming port"))
            {
                let Some(expr) = self.get_copyable_expression(n, p, circuit) else {
                    continue;
                };
                let Some(subgraph) = expr.as_subgraph() else {
                    continue;
                };
                func_calls.extend(subgraph.function_calls().clone());
            }
        }

        func_calls
    }

    /// Number of nodes in the subcircuit.
    pub fn node_count(&self, circuit: &ResourceScope<impl HugrView<Node = N>>) -> usize {
        self.nodes_on_resource_paths(circuit).count()
    }

    /// Whether the subcircuit is empty.
    pub fn is_empty(&self) -> bool {
        self.intervals.is_empty()
    }

    /// Get the interval for the given line.
    pub fn get_interval(
        &self,
        resource: ResourceId,
        circuit: &ResourceScope<impl HugrView<Node = N>>,
    ) -> Option<Interval<N>> {
        self.intervals
            .iter()
            .find(|interval| interval.resource_id(circuit) == resource)
            .copied()
    }

    fn get_interval_mut(
        &mut self,
        resource: ResourceId,
        circuit: &ResourceScope<impl HugrView<Node = N>>,
    ) -> Option<&mut Interval<N>> {
        self.intervals
            .iter_mut()
            .find(|interval| interval.resource_id(circuit) == resource)
    }

    /// Iterate over the line indices of the subcircuit and their intervals.
    pub fn intervals_iter(&self) -> impl Iterator<Item = Interval<N>> + '_ {
        self.intervals.iter().copied()
    }

    /// Number of intervals in the subcircuit.
    pub fn num_intervals(&self) -> usize {
        self.intervals.len()
    }

    /// Get the input ports of the subcircuit.
    ///
    /// The linear ports will come first, followed by all copyable values used
    /// in the subcircuit. Within each group, the ports are ordered in the order
    /// in which they were added to the subcircuit.
    pub fn input_ports(
        &self,
        circuit: &ResourceScope<impl HugrView<Node = N>>,
    ) -> IncomingPorts<N> {
        let resource_ports = self.resource_inputs(circuit);
        resource_ports
            .chain(self.copyable_inputs(circuit))
            .map(|(node, port)| vec![(node, port)])
            .collect_vec()
    }

    /// Get the output ports of the subcircuit.
    ///
    /// This will only contain linear ports (copyable outputs are not supported
    /// at the moment). The ports are ordered in the order in which they were
    /// added to the subcircuit.
    pub fn output_ports(
        &self,
        circuit: &ResourceScope<impl HugrView<Node = N>>,
    ) -> OutgoingPorts<N> {
        self.resource_boundary(circuit, Direction::Outgoing)
            .map(|(node, port)| {
                let port = port.as_outgoing().expect("boundary_resource_ports dir");
                (node, port)
            })
            .collect_vec()
    }

    /// Get the dataflow signature of the subcircuit.
    pub fn dataflow_signature(
        &self,
        circuit: &ResourceScope<impl HugrView<Node = N>>,
    ) -> Signature {
        let port_type = |n: N, p: Port| {
            let op = circuit.hugr().get_optype(n);
            let signature = op.dataflow_signature().expect("dataflow op");
            signature.port_type(p).expect("valid dfg port").clone()
        };

        let input_types = self.input_ports(circuit).into_iter().map(|all_uses| {
            let (n, p) = all_uses.into_iter().next().expect("all inputs are used");
            port_type(n, p.into())
        });
        let output_types = self
            .output_ports(circuit)
            .into_iter()
            .map(|(n, p)| port_type(n, p.into()));

        Signature::new(input_types.collect_vec(), output_types.collect_vec())
    }

    /// Whether the subcircuit is a valid [`SiblingSubgraph`].
    ///
    /// Calling this method will succeed if and only if the subcircuit can be
    /// converted to a [`SiblingSubgraph`] using [`Self::try_to_subgraph`].
    pub fn validate_subgraph(
        &self,
        circuit: &ResourceScope<impl HugrView<Node = N>>,
    ) -> Result<(), InvalidSubgraph<N>> {
        if self.is_empty() {
            return Err(InvalidSubgraph::EmptySubgraph);
        }

        if !circuit.is_convex(self) {
            return Err(InvalidSubgraph::NotConvex);
        }

        Ok(())
    }

    /// Whether the subcircuit is a valid subcircuit.
    pub fn validate_subcircuit(&self, circuit: &ResourceScope<impl HugrView<Node = N>>) {
        for interval in self.intervals_iter() {
            let node = interval.start_node();
            assert_eq!(
                circuit.get_position(node),
                Some(interval.start_pos(circuit)),
                "start node has position {:?}, expected {:?}",
                circuit.get_position(node),
                interval.start_pos(circuit)
            );
            assert!(
                circuit
                    .nodes_in_interval(interval)
                    .is_sorted_by_key(|n| circuit.get_position(n).unwrap()),
                "nodes in interval are not sorted by position"
            );
            let end_node = interval.end_node();
            assert_eq!(
                circuit.get_position(end_node),
                Some(interval.end_pos(circuit)),
                "end node has position {:?}, expected {:?}",
                circuit.get_position(end_node),
                interval.end_pos(circuit)
            );
        }
    }

    /// Convert the subcircuit to a [`SiblingSubgraph`].
    ///
    /// You may use [`Self::validate_subgraph`] to check whether converting the
    /// subcircuit to a [`SiblingSubgraph`] will succeed.
    pub fn try_to_subgraph(
        &self,
        circuit: &ResourceScope<impl HugrView<Node = N>>,
    ) -> Result<SiblingSubgraph<N>, InvalidSubgraph<N>> {
        self.validate_subgraph(circuit)?;

        Ok(SiblingSubgraph::new_unchecked(
            self.input_ports(circuit),
            self.output_ports(circuit),
            self.function_calls(circuit).into_iter().collect(),
            self.nodes(circuit).into_iter().collect(),
        ))
    }

    /// Create a rewrite rule to replace the subcircuit with a new circuit.
    ///
    /// # Parameters
    /// * `circuit` - The base circuit that contains the subcircuit.
    /// * `replacement` - The new circuit to replace the subcircuit with.
    pub fn create_rewrite(
        &self,
        replacement: Circuit<impl HugrView<Node = hugr::Node>>,
        circuit: &ResourceScope<impl HugrView<Node = N>>,
    ) -> Result<CircuitRewrite<N>, InvalidReplacement> {
        CircuitRewrite::try_new(
            &self
                .try_to_subgraph(circuit)
                .map_err(|_| InvalidReplacement::NonConvexSubgraph)?,
            circuit.hugr(),
            replacement,
        )
    }

    /// Get the linear input ports of the subcircuit.
    pub fn resource_inputs<'a>(
        &'a self,
        scope: &'a ResourceScope<impl HugrView<Node = N>>,
    ) -> impl Iterator<Item = (N, IncomingPort)> + 'a {
        self.intervals
            .iter()
            .filter_map(|interval| interval.incoming_boundary_port(scope))
    }

    /// Get the linear output ports of the subcircuit.
    pub fn resource_outputs<'a>(
        &'a self,
        scope: &'a ResourceScope<impl HugrView<Node = N>>,
    ) -> impl Iterator<Item = (N, OutgoingPort)> + 'a {
        self.intervals
            .iter()
            .filter_map(|interval| interval.outgoing_boundary_port(scope))
    }

    /// Get the linear input or output ports of the subcircuit.
    pub fn resource_boundary<'a>(
        &'a self,
        circuit: &'a ResourceScope<impl HugrView<Node = N>>,
        dir: Direction,
    ) -> impl Iterator<Item = (N, Port)> + 'a {
        match dir {
            Direction::Incoming => {
                Either::Left(self.resource_inputs(circuit).map(|(n, p)| (n, p.into())))
            }
            Direction::Outgoing => {
                Either::Right(self.resource_outputs(circuit).map(|(n, p)| (n, p.into())))
            }
        }
    }

    /// Get the copyable input ports of the subcircuit.
    pub fn copyable_inputs(
        &self,
        _scope: &ResourceScope<impl HugrView<Node = N>>,
    ) -> impl Iterator<Item = (N, IncomingPort)> + '_ {
        self.input_copyable.iter().flatten().copied()
    }

    /// Get the copyable output ports of the subcircuit.
    pub fn copyable_outputs(
        &self,
        _scope: &ResourceScope<impl HugrView<Node = N>>,
    ) -> impl Iterator<Item = (N, OutgoingPort)> + '_ {
        self.output_copyable.iter().copied()
    }

    /// Get the indices and values of the subcircuit inputs that can be
    /// statically resolved to constants.
    pub fn get_const_inputs<'c>(
        &'c self,
        circuit: &'c ResourceScope<impl HugrView<Node = N>>,
    ) -> impl Iterator<Item = ((N, IncomingPort), constant::Value)> + 'c {
        self.copyable_inputs(circuit).filter_map(move |(n, p)| {
            let wire = circuit.get_copyable_wire(n, p)?;
            let val = circuit.as_const_value(wire.into())?;
            Some(((n, p), val.clone()))
        })
    }

    /// Get the copyable expression for the given input port, if it is a
    /// copyable port of the subcircuit.
    ///
    /// This panics if the subcircuit is not valid in `circuit`.
    pub fn get_copyable_expression(
        &self,
        node: N,
        port: IncomingPort,
        circuit: &ResourceScope<impl HugrView<Node = N>>,
    ) -> Option<CopyableExpr<N>> {
        if circuit
            .get_circuit_unit(node, port)
            .is_none_or(|unit| unit.is_resource())
        {
            // Not a known copyable unit
            return None;
        }

        if !self.nodes_on_resource_paths(circuit).contains(&node) {
            // Node is not on an interval of the subcircuit
            return None;
        }

        let value = circuit
            .hugr()
            .single_linked_output(node, port)
            .expect("valid dataflow wire");

        if self.copyable_inputs(circuit).contains(&(node, port)) {
            return Some(CopyableExpr::Wire(Wire::new(value.0, value.1)));
        }

        let expr = CopyableExpr::try_new(
            value,
            self.copyable_inputs(circuit).collect(),
            iter::empty().collect(),
            circuit,
        )
        .expect("valid copyable expression");

        Some(expr)
    }
}

#[allow(clippy::type_complexity)]
fn parse_input_boundary<N: HugrNode>(
    subgraph: &SiblingSubgraph<N>,
    circuit: &ResourceScope<impl HugrView<Node = N>>,
) -> Result<(Vec<(N, IncomingPort)>, IncomingPorts<N>), InvalidSubcircuit<N>> {
    let mut inp_iter = subgraph.incoming_ports().iter().peekable();

    let is_resource = |inps: &&Vec<_>| {
        let Some(&(node, port)) = inps.iter().exactly_one().ok() else {
            return false;
        };
        circuit
            .get_circuit_unit(node, port)
            .is_some_and(|unit| unit.is_resource())
    };
    let resource_inputs = inp_iter
        .peeking_take_while(is_resource)
        .map(|vec| vec[0])
        .collect_vec();
    let other_inputs = inp_iter.cloned().collect_vec();

    if other_inputs.iter().flatten().any(|(n, p)| {
        circuit
            .get_circuit_unit(*n, *p)
            .is_none_or(|u| u.is_resource())
    }) {
        return Err(InvalidSubcircuit::UnsupportedBoundary(
            "resource inputs must precede copyable inputs".to_string(),
        ));
    }

    Ok((resource_inputs, other_inputs))
}

fn parse_output_boundary<N: HugrNode>(
    subgraph: &SiblingSubgraph<N>,
    circuit: &ResourceScope<impl HugrView<Node = N>>,
) -> Result<(OutgoingPorts<N>, OutgoingPorts<N>), InvalidSubcircuit<N>> {
    let mut out_iter = subgraph.outgoing_ports().iter().copied().peekable();

    let resource_outputs = out_iter
        .peeking_take_while(|&(node, port)| {
            circuit
                .get_circuit_unit(node, port)
                .is_some_and(|unit| unit.is_resource())
        })
        .collect_vec();
    let other_outputs = out_iter.collect_vec();

    if other_outputs.iter().any(|&(node, port)| {
        circuit
            .get_circuit_unit(node, port)
            .is_some_and(|unit| unit.is_resource())
    }) {
        return Err(InvalidSubcircuit::UnsupportedBoundary(
            "resource outputs must precede copyable outputs".to_string(),
        ));
    }

    Ok((resource_outputs, other_outputs))
}

fn extend_unique<E: PartialEq>(vec: &mut Vec<E>, other: impl IntoIterator<Item = E>) {
    let other_unique = other.into_iter().filter(|x| !vec.contains(x)).collect_vec();
    vec.extend(other_unique);
}

impl<H: HugrView> ResourceScope<H> {
    fn is_convex(&self, _subcircuit: &Subcircuit<H::Node>) -> bool {
        unimplemented!("is_convex is not yet implemented")
    }
}

/// Extend the intervals such that the given node is included.
fn extend_intervals<N: HugrNode>(
    intervals: &mut IndexMap<ResourceId, (Interval<N>, usize)>,
    node: N,
    circuit: &ResourceScope<impl HugrView<Node = N>>,
) {
    for res in circuit.get_all_resources(node) {
        let (interval, num_nodes) = intervals.entry(res).or_insert_with(|| {
            (
                Interval::new_singleton(res, node, circuit).expect("node on resource path"),
                0,
            )
        });
        interval.add_node_unchecked(node, circuit);
        *num_nodes += 1;
    }
}

// Private methods
impl<N: HugrNode> Subcircuit<N> {
    fn try_extend_resources(
        &mut self,
        node: N,
        circuit: &ResourceScope<impl HugrView<Node = N>>,
    ) -> Result<bool, InvalidSubcircuit<N>> {
        let mut was_changed = false;

        for resource_id in circuit.get_all_resources(node) {
            let interval = self.get_interval_mut(resource_id, circuit);
            if let Some(interval) = interval {
                match interval.try_extend(node, circuit) {
                    Ok(None) => { /* nothing to do */ }
                    Ok(Some(Direction::Incoming)) => {
                        // Added node to the left of the interval
                        was_changed = true;
                    }
                    Ok(Some(Direction::Outgoing)) => {
                        // Added node to the right of the interval
                        was_changed = true;
                    }
                    Err(InvalidInterval::NotContiguous(node)) => {
                        return Err(InvalidSubcircuit::NotContiguous(node));
                    }
                    Err(InvalidInterval::NotOnResourcePath(node)) => {
                        panic!("{resource_id:?} is not a valid resource for node {node:?}")
                    }
                    Err(InvalidInterval::StartAfterEnd(_, _, _)) => {
                        panic!("invalid interval for resource {resource_id:?}")
                    }
                }
            } else {
                was_changed = true;
                self.intervals.push(
                    Interval::new_singleton(resource_id, node, circuit)
                        .expect("node on resource path"),
                );
            }
        }

        Ok(was_changed)
    }

    fn extend_copyable_io(
        &mut self,
        node: N,
        circuit: &ResourceScope<impl HugrView<Node = N>>,
    ) -> bool {
        let mut was_changed = false;

        for dir in Direction::BOTH {
            let copyable_ports = circuit.get_copyable_ports(node, dir);

            match dir {
                Direction::Incoming => {
                    let new_inputs = copyable_ports
                        .map(|p| vec![(node, p.as_incoming().expect("incoming port"))]);

                    let len = self.input_copyable.len();
                    extend_unique(&mut self.input_copyable, new_inputs);
                    was_changed |= self.input_copyable.len() > len;
                }
                Direction::Outgoing => {
                    let new_outputs =
                        copyable_ports.map(|p| (node, p.as_outgoing().expect("outgoing port")));

                    let len = self.output_copyable.len();
                    extend_unique(&mut self.output_copyable, new_outputs);
                    was_changed |= self.output_copyable.len() > len;
                }
            }
        }

        was_changed
    }

    fn reorder_intervals(
        &mut self,
        resource_inputs: &[(N, IncomingPort)],
        resource_outputs: &[(N, OutgoingPort)],
        circuit: &ResourceScope<impl HugrView<Node = N>>,
    ) -> Result<(), InvalidSubcircuit<N>> {
        if self
            .resource_inputs(circuit)
            .any(|np| !resource_inputs.contains(&np))
        {
            return Err(InvalidSubcircuit::UnsupportedBoundary(
                "resource inputs in subcircuit do not match subgraph".to_string(),
            ));
        }

        let inp_pos = |interval: &Interval<N>| {
            let (node, port) = interval.incoming_boundary_port(circuit)?;
            resource_inputs.iter().position(|&np| np == (node, port))
        };
        let out_pos = |interval: &Interval<N>| {
            let (node, port) = interval.outgoing_boundary_port(circuit)?;
            resource_outputs.iter().position(|&np| np == (node, port))
        };
        self.intervals.sort_unstable_by_key(out_pos);
        // important: use stable sort to preserve output ordering where possible
        self.intervals.sort_by_key(inp_pos);

        if !self.intervals.iter().is_sorted_by_key(out_pos) {
            // There is no interval ordering that satisfies both input and output orderings
            return Err(InvalidSubcircuit::UnsupportedBoundary(
                "cannot order intervals to match subgraph boundary".to_string(),
            ));
        }

        Ok(())
    }
}

/// Errors that can occur when removing a copyable input from a subcircuit.
#[derive(Debug, Clone, PartialEq, Display, Error)]
pub enum RemoveCopyableInputError<N> {
    /// The specified input was not found in the subcircuit.
    #[display("input ({_0:?}, {_1:?}) not found in subcircuit")]
    InputNotFound(N, IncomingPort),
    /// The value at the port cannot be expressed as a convex AST.
    #[display("value at port ({_0:?}, {_1:?}) cannot be expressed as convex AST")]
    NonConvexAST(N, OutgoingPort),
    /// The value at the port is a trivial expression that cannot be expanded.
    #[display("value at port ({_0:?}, {_1:?}) cannot be replaced by non-trivial AST")]
    TrivialExpression(N, OutgoingPort),
    /// The subcircuit is missing inputs required to replace the value with the
    /// AST.
    #[display("input ({_0:?}, {_1:?}) is required to replace value with AST, but is missing")]
    MissingInputs(N, IncomingPort),
}

#[cfg(test)]
mod tests {
    use super::expression::tests::hugr_with_midcircuit_meas;
    use super::*;
    use crate::{
        extension::rotation::rotation_type,
        resource::{
            tests::{cx_circuit, cx_rz_circuit},
            ResourceAllocator,
        },
        utils::build_simple_circuit,
        TketOp,
    };
    use cool_asserts::assert_matches;
    use hugr::{extension::prelude::qb_t, types::Signature, CircuitUnit, Hugr, Node, OutgoingPort};
    use rstest::{fixture, rstest};

    #[rstest]
    #[case::empty_set(vec![], true, "empty subcircuit is valid")]
    #[case::single_node(vec![0], true, "single node should succeed")]
    #[case::two_adjacent_nodes(vec![0, 1], true, "two adjacent nodes should succeed")]
    #[case::three_adjacent_nodes(vec![0, 1, 2], true, "three adjacent nodes should succeed")]
    #[case::all_nodes(vec![0, 1, 2, 3, 4], true, "all nodes should succeed")]
    #[case::non_adjacent_nodes(vec![0, 2], false, "non-adjacent nodes should fail")]
    #[case::gap_in_middle(vec![0, 1, 3, 4], false, "gap in middle should fail")]
    #[case::last_two_nodes(vec![3, 4], true, "last two nodes should succeed")]
    fn test_try_from_nodes_cx_circuit(
        #[case] node_indices: Vec<usize>,
        #[case] should_succeed: bool,
        #[case] description: &str,
    ) {
        let circ = cx_circuit(5);
        let subgraph = Circuit::from(&circ).subgraph().unwrap();
        let cx_nodes = subgraph.nodes().to_owned();
        let scope = ResourceScope::new(&circ, subgraph);

        let nodes: Vec<_> = node_indices.into_iter().map(|i| cx_nodes[i]).collect();

        let result = Subcircuit::try_from_resource_nodes(nodes.iter().copied(), &scope);

        if should_succeed {
            assert!(result.is_ok(), "Expected success for case: {description}");
            let subcircuit = result.unwrap();
            assert_eq!(
                subcircuit.nodes_on_resource_paths(&scope).collect_vec(),
                nodes
            );
        } else {
            assert!(result.is_err(), "Expected failure for case: {description}");
        }
    }

    #[rstest]
    #[case::empty_set(vec![], true, 0, 0, 0)]
    #[case::singe_h_gate(vec![7], true, 1, 1, 0)]
    #[case::two_h_gates(vec![7, 8], true, 2, 2, 0)]
    #[case::h_and_cx_gate(vec![7, 9], true, 2, 2, 0)]
    #[case::cx_rz_rz_same_angle(vec![9, 10, 11], true, 2, 2, 2)]
    #[case::cx_rz_rz_diff_angle(vec![9, 10, 15], true, 2, 2, 2)]
    fn test_try_from_nodes_cx_rz_circuit(
        #[case] node_indices: Vec<usize>,
        #[case] should_succeed: bool,
        #[case] expected_input_resources: usize,
        #[case] expected_output_resources: usize,
        #[case] expected_copyable_inputs: usize,
    ) {
        let circ = cx_rz_circuit(2, true, true);
        let subgraph = Circuit::from(&circ).subgraph().unwrap();
        let scope = ResourceScope::new(&circ, subgraph);

        let selected_nodes: Vec<_> = node_indices
            .into_iter()
            .map(|i| Node::from(portgraph::NodeIndex::new(i)))
            .collect();

        let result = Subcircuit::try_from_resource_nodes(selected_nodes.iter().copied(), &scope);

        if should_succeed {
            assert!(result.is_ok());
            let subcircuit = result.unwrap();
            assert_eq!(
                subcircuit.nodes_on_resource_paths(&scope).collect_vec(),
                selected_nodes
            );
            assert_eq!(
                subcircuit.resource_inputs(&scope).count(),
                expected_input_resources,
                "Wrong number of input resources"
            );
            assert_eq!(
                subcircuit.resource_outputs(&scope).count(),
                expected_output_resources,
                "Wrong number of output resources"
            );
            assert_eq!(
                subcircuit.copyable_inputs(&scope).count(),
                expected_copyable_inputs,
                "Wrong number of copyable inputs"
            );
        } else {
            assert!(result.is_err());
        }
    }

    #[test]
    fn try_extend_cx_rz_circuit() {
        let circ = cx_rz_circuit(2, true, true);
        let subgraph = Circuit::from(&circ).subgraph().unwrap();
        let circ = ResourceScope::new(circ, subgraph);

        let mut subcircuit = Subcircuit::new_empty();

        let node = |i: usize| Node::from(portgraph::NodeIndex::new(i));
        let resources = {
            let mut alloc = ResourceAllocator::new();
            [alloc.allocate(), alloc.allocate()]
        };

        // Add first a H gate
        assert_eq!(subcircuit.try_add_node(node(7), &circ), Ok(true));
        assert_eq!(subcircuit.resources(&circ).collect_vec(), [resources[0]]);
        assert_eq!(subcircuit.resource_inputs(&circ).count(), 1);
        assert_eq!(subcircuit.resource_outputs(&circ).count(), 1);
        assert_eq!(subcircuit.try_add_node(node(7), &circ), Ok(false));

        // Now add a two-qubit CX gate
        assert_eq!(subcircuit.try_add_node(node(9), &circ), Ok(true));
        assert_eq!(subcircuit.resources(&circ).collect_vec(), resources);
        assert_eq!(subcircuit.resource_inputs(&circ).count(), 2);
        assert_eq!(subcircuit.resource_outputs(&circ).count(), 2);
        assert_eq!(subcircuit.input_copyable, IncomingPorts::new());
        assert_eq!(subcircuit.try_add_node(node(9), &circ), Ok(false));

        // Cannot add this non-contiguous rotation
        let subcircuit_clone = subcircuit.clone();
        assert_eq!(
            subcircuit.try_add_node(node(16), &circ),
            Err(InvalidSubcircuit::NotContiguous(node(16)))
        );
        assert_eq!(subcircuit, subcircuit_clone);

        // Now add a contiguous rotation
        assert_eq!(subcircuit.try_add_node(node(10), &circ), Ok(true));
        assert_eq!(subcircuit.resources(&circ).collect_vec(), resources);
        assert_eq!(subcircuit.resource_inputs(&circ).count(), 2);
        assert_eq!(subcircuit.resource_outputs(&circ).count(), 2);
        assert_eq!(subcircuit.input_copyable.len(), 1);
        assert_eq!(subcircuit.try_add_node(node(10), &circ), Ok(false));

        // One more rotation, same angle
        assert_eq!(subcircuit.try_add_node(node(11), &circ), Ok(true));
        assert_eq!(subcircuit.resources(&circ).collect_vec(), resources);
        assert_eq!(subcircuit.resource_inputs(&circ).count(), 2);
        assert_eq!(subcircuit.resource_outputs(&circ).count(), 2);
        assert_eq!(subcircuit.input_copyable.len(), 2);
        assert_eq!(subcircuit.try_add_node(node(11), &circ), Ok(false));

        // Last rotation, different angle
        // now the previously non-contiguous rotation is contiguous
        assert_eq!(subcircuit.try_add_node(node(16), &circ), Ok(true));
        assert_eq!(subcircuit.resources(&circ).collect_vec(), resources);
        assert_eq!(subcircuit.resource_inputs(&circ).count(), 2);
        assert_eq!(subcircuit.resource_outputs(&circ).count(), 2);
        assert_eq!(subcircuit.input_copyable.len(), 3);
        assert_eq!(subcircuit.try_add_node(node(16), &circ), Ok(false));

        assert_eq!(subcircuit.node_count(&circ), 5);
    }

    #[fixture]
    fn ancilla_circ() -> ResourceScope<Hugr> {
        let circ = build_simple_circuit(1, |circ| {
            let empty: [CircuitUnit; 0] = []; // requires type annotation
            let ancilla = circ.append_with_outputs(TketOp::QAlloc, empty)?[0];

            let ancilla = circ.append_with_outputs(
                TketOp::CX,
                [CircuitUnit::Linear(0), CircuitUnit::Wire(ancilla)],
            )?[0];
            circ.append_and_consume(TketOp::QFree, [ancilla])?;

            Ok(())
        })
        .unwrap();
        let subgraph = circ.subgraph().unwrap();
        ResourceScope::new(circ.into_hugr(), subgraph)
    }

    #[rstest]
    fn try_extend_remove_input_output(ancilla_circ: ResourceScope<Hugr>) {
        let mut subcircuit = Subcircuit::new_empty();
        let node = |i: usize| Node::from(portgraph::NodeIndex::new(i));
        let resources = {
            let mut alloc = ResourceAllocator::new();
            [alloc.allocate(), alloc.allocate()]
        };

        // Add a two-qubit CX gates, as usual => two inputs, two outputs
        assert_eq!(subcircuit.try_add_node(node(5), &ancilla_circ), Ok(true));
        assert_eq!(subcircuit.resources(&ancilla_circ).collect_vec(), resources);
        assert_eq!(subcircuit.resource_inputs(&ancilla_circ).count(), 2);
        assert_eq!(subcircuit.resource_outputs(&ancilla_circ).count(), 2);
        assert_eq!(subcircuit.input_copyable.len(), 0);

        // Add the qalloc; now the second qubit is no more an input
        assert_eq!(subcircuit.try_add_node(node(4), &ancilla_circ), Ok(true));
        assert_eq!(subcircuit.resources(&ancilla_circ).collect_vec(), resources);
        assert_eq!(subcircuit.resource_inputs(&ancilla_circ).count(), 1);
        assert_eq!(subcircuit.resource_outputs(&ancilla_circ).count(), 2);
        assert_eq!(subcircuit.input_copyable.len(), 0);

        // Add the qfree; the second qubit is no longer an output either
        assert_eq!(subcircuit.try_add_node(node(6), &ancilla_circ), Ok(true));
        assert_eq!(subcircuit.resources(&ancilla_circ).collect_vec(), resources);
        assert_eq!(subcircuit.resource_inputs(&ancilla_circ).count(), 1);
        assert_eq!(subcircuit.resource_outputs(&ancilla_circ).count(), 1);
        assert_eq!(subcircuit.input_copyable.len(), 0);
    }

    #[test]
    #[should_panic(expected = "is_convex is not yet implemented")]
    fn test_to_subgraph() {
        let circ = cx_rz_circuit(2, true, false);
        let subgraph = Circuit::from(&circ).subgraph().unwrap();
        let circ = ResourceScope::new(circ, subgraph);

        let mut subcircuit = Subcircuit::new_empty();

        let node = |i: usize| Node::from(portgraph::NodeIndex::new(i));

        // Add first a H gate
        subcircuit.try_add_node(node(7), &circ).unwrap();
        assert_eq!(
            subcircuit.input_ports(&circ),
            vec![vec![(node(7), IncomingPort::from(0))]]
        );
        assert_eq!(
            subcircuit.output_ports(&circ),
            vec![(node(7), OutgoingPort::from(0))]
        );

        // Now add a two-qubit CX gate
        subcircuit.try_add_node(node(9), &circ).unwrap();
        assert_eq!(
            subcircuit.input_ports(&circ),
            vec![
                vec![(node(7), IncomingPort::from(0))],
                vec![(node(9), IncomingPort::from(1))]
            ]
        );
        assert_eq!(
            subcircuit.output_ports(&circ),
            vec![
                (node(9), OutgoingPort::from(0)),
                (node(9), OutgoingPort::from(1))
            ]
        );

        // Now add two contiguous rotation
        subcircuit.try_add_node(node(10), &circ).unwrap();
        subcircuit.try_add_node(node(11), &circ).unwrap();
        assert_eq!(
            subcircuit.input_ports(&circ),
            vec![
                vec![(node(7), IncomingPort::from(0))],
                vec![(node(9), IncomingPort::from(1))],
                vec![(node(10), IncomingPort::from(1)),],
                vec![(node(11), IncomingPort::from(1))],
            ]
        );
        assert_eq!(
            subcircuit.output_ports(&circ),
            vec![
                (node(10), OutgoingPort::from(0)),
                (node(11), OutgoingPort::from(0)),
            ]
        );

        let subgraph = subcircuit.try_to_subgraph(&circ).unwrap();
        assert!(subgraph.validate(circ.hugr(), Default::default()).is_ok());
        let mut nodes = subgraph.nodes().to_owned();
        nodes.sort_unstable();
        assert_eq!(nodes, vec![node(7), node(9), node(10), node(11)]);
        assert_eq!(
            subgraph.signature(circ.hugr()),
            Signature::new(vec![qb_t(), qb_t(), rotation_type()], vec![qb_t(), qb_t()],)
        );
    }

    #[test]
    #[should_panic(expected = "is_convex is not yet implemented")] // TODO: remove this once is_convex is implemented
    fn test_to_subgraph_invalid() {
        let circ = cx_rz_circuit(2, true, false);
        let subgraph = Circuit::from(&circ).subgraph().unwrap();
        let circ = ResourceScope::new(circ, subgraph);

        let mut subcircuit = Subcircuit::new_empty();

        assert_eq!(
            subcircuit.try_to_subgraph(&circ),
            Err(InvalidSubgraph::EmptySubgraph)
        );

        let node = |i: usize| Node::from(portgraph::NodeIndex::new(i));

        // Add a H gate and a Rz gate, but omitting the CX gate in-between
        subcircuit.try_add_node(node(7), &circ).unwrap();
        subcircuit.try_add_node(node(11), &circ).unwrap();

        assert_eq!(
            subcircuit.try_to_subgraph(&circ),
            Err(InvalidSubgraph::NotConvex)
        );
    }

    #[rstest]
    fn test_remove_expr(hugr_with_midcircuit_meas: Hugr) {
        let circ = ResourceScope::from_circuit(Circuit::new(hugr_with_midcircuit_meas));

        let mut subcircuit = Subcircuit::try_from_resource_nodes(
            [
                Node::from(portgraph::NodeIndex::new(5)),
                Node::from(portgraph::NodeIndex::new(10)),
            ],
            &circ,
        )
        .unwrap();

        assert_eq!(subcircuit.copyable_inputs(&circ).count(), 1);
        assert_eq!(subcircuit.copyable_outputs(&circ).count(), 1);

        let inp = subcircuit
            .copyable_inputs(&circ)
            .exactly_one()
            .ok()
            .unwrap();
        assert_matches!(
            subcircuit
                .get_copyable_expression(inp.0, inp.1, &circ)
                .unwrap(),
            CopyableExpr::Wire { .. }
        );

        subcircuit
            .try_remove_copyable_input(inp.0, inp.1, &circ)
            .unwrap();
        let CopyableExpr::Composite { subgraph } = subcircuit
            .get_copyable_expression(inp.0, inp.1, &circ)
            .unwrap()
        else {
            panic!("expected composite expression");
        };
        assert_eq!(
            subgraph.nodes(),
            (6..=8)
                .map(|i| Node::from(portgraph::NodeIndex::new(i)),)
                .collect_vec()
        )
    }

    #[rstest]
    #[case::simple_subgraph(
        vec![
            vec![(
                Node::from(portgraph::NodeIndex::new(5)),
                IncomingPort::from(0),
            )],
            vec![(
                Node::from(portgraph::NodeIndex::new(6)),
                IncomingPort::from(1),
            )],
        ],
        vec![(
            Node::from(portgraph::NodeIndex::new(10)),
            OutgoingPort::from(0),
        )],
    )]
    #[case::more_complex_subgraph(
        vec![
            vec![(
                Node::from(portgraph::NodeIndex::new(10)),
                IncomingPort::from(0),
            )],
            vec![(
                Node::from(portgraph::NodeIndex::new(7)),
                IncomingPort::from(0),
            )],
        ],
        vec![(
            Node::from(portgraph::NodeIndex::new(10)),
            OutgoingPort::from(0),
        )],
    )]
    fn test_from_subgraph(
        hugr_with_midcircuit_meas: Hugr,
        #[case] inputs: IncomingPorts,
        #[case] outputs: OutgoingPorts,
    ) {
        let circ = ResourceScope::from_circuit(Circuit::new(hugr_with_midcircuit_meas));

        let subgraph = SiblingSubgraph::try_new(inputs, outputs, circ.hugr()).unwrap();
        let subcircuit = Subcircuit::try_from_subgraph(&subgraph, &circ).unwrap();

        let exp_resource_nodes = subgraph
            .nodes()
            .iter()
            .copied()
            .filter(|&n| !is_pure_copyable(n, circ.hugr()))
            .collect_vec();
        assert_eq!(subgraph.incoming_ports(), &subcircuit.input_ports(&circ));
        assert_eq!(subgraph.outgoing_ports(), &subcircuit.output_ports(&circ));
        assert_eq!(
            subcircuit.nodes_on_resource_paths(&circ).collect_vec(),
            exp_resource_nodes
        );
        assert_eq!(
            subcircuit.nodes(&circ).into_iter().collect::<BTreeSet<_>>(),
            subgraph.nodes().iter().copied().collect::<BTreeSet<_>>()
        );
    }
}
