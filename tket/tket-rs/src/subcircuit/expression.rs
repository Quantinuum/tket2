use std::collections::BTreeSet;

use hugr::{
    Direction, HugrView, IncomingPort, OutgoingPort, Wire,
    core::HugrNode,
    hugr::views::{SiblingSubgraph, sibling_subgraph::InvalidSubgraph},
    ops::OpTrait,
    types::EdgeKind,
};
use indexmap::IndexSet;
use itertools::{Either, Itertools};
use priority_queue::PriorityQueue;

use crate::resource::ResourceScope;

/// Hugr fragment for computing a copyable value - either a subgraph, or a
/// single outport
///
/// The value of the expression is `value` if the expression is
/// [CopyableExpr::Wire], or is given by the first output of the
/// subgraph otherwise. The subgraph of [CopyableExpr::Composite] only
/// contains nodes such that all its inputs and outputs are copyable types;
/// further, all inputs and outputs of the subgraph are in the past of the first
/// output (and hence the subgraph is always convex).
///
/// The subgraph may have other (copyable) outputs, which are ignored in the
/// context of the expression.
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum CopyableExpr<N: HugrNode = hugr::Node> {
    /// A trivial expression that is just the value at that wire.
    Wire(Wire<N>),
    /// A composite expression, represented as a hugr subgraph.
    Composite {
        /// The hugr subgraph representing the expression.
        ///
        /// The output value is the first output of the subgraph.
        subgraph: SiblingSubgraph<N>,
    },
}

impl<N: HugrNode> CopyableExpr<N> {
    /// Construct the expression for the value at `output` in `circuit`.
    ///
    /// The inputs of the expr will be determined by traversing `circuit`
    /// backwards starting from `output` and stopping as follows:
    /// - any input port that is contained in `allowed_boundary_inputs` is added
    ///   as an input to the expression;
    /// - any input port that is linked to an outgoing port of a node in
    ///   `nodes_to_exclude` is added as an input to the expression;
    /// - any input port that is linked to an outgoing port of a node that is
    ///   not pure copyable (i.e. has non-copyable inputs or outputs), or that
    ///   is not in `circuit`, is added as an input to the expression.
    ///
    /// The traversal does not progress beyond an incoming port that is added as
    /// input, nor does it progress to nodes outside of `circuit`. The set of
    /// inputs are ordered according to the following ordering:
    /// 1. first by the order in which they appear in `allowed_boundary_inputs`,
    /// 2. then by the order in which their source nodes appear in
    ///    `nodes_to_exclude`,
    /// 3. and finally in the order in which they are encountered during
    ///    traversal.
    ///
    /// If `output` is not attached to a pure copyable node, or is in
    /// `nodes_to_exclude`, then the expression is trivial and a
    /// [CopyableExpr::Wire] variant is returned.
    ///
    /// If the subgraph is not convex, then an [InvalidSubgraph::NotConvex]
    /// error is returned.
    ///
    /// ## Panics
    ///
    /// This will panic if `output` is not a value of copyable type in
    /// `circuit`.
    pub fn try_new(
        output: (N, OutgoingPort),
        allowed_boundary_inputs: IndexSet<(N, IncomingPort)>,
        nodes_to_exclude: IndexSet<N>,
        circuit: &ResourceScope<impl HugrView<Node = N>>,
    ) -> Result<Self, InvalidSubgraph> {
        Self::validate_output_copyable(output, circuit);
        let circuit_nodes = circuit.nodes().iter().copied().collect::<BTreeSet<_>>();

        if !admissible_node(output.0, &nodes_to_exclude, circuit.hugr(), &circuit_nodes) {
            return Ok(Self::Wire(Wire::new(output.0, output.1)));
        }

        let ExprArgs {
            inputs,
            outputs,
            function_calls,
            mut nodes,
        } = Self::traverse_expression(output, &allowed_boundary_inputs, circuit, |node| {
            admissible_node(node, &nodes_to_exclude, circuit.hugr(), &circuit_nodes)
        });

        if circuit.any_reachable_from(outputs.iter().copied(), inputs.iter().copied()) {
            return Err(InvalidSubgraph::NotConvex);
        }

        let inputs = sort_inputs(
            inputs,
            &allowed_boundary_inputs,
            &nodes_to_exclude,
            circuit.hugr(),
        );
        let outputs = outputs.into_iter().collect_vec();
        // Flip topological order
        nodes.reverse();
        let function_calls = group_function_calls(function_calls, circuit);

        let subgraph = SiblingSubgraph::new_unchecked(inputs, outputs, function_calls, nodes);
        Ok(Self::Composite { subgraph })
    }

    fn validate_output_copyable(
        output: (N, OutgoingPort),
        circuit: &ResourceScope<impl HugrView<Node = N>>,
    ) {
        let port_kind = circuit
            .hugr()
            .get_optype(output.0)
            .port_kind(output.1)
            .expect("known output port");
        assert!(matches!(port_kind, EdgeKind::Value(t) if t.copyable()));
    }

    /// Traverse the circuit backwards from `output`, collecting the inputs,
    /// outputs, function calls, and nodes that define the expression subgraph.
    fn traverse_expression(
        output: (N, OutgoingPort),
        allowed_input_ports: &IndexSet<(N, IncomingPort)>,
        circuit: &ResourceScope<impl HugrView<Node = N>>,
        admissible_node: impl Fn(N) -> bool,
    ) -> ExprArgs<N> {
        let mut args = ExprArgs::default();

        // Queues and set useful during traversal.
        let mut curr_nodes = PriorityQueue::new();
        let prio = |node: N| circuit.get_position(node).expect("known node has position");
        let mut visited = BTreeSet::new();
        curr_nodes.push(output.0, prio(output.0));
        let mut visited_incoming_ports = IndexSet::new();

        // Traverse nodes in reverse dataflow order (so that any incoming ports at
        // `node` not in `visited_incoming_ports` must be outside the expr).
        while let Some((node, _)) = curr_nodes.pop() {
            if !visited.insert(node) {
                continue; // been here before
            }

            debug_assert!(
                admissible_node(node),
                "cannot include a non-pure-copyable or allowed input into CopyableExpression"
            );

            // Add node to expression subgraph
            args.nodes.push(node);

            // Add all node outputs that we have not traversed to `outputs`
            let node_outputs = circuit.hugr().out_value_types(node).map(|(p, _)| (node, p));
            args.outputs.extend(node_outputs.filter(|&(n, p)| {
                circuit
                    .hugr()
                    .linked_inputs(n, p)
                    .any(|inp| !visited_incoming_ports.contains(&inp))
            }));
            for (in_port, t) in circuit.hugr().in_value_types(node) {
                debug_assert!(t.copyable());
                let (prev_node, _out_port) = circuit
                    .hugr()
                    .single_linked_output(node, in_port)
                    .expect("valid dataflow wire");

                if allowed_input_ports.contains(&(node, in_port)) || !admissible_node(prev_node) {
                    // Add in_port to expression inputs
                    args.inputs.insert((node, in_port));
                } else {
                    // Continue traversing expression backwards
                    visited_incoming_ports.insert((node, in_port));
                    curr_nodes.push(prev_node, prio(prev_node));
                }
            }

            // Add function call input if there is one
            let op = circuit.hugr().get_optype(node);
            if op.static_input().is_some_and(|edge| edge.is_function()) {
                args.function_calls
                    .push((node, op.static_input_port().expect("function input port")));
            }
        }

        args
    }

    /// Get the output value of the expression, as a port in the hugr graph.
    pub fn output(&self) -> (N, OutgoingPort) {
        match self {
            Self::Wire(wire) => (wire.node(), wire.source()),
            Self::Composite { subgraph } => subgraph.outgoing_ports()[0],
        }
    }

    /// The expression as a hugr subgraph, if it is not the identity.
    pub fn as_subgraph(&self) -> Option<&SiblingSubgraph<N>> {
        match self {
            Self::Wire { .. } => None,
            Self::Composite { subgraph } => Some(subgraph),
        }
    }
}

/// Whether a node only contains copyable inputs and output values.
fn pure_copyable<N: HugrNode>(node: N, hugr: impl HugrView<Node = N>) -> bool {
    let mut all_port_types = Direction::BOTH
        .iter()
        .flat_map(|&dir| hugr.value_types(node, dir));
    all_port_types.all(|(_, t)| t.copyable())
}

/// Whether a node can be added to the copyable expression.
fn admissible_node<N: HugrNode>(
    node: N,
    allowed_input_nodes: &IndexSet<N>,
    hugr: &impl HugrView<Node = N>,
    circuit_nodes: &BTreeSet<N>,
) -> bool {
    !allowed_input_nodes.contains(&node)
        && pure_copyable(node, hugr)
        && circuit_nodes.contains(&node)
}

#[derive(Debug)]
struct ExprArgs<N> {
    inputs: IndexSet<(N, IncomingPort)>,
    outputs: IndexSet<(N, OutgoingPort)>,
    function_calls: Vec<(N, IncomingPort)>,
    nodes: Vec<N>,
}

impl<N> Default for ExprArgs<N> {
    fn default() -> Self {
        Self {
            inputs: IndexSet::new(),
            outputs: IndexSet::new(),
            function_calls: Vec::new(),
            nodes: Vec::new(),
        }
    }
}

/// Group function calls by the function they call.
fn group_function_calls<N: HugrNode>(
    function_calls: Vec<(N, IncomingPort)>,
    circuit: &ResourceScope<impl HugrView<Node = N>>,
) -> Vec<Vec<(N, IncomingPort)>> {
    function_calls
        .into_iter()
        .into_group_map_by(|&(node, port)| {
            circuit
                .hugr()
                .single_linked_output(node, port)
                .expect("valid function call")
        })
        .into_values()
        .collect_vec()
}

/// Sort inputs according to the ordering defined in `allowed_boundary_inputs`
/// and `nodes_to_exclude`.
fn sort_inputs<N: HugrNode>(
    inputs: IndexSet<(N, IncomingPort)>,
    allowed_boundary_inputs: &IndexSet<(N, IncomingPort)>,
    nodes_to_exclude: &IndexSet<N>,
    hugr: &impl HugrView<Node = N>,
) -> Vec<Vec<(N, IncomingPort)>> {
    inputs
        .into_iter()
        .sorted_by_key(|&(node, port)| {
            if let Some(pos) = allowed_boundary_inputs.get_index_of(&(node, port)) {
                // Order by allowed input ports first (Left comes first)
                Either::Left(pos)
            } else {
                // Then order by nodes_to_exclude order (Right comes last)
                let (prev_node, _) = hugr
                    .single_linked_output(node, port)
                    .expect("valid dataflow wire");
                Either::Right(nodes_to_exclude.get_index_of(&prev_node))
            }
        })
        .map(|np| vec![np])
        .collect_vec()
}

#[cfg(test)]
mod tests {
    use std::iter;

    use crate::{Circuit, TketOp, extension::rotation::RotationOp};

    use super::*;
    use hugr::{
        Hugr, HugrView, IncomingPort, Node, OutgoingPort,
        builder::{Dataflow, DataflowHugr, FunctionBuilder, HugrBuilder},
        extension::prelude::{bool_t, qb_t},
        hugr::views::SiblingSubgraph,
        std_extensions::arithmetic::float_types::float64_type,
        types::Signature,
    };
    use rstest::{fixture, rstest};

    #[fixture]
    fn hugr_with_midcircuit_meas() -> Hugr {
        let qb_row = vec![qb_t()];
        let signature = Signature::new_endo(qb_row);
        let mut h = FunctionBuilder::new("main", signature).unwrap();
        let bool_to_float = h
            .module_root_builder()
            .declare(
                "bool_to_float",
                Signature::new(vec![bool_t()], vec![float64_type()]).into(),
            )
            .unwrap();

        let mut circ = h.as_circuit(h.input_wires());
        let [meas] = circ.append_with_outputs_arr(TketOp::Measure, [0]).unwrap();
        let qbs = circ.finish();
        let [float] = h.call(&bool_to_float, &[], [meas]).unwrap().outputs_arr();

        let mut circ = h.as_circuit(qbs);
        let [rot_angle] = circ
            .append_with_outputs_arr(RotationOp::from_halfturns_unchecked, [float])
            .unwrap();
        let [two_rot_angle] = circ
            .append_with_outputs_arr(RotationOp::radd, [rot_angle, rot_angle])
            .unwrap();
        circ.append_and_consume(
            TketOp::Rx,
            [
                hugr::CircuitUnit::Linear(0),
                hugr::CircuitUnit::Wire(two_rot_angle),
            ],
        )
        .unwrap();
        let circ_out = circ.finish();
        h.finish_hugr_with_outputs(circ_out).unwrap()
        // (hugr, (two_rot_angle.node(), two_rot_angle.source()))
    }

    #[rstest]
    #[case::identity_expression(
        // Single-qubit circuit where a measurement output is directly used as expr
        {
            let qb_row = vec![qb_t()];
            let signature = Signature::new_endo(qb_row);
            let mut h = FunctionBuilder::new("main", signature).unwrap();
            let mut circ = h.as_circuit(h.input_wires());
            let [meas] = circ.append_with_outputs_arr(TketOp::Measure, [0]).unwrap();
            let circ_out = circ.finish();
            let hugr = h.finish_hugr_with_outputs(circ_out).unwrap();
            (hugr, (meas.node(), meas.source()))
        },
        CopyableExpr::Wire (
            Wire::new(Node::from(portgraph::NodeIndex::new(4)), OutgoingPort::from(1)),
        )
    )]
    #[case::midcircuit_meas(
        // Circuit where a measurement output is converted to float then to
        // rotation, multiplied by two and passed to a Rx rotation
        {
            let hugr = hugr_with_midcircuit_meas();
            let rx = hugr
                .nodes()
                .find(|&n| hugr.get_optype(n) == &(TketOp::Rx.into()))
                .unwrap();
            let two_rot_angle_input = (rx, IncomingPort::from(1));
            let two_rot_angle_output = hugr
                .single_linked_output(two_rot_angle_input.0, two_rot_angle_input.1)
                .unwrap();

            (hugr, two_rot_angle_output)
        },
        CopyableExpr::Composite {
            subgraph: SiblingSubgraph::new_unchecked(
                vec![vec![(Node::from(portgraph::NodeIndex::new(6)), IncomingPort::from(0))]],
                vec![(Node::from(portgraph::NodeIndex::new(8)), OutgoingPort::from(0))],
                vec![vec![(Node::from(portgraph::NodeIndex::new(6)), IncomingPort::from(1))]],
                (6..=8).map(|i| Node::from(portgraph::NodeIndex::new(i))).collect(),
            ),
        }
    )]
    fn test_copyable_expression_ast_new(
        #[case] input: (Hugr, (Node, OutgoingPort)),
        #[case] expected: CopyableExpr,
    ) {
        let (hugr, output) = input;

        let circuit = ResourceScope::from_circuit(Circuit::new(hugr));

        let result = CopyableExpr::try_new(
            output,
            iter::empty().collect(),
            iter::empty().collect(),
            &circuit,
        )
        .unwrap();
        assert_eq!(result, expected);
    }

    #[rstest]
    fn test_copyable_expression_ast_new_bounded_inputs() {
        let hugr = hugr_with_midcircuit_meas();

        let rot_cast = hugr
            .nodes()
            .find(|&n| hugr.get_optype(n) == &(RotationOp::from_halfturns_unchecked.into()))
            .unwrap();
        let circuit = ResourceScope::from_circuit(Circuit::new(&hugr));
        let two_rot_angle_output = {
            let rx = hugr
                .nodes()
                .find(|&n| hugr.get_optype(n) == &(TketOp::Rx.into()))
                .unwrap();
            let two_rot_angle_input = (rx, IncomingPort::from(1));

            hugr.single_linked_output(two_rot_angle_input.0, two_rot_angle_input.1)
                .unwrap()
        };

        let expr = CopyableExpr::try_new(
            two_rot_angle_output,
            iter::empty().collect(),
            [rot_cast].into_iter().collect(),
            &circuit,
        )
        .unwrap();

        let subgraph = expr.as_subgraph().expect("non-identity expression");

        assert_eq!(subgraph.nodes(), [Node::from(portgraph::NodeIndex::new(8))]);
        let &[radd] = subgraph.nodes() else {
            panic!("expected radd node")
        };
        assert_eq!(
            subgraph.incoming_ports(),
            &vec![
                vec![(radd, IncomingPort::from(0))],
                vec![(radd, IncomingPort::from(1))]
            ]
        );
        assert_eq!(
            subgraph.outgoing_ports(),
            &vec![(radd, OutgoingPort::from(0))]
        );
    }

    #[test]
    fn test_copyable_expression_ast_new_non_convex() {
        let hugr = hugr_with_midcircuit_meas();

        let radd = hugr
            .nodes()
            .find(|&n| hugr.get_optype(n) == &(RotationOp::radd.into()))
            .unwrap();
        let circuit = ResourceScope::from_circuit(Circuit::new(&hugr));

        let _expr = CopyableExpr::try_new(
            (radd, OutgoingPort::from(0)),
            [(radd, IncomingPort::from(0))].into_iter().collect(),
            iter::empty().collect(),
            &circuit,
        )
        .expect_err("is not convex");
    }
}
