//! Pass for removing redundant tuple pack/unpack operations in either order.

use std::collections::VecDeque;

use fxhash::FxHashSet;
use hugr::hugr::views::sibling_subgraph::SchedGraphChecker;
use hugr_core::builder::{Dataflow, DataflowHugr, FunctionBuilder};
use hugr_core::extension::prelude::{MakeTuple, UnpackTuple};
use hugr_core::hugr::SimpleReplacementError;
use hugr_core::hugr::hugrmut::HugrMut;
use hugr_core::hugr::views::SiblingSubgraph;
use hugr_core::ops::OpType;
use hugr_core::types::Type;
use hugr_core::{HugrView, Node, PortIndex, SimpleReplacement};
use itertools::Itertools;

use crate::passes::composable::WithScope;
use crate::passes::{ComposablePass, PassScope};

/// A pass that removes unnecessary `MakeTuple` operations immediately followed
/// by `UnpackTuple`s.
///
/// If the tuple output is consumed by other operations, only the `UnpackTuple`s
/// are removed and their outputs are connected to the original values
/// accordingly.
///
/// Currently only unpack operations in the same region as the `MakeTuple` are
/// removed. This may be extended in the future.
///
/// Removes `MakeTuple` operations that are not consumed by any other
/// operations.
///
/// Also removes `UnpackTuple` operations followed by `MakeTuple`s that repack
/// all their fields in the original order. If the unpacked fields have other
/// consumers and the tuple is copyable, the `UnpackTuple` is retained and only
/// the repacks are removed. Linear tuples with other field consumers are left
/// unchanged.
/// These operations must also be in the same region.
///
/// Repeats simplification until no further rewrites are possible, including
/// pairs exposed by simplifying nested tuples.
///
/// Ignores pack/unpack nodes with order edges.
// TODO: Supporting those requires updating the `SiblingSubgraph` implementation. See <https://github.com/CQCL/hugr/issues/1974>.
#[derive(Debug, Clone, Default)]
pub struct UntuplePass {
    scope: PassScope,
}

#[derive(Debug, derive_more::Display, derive_more::Error, derive_more::From)]
#[non_exhaustive]
/// Errors produced by [`UntuplePass`].
pub enum UntupleError {
    /// Rewriting the circuit failed.
    RewriteError(SimpleReplacementError),
}

/// Result type for the untuple pass.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct UntupleResult {
    /// Number of tuple pack/unpack rewrites applied.
    pub rewrites_applied: usize,
}

impl UntuplePass {
    /// Find redundant tuple pack/unpack operations in either order and generate
    /// rewrites to remove them.
    ///
    /// The returned rewrites are guaranteed to be independent of each other;
    /// overlapping matches are left for subsequent iterations of the pass.
    pub fn all_rewrites<H: HugrView<Node = Node>>(
        &self,
        hugr: &H,
    ) -> Vec<SimpleReplacement<H::Node>> {
        let Some(parent) = self.scope.root(hugr) else {
            return vec![];
        };
        find_rewrites(hugr, parent, self.scope.recursive())
    }
}

fn find_rewrites<H: HugrView>(
    hugr: &H,
    parent: H::Node,
    recursive: bool,
) -> Vec<SimpleReplacement<H::Node>> {
    let mut res = Vec::new();
    let mut children_queue = VecDeque::new();
    children_queue.push_back(parent);

    while let Some(parent) = children_queue.pop_front() {
        // Required to create SimpleReplacements.
        // Reset for each parent as `SchedGraphChecker` and all `HugrConvexChecker`s
        // are tied to a specific parent node.
        let mut convex_checker: Option<SchedGraphChecker<H>> = None;
        let mut rewritten_nodes: FxHashSet<H::Node> = FxHashSet::default();

        for node in hugr.children(parent) {
            let op = hugr.get_optype(node);
            let rewrite = if is_make_tuple(op) {
                make_pack_unpack_rewrite(hugr, &mut convex_checker, node)
            } else if is_unpack_tuple(op) {
                make_unpack_pack_rewrite(hugr, &mut convex_checker, node)
            } else {
                None
            };
            if let Some(rw) = rewrite.filter(|rw| {
                rw.subgraph()
                    .nodes()
                    .iter()
                    .all(|n| !rewritten_nodes.contains(n))
            }) {
                rewritten_nodes.extend(rw.subgraph().nodes().iter().copied());
                res.push(rw);
            }
            if recursive && op.is_container() {
                children_queue.push_back(node);
            }
        }
    }
    res
}

impl<H: HugrMut<Node = Node>> ComposablePass<H> for UntuplePass {
    type Error = UntupleError;
    type Result = UntupleResult;

    fn run(&self, hugr: &mut H) -> Result<Self::Result, Self::Error> {
        let mut rewrites_applied = 0;
        loop {
            let rewrites = self.all_rewrites(hugr);
            if rewrites.is_empty() {
                break;
            }
            rewrites_applied += rewrites.len();
            // The rewrites are independent, so we can always apply them all.
            for rewrite in rewrites {
                hugr.apply_patch(rewrite)?;
            }
            // Removing a pair can expose another pair, e.g. when tuples are nested.
            // Each rewrite removes at least one node, so this terminates.
        }
        Ok(UntupleResult { rewrites_applied })
    }
}

impl WithScope for UntuplePass {
    fn with_scope(mut self, scope: impl Into<PassScope>) -> Self {
        self.scope = scope.into();
        self
    }
}

/// Returns true if the given optype is a `MakeTuple` operation.
///
/// Boilerplate required due to <https://github.com/CQCL/hugr/issues/1496>
fn is_make_tuple(optype: &OpType) -> bool {
    optype.cast::<MakeTuple>().is_some()
}

/// Returns true if the given optype is an `UnpackTuple` operation.
///
/// Boilerplate required due to <https://github.com/CQCL/hugr/issues/1496>
fn is_unpack_tuple(optype: &OpType) -> bool {
    optype.cast::<UnpackTuple>().is_some()
}

/// Returns whether the node has any incident order edges.
fn has_order_edges<H: HugrView>(hugr: &H, node: H::Node) -> bool {
    let op = hugr.get_optype(node);
    let has_input_order = op
        .other_input_port()
        .and_then(|p| hugr.linked_outputs(node, p).next())
        .is_some();
    let has_output_order = op
        .other_output_port()
        .and_then(|p| hugr.linked_inputs(node, p).next())
        .is_some();
    has_input_order || has_output_order
}

/// If this is a `MakeTuple` operation followed by some number of `UnpackTuple` operations
/// on the same region, return a rewrite to remove them.
///
/// Otherwise, return None.
fn make_pack_unpack_rewrite<'h, T: HugrView>(
    hugr: &'h T,
    convex_checker: &mut Option<SchedGraphChecker<'h, T>>,
    node: T::Node,
) -> Option<SimpleReplacement<T::Node>> {
    // If the node has order edges, ignore it.
    if has_order_edges(hugr, node) {
        return None;
    }

    let tuple_types = hugr.in_value_types(node).map(|(_, ty)| ty).collect_vec();
    let node_parent = hugr.get_parent(node);

    // See if it is followed by a tuple unpack
    let links = hugr
        .linked_inputs(node, 0)
        .map(|(neigh, _)| neigh)
        .collect_vec();

    let unpack_nodes = links
        .iter()
        .filter(|&&neigh| hugr.get_parent(neigh) == node_parent)
        .filter(|&&neigh| is_unpack_tuple(hugr.get_optype(neigh)))
        .filter(|&&neigh| !has_order_edges(hugr, neigh))
        .copied()
        .collect_vec();

    // If there are no unpacks but the tuple is being used, there's nothing to do.
    if unpack_nodes.is_empty() && !links.is_empty() {
        return None;
    }

    // Remove all unpack operations, and remove the pack operation if all neighbours are unpacks.
    let num_other_outputs = links.len() - unpack_nodes.len();
    Some(remove_pack_unpack(
        hugr,
        convex_checker,
        tuple_types,
        node,
        unpack_nodes,
        num_other_outputs,
    ))
}

/// Remove repacks of all the fields of an `UnpackTuple`, in their original order.
fn make_unpack_pack_rewrite<'h, T: HugrView>(
    hugr: &'h T,
    convex_checker: &mut Option<SchedGraphChecker<'h, T>>,
    unpack_node: T::Node,
) -> Option<SimpleReplacement<T::Node>> {
    if has_order_edges(hugr, unpack_node) {
        return None;
    }

    let tuple_types = hugr
        .out_value_types(unpack_node)
        .map(|(_, ty)| ty)
        .collect_vec();
    let parent = hugr.get_parent(unpack_node)?;
    // Any repack must consume the first field. Empty tuples have no fields
    // connecting the unpack to a pack, so cannot be matched this way.
    if tuple_types.is_empty() {
        return None;
    }
    let pack_nodes = hugr
        .linked_inputs(unpack_node, 0)
        .filter_map(|(node, port)| {
            (port.index() == 0
                && hugr.get_parent(node) == Some(parent)
                && is_make_tuple(hugr.get_optype(node))
                && !has_order_edges(hugr, node)
                && hugr.in_value_types(node).count() == tuple_types.len()
                && (0..tuple_types.len())
                    .all(|i| hugr.single_linked_output(node, i) == Some((unpack_node, i.into()))))
            .then_some(node)
        })
        .collect_vec();
    if pack_nodes.is_empty() {
        return None;
    }

    let checker =
        convex_checker.get_or_insert_with(|| SchedGraphChecker::new(hugr.scheduling_graph(parent)));
    let nodes = std::iter::once(unpack_node).chain(pack_nodes).collect_vec();
    let subgraph = SiblingSubgraph::try_from_nodes_with_checker(nodes, hugr, checker).unwrap();
    let retain_unpack = subgraph
        .outgoing_ports()
        .iter()
        .any(|(node, _)| *node == unpack_node);
    // Passing the original tuple through while also unpacking it for other
    // field consumers would copy it, which is invalid for a linear tuple.
    if retain_unpack && !tuple_types.iter().all(Type::copyable) {
        return None;
    }
    let mut replacement = FunctionBuilder::new("<repl>", subgraph.poly_func_type(hugr)).unwrap();
    let [tuple] = replacement.input_wires_arr();

    // Keep an unpack only if some fields are used outside the removed repacks.
    let fields = if retain_unpack {
        replacement
            .add_dataflow_op(UnpackTuple::new(tuple_types.into()), [tuple])
            .unwrap()
            .outputs()
            .collect_vec()
    } else {
        vec![]
    };
    let outputs = subgraph.outgoing_ports().iter().map(|&(node, port)| {
        if node == unpack_node {
            fields[port.index()]
        } else {
            tuple
        }
    });
    let replacement = replacement.finish_hugr_with_outputs(outputs).unwrap();
    Some(
        subgraph
            .create_simple_replacement(hugr, replacement)
            .unwrap(),
    )
}

/// Returns a rewrite to remove a tuple pack operation that's followed by unpack operations,
/// and `other_tuple_links` other operations.
fn remove_pack_unpack<'h, T: HugrView>(
    hugr: &'h T,
    convex_checker: &mut Option<SchedGraphChecker<'h, T>>,
    tuple_types: Vec<Type>,
    pack_node: T::Node,
    unpack_nodes: Vec<T::Node>,
    num_other_outputs: usize,
) -> SimpleReplacement<T::Node> {
    let parent = hugr.get_parent(pack_node).expect("pack_node has no parent");
    let checker =
        convex_checker.get_or_insert_with(|| SchedGraphChecker::new(hugr.scheduling_graph(parent)));

    let mut nodes = unpack_nodes.clone();
    nodes.push(pack_node);
    let subgraph = SiblingSubgraph::try_from_nodes_with_checker(nodes, hugr, checker).unwrap();
    let subgraph_type = subgraph.poly_func_type(hugr);

    let mut replacement = FunctionBuilder::new("<repl>", subgraph_type).unwrap();

    // Wire the inputs directly to the unpack outputs
    // We need to list the **connected** output ports from the unpack nodes.
    // SiblingSubgraph ignores disconnected outputs, so we need these when building the replacement.
    let mut replacement_outputs =
        Vec::with_capacity(unpack_nodes.len() * tuple_types.len() + num_other_outputs);
    let replacement_inputs = replacement.input_wires().collect_vec();
    for unpack_node in unpack_nodes {
        for out_port in hugr.node_outputs(unpack_node) {
            if hugr.is_linked(unpack_node, out_port) {
                let input = replacement_inputs[out_port.index()];
                replacement_outputs.push(input);
            }
        }
    }

    // If needed, re-add the tuple pack node and connect its output to the tuple outputs.
    if num_other_outputs > 0 {
        let op = MakeTuple::new(tuple_types.into());
        let [tuple] = replacement
            .add_dataflow_op(op, replacement.input_wires())
            .unwrap()
            .outputs_arr();
        replacement_outputs.extend(std::iter::repeat_n(tuple, num_other_outputs));
    }

    // These should never fail, as we are defining the replacement ourselves.
    let replacement = replacement
        .finish_hugr_with_outputs(replacement_outputs)
        .unwrap_or_else(|e| {
            panic!("Failed to create replacement for removing tuple pack/unpack operations. {e}")
        });
    subgraph
        .create_simple_replacement(hugr, replacement)
        .unwrap_or_else(|e| {
            panic!("Failed to create rewrite for removing tuple pack/unpack operations. {e}")
        })
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::passes::composable::WithScope;
    use hugr_core::Hugr;
    use hugr_core::builder::{DFGBuilder, DataflowSubContainer, FunctionBuilder};
    use hugr_core::extension::prelude::{UnpackTuple, bool_t, qb_t};
    use hugr_core::ops::handle::NodeHandle;
    use hugr_core::std_extensions::arithmetic::float_types::float64_type;
    use hugr_core::std_extensions::collections::array::array_type_parametric;
    use hugr_core::types::type_param::TypeParam;
    use hugr_core::types::{PolyFuncType, Signature, TypeArg};
    use rstest::{fixture, rstest};

    /// A simple pack operation with unused output.
    ///
    /// These can be removed entirely.
    #[fixture]
    fn unused_pack() -> Hugr {
        let mut h = DFGBuilder::new(Signature::new(vec![bool_t(), bool_t()], vec![])).unwrap();
        let mut inps = h.input_wires();
        let b1 = inps.next().unwrap();
        let b2 = inps.next().unwrap();

        let _tuple = h.make_tuple([b1, b2]).unwrap();

        h.finish_hugr_with_outputs([]).unwrap()
    }

    /// A simple pack operation followed by an unpack operation.
    ///
    /// These can be removed entirely.
    #[fixture]
    fn simple_pack_unpack() -> Hugr {
        let mut h = DFGBuilder::new(Signature::new_endo([qb_t(), bool_t()])).unwrap();
        let mut inps = h.input_wires();
        let qb1 = inps.next().unwrap();
        let b2 = inps.next().unwrap();

        let tuple = h.make_tuple([qb1, b2]).unwrap();

        let op = UnpackTuple::new(vec![qb_t(), bool_t()].into());
        let [qb1, b2] = h.add_dataflow_op(op, [tuple]).unwrap().outputs_arr();

        h.finish_hugr_with_outputs([qb1, b2]).unwrap()
    }

    /// A simple pack/unpack pair with order edges between them.
    ///
    /// In the future we should be able to preserve some order edges, but for now
    /// we just remove everything.
    #[fixture]
    fn ordered_pack_unpack() -> Hugr {
        let mut h = DFGBuilder::new(Signature::new_endo(vec![qb_t(), bool_t()])).unwrap();
        let mut inps = h.input_wires();
        let qb1 = inps.next().unwrap();
        let b2 = inps.next().unwrap();

        let tuple = h.make_tuple([qb1, b2]).unwrap();
        h.set_order(&h.input(), &tuple.node());

        let op = UnpackTuple::new(vec![qb_t(), bool_t()].into());
        let untuple = h.add_dataflow_op(op, [tuple]).unwrap();
        let [qb1, b2] = untuple.outputs_arr();
        h.set_order(&tuple.node(), &untuple.node());

        h.set_order(&untuple.node(), &h.output());
        h.finish_hugr_with_outputs([qb1, b2]).unwrap()
    }

    /// A simple pack/unpack pair with an order from the pack node to a downstream node.
    ///
    /// The order edge should be preserved, so we move it to the predecessor of the pack node.
    #[fixture]
    fn outgoing_ordered_pack_unpack() -> Hugr {
        let mut h = DFGBuilder::new(Signature::new_endo(vec![qb_t(), bool_t()])).unwrap();
        let mut inps = h.input_wires();
        let qb1 = inps.next().unwrap();
        let b2 = inps.next().unwrap();

        let tuple = h.make_tuple([qb1, b2]).unwrap();

        let op = UnpackTuple::new(vec![qb_t(), bool_t()].into());
        let untuple = h.add_dataflow_op(op, [tuple]).unwrap();
        let [qb1, b2] = untuple.outputs_arr();

        h.set_order(&tuple.node(), &h.output());
        h.finish_hugr_with_outputs([qb1, b2]).unwrap()
    }

    /// A simple pack/unpack pair with an order from a downstream node to the pack node.
    ///
    /// The order edge should be preserved, so we move it to the successor of the unpack node.
    #[fixture]
    fn incoming_ordered_pack_unpack() -> Hugr {
        let mut h = DFGBuilder::new(Signature::new_endo(vec![qb_t(), bool_t()])).unwrap();
        let mut inps = h.input_wires();
        let qb1 = inps.next().unwrap();
        let b2 = inps.next().unwrap();

        let tuple = h.make_tuple([qb1, b2]).unwrap();

        let op = UnpackTuple::new(vec![qb_t(), bool_t()].into());
        let untuple = h.add_dataflow_op(op, [tuple]).unwrap();
        let [qb1, b2] = untuple.outputs_arr();

        h.set_order(&h.input(), &untuple.node());
        h.finish_hugr_with_outputs([qb1, b2]).unwrap()
    }

    /// A pack operation followed by three unpack operations from the same tuple.
    ///
    /// These can be removed entirely.
    #[fixture]
    fn multi_unpack() -> Hugr {
        let mut h = DFGBuilder::new(Signature::new(
            vec![bool_t(), bool_t()],
            vec![bool_t(), bool_t(), bool_t(), bool_t()],
        ))
        .unwrap();
        let mut inps = h.input_wires();
        let b1 = inps.next().unwrap();
        let b2 = inps.next().unwrap();

        let tuple = h.make_tuple([b1, b2]).unwrap();

        let op = UnpackTuple::new(vec![bool_t(), bool_t()].into());
        let [b1, b2] = h.add_dataflow_op(op, [tuple]).unwrap().outputs_arr();

        let op = UnpackTuple::new(vec![bool_t(), bool_t()].into());
        let [b3, b4] = h.add_dataflow_op(op, [tuple]).unwrap().outputs_arr();

        // The last one's outputs are disconnected.
        // TODO: Adding this causes the test to fail due to a `NonCovex` error.
        //let op = UnpackTuple::new(vec![bool_t(), bool_t()].into());
        //let _ = h.add_dataflow_op(op, [tuple]).unwrap();

        h.finish_hugr_with_outputs([b1, b2, b3, b4]).unwrap()
    }

    /// A pack operation followed by an unpack operation, where the tuple is also returned.
    ///
    /// The unpack operation can be removed, but the pack operation cannot.
    #[fixture]
    fn partial_unpack() -> Hugr {
        let mut h = DFGBuilder::new(Signature::new(
            vec![bool_t(), bool_t()],
            vec![
                bool_t(),
                bool_t(),
                Type::new_tuple(vec![bool_t(), bool_t()]),
            ],
        ))
        .unwrap();
        let mut inps = h.input_wires();
        let b1 = inps.next().unwrap();
        let b2 = inps.next().unwrap();

        let tuple = h.make_tuple([b1, b2]).unwrap();

        let op = UnpackTuple::new(vec![bool_t(), bool_t()].into());
        let [b1, b2] = h.add_dataflow_op(op, [tuple]).unwrap().outputs_arr();

        h.finish_hugr_with_outputs([b1, b2, tuple]).unwrap()
    }

    /// A pack operation followed by an unpack that discards its first output.
    ///
    /// The unpack operation can be removed, but the pack operation cannot.
    ///
    /// This is a minimal error case for <https://github.com/Quantinuum/tket2/issues/1347>.
    #[fixture]
    fn unpack_discard_first() -> Hugr {
        let mut h = FunctionBuilder::new(
            "test",
            Signature::new(vec![bool_t(), float64_type()], vec![float64_type()]),
        )
        .unwrap();
        let [b, f] = h.input_wires_arr();

        let tuple = h.make_tuple([b, f]).unwrap();

        let op = UnpackTuple::new(vec![bool_t(), float64_type()].into());
        let [_b, f] = h.add_dataflow_op(op, [tuple]).unwrap().outputs_arr();

        h.finish_hugr_with_outputs([f]).unwrap()
    }

    /// A pack operation followed by an unpack, where at least one tuple member contains a type
    /// argument that is declared in the contained function.
    ///
    /// Both operations can be removed.
    ///
    /// Regression smoke test for <https://github.com/Quantinuum/tket2/issues/1486>.
    #[fixture]
    fn unpack_type_parameters() -> Hugr {
        let arr_type =
            array_type_parametric(TypeArg::new_var_use(0, TypeParam::max_nat_kind()), bool_t())
                .unwrap();
        let mut h = FunctionBuilder::new(
            "inner",
            PolyFuncType::new(
                vec![TypeParam::max_nat_kind()],
                Signature::new_endo(vec![arr_type.clone(), bool_t()]),
            ),
        )
        .unwrap();
        let [arr, b] = h.input_wires_arr();
        let tuple = h.make_tuple([arr, b]).unwrap();
        let op = UnpackTuple::new(vec![arr_type, bool_t()].into());
        let [arr, b] = h.add_dataflow_op(op, [tuple]).unwrap().outputs_arr();
        h.finish_hugr_with_outputs([arr, b]).unwrap()
    }

    fn unpack_repack(types: Vec<Type>, packs: usize, extra_field: Option<usize>) -> Hugr {
        let tuple_type = Type::new_tuple(types.clone());
        let mut outputs = vec![tuple_type.clone(); packs];
        outputs.extend(extra_field.map(|i| types[i].clone()));
        let mut h = DFGBuilder::new(Signature::new(vec![tuple_type], outputs)).unwrap();
        let [tuple] = h.input_wires_arr();
        let fields = h
            .add_dataflow_op(UnpackTuple::new(types.into()), [tuple])
            .unwrap()
            .outputs()
            .collect_vec();
        let mut outputs = (0..packs)
            .map(|_| h.make_tuple(fields.iter().copied()).unwrap())
            .collect_vec();
        outputs.extend(extra_field.map(|i| fields[i]));
        h.finish_hugr_with_outputs(outputs).unwrap()
    }

    #[rstest]
    #[case::linear(vec![qb_t(), bool_t()], 1, None)]
    #[case::singleton(vec![qb_t()], 1, None)]
    #[case::multiple_repacks(vec![bool_t(), bool_t()], 3, None)]
    #[case::extra_field_use(vec![bool_t(), bool_t()], 2, Some(1))]
    fn test_unpack_repack(
        #[case] types: Vec<Type>,
        #[case] packs: usize,
        #[case] extra_field: Option<usize>,
    ) {
        let mut hugr = unpack_repack(types, packs, extra_field);
        hugr.validate().unwrap();
        let pass = UntuplePass::default();
        assert_eq!(pass.run(&mut hugr).unwrap().rewrites_applied, 1);
        hugr.validate().unwrap();

        let [input, output] = hugr.get_io(hugr.entrypoint()).unwrap();
        for i in 0..packs {
            assert_eq!(
                hugr.single_linked_output(output, i),
                Some((input, 0.into()))
            );
        }
        if let Some(field) = extra_field {
            let (unpack, port) = hugr.single_linked_output(output, packs).unwrap();
            assert!(is_unpack_tuple(hugr.get_optype(unpack)));
            assert_eq!(port.index(), field);
            assert_eq!(
                hugr.single_linked_output(unpack, 0),
                Some((input, 0.into()))
            );
        }
        assert_eq!(
            hugr.children(hugr.entrypoint()).count(),
            2 + usize::from(extra_field.is_some())
        );
        assert_eq!(pass.run(&mut hugr).unwrap().rewrites_applied, 0);
    }

    #[test]
    fn test_unpack_repack_with_linear_tuple_and_extra_field_use() {
        let mut hugr = unpack_repack(vec![qb_t(), bool_t()], 1, Some(1));
        hugr.validate().unwrap();
        let original = hugr.clone();
        // The extra boolean use requires retaining the unpack. Bypassing the
        // repack would then duplicate the input tuple, including its qubit.
        assert_eq!(
            UntuplePass::default()
                .run(&mut hugr)
                .unwrap()
                .rewrites_applied,
            0
        );
        assert_eq!(hugr, original);
    }

    #[rstest]
    #[case::reordered(vec![(0, 1), (0, 0)])]
    #[case::duplicated(vec![(0, 0), (0, 0)])]
    #[case::mixed_tuples(vec![(0, 0), (1, 1)])]
    #[case::partial(vec![(0, 0)])]
    #[case::extra_field(vec![(0, 0), (0, 1), (0, 1)])]
    fn test_non_identity_repack(#[case] sources: Vec<(usize, usize)>) {
        let tuple_type = Type::new_tuple(vec![bool_t(), bool_t()]);
        let mut h = DFGBuilder::new(Signature::new(
            vec![tuple_type.clone(), tuple_type],
            vec![Type::new_tuple(vec![bool_t(); sources.len()])],
        ))
        .unwrap();
        let fields = h
            .input_wires()
            .collect_vec()
            .into_iter()
            .map(|tuple| {
                h.add_dataflow_op(UnpackTuple::new(vec![bool_t(), bool_t()].into()), [tuple])
                    .unwrap()
                    .outputs()
                    .collect_vec()
            })
            .collect_vec();
        let packed = h
            .make_tuple(
                sources
                    .into_iter()
                    .map(|(tuple, field)| fields[tuple][field]),
            )
            .unwrap();
        let mut hugr = h.finish_hugr_with_outputs([packed]).unwrap();
        hugr.validate().unwrap();
        let original = hugr.clone();
        assert_eq!(
            UntuplePass::default()
                .run(&mut hugr)
                .unwrap()
                .rewrites_applied,
            0
        );
        assert_eq!(hugr, original);
    }

    #[rstest]
    #[case::unpack_incoming(false, false)]
    #[case::unpack_outgoing(false, true)]
    #[case::pack_incoming(true, false)]
    #[case::pack_outgoing(true, true)]
    fn test_ordered_unpack_repack(#[case] pack: bool, #[case] outgoing: bool) {
        let mut hugr = unpack_repack(vec![qb_t(), bool_t()], 1, None);
        let [input, output] = hugr.get_io(hugr.entrypoint()).unwrap();
        let node = hugr
            .children(hugr.entrypoint())
            .find(|&n| {
                if pack {
                    is_make_tuple(hugr.get_optype(n))
                } else {
                    is_unpack_tuple(hugr.get_optype(n))
                }
            })
            .unwrap();
        if outgoing {
            hugr.add_other_edge(node, output);
        } else {
            hugr.add_other_edge(input, node);
        }
        hugr.validate().unwrap();
        let original = hugr.clone();
        assert_eq!(
            UntuplePass::default()
                .run(&mut hugr)
                .unwrap()
                .rewrites_applied,
            0
        );
        assert_eq!(hugr, original);
    }

    #[test]
    fn test_nested_unpack_repack() {
        let inner_types = vec![qb_t(), bool_t()];
        let outer_types = vec![Type::new_tuple(inner_types.clone()), bool_t()];
        let mut h = DFGBuilder::new(Signature::new_endo(vec![Type::new_tuple(
            outer_types.clone(),
        )]))
        .unwrap();
        let [tuple] = h.input_wires_arr();
        let [inner, b] = h
            .add_dataflow_op(UnpackTuple::new(outer_types.into()), [tuple])
            .unwrap()
            .outputs_arr();
        let [q, b_inner] = h
            .add_dataflow_op(UnpackTuple::new(inner_types.into()), [inner])
            .unwrap()
            .outputs_arr();
        let inner = h.make_tuple([q, b_inner]).unwrap();
        let tuple = h.make_tuple([inner, b]).unwrap();
        let mut hugr = h.finish_hugr_with_outputs([tuple]).unwrap();
        assert_eq!(
            UntuplePass::default()
                .run(&mut hugr)
                .unwrap()
                .rewrites_applied,
            2
        );
        hugr.validate().unwrap();
        assert_eq!(hugr.children(hugr.entrypoint()).count(), 2);
    }

    #[test]
    fn test_overlapping_pack_unpack_repack() {
        let mut h = DFGBuilder::new(Signature::new(
            vec![qb_t(), bool_t()],
            vec![Type::new_tuple(vec![qb_t(), bool_t()])],
        ))
        .unwrap();
        let tuple = h.make_tuple(h.input_wires()).unwrap();
        let fields = h
            .add_dataflow_op(UnpackTuple::new(vec![qb_t(), bool_t()].into()), [tuple])
            .unwrap()
            .outputs()
            .collect_vec();
        let tuple = h.make_tuple(fields).unwrap();
        let mut hugr = h.finish_hugr_with_outputs([tuple]).unwrap();
        let pass = UntuplePass::default();
        let rewrites = pass.all_rewrites(&hugr);
        // The two possible rewrites overlap, so only one may be returned.
        assert_eq!(rewrites.len(), 1);
        for rewrite in rewrites {
            hugr.apply_patch(rewrite).unwrap();
        }
        hugr.validate().unwrap();
        assert_eq!(hugr.children(hugr.entrypoint()).count(), 3);
        assert_eq!(pass.run(&mut hugr).unwrap().rewrites_applied, 0);
    }

    #[rstest]
    #[case::flat(PassScope::EntrypointFlat, 0)]
    #[case::recursive(PassScope::EntrypointRecursive, 1)]
    #[case::global(PassScope::default(), 1)]
    fn test_unpack_repack_scope(#[case] scope: PassScope, #[case] expected: usize) {
        let types = vec![qb_t(), bool_t()];
        let signature = Signature::new_endo(vec![Type::new_tuple(types.clone())]);
        let mut h = FunctionBuilder::new("main", signature.clone()).unwrap();
        let mut inner = h.dfg_builder(signature, h.input_wires()).unwrap();
        let [tuple] = inner.input_wires_arr();
        let fields = inner
            .add_dataflow_op(UnpackTuple::new(types.into()), [tuple])
            .unwrap()
            .outputs()
            .collect_vec();
        let tuple = inner.make_tuple(fields).unwrap();
        let [tuple] = inner.finish_with_outputs([tuple]).unwrap().outputs_arr();
        let mut hugr = h.finish_hugr_with_outputs([tuple]).unwrap();
        assert_eq!(
            UntuplePass::default_with_scope(scope)
                .run(&mut hugr)
                .unwrap()
                .rewrites_applied,
            expected
        );
        hugr.validate().unwrap();
    }

    #[test]
    fn test_unpack_repack_type_parameters() {
        let arr_type =
            array_type_parametric(TypeArg::new_var_use(0, TypeParam::max_nat_kind()), bool_t())
                .unwrap();
        let fields = vec![arr_type, bool_t()];
        let mut h = FunctionBuilder::new(
            "main",
            PolyFuncType::new(
                vec![TypeParam::max_nat_kind()],
                Signature::new_endo(vec![Type::new_tuple(fields.clone())]),
            ),
        )
        .unwrap();
        let [tuple] = h.input_wires_arr();
        let fields = h
            .add_dataflow_op(UnpackTuple::new(fields.into()), [tuple])
            .unwrap()
            .outputs()
            .collect_vec();
        let tuple = h.make_tuple(fields).unwrap();
        let mut hugr = h.finish_hugr_with_outputs([tuple]).unwrap();
        assert_eq!(
            UntuplePass::default()
                .run(&mut hugr)
                .unwrap()
                .rewrites_applied,
            1
        );
        hugr.validate().unwrap();
        assert_eq!(hugr.children(hugr.entrypoint()).count(), 2);
    }

    #[rstest]
    #[case::unused(unused_pack(), 1, 2)]
    #[cfg_attr(miri, ignore = "bitvec 1.1.1 violates Miri Stacked Borrows checks.")]
    #[case::simple(simple_pack_unpack(), 1, 2)]
    #[cfg_attr(miri, ignore = "bitvec 1.1.1 violates Miri Stacked Borrows checks.")]
    #[case::multi(multi_unpack(), 1, 2)]
    #[cfg_attr(miri, ignore = "bitvec 1.1.1 violates Miri Stacked Borrows checks.")]
    #[case::partial(partial_unpack(), 1, 3)]
    #[cfg_attr(miri, ignore = "bitvec 1.1.1 violates Miri Stacked Borrows checks.")]
    #[case::unpack_discard_first(unpack_discard_first(), 1, 2)]
    // Nodes with order edges are ignored.
    #[case::ordered(ordered_pack_unpack(), 0, 4)]
    #[case::outgoing_ordered(outgoing_ordered_pack_unpack(), 0, 4)]
    #[case::incoming_ordered(incoming_ordered_pack_unpack(), 0, 4)]
    #[cfg_attr(miri, ignore = "bitvec 1.1.1 violates Miri Stacked Borrows checks.")]
    #[case::type_parameters(unpack_type_parameters(), 1, 2)]
    fn test_pack_unpack(
        #[case] mut hugr: Hugr,
        #[case] expected_rewrites: usize,
        #[case] remaining_nodes: usize,
    ) {
        let parent = hugr.entrypoint();
        let pass = UntuplePass::default().with_scope(PassScope::EntrypointFlat);
        let res = pass.run(&mut hugr).unwrap_or_else(|e| panic!("{e}"));
        hugr.validate().unwrap();
        assert_eq!(res.rewrites_applied, expected_rewrites);
        assert_eq!(hugr.children(parent).count(), remaining_nodes);
    }
}
