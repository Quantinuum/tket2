//! A pass for removing redundant order edges in a Hugr region.

use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};

use hugr::{IncomingPort, OutgoingPort};
use hugr_core::hugr::{HugrError, hugrmut::HugrMut};
use hugr_core::ops::{OpTag, OpTrait};
use hugr_core::{HugrView, Node};
use petgraph::visit::Walker;

use crate::passes::composable::WithScope;
use crate::passes::{ComposablePass, PassScope};

/// A pass for removing order edges in a Hugr region that are already implied by
/// other order dependencies.
///
/// Note we cannot remove order edges that are implied by value edges: the former
/// enforce an order on side effects, whereas the latter only on the values themselves,
/// and do not imply an order of (pure functional, invisible) "evaluation".
///
/// TODO: consider adding a whitelist of ops that have no side effects, for which
/// we can remove order edges entirely (i.e. reroute around the node).
///
/// Each evaluation on a region runs in `O(e + n log(n) * #order_edges)` time,
/// where `e` and `n` are the number of edges and nodes in the region,
/// respectively.
#[derive(Debug, Default, Clone)]
pub struct RedundantOrderEdgesPass {
    /// On what part of the Hugr to run
    scope: PassScope,
}

/// Result type for the redundant order edges pass.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, derive_more::AddAssign)]
pub struct RedundantOrderEdgesResult {
    /// Number of edges removed.
    pub edges_removed: usize,
}

impl RedundantOrderEdgesPass {
    /// Evaluate the pass on the given dataflow region.
    ///
    /// # Arguments
    ///
    /// * `hugr`: The hugr to evaluate the pass on.
    /// * `region`: The region to evaluate the pass on.
    /// * `region_candidates`: A queue of nodes to explore in the region. If
    ///   `self.recursive`, we will add to this list any children nodes of the
    ///   region.
    pub fn run_on_df_region<H: HugrMut>(
        &self,
        hugr: &mut H,
        parent: H::Node,
        region_candidates: &mut VecDeque<H::Node>,
    ) -> Result<RedundantOrderEdgesResult, HugrError> {
        type OutPorts<N> = HashSet<(N, OutgoingPort)>;
        // A map that for node n contains both
        // * a count of the unprocessed order-successors of n (allowing to clear the map as these are processed)
        // * a set of nodes that reach n via (chains of) order edges.
        let mut order_nodes_reaching: HashMap<H::Node, (usize, OutPorts<H::Node>)> = HashMap::new();
        // Order edges to be removed.
        let mut to_remove = BTreeMap::<(H::Node, H::Node), (OutgoingPort, IncomingPort)>::new();

        // Traverse the region in topological order. We actually only need an order
        // that respects the order edges, but this is a simple (albeit expensive) way
        // to get one.
        let sg = hugr.scheduling_graph(parent);
        let postorder = petgraph::visit::Topo::new(sg.petgraph());
        for pg_child in postorder.iter(sg.petgraph()) {
            let child = sg.pg_to_node(pg_child);

            let op = hugr.get_optype(child);

            // If the child itself is a region (parent) and we are running recursively, add the child to the region candidates.
            if self.scope.recursive() && hugr.first_child(child).is_some() {
                region_candidates.push_back(child);
            }

            let mut reaching_child = HashSet::new();
            if let Some(ord_in) = op.other_input_port() {
                let order_preds = hugr
                    .linked_outputs(child, ord_in)
                    .collect::<BTreeMap<_, _>>();
                for (order_pred, pred_in) in order_preds.iter() {
                    let (pred_count, reaching_pred) =
                        order_nodes_reaching.get_mut(order_pred).unwrap();
                    for (&other_pred, &other_pred_port) in order_preds.iter() {
                        if reaching_pred.contains(&(other_pred, other_pred_port)) {
                            // `other_pred` reaches predecessor `order_pred` of child, and has a direct edge to child.
                            to_remove.insert((other_pred, child), (other_pred_port, ord_in));
                        }
                    }
                    reaching_child.extend(reaching_pred.iter().copied());
                    reaching_child.insert((*order_pred, *pred_in));
                    (*pred_count) -= 1;
                    if *pred_count == 0 {
                        order_nodes_reaching.remove(order_pred);
                    }
                }
            }
            if let Some(ord_out) = op.other_output_port() {
                order_nodes_reaching.insert(
                    child,
                    (hugr.linked_inputs(child, ord_out).count(), reaching_child),
                );
            }
        }
        // Release the hugr borrow so we can mutate it.
        drop(sg);
        let edges_removed = to_remove.len();

        for ((src_n, tgt_n), (src_p, tgt_p)) in to_remove {
            hugr.disconnect_edge(src_n, src_p, tgt_n, tgt_p);
        }

        Ok(RedundantOrderEdgesResult { edges_removed })
    }
}

impl<H: HugrMut<Node = Node>> ComposablePass<H> for RedundantOrderEdgesPass {
    type Error = HugrError;
    type Result = RedundantOrderEdgesResult;

    fn run(&self, hugr: &mut H) -> Result<Self::Result, Self::Error> {
        // Nodes to explore in the hugr.
        let mut region_candidates = VecDeque::from_iter(self.scope.root(hugr));
        let mut result = RedundantOrderEdgesResult::default();

        while let Some(region) = region_candidates.pop_front() {
            let op = hugr.get_optype(region);

            if OpTag::DataflowParent.is_superset(op.tag()) {
                result += self.run_on_df_region(hugr, region, &mut region_candidates)?;
            } else if self.scope.recursive() {
                region_candidates.extend(hugr.children(region));
            }
        }

        Ok(result)
    }
}

impl WithScope for RedundantOrderEdgesPass {
    fn with_scope(mut self, scope: impl Into<PassScope>) -> Self {
        self.scope = scope.into();
        self
    }
}

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use hugr_core::builder::{Dataflow, DataflowHugr, FunctionBuilder, SubContainer};
    use hugr_core::extension::prelude::{Noop, bool_t};
    use hugr_core::ops::handle::NodeHandle;
    use hugr_core::types::Signature;

    use super::*;

    /// Construct a simple hugr with a bunch of noops
    ///
    /// ```
    /// input -> noop1 --> noop2 --> noop3 -> nested_op
    ///       |
    ///       v
    ///       noop4 --> noop5 --> output
    /// ```
    #[rstest]
    #[case([("input", "noop2"), ("noop1", "output"), ("noop4", "noop3"), ("noop5", "noop2"), ("noop3", "nested_op")],
           [("input", "noop2"), ("noop1", "output"), ("noop4", "noop3"), ("noop5", "noop2"), ("noop3", "nested_op")])]
    #[case([("input", "noop2"), ("noop2", "output"), ("input", "output")],
           [("input", "noop2"), ("noop2", "output")])]
    #[case([("input", "noop1"), ("noop1", "noop5"), ("input", "noop4"), ("noop4", "noop5"), ("noop5", "output")],
           [("input", "noop1"), ("noop1", "noop5"), ("input", "noop4"), ("noop4", "noop5"), ("noop5", "output")])]
    #[case([("input", "noop1"), ("noop1", "noop5"), ("input", "noop4"), ("noop4", "noop5"), ("noop5", "output"),
            ("input", "noop5"), ("noop1", "output"), ("noop4", "output")],
           [("input", "noop1"), ("noop1", "noop5"), ("input", "noop4"), ("noop4", "noop5"), ("noop5", "output")])]
    #[case([("noop1", "noop4"), ("noop4", "noop2"), ("noop2", "noop5"), ("noop4", "noop5"), ("noop1", "noop5")],
           [("noop1", "noop4"), ("noop4", "noop2"), ("noop2", "noop5"), ("noop1", "noop4")])]
    fn test_redundant_order_edges(
        #[case] start_edges: impl IntoIterator<Item = (&'static str, &'static str)>,
        #[case] end_edges: impl IntoIterator<Item = (&'static str, &'static str)>,
    ) {
        let mut hugr = FunctionBuilder::new("f", Signature::new_endo([bool_t()])).unwrap();
        let op = Noop::new(bool_t());

        let [input, output] = hugr.io();
        let mut named_nodes = HashMap::from([("input", input.node()), ("output", output.node())]);

        let [b1] = hugr.input_wires_arr();
        let noop1 = hugr.add_dataflow_op(Noop::new(bool_t()), [b1]).unwrap();
        named_nodes.insert("noop1", noop1.node());
        let noop2 = hugr
            .add_dataflow_op(op.clone(), [noop1.out_wire(0)])
            .unwrap();
        named_nodes.insert("noop2", noop2.node());
        let noop3 = hugr
            .add_dataflow_op(op.clone(), [noop2.out_wire(0)])
            .unwrap();
        named_nodes.insert("noop3", noop3.node());
        let noop4 = hugr.add_dataflow_op(op.clone(), [b1]).unwrap();
        named_nodes.insert("noop4", noop4.node());
        let noop5 = hugr
            .add_dataflow_op(op.clone(), [noop4.out_wire(0)])
            .unwrap();
        named_nodes.insert("noop5", noop5.node());
        let nested_op = hugr
            .dfg_builder(Signature::new(vec![bool_t()], vec![]), [noop5.out_wire(0)])
            .unwrap()
            .finish_sub_container()
            .unwrap();
        named_nodes.insert("nested_op", nested_op.node());

        // Set the order edges before optimization
        let start_edges = start_edges.into_iter().collect::<Vec<_>>();
        for (src, tgt) in &start_edges {
            let src_node = named_nodes.get(src).unwrap();
            let tgt_node = named_nodes.get(tgt).unwrap();
            hugr.set_order(src_node, tgt_node);
        }

        let mut hugr = hugr.finish_hugr_with_outputs([noop5.out_wire(0)]).unwrap();

        // Run the pass
        let result = RedundantOrderEdgesPass::default().run(&mut hugr).unwrap();
        let end_edges = end_edges
            .into_iter()
            .map(|(src, tgt)| {
                let src_node = named_nodes.get(src).unwrap();
                let tgt_node = named_nodes.get(tgt).unwrap();
                (*src_node, *tgt_node)
            })
            .collect::<HashSet<_>>();
        assert_eq!(result.edges_removed, start_edges.len() - end_edges.len());

        let remaining_edges = hugr
            .nodes()
            .filter_map(|src| {
                hugr.get_optype(src)
                    .other_output_port()
                    .map(|ord_out| (src, ord_out))
            })
            .flat_map(|(src, ord_out)| {
                hugr.linked_inputs(src, ord_out)
                    .map(move |(tgt, _ord_in)| (src, tgt))
            })
            .collect::<HashSet<_>>();

        assert_eq!(remaining_edges, end_edges);
    }
}
