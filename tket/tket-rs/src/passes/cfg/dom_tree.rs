//! Analysis of Control-Flow Graphs using dominator-strong components decomposition
use std::collections::{BTreeSet, HashMap, HashSet};
use std::fmt::Debug;

use hugr::core::HugrNode;
use hugr::{HugrView, OutgoingPort};
use hugr_core::hugr::internal::PortgraphNodeMap;
use petgraph::algo::dominators::{self, Dominators};
use portgraph::NodeIndex;

/// Trait for a description of a non-empty set of control-flow paths around a tree of [DomTreeNode]s.
///
/// Each instance describes a set of paths all of which *either*
/// * lead to the same point (node), starting from the same dominator thereof, *or*
/// * leave a particular subtree, starting from its root.
pub trait PathSet<N>: Debug + Sized {
    /// Make a singleton set: one path being a single control-flow edge.
    fn new_edge(node: N, outport: OutgoingPort) -> Self;
    /// Make a new set of paths that start with the edge from `outport`, among `num_outports`
    /// leaving `node`, followed by the set of paths `ch` (that must go from the target of said edge).
    fn new_branch(node: N, num_outports: usize, outport: OutgoingPort, ch: Self) -> Self;
    /// Merge two path sets, both describing paths to the same location / out of the same subtree,
    /// starting from the same point.
    fn union(&mut self, other: &Self);
    /// Concatenate another path, i.e. that describes a set starting from the node reached by this path.
    ///
    /// (Will be called only when every element in `self` ends at the same node, not for [PathSet]s that
    /// describe exits from a subtree to multiple targets.)
    fn concat(&self, other: &Self) -> Self;

    /// Split this PathSet into disjoint subsets according to the final edge.
    ///
    /// If `self` describes paths to a node, each element will describe a different incoming edge to that node;
    /// if this describes paths leaving a subtree, each element will be a different edge exiting that subtree.
    ///
    /// In either case, each element is a tuple, the first element indicating the final control-flow edge,
    /// common to all paths in the PathSet being the second element of the tuple.
    fn split_by_last_edge(&self) -> impl Iterator<Item = ((N, OutgoingPort), Self)>;

    /// Type describing a set of paths that may be empty. The type `Option<Self>` would be
    /// valid for any implementation of [PathSet], but this allows for more efficient representations in some cases.
    type OptPath: From<Self> + Debug + Default;

    /// If an `OptPath` is non-empty, return a reference to the contained `PathSet`; otherwise return `None`.
    fn opt_as_ref(opt: &Self::OptPath) -> Option<&Self>;
    /// Updates an `OptPath` by unioning it with another `PathSet`. (After update, `opt` will
    /// necessarily be non-empty, but we leave it as type `OptPath`.)
    fn union_opt(opt: &mut Self::OptPath, other: &Self);
}

/// Dominator-strong components decomposition of a CFG.
///
/// (For reducible CFGs only.)
///
/// A 1-1 representation of the dominator tree of the CFG, with
/// * The dominator children of each node in topsort order
/// * The control flow edges leaving this dominator subtree represented by type `P`.
/// * Extra information about loops (according to `LOOP`) - see e.g. [DomTreeWithBackedges]
pub struct DomTreeNode<N, P: PathSet<N>, LOOP> {
    /// The CFG node at the root of this dominator subtree
    pub node: N,
    /// In topsort order (each child before any sibling to which the child has an exit edge);
    /// loop back edges from a child subtree back to [Self::node] do not affect topsort order.
    pub children: Vec<(P, DomTreeNode<N, P, LOOP>)>,
    /// The control flow edges leaving this dominator subtree (i.e. whose
    /// source is within this subtree but whose target is not)
    pub exit_edges: P::OptPath,
    /// Information describing any loop for which this is the header.
    /// (In a reducible CFG, loops can only be formed by back edges to the header,
    ///  which dominates all nodes in the loop body; the body being those nodes
    ///  which have a forwards path to the header)
    pub loop_: LOOP,
}

impl<N: HugrNode, P: PathSet<N>, LOOP: Default> DomTreeNode<N, P, LOOP> {
    /// Make a new instance by combining the children (provided in topsort order).
    /// `loop_fn` is called to update the [DomTreeNode::loop_] with each child exit edge
    /// targetting the main node (i.e. the loop header).
    pub fn new_with_loop_callback(
        node: N,
        children: Vec<Self>,
        loop_fn: impl Fn(&mut LOOP, &P),
        hugr: &impl HugrView<Node = N>,
    ) -> Self {
        let child_blocks: HashSet<N> = children.iter().map(|c| c.node).collect();
        let mut child_paths = HashMap::<N, P>::new();
        let mut loop_: LOOP = LOOP::default();
        let mut exit_edges: P::OptPath = P::OptPath::default();
        // Process edges from this node (perhaps a loop header)
        let outports = hugr.node_outputs(node).collect::<Vec<_>>();
        for outport in &outports {
            let mut path = P::new_edge(node, *outport);
            if outports.len() > 1 {
                path = P::new_branch(node, outports.len(), *outport, path);
            }
            // Control Flow outports should have exactly one outgoing edge
            let (tgt, _) = hugr.single_linked_input(node, *outport).unwrap();
            if child_blocks.contains(&tgt) {
                child_paths
                    .entry(tgt)
                    .and_modify(|p| p.union(&path))
                    .or_insert(path);
            } else if tgt == node {
                loop_fn(&mut loop_, &path)
            } else {
                P::union_opt(&mut exit_edges, &path)
            }
        }

        // Now process children in the determined order
        let children = children
            .into_iter()
            .map(|child_dtn| {
                let path_to_child = child_paths.remove(&child_dtn.node).unwrap();
                // ALAN shouldn't need opt_as_ref here if concat does right thing
                if let Some(child_exit_edges) = P::opt_as_ref(&child_dtn.exit_edges) {
                    let path_to_child_exit = path_to_child.concat(child_exit_edges);
                    for (tgt, path_to_exit) in split_by_target(&path_to_child_exit, hugr) {
                        /*assert!(
                            // if dst has no dominator, dst is the entry node
                            doms.immediate_dominator(node_map.to_portgraph(lp.tgt))
                                .is_none_or(|tgt_dom|
                            //otherwise, tgt_dom must be ni or some dominator thereof
                            // (i.e. tgt is a sibling of an nonstrict-ancestor of ni).
                        doms.dominators(ni).unwrap().contains(&tgt_dom))
                        );*/
                        if child_blocks.contains(&tgt) {
                            child_paths
                                .entry(tgt)
                                .and_modify(|p| p.union(&path_to_exit))
                                .or_insert(path_to_exit);
                        } else if tgt == node {
                            loop_fn(&mut loop_, &path_to_exit)
                        } else {
                            P::union_opt(&mut exit_edges, &path_to_exit);
                        }
                    }
                }
                (path_to_child, child_dtn)
            })
            .collect();

        DomTreeNode {
            node,
            children,
            exit_edges,
            loop_,
        }
    }
}

/// A [DomTreeNode] representing a loop as the paths back to the loop header.
pub type DomTreeWithBackedges<N, P> = DomTreeNode<N, P, <P as PathSet<N>>::OptPath>;

impl<N: HugrNode, P: PathSet<N>> DomTreeWithBackedges<N, P> {
    /// Creates a new instance as per [DomTreeNode::new], updating the [DomTreeNode::loop_]
    /// with gating paths that target the loop header.
    pub fn new(node: N, children: Vec<Self>, hugr: &impl HugrView<Node = N>) -> Self {
        Self::new_with_loop_callback(node, children, P::union_opt, hugr)
    }

    /// Builds a dominator tree for the given control flow graph (CFG).
    pub fn new_for_cfg(hugr: &impl HugrView<Node = N>, cfg: N) -> Self {
        fn rev_sort<N: HugrNode, P: PathSet<N>>(
            hugr: &impl HugrView<Node = N>,
            ordered: &mut Vec<N>,
            child: N,
            remaining_children: &mut HashMap<N, &DomTreeWithBackedges<N, P>>,
        ) {
            let Some(dtn) = remaining_children.remove(&child) else {
                return;
            };
            // Targets of exit edges pushed onto <ordered> first
            for tgt in leaf_targets::<_, P>(&dtn.exit_edges, hugr) {
                rev_sort(hugr, ordered, tgt, remaining_children);
            }
            ordered.push(child);
        }
        fn build<H: HugrView, P: PathSet<H::Node>>(
            hugr: &H,
            doms: &Dominators<NodeIndex>,
            n: H::Node,
            node_map: &H::RegionPortgraphNodes,
        ) -> DomTreeWithBackedges<H::Node, P> {
            let ni = node_map.to_portgraph(n);
            let mut children_by_bb = doms
                .immediately_dominated_by(ni)
                .map(|c| build(hugr, doms, node_map.from_portgraph(c), node_map))
                .map(|c| (c.node, c))
                .collect::<HashMap<_, _>>();

            // We want to process children in reverse topsort order: any child C1 with an exit edge to C2, must be processed *before* C2.
            let mut ordered_children = Vec::new();

            let mut remaining_children = children_by_bb.iter().map(|(n, c)| (*n, c)).collect();
            for child in children_by_bb.values() {
                rev_sort(
                    &hugr,
                    &mut ordered_children,
                    child.node,
                    &mut remaining_children,
                );
            }
            // This means targets of exit edges will be processed *after* all the sources of said exit edges:
            ordered_children.reverse();

            DomTreeWithBackedges::<H::Node, P>::new(
                n,
                ordered_children
                    .into_iter()
                    .map(|n| children_by_bb.remove(&n).unwrap())
                    .collect(),
                hugr,
            )
        }

        let (doms, node_map) = compute_dominators(hugr, cfg);
        let entry = hugr.children(cfg).next().unwrap();
        assert_eq!(doms.root(), node_map.to_portgraph(entry));
        build(&hugr, &doms, entry, &node_map)
    }
}

fn compute_dominators<H: HugrView>(
    hugr: &H,
    parent: H::Node,
) -> (Dominators<NodeIndex>, H::RegionPortgraphNodes) {
    let sg = hugr.scheduling_graph(parent);
    let entry_node = hugr.children(parent).next().unwrap();
    let doms = dominators::simple_fast(sg.petgraph(), sg.node_to_pg(entry_node));
    (doms, sg.into_node_map())
}

pub(super) fn split_by_target<H: HugrView, P: PathSet<H::Node>>(
    gp: &P,
    hugr: &H,
) -> HashMap<H::Node, P> {
    let mut res = HashMap::<H::Node, P>::new();
    for ((n, outp), l) in gp.split_by_last_edge() {
        let (target, _) = hugr.single_linked_input(n, outp).unwrap();
        res.entry(target)
            .and_modify(|existing| existing.union(&l))
            .or_insert(l);
    }
    res
}

pub(super) fn leaf_targets<'a, H: HugrView, P: PathSet<H::Node> + 'a>(
    gp: &'a P::OptPath,
    hugr: &'a H,
) -> impl Iterator<Item = H::Node> + 'a {
    P::opt_as_ref(gp).into_iter().flat_map(move |gp| {
        gp.split_by_last_edge()
            .map(|((node, port), _)| hugr.single_linked_input(node, port).unwrap().0)
    })
}

/// Implements [PathSet] merely as the set containing the final edge of each path.
pub type EdgeSources<N> = BTreeSet<(N, OutgoingPort)>;

impl<N: HugrNode> PathSet<N> for EdgeSources<N> {
    type OptPath = Self;

    fn new_edge(node: N, outport: OutgoingPort) -> Self {
        Self::from([(node, outport)])
    }

    fn new_branch(_node: N, _num_outports: usize, _outport: OutgoingPort, ch: Self) -> Self {
        ch
    }

    fn union(&mut self, other: &Self) {
        self.extend(other);
    }

    fn concat(&self, other: &Self) -> Self {
        assert!(!self.is_empty());
        other.clone()
    }

    fn split_by_last_edge(&self) -> impl Iterator<Item = ((N, OutgoingPort), Self)> {
        self.iter()
            .map(|&(node, outport)| ((node, outport), Self::from([(node, outport)])))
    }

    fn opt_as_ref(opt: &Self::OptPath) -> Option<&Self> {
        (!opt.is_empty()).then_some(opt)
    }

    fn union_opt(opt: &mut Self::OptPath, other: &Self) {
        opt.extend(other);
    }
}
