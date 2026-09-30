//! Analysis of Control-Flow Graphs using dominator-strong components decomposition
use itertools::Itertools;
use std::collections::{HashMap, HashSet};
use std::iter;

use hugr::core::HugrNode;
use hugr::{HugrView, OutgoingPort, PortIndex as _};
use hugr_core::hugr::internal::PortgraphNodeMap;
use petgraph::algo::dominators::{self, Dominators};
use portgraph::NodeIndex;

/// Dominator-strong components decomposition of a CFG.
///
/// (For reducible CFGs only.)
///
/// A 1-1 representation of the dominator tree of the CFG, with
/// * The dominator children of each node in topsort order
/// * The control flow edges leaving this dominator subtree
/// * Extra information about loops (according to `LOOP`) - see e.g. [DomTreeWithBackedges]
pub struct DomTreeNode<N, LOOP> {
    /// The CFG node at the root of this dominator subtree
    pub node: N,
    /// In topsort order (each child before any sibling to which the child has an exit edge);
    /// loop back edges from a child subtree back to [Self::node] do not affect topsort order.
    pub children: Vec<(GatingPath<N>, DomTreeNode<N, LOOP>)>,
    /// The control flow edges leaving this dominator subtree (i.e. whose
    /// source is within this subtree but whose target is not)
    pub exit_edges: Option<GatingPath<N>>,
    /// Information describing any loop for which this is the header.
    /// (In a reducible CFG, loops can only be formed by back edges to the header,
    ///  which dominates all nodes in the loop body; the body being those nodes
    ///  which have a forwards path to the header)
    pub loop_: Option<LOOP>,
}

impl<N: HugrNode, LOOP> DomTreeNode<N, LOOP> {
    /// Make a new instance by combining the children (provided in topsort order)
    /// `loop_fn` updates the [DomTreeNode::loop_] with an exit path, returning `true`
    /// if it did so, or `false` if the exit path was not part of the loop and should be
    ///  recorded as an exit edge of the parent instance being created.
    pub(super) fn new(
        node: N,
        children: Vec<Self>,
        loop_fn: impl Fn(&mut Option<LOOP>, &LeafPath<N>) -> bool,
        hugr: &impl HugrView<Node = N>,
    ) -> Self {
        let child_blocks: HashSet<N> = children.iter().map(|c| c.node).collect();
        let mut child_paths = HashMap::<N, GatingPath<N>>::new();
        let mut loop_: Option<LOOP> = None;
        let mut exit_edges: Option<GatingPath<N>> = None;
        // Process edges from this node (perhaps a loop header)
        let path = hugr
            .node_outputs(node)
            .exactly_one()
            .ok()
            .map(|p| LeafPath {
                src: (node, p),
                branches: vec![],
                tgt: hugr.single_linked_input(node, p).unwrap().0,
            });
        let outports = hugr.node_outputs(node).collect::<Vec<_>>();
        for outport in &outports {
            let path = path.clone().unwrap_or(LeafPath {
                src: (node, *outport),
                branches: vec![(node, *outport, outports.len())],
                tgt: hugr.single_linked_input(node, *outport).unwrap().0,
            });
            // Control Flow outports should have exactly one outgoing edge
            let (tgt, _) = hugr.single_linked_input(node.into(), *outport).unwrap();
            if child_blocks.contains(&tgt) {
                let path = path.into();
                child_paths
                    .entry(tgt)
                    .and_modify(|p| p.union(&path))
                    .or_insert_with(|| path);
            } else if !loop_fn(&mut loop_, &path) {
                union_opt(&mut exit_edges, &path.into())
            }
        }

        // Now process children in the determined order
        let children = children
            .into_iter()
            .map(|child_dtn| {
                let path_to_child = child_paths.remove(&child_dtn.node).unwrap();
                if let Some(child_exit_edges) = child_dtn.exit_edges.as_ref() {
                    let path_to_child_exit = path_to_child.concat(child_exit_edges);
                    for lp in path_to_child_exit.leaves(hugr) {
                        /*assert!(
                            // if dst has no dominator, dst is the entry node
                            doms.immediate_dominator(node_map.to_portgraph(lp.tgt))
                                .is_none_or(|tgt_dom|
                            //otherwise, tgt_dom must be ni or some dominator thereof
                            // (i.e. tgt is a sibling of an nonstrict-ancestor of ni).
                        doms.dominators(ni).unwrap().contains(&tgt_dom))
                        );*/
                        if child_blocks.contains(&lp.tgt) {
                            let e = child_paths.entry(lp.tgt);
                            let path_to_exit = lp.into();
                            e.and_modify(|p| p.union(&path_to_exit))
                                .or_insert_with(|| path_to_exit);
                        } else if !loop_fn(&mut loop_, &lp) {
                            union_opt(&mut exit_edges, &lp.into());
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

/// A [DomTreeNode] representing a loop as the [GatingPath] back to the loop header.
pub type DomTreeWithBackedges<N> = DomTreeNode<N, GatingPath<N>>;

impl<N: HugrNode> DomTreeWithBackedges<N> {
    /// Creates a new instance as per [DomTreeNode::new], updating the [DomTreeNode::loop_]
    /// with gating paths that target the loop header.
    pub(super) fn new_with_children(
        node: N,
        children: Vec<Self>,
        hugr: &impl HugrView<Node = N>,
    ) -> Self {
        Self::new(
            node,
            children,
            |loop_, child_exit_path| {
                if child_exit_path.tgt == node {
                    union_opt(loop_, &child_exit_path.clone().into());
                    true
                } else {
                    false
                }
            },
            hugr,
        )
    }

    /// Builds a dominator tree for the given control flow graph (CFG).
    pub fn new_for_cfg(hugr: &impl HugrView<Node = N>, cfg: N) -> Self {
        fn rev_sort<N: HugrNode>(
            hugr: &impl HugrView<Node = N>,
            ordered: &mut Vec<N>,
            child: N,
            remaining_children: &mut HashMap<N, &DomTreeWithBackedges<N>>,
        ) {
            let Some(dtn) = remaining_children.remove(&child) else {
                return;
            };
            // Targets of exit edges pushed onto <ordered> first
            for tgt in leaf_targets(&dtn.exit_edges, hugr) {
                rev_sort(hugr, ordered, tgt, remaining_children);
            }
            ordered.push(child);
        }
        fn build<H: HugrView>(
            hugr: &H,
            doms: &Dominators<NodeIndex>,
            n: H::Node,
            node_map: &H::RegionPortgraphNodes,
        ) -> DomTreeWithBackedges<H::Node> {
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

            DomTreeWithBackedges::<H::Node>::new_with_children(
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

/// A collection (0 or more) paths from some given CFG node, within that node's dominator tree.
/// `None` (rather than a `GatingPath`) represents the empty collection (no paths).
#[derive(Clone, Debug)]
pub enum GatingPath<N> {
    /// Source of an edge leaving the dominator tree
    Always(N, OutgoingPort),
    /// One element (None if no path via that port) for each outgoing port of the branch node
    Branch(N, Vec<Option<GatingPath<N>>>),
}

/// A single path from some given CFG node, stopping at the boundary of its dominator tree.
#[derive(Clone, Debug, Default)]
pub(super) struct LeafPath<N> {
    /// Previous branches passed through on the way to [Self::src]. The `usize` caches the number of outgoing ports.
    branches: Vec<(N, OutgoingPort, usize)>,
    /// The source node (last node dominated by the start), and the outgoing port
    /// whose edge leaves the dominator tree
    pub(super) src: (N, OutgoingPort),
    /// Target of that edge (outside the dominator tree)
    pub(super) tgt: N,
}

impl<N> From<LeafPath<N>> for GatingPath<N> {
    fn from(leaf: LeafPath<N>) -> Self {
        let mut path = GatingPath::Always(leaf.src.0, leaf.src.1);
        for (node, port, len) in leaf.branches.into_iter().rev() {
            let mut branches = Vec::from_iter(iter::repeat_with(|| None).take(len));
            branches[port.index()] = Some(path);
            path = GatingPath::Branch(node, branches);
        }
        path
    }
}

/// Merges `other` into the accumulated path `acc`, which may not have any path yet.
fn union_opt<N: HugrNode>(acc: &mut Option<GatingPath<N>>, other: &GatingPath<N>) {
    match acc {
        Some(existing) => existing.union(other),
        None => *acc = Some(other.clone()),
    }
}

impl<N: HugrNode> GatingPath<N> {
    fn concat(&self, other: &GatingPath<N>) -> Self {
        fn has_none<N>(gp: &Option<GatingPath<N>>) -> bool {
            match gp {
                None => true,
                Some(GatingPath::Always(_, _)) => false,
                Some(GatingPath::Branch(_, opts)) => opts.iter().any(has_none),
            }
        }
        match self {
            GatingPath::Always(_, _) => other.clone(),
            GatingPath::Branch(node, opts) => {
                if opts.iter().any(has_none) {
                    GatingPath::Branch(
                        *node,
                        opts.iter()
                            .map(|v| v.as_ref().map(|v| v.concat(other)))
                            .collect(),
                    )
                } else {
                    other.clone()
                }
            }
        }
    }

    pub(super) fn leaves(&self, hugr: &impl HugrView<Node = N>) -> Vec<LeafPath<N>> {
        fn traverse<H: HugrView>(
            hugr: &H,
            gp: &GatingPath<H::Node>,
            path_to_here: &mut Vec<(H::Node, OutgoingPort, usize)>,
        ) -> Vec<LeafPath<H::Node>> {
            match gp {
                GatingPath::Always(node, port) => {
                    let (tgt, _) = hugr.single_linked_input(*node, *port).unwrap();
                    vec![LeafPath {
                        branches: path_to_here.clone(),
                        src: (*node, *port),
                        tgt,
                    }]
                }
                GatingPath::Branch(br, opts) => opts
                    .iter()
                    .enumerate()
                    .flat_map(|(i, gp)| {
                        path_to_here.push((*br, i.into(), opts.len()));
                        let leaves = gp
                            .as_ref()
                            .map(|gp| traverse(hugr, gp, path_to_here))
                            .unwrap_or_default();
                        path_to_here.pop();
                        leaves
                    })
                    .collect(),
            }
        }
        traverse(hugr, self, &mut Vec::new())
    }

    fn union(&mut self, other: &GatingPath<N>) {
        match self {
            GatingPath::Always(_, _) => {
                panic!("Union of Always with {other:?}");
            }
            GatingPath::Branch(n, opts) => {
                if let GatingPath::Branch(n2, opts2) = other
                    && n == n2
                {
                    for (this, other) in opts.iter_mut().zip_eq(opts2.iter()) {
                        let Some(other) = other else { continue };
                        match this {
                            None => *this = Some(other.clone()),
                            Some(this) => this.union(other),
                        }
                    }
                } else {
                    panic!("Cannot union Branch({n:?}) with {other:?}");
                }
            }
        }
    }
}

pub(super) fn leaf_targets<H: HugrView>(
    gp: &Option<GatingPath<H::Node>>,
    hugr: &H,
) -> impl Iterator<Item = H::Node> {
    gp.into_iter()
        .flat_map(move |gp| gp.leaves(hugr).into_iter().map(|lp| lp.tgt))
}
