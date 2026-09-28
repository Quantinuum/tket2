//! Analysis of Control-Flow Graphs using dominator-strong components decomposition
use itertools::Itertools;
use std::collections::hash_map::Entry;
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

/// A [DomTreeNode] representing a loop as the [GatingPath] back to the loop header.
pub type DomTreeWithBackedges<N> = DomTreeNode<N, GatingPath<N>>;

impl<N: HugrNode> DomTreeWithBackedges<N> {
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
            for lp in leaves(&dtn.exit_edges, hugr) {
                rev_sort(hugr, ordered, lp.tgt, remaining_children);
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
            let children_by_bb = doms
                .immediately_dominated_by(ni)
                .map(|c| build(hugr, doms, node_map.from_portgraph(c), node_map))
                .map(|c| (c.node, c))
                .collect::<HashMap<_, _>>();
            let mut exit_edges: Option<GatingPath<H::Node>> = None;
            let mut loop_backedges: Option<GatingPath<H::Node>> = None;

            let mut child_paths = HashMap::<H::Node, GatingPath<H::Node>>::new();
            // Process edges from this node (perhaps a loop header)
            let path = hugr
                .node_outputs(n.into())
                .exactly_one()
                .ok()
                .map(|p| GatingPath::Always(n, p));
            let outports = hugr.node_outputs(n.into()).collect::<Vec<_>>();
            for outport in &outports {
                let path = path
                    .clone()
                    .unwrap_or(GatingPath::branch(n, *outport, outports.len()));
                // Control Flow outports should have exactly one outgoing edge
                let (tgt, _) = hugr
                    .linked_inputs(n.into(), *outport)
                    .exactly_one()
                    .ok()
                    .unwrap();
                if children_by_bb.contains_key(&tgt) {
                    child_paths
                        .entry(tgt)
                        .and_modify(|p| p.union(&path))
                        .or_insert_with(|| path.clone());
                } else if tgt == n {
                    union_opt(&mut loop_backedges, &path);
                } else {
                    union_opt(&mut exit_edges, &path)
                }
            }
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

            // Now process children in the determined order
            let mut children_by_bb = children_by_bb;
            let children = ordered_children
                .into_iter()
                .map(|child| {
                    let path_to_child = child_paths.remove(&child).unwrap();
                    let child_dtn = children_by_bb.remove(&child).unwrap();
                    let child_exit_leaves = leaves(&child_dtn.exit_edges, hugr);
                    for lp in child_exit_leaves {
                        let path_from_child_to_exit: GatingPath<H::Node> = lp.clone().into();
                        let path_to_exit = path_to_child.concat(&path_from_child_to_exit);
                        assert!(
                            // if dst has no dominator, dst is the entry node
                            doms.immediate_dominator(node_map.to_portgraph(lp.tgt))
                                .is_none_or(|tgt_dom|
                            //otherwise, tgt_dom must be ni or some dominator thereof
                            // (i.e. tgt is a sibling of an nonstrict-ancestor of ni).
                        doms.dominators(ni).unwrap().contains(&tgt_dom))
                        );
                        if children_by_bb.contains_key(&lp.tgt) {
                            child_paths
                                .entry(lp.tgt)
                                .and_modify(|p| p.union(&path_to_exit))
                                .or_insert_with(|| path_to_exit.clone());
                        } else if lp.tgt == n {
                            union_opt(&mut loop_backedges, &path_to_exit);
                        } else {
                            union_opt(&mut exit_edges, &path_to_exit);
                        }
                    }
                    (path_to_child, child_dtn)
                })
                .collect();

            DomTreeNode {
                node: n,
                children,
                exit_edges,
                loop_: loop_backedges,
            }
        }

        let (doms, node_map) = compute_dominators(hugr, cfg);
        let entry = hugr.children(cfg).next().unwrap();
        assert_eq!(doms.root(), node_map.to_portgraph(entry));
        build(&hugr, &doms, entry, &node_map)
    }

    /// Return values are:
    /// * `Option<Self>`: The remaining part of the current node after detaching the
    ///   non-loop blocks.
    /// * `Vec<Self>`: The subtrees that were detached as being outside the loop
    /// * `HashMap<N, Vec<N>>`: A mapping, from each node that is destination of a loop-exit
    ///   edge, to a representation of the LCA in the dominator tree of all such edges,
    ///   given as a list of dominators starting from `self` and moving down the dominator
    ///   tree one node at a time until the LCA is reached.
    pub(super) fn detach<H: HugrView<Node = N>>(
        self,
        hugr: &H,
        loop_blocks: &HashSet<N>,
    ) -> (Option<Self>, Vec<Self>, HashMap<N, Vec<N>>) {
        if !loop_blocks.contains(&self.node) {
            let n = self.node;
            return (None, vec![self], HashMap::from([(n, vec![])]));
        }
        // TODO need to consider exit_edges here as well as children. If these exit the loop,
        // - in the inner loop, they'll go to tag_exit and then the inner ExitBlock;
        //   it's not clear where those will be attached into the dominator tree.
        // - for the outer (loop-containing node), we need to return the edge source (rather than a child),
        //   to add to the *outer* block's DomTreeNode (as exit edges directly from the outer block i.e. the header).
        let mut remaining_children = Vec::new();
        let mut detached = Vec::new();
        let mut exit_targets = HashMap::new();
        for (gp, ch) in self.children {
            let (ch, ch_detached, ch_exit_targets) = ch.detach(hugr, loop_blocks);
            if let Some(ch) = ch {
                remaining_children.push((gp, ch));
            }
            detached.extend(ch_detached);
            for (k, mut v) in ch_exit_targets {
                match exit_targets.entry(k) {
                    Entry::Occupied(occupied_entry) => *occupied_entry.into_mut() = vec![self.node],
                    Entry::Vacant(vacant_entry) => {
                        v.insert(0, self.node);
                        vacant_entry.insert(v);
                    }
                }
            }
        }
        for lp in leaves(&self.exit_edges, hugr) {
            // Override any existing entry as self.node is LCA.
            exit_targets.insert(lp.tgt, vec![self.node]);
        }
        (
            Some(DomTreeNode {
                node: self.node,
                children: remaining_children,
                exit_edges: self.exit_edges, // No recompute (?)
                loop_: self.loop_,           // we have not detached the backedges
            }),
            detached,
            exit_targets,
        )
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
///
/// TODO this may not be required.
#[derive(Clone, Debug, Default)]
pub(super) struct LeafPath<N> {
    /// Previous branches passed through on the way to [Self::src]. The `usize` caches the number of outgoing ports.
    branches: Vec<(N, OutgoingPort, usize)>,
    /// The source node (last node dominated by the start), and the outgoing port
    /// whose edge leaves the dominator tree
    pub(super) src: (N, OutgoingPort),
    /// Target of that edge (outside the dominator tree)
    tgt: N,
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
    fn branch(node: N, port: OutgoingPort, num_ports: usize) -> Self {
        let mut branches = vec![None; num_ports];
        branches[port.index()] = Some(GatingPath::Always(node, port));
        GatingPath::Branch(node, branches)
    }

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

    /// TODO client (remove_cycles) only needs the [LeafPath::src] of each here
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

fn leaves<H: HugrView>(gp: &Option<GatingPath<H::Node>>, hugr: &H) -> Vec<LeafPath<H::Node>> {
    match gp {
        Some(gp) => gp.leaves(hugr),
        None => Vec::new(),
    }
}
