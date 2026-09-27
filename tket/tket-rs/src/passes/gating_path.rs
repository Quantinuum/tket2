//! Analysis of Control-Flow Graphs using dominator-strong components decomposition
use hugr::hugr::hugrmut::HugrMut;
use hugr::ops::{CFG, DataflowBlock, ExitBlock, TailLoop};
use hugr::types::{Signature, Type, TypeRow};
use itertools::Itertools;
use std::collections::{HashMap, HashSet, VecDeque};
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
    node: N,
    /// In topsort order (each child before any sibling to which the child has an exit edge);
    /// loop back edges from a child subtree back to [Self::node] do not affect topsort order.
    children: Vec<(GatingPath<N>, DomTreeNode<N, LOOP>)>,
    /// The control flow edges leaving this dominator subtree (i.e. whose
    /// source is within this subtree but whose target is not)
    exit_edges: Option<GatingPath<N>>,
    /// Information describing any loop for which this is the header.
    /// (In a reducible CFG, loops can only be formed by back edges to the header,
    ///  which dominates all nodes in the loop body; the body being those nodes
    ///  which have a forwards path to the header)
    loop_: Option<LOOP>,
}

/// A [DomTreeNode] representing a loop as the [GatingPath] back to the loop header.
type DomTreeWithBackedges<N> = DomTreeNode<N, GatingPath<N>>;

/// Used as [DomTreeNode::loop_], records information about a CFG within a TailLoop within
/// the [DomTreeNode::node]
struct InnerTailLoop<N>(Box<DomTreeNode<N, InnerTailLoop<N>>>);

impl<N: HugrNode, LOOP> DomTreeNode<N, LOOP> {
    fn disconnect(&mut self, doms: &[N]) -> (GatingPath<N>, Self) {
        for (child_idx, (child_path, child)) in self.children.iter_mut().enumerate() {
            if child.node == doms[0] {
                if doms.len() == 1 {
                    return self.children.remove(child_idx);
                }
                let (ep, dtn) = child.disconnect(&doms[1..]);
                return (child_path.concat(&ep), dtn);
            }
        }
        panic!("Node not found in children");
    }
}

enum Void {}

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

    fn nest_loop(self, hugr: &mut impl HugrMut<Node = N>) -> DomTreeNode<N, InnerTailLoop<N>> {
        let Some(backedges) = self.loop_ else {
            return DomTreeNode {
                node: self.node,
                children: self
                    .children
                    .into_iter()
                    .map(|(cp, cn)| (cp, cn.nest_loop(hugr)))
                    .collect(),
                exit_edges: self.exit_edges,
                loop_: None,
            };
        };
        let loop_blocks = loop_blocks(hugr, self.node, &backedges);
        let mut in_loop_children = Vec::new();
        let mut out_of_loop_children = Vec::new();
        for (cp, cn) in self.children {
            let (in_loop, out_of_loop) = cn.detach(hugr, &loop_blocks);
            if let Some(in_loop) = in_loop {
                in_loop_children.push((cp, in_loop));
            }
            out_of_loop_children.extend(out_of_loop);
        }
        let header_inputs = hugr
            .get_optype(self.node)
            .as_dataflow_block()
            .unwrap()
            .inputs
            .clone();
        let exit_type_rows = out_of_loop_children
            .iter()
            .map(|n| {
                hugr.get_optype(n.node)
                    .as_dataflow_block()
                    .unwrap()
                    .inputs
                    .clone()
            })
            .collect::<Vec<_>>();
        let outer_cfg = hugr.get_parent(self.node).unwrap();
        let loop_node = hugr.add_node_with_parent(
            outer_cfg,
            DataflowBlock {
                inputs: header_inputs.clone(),
                other_outputs: TypeRow::new(),
                sum_rows: exit_type_rows.clone(),
            },
        );
        // TODO(?) If exit_type_rows of length one, then no need for this, hmmm.
        let exit_sum_row = TypeRow::from([Type::new_sum(exit_type_rows)]);
        let tail_loop = hugr.add_node_with_parent(
            loop_node,
            TailLoop {
                just_inputs: header_inputs.clone(),
                just_outputs: exit_sum_row.clone(),
                rest: TypeRow::new(),
            },
        ); // TODO and wire up (loop_node needs input/output first)
        let exit_or_continue_row =
            TypeRow::from([Type::new_sum([header_inputs.clone(), exit_sum_row.clone()])]);
        let inner_cfg = hugr.add_node_with_parent(
            tail_loop,
            CFG {
                signature: Signature::new(header_inputs.clone(), exit_or_continue_row.clone()),
            },
        ); // TODO and wire up (tail_loop needs input/output first)
        hugr.set_parent(self.node, inner_cfg);
        let exit_block = hugr.add_node_with_parent(
            inner_cfg,
            ExitBlock {
                cfg_outputs: exit_or_continue_row.clone(),
            },
        );

        let tag_continue = hugr.add_node_with_parent(
            inner_cfg,
            DataflowBlock {
                inputs: header_inputs.clone(),
                other_outputs: TypeRow::new(),
                sum_rows: vec![exit_or_continue_row.clone()],
            },
        );
        // TODO add and wire up tag_continue's children
        let tag_exit = hugr.add_node_with_parent(
            inner_cfg,
            DataflowBlock {
                inputs: exit_sum_row,
                other_outputs: TypeRow::new(),
                sum_rows: vec![exit_or_continue_row.clone()],
            },
        );
        // TODO add and wire up tag_exit's children
        hugr.connect(tag_continue, 0, exit_block, 0);
        hugr.connect(tag_exit, 0, exit_block, 0);
        for &n in &loop_blocks {
            hugr.set_parent(n, inner_cfg);
            for succ_port in hugr.node_outputs(n).collect::<Vec<_>>() {
                let (succ, _) = hugr.single_linked_input(n, succ_port).unwrap();
                if succ == self.node {
                    //loop backedge
                    hugr.disconnect(n, succ_port);
                    hugr.connect(n, succ_port, tag_continue, 0);
                } else if !loop_blocks.contains(&succ) {
                    hugr.disconnect(n, succ_port);
                    hugr.connect(n, succ_port, tag_exit, 0);
                }
            }
        }

        for (pred_n, outport) in hugr.all_linked_outputs(self.node).collect::<Vec<_>>() {
            assert_eq!(
                hugr.single_linked_input(pred_n, outport),
                Some((self.node, 0.into()))
            );
            hugr.disconnect(pred_n, outport);
            assert!(!loop_blocks.contains(&pred_n)); // loop predecessors disconnected above
            hugr.connect(pred_n, outport, loop_node, 0);
        }

        DomTreeNode {
            node: loop_node,
            // TODO we need also to include edges that exitted both the loop and the header's dom tree.
            // these do not produce "children" here but instead add to exit_edges.
            children: out_of_loop_children
                .into_iter()
                .enumerate()
                .map(|(exit_num, n)| {
                    (
                        GatingPath::Always(loop_node, exit_num.into()),
                        n.nest_loop(hugr),
                    )
                })
                .collect(),
            // TODO no recompute here. Old exit edges could include edges directly from the loop,
            // these are now edges from the new header (loop_node)
            exit_edges: self.exit_edges,
            loop_: Some(InnerTailLoop(Box::new(DomTreeNode {
                node: self.node,
                children: in_loop_children
                    .into_iter()
                    .map(|(cp, cn)| (cp, cn.nest_loop(hugr)))
                    .collect(),
                exit_edges: None, // every node in inner CFG is dominated by the header, including the ExitBlock
                loop_: None,
            }))),
        }
    }

    fn detach<H: HugrView<Node = N>>(
        self,
        hugr: &H,
        loop_blocks: &HashSet<N>,
    ) -> (Option<Self>, Vec<Self>) {
        if !loop_blocks.contains(&self.node) {
            return (None, vec![self]);
        }
        // TODO need to consider exit_edges here as well as children. If these exit the loop,
        // - in the inner loop, they'll go to tag_exit and then the inner ExitBlock;
        //   it's not clear where those will be attached into the dominator tree.
        // - for the outer (loop-containing node), we need to return the edge source (rather than a child),
        //   to add to the *outer* block's DomTreeNode (as exit edges directly from the outer block i.e. the header).
        let mut remaining_children = Vec::new();
        let mut detached = Vec::new();
        for (gp, ch) in self.children {
            let (ch, ch_detached) = ch.detach(hugr, loop_blocks);
            if let Some(ch) = ch {
                remaining_children.push((gp, ch));
            }
            detached.extend(ch_detached);
        }
        (
            Some(DomTreeNode {
                node: self.node,
                children: remaining_children,
                exit_edges: self.exit_edges, // No recompute (?)
                loop_: self.loop_,           // we have not detached the backedges
            }),
            detached,
        )
    }
}

fn loop_blocks<H: HugrView>(
    hugr: &H,
    loop_header: H::Node,
    backedges: &GatingPath<H::Node>,
) -> HashSet<H::Node> {
    let mut blocks = HashSet::new();
    let mut queue = VecDeque::from_iter(backedges.leaves(hugr).into_iter().map(|lp| lp.src.0));
    while let Some(n) = queue.pop_front() {
        if n == loop_header || !blocks.insert(n) {
            continue;
        }
        queue.extend(hugr.input_neighbours(n));
    }
    blocks
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
enum GatingPath<N> {
    // Source of an edge leaving the dom tree
    Always(N, OutgoingPort),
    // One element (None if no path via that port) for each outgoing port of the branch node
    Branch(N, Vec<Option<GatingPath<N>>>),
}

/// A single path from some given CFG node, stopping at the boundary of its dominator tree.
#[derive(Clone, Debug, Default)]
struct LeafPath<N> {
    /// Previous branches passed through on the way to [Self::src]. The `usize` caches the number of outgoing ports.
    branches: Vec<(N, OutgoingPort, usize)>,
    /// The source node (last node dominated by the start), and the outgoing port
    /// whose edge leaves the dominator tree
    src: (N, OutgoingPort),
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

    fn leaves(&self, hugr: &impl HugrView<Node = N>) -> Vec<LeafPath<N>> {
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
