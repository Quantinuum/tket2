//! Reducible-CFG to RVSDG construction.
//!
//! The builder uses dominators, postdominators, and natural-loop discovery to
//! structure a reducible CFG into nested RVSDG regions with explicit `gamma`
//! and `theta` nodes, ordered arguments/results, and bundled control
//! variables.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use hugr::ops::OpType;
use hugr::types::{Type, TypeRow};
use hugr::{HugrView, Node};
use itertools::Itertools;
use petgraph::algo::{dominators::simple_fast, kosaraju_scc};
use petgraph::graphmap::DiGraphMap;

use crate::control::{CfgNodeMap, IdentityCfgMap};

use super::error::RvsdgBuildError;
use super::ir::{
    BlockNode, BranchJoinKind, GammaEntryVar, GammaNode, GammaOutputVar, LoopKind, Region,
    RegionVar, Rvsdg, RvsdgNode, ThetaLoopVar, ThetaNode, VarId,
};

/// Builds an RVSDG for one reducible CFG.
///
/// # Errors
///
/// Returns an error when the CFG is irreducible, when a required branch/loop
/// fact cannot be derived, or when a node expected to be a CFG block is not a
/// dataflow/exit block.
pub(crate) fn build_cfg_rvsdg<H: HugrView<Node = Node>>(
    cfg_view: &H,
    cfg: &IdentityCfgMap<H>,
) -> Result<Rvsdg, RvsdgBuildError<Node>> {
    let info = CfgInfo::new(cfg_view.entrypoint(), cfg)?;
    let mut builder = RvsdgBuilder::new(cfg_view);
    let root = builder.build_root(cfg.entry_node(), &info.scope, &info, cfg)?;
    Ok(Rvsdg { root })
}

/// Stateful builder allocating deterministic RVSDG variables.
struct RvsdgBuilder<'a, H> {
    /// Immutable view of the CFG being structured.
    cfg_view: &'a H,
    /// Monotonic variable allocator.
    next_var: usize,
}

impl<'a, H: HugrView<Node = Node>> RvsdgBuilder<'a, H> {
    /// Creates a builder for one CFG.
    fn new(cfg_view: &'a H) -> Self {
        Self {
            cfg_view,
            next_var: 0,
        }
    }

    /// Allocates one ordered variable of the given type.
    fn fresh_var(&mut self, ty: Type) -> RegionVar {
        let id = VarId(self.next_var);
        self.next_var += 1;
        RegionVar { id, ty }
    }

    /// Allocates one ordered variable per element in a row.
    fn fresh_vars(&mut self, row: &TypeRow) -> Vec<RegionVar> {
        row.iter().cloned().map(|ty| self.fresh_var(ty)).collect()
    }

    /// Builds the root region for the whole CFG.
    fn build_root(
        &mut self,
        start: Node,
        scope: &BTreeSet<Node>,
        info: &CfgInfo,
        cfg: &impl CfgNodeMap<Node>,
    ) -> Result<Region, RvsdgBuildError<Node>> {
        let arguments = self.cfg_signature_inputs()?;
        let results = self.cfg_signature_outputs()?;
        let body = self.build_scope(start, scope, None, None, info, cfg)?;
        Ok(Region {
            arguments,
            body,
            results,
        })
    }

    /// Returns the ordered CFG signature inputs as region arguments.
    fn cfg_signature_inputs(&mut self) -> Result<Vec<RegionVar>, RvsdgBuildError<Node>> {
        let cfg = self
            .cfg_view
            .get_optype(self.cfg_view.entrypoint())
            .as_cfg()
            .ok_or(RvsdgBuildError::ExpectedBlock {
                node: self.cfg_view.entrypoint(),
            })?;
        Ok(self.fresh_vars(&cfg.signature.input))
    }

    /// Returns the ordered CFG signature outputs as region results.
    fn cfg_signature_outputs(&mut self) -> Result<Vec<RegionVar>, RvsdgBuildError<Node>> {
        let cfg = self
            .cfg_view
            .get_optype(self.cfg_view.entrypoint())
            .as_cfg()
            .ok_or(RvsdgBuildError::ExpectedBlock {
                node: self.cfg_view.entrypoint(),
            })?;
        Ok(self.fresh_vars(&cfg.signature.output))
    }

    /// Structures one linear scope until an explicit stop node is reached.
    fn build_scope(
        &mut self,
        start: Node,
        scope: &BTreeSet<Node>,
        stop: Option<Node>,
        active_loop: Option<Node>,
        info: &CfgInfo,
        cfg: &impl CfgNodeMap<Node>,
    ) -> Result<Vec<RvsdgNode>, RvsdgBuildError<Node>> {
        let mut items = Vec::new();
        let mut current = Some(start);
        let mut seen = BTreeSet::new();

        while let Some(node) = current {
            if Some(node) == stop || !scope.contains(&node) {
                break;
            }
            if !seen.insert(node) {
                return Err(RvsdgBuildError::MalformedScope {
                    start,
                    reason: format!("scope walk revisited node {node}"),
                });
            }

            if info.is_nested_loop_header(node, scope, active_loop) {
                let (theta, next) = self.build_theta(node, scope, info, cfg)?;
                items.push(RvsdgNode::Theta(Box::new(theta)));
                current = next;
                continue;
            }

            let succs = info.scope_successors(node, scope, active_loop);
            if succs.len() > 1 {
                let (gamma, next) = self.build_gamma(node, scope, stop, active_loop, info, cfg)?;
                items.push(RvsdgNode::Gamma(Box::new(gamma)));
                current = next;
                continue;
            }

            let block = self.build_block(node)?;
            let is_exit = matches!(block, BlockNode::Exit { .. });
            items.push(RvsdgNode::Block(block));
            current = succs.into_iter().next();
            if is_exit {
                break;
            }
        }

        Ok(items)
    }

    /// Converts one CFG block into a typed RVSDG leaf.
    fn build_block(&mut self, node: Node) -> Result<BlockNode, RvsdgBuildError<Node>> {
        match self.cfg_view.get_optype(node) {
            OpType::DataflowBlock(block) => Ok(BlockNode::Dataflow {
                node,
                inputs: self.fresh_vars(&block.inputs),
                sum_rows: block
                    .sum_rows
                    .iter()
                    .map(|row| self.fresh_vars(row))
                    .collect(),
                outputs: self.fresh_vars(&block.other_outputs),
            }),
            OpType::ExitBlock(exit) => Ok(BlockNode::Exit {
                node,
                inputs: self.fresh_vars(&exit.cfg_outputs),
            }),
            _ => Err(RvsdgBuildError::ExpectedBlock { node }),
        }
    }

    /// Structures one reducible branch into a `gamma` node.
    fn build_gamma(
        &mut self,
        split_node: Node,
        scope: &BTreeSet<Node>,
        stop: Option<Node>,
        active_loop: Option<Node>,
        info: &CfgInfo,
        cfg: &impl CfgNodeMap<Node>,
    ) -> Result<(GammaNode, Option<Node>), RvsdgBuildError<Node>> {
        let split = self.build_block(split_node)?;
        let (inputs, sum_rows, outputs) = match &split {
            BlockNode::Dataflow {
                inputs,
                sum_rows,
                outputs,
                ..
            } => (inputs.clone(), sum_rows.clone(), outputs.clone()),
            _ => {
                return Err(RvsdgBuildError::ExpectedBlock { node: split_node });
            }
        };
        if sum_rows.is_empty() {
            return Err(RvsdgBuildError::ExpectedBlock { node: split_node });
        }
        let join_node = info.branch_join(split_node, scope, stop)?;
        let join = self.build_block(join_node)?;
        let join_inputs = join.inputs().to_vec();

        let arms = info.scope_successors(split_node, scope, active_loop);
        let mut branches = Vec::with_capacity(arms.len());
        for (case_idx, succ) in arms.iter().copied().enumerate() {
            let branch_arguments = branch_arguments(&sum_rows, &outputs, case_idx);
            let branch_results = join_inputs
                .iter()
                .map(|var| self.fresh_var(var.ty.clone()))
                .collect_vec();
            let body = self.build_scope(succ, scope, Some(join_node), active_loop, info, cfg)?;
            branches.push(Region {
                arguments: branch_arguments,
                body,
                results: branch_results,
            });
        }

        let entry_vars = outputs
            .iter()
            .cloned()
            .enumerate()
            .map(|(idx, input)| GammaEntryVar {
                input,
                branch_arguments: branches
                    .iter()
                    .map(|branch| branch.arguments[sum_rows[0].len() + idx].clone())
                    .collect(),
            })
            .collect_vec();
        let outputs = join_inputs
            .iter()
            .cloned()
            .enumerate()
            .map(|(idx, output)| GammaOutputVar {
                branch_results: branches
                    .iter()
                    .map(|branch| branch.results[idx].clone())
                    .collect(),
                output,
            })
            .collect_vec();
        let (join_kind, next) = info.branch_continuation(join_node, scope, active_loop);

        Ok((
            GammaNode {
                inputs,
                split,
                entry_vars,
                match_rows: sum_rows,
                branches,
                outputs,
                join,
                join_kind,
            },
            next,
        ))
    }

    /// Structures one reducible loop into a `theta` node.
    fn build_theta(
        &mut self,
        header: Node,
        scope: &BTreeSet<Node>,
        info: &CfgInfo,
        cfg: &impl CfgNodeMap<Node>,
    ) -> Result<(ThetaNode, Option<Node>), RvsdgBuildError<Node>> {
        let loop_blocks = info
            .loop_blocks
            .get(&header)
            .ok_or_else(|| RvsdgBuildError::UnsupportedLoop {
                header,
                reason: "missing loop block set".into(),
            })?
            .intersection(scope)
            .copied()
            .collect::<BTreeSet<_>>();
        let exit_edges = loop_blocks
            .iter()
            .copied()
            .flat_map(|src| {
                info.succs
                    .get(&src)
                    .into_iter()
                    .flatten()
                    .copied()
                    .map(move |dst| (src, dst))
            })
            .filter(|(src, dst)| {
                !loop_blocks.contains(dst) && !info.is_loop_backedge(*src, *dst, header)
            })
            .collect_vec();
        let exit_target = exit_edges
            .iter()
            .map(|(_, dst)| *dst)
            .dedup()
            .exactly_one()
            .map_err(|_| RvsdgBuildError::UnsupportedLoop {
                header,
                reason: "loop does not have a unique exit target".into(),
            })?;
        let backedge_source = info
            .backedges
            .get(&header)
            .into_iter()
            .flatten()
            .copied()
            .filter(|source| loop_blocks.contains(source))
            .exactly_one()
            .map_err(|_| RvsdgBuildError::UnsupportedLoop {
                header,
                reason: "loop does not have a unique backedge source".into(),
            })?;

        let header_block = self.build_block(header)?;
        let header_inputs = header_block.inputs().to_vec();
        let header_succs = info.succs.get(&header).cloned().unwrap_or_default();
        let in_loop_succs = header_succs
            .iter()
            .copied()
            .filter(|succ| loop_blocks.contains(succ))
            .collect_vec();
        let out_of_loop_succs = header_succs
            .iter()
            .copied()
            .filter(|succ| !loop_blocks.contains(succ))
            .collect_vec();

        let (kind, continue_case, break_case, body_start) = if !out_of_loop_succs.is_empty() {
            let continue_target = in_loop_succs.into_iter().exactly_one().map_err(|_| {
                RvsdgBuildError::UnsupportedLoop {
                    header,
                    reason: "header-controlled loop must have one in-loop successor".into(),
                }
            })?;
            let break_target = out_of_loop_succs.into_iter().exactly_one().map_err(|_| {
                RvsdgBuildError::UnsupportedLoop {
                    header,
                    reason: "header-controlled loop must have one exit successor".into(),
                }
            })?;
            let continue_case = header_succs
                .iter()
                .position(|succ| *succ == continue_target)
                .ok_or_else(|| RvsdgBuildError::UnsupportedLoop {
                    header,
                    reason: "header-controlled loop is missing the continue edge".into(),
                })?;
            let break_case = header_succs
                .iter()
                .position(|succ| *succ == break_target)
                .ok_or_else(|| RvsdgBuildError::UnsupportedLoop {
                    header,
                    reason: "header-controlled loop is missing the break edge".into(),
                })?;
            (
                LoopKind::HeaderControlled,
                continue_case,
                break_case,
                continue_target,
            )
        } else {
            let exit_source = exit_edges
                .iter()
                .map(|(src, _)| *src)
                .dedup()
                .exactly_one()
                .map_err(|_| RvsdgBuildError::UnsupportedLoop {
                    header,
                    reason: "tail-controlled loop must have one exit source".into(),
                })?;
            let latch_succs = info.succs.get(&exit_source).cloned().unwrap_or_default();
            let continue_case = latch_succs
                .iter()
                .position(|succ| *succ == header)
                .ok_or_else(|| RvsdgBuildError::UnsupportedLoop {
                    header,
                    reason: "loop latch is missing the backedge".into(),
                })?;
            let break_case = latch_succs
                .iter()
                .position(|succ| *succ == exit_target)
                .ok_or_else(|| RvsdgBuildError::UnsupportedLoop {
                    header,
                    reason: "loop latch is missing the exit edge".into(),
                })?;
            (LoopKind::TailControlled, continue_case, break_case, header)
        };

        let continue_row = block_successor_row(
            self.cfg_view,
            continuation_source(kind, header, backedge_source),
            continue_case,
        )
        .map_err(|reason| RvsdgBuildError::UnsupportedLoop { header, reason })?;
        let break_row = block_successor_row(
            self.cfg_view,
            continuation_source(kind, header, backedge_source),
            break_case,
        )
        .map_err(|reason| RvsdgBuildError::UnsupportedLoop { header, reason })?;

        let loop_vars = continue_row
            .iter()
            .cloned()
            .map(|continue_ty| ThetaLoopVar {
                input: self.fresh_var(continue_ty.clone()),
                pre: self.fresh_var(continue_ty.clone()),
                post: self.fresh_var(continue_ty),
            })
            .collect_vec();
        let outputs = break_row
            .iter()
            .cloned()
            .map(|ty| self.fresh_var(ty))
            .collect_vec();
        let body = Region {
            arguments: loop_vars.iter().map(|var| var.pre.clone()).collect(),
            body: self.build_scope(body_start, &loop_blocks, None, Some(header), info, cfg)?,
            results: loop_vars.iter().map(|var| var.post.clone()).collect(),
        };

        Ok((
            ThetaNode {
                inputs: header_inputs,
                outputs,
                kind,
                header: header_block,
                body,
                backedge_source,
                continue_case,
                break_case,
                loop_vars,
            },
            Some(exit_target),
        ))
    }
}

/// Ordered shared branch arguments for one branch case.
fn branch_arguments(
    match_rows: &[Vec<RegionVar>],
    entry_outputs: &[RegionVar],
    case_idx: usize,
) -> Vec<RegionVar> {
    match_rows[case_idx]
        .iter()
        .cloned()
        .chain(entry_outputs.iter().cloned())
        .collect()
}

/// Chooses the block that defines loop successor payload rows.
fn continuation_source(kind: LoopKind, header: Node, backedge_source: Node) -> Node {
    match kind {
        LoopKind::HeaderControlled => header,
        LoopKind::TailControlled => backedge_source,
    }
}

/// Returns the typed payload row emitted along one successor edge.
fn block_successor_row<H: HugrView<Node = Node>>(
    cfg_view: &H,
    node: Node,
    case_idx: usize,
) -> Result<TypeRow, String> {
    cfg_view
        .get_optype(node)
        .as_dataflow_block()
        .ok_or_else(|| format!("node {node} is not a dataflow block"))?
        .successor_input(case_idx)
        .ok_or_else(|| format!("successor case {case_idx} is out of range for node {node}"))
}

/// Deterministic reducible-CFG facts used during RVSDG construction.
struct CfgInfo {
    /// Reachable nodes that also reach the exit.
    scope: BTreeSet<Node>,
    /// Deterministic in-scope successor order per node.
    succs: BTreeMap<Node, Vec<Node>>,
    /// Immediate postdominator relation.
    ipostdom: BTreeMap<Node, Option<Node>>,
    /// Loop-header to backedge-source mapping.
    backedges: BTreeMap<Node, Vec<Node>>,
    /// Loop-header to natural-loop-block set mapping.
    loop_blocks: BTreeMap<Node, BTreeSet<Node>>,
}

impl CfgInfo {
    /// Computes the reducible-CFG facts needed by the RVSDG builder.
    fn new(cfg_root: Node, cfg: &impl CfgNodeMap<Node>) -> Result<Self, RvsdgBuildError<Node>> {
        let reachable = forward_reachable(cfg);
        let can_reach_exit = backward_reachable_from_exit(cfg);
        let scope = reachable
            .intersection(&can_reach_exit)
            .copied()
            .collect::<BTreeSet<_>>();
        let graph = build_graph(cfg, &scope);
        ensure_reducible(cfg_root, cfg, &scope, &graph)?;

        let entry = cfg.entry_node();
        let exit = cfg.exit_node();
        let doms = simple_fast(&graph, entry);
        let reversed = reverse_graph(&graph);
        let postdoms = simple_fast(&reversed, exit);

        let succs = scope
            .iter()
            .copied()
            .map(|node| {
                (
                    node,
                    cfg.successors(node)
                        .filter(|succ: &Node| scope.contains(succ))
                        .collect_vec(),
                )
            })
            .collect::<BTreeMap<_, _>>();
        let preds = scope
            .iter()
            .copied()
            .map(|node| {
                (
                    node,
                    cfg.predecessors(node)
                        .filter(|pred: &Node| scope.contains(pred))
                        .collect_vec(),
                )
            })
            .collect::<BTreeMap<_, _>>();

        let idom = scope
            .iter()
            .copied()
            .map(|node| (node, doms.immediate_dominator(node)))
            .collect::<BTreeMap<_, _>>();
        let ipostdom = scope
            .iter()
            .copied()
            .map(|node| (node, postdoms.immediate_dominator(node)))
            .collect::<BTreeMap<_, _>>();

        let backedges = succs
            .iter()
            .flat_map(|(&src, succs)| succs.iter().copied().map(move |dst| (src, dst)))
            .filter(|(src, dst)| dominates(*dst, *src, &idom))
            .fold(BTreeMap::<Node, Vec<Node>>::new(), |mut map, (src, dst)| {
                map.entry(dst).or_default().push(src);
                map
            });

        let loop_blocks = backedges
            .iter()
            .map(|(&header, sources)| {
                (
                    header,
                    natural_loop_blocks(header, sources, &preds, &idom, &scope),
                )
            })
            .collect::<BTreeMap<_, _>>();

        Ok(Self {
            scope,
            succs,
            ipostdom,
            backedges,
            loop_blocks,
        })
    }

    /// Returns in-scope successors visible from the current structured walk.
    fn scope_successors(
        &self,
        node: Node,
        scope: &BTreeSet<Node>,
        active_loop: Option<Node>,
    ) -> Vec<Node> {
        self.succs
            .get(&node)
            .into_iter()
            .flatten()
            .copied()
            .filter(|succ| scope.contains(succ))
            .filter(|succ| {
                active_loop.is_none_or(|header| !self.is_loop_backedge(node, *succ, header))
            })
            .collect()
    }

    /// Tells whether an edge is the suppressed backedge of the active loop.
    fn is_loop_backedge(&self, src: Node, dst: Node, header: Node) -> bool {
        dst == header
            && self
                .backedges
                .get(&header)
                .is_some_and(|sources| sources.contains(&src))
    }

    /// Tells whether a node should be structured as a nested loop in the current scope.
    fn is_nested_loop_header(
        &self,
        node: Node,
        scope: &BTreeSet<Node>,
        active_loop: Option<Node>,
    ) -> bool {
        active_loop != Some(node)
            && self
                .loop_blocks
                .get(&node)
                .is_some_and(|blocks| blocks.is_subset(scope))
    }

    /// Returns the join node for a structured branch.
    fn branch_join(
        &self,
        split_node: Node,
        scope: &BTreeSet<Node>,
        stop: Option<Node>,
    ) -> Result<Node, RvsdgBuildError<Node>> {
        let mut current = self.ipostdom.get(&split_node).copied().flatten();
        while let Some(join) = current {
            if join != split_node && scope.contains(&join) {
                return Ok(join);
            }
            current = self.ipostdom.get(&join).copied().flatten();
        }
        if let Some(stop) = stop.filter(|node| scope.contains(node)) {
            return Ok(stop);
        }
        Err(RvsdgBuildError::UnsupportedBranch {
            split: split_node,
            reason: "branch has no in-scope join".into(),
        })
    }

    /// Classifies how control should continue after a branch join.
    fn branch_continuation(
        &self,
        node: Node,
        scope: &BTreeSet<Node>,
        active_loop: Option<Node>,
    ) -> (BranchJoinKind, Option<Node>) {
        let successors = self.scope_successors(node, scope, active_loop);
        match successors.as_slice() {
            [] => (BranchJoinKind::Inline, None),
            [next] => (BranchJoinKind::Inline, Some(*next)),
            _ => (BranchJoinKind::Deferred, Some(node)),
        }
    }
}

/// Builds the scoped CFG graph used for dominance queries.
fn build_graph(cfg: &impl CfgNodeMap<Node>, scope: &BTreeSet<Node>) -> DiGraphMap<Node, ()> {
    let mut graph = DiGraphMap::new();
    for &node in scope {
        graph.add_node(node);
    }
    for &node in scope {
        for succ in cfg.successors(node) {
            if scope.contains(&succ) {
                graph.add_edge(node, succ, ());
            }
        }
    }
    graph
}

/// Reverses a graph for postdominator computation.
fn reverse_graph(graph: &DiGraphMap<Node, ()>) -> DiGraphMap<Node, ()> {
    let mut reversed = DiGraphMap::new();
    for node in graph.nodes() {
        reversed.add_node(node);
    }
    for (src, dst, _) in graph.all_edges() {
        reversed.add_edge(dst, src, ());
    }
    reversed
}

/// Returns all nodes reachable from the CFG entry.
fn forward_reachable(cfg: &impl CfgNodeMap<Node>) -> BTreeSet<Node> {
    let mut seen = BTreeSet::new();
    let mut pending = VecDeque::from([cfg.entry_node()]);
    while let Some(node) = pending.pop_front() {
        if !seen.insert(node) {
            continue;
        }
        pending.extend(cfg.successors(node));
    }
    seen
}

/// Returns all nodes that can reach the CFG exit.
fn backward_reachable_from_exit(cfg: &impl CfgNodeMap<Node>) -> BTreeSet<Node> {
    let mut seen = BTreeSet::new();
    let mut pending = VecDeque::from([cfg.exit_node()]);
    while let Some(node) = pending.pop_front() {
        if !seen.insert(node) {
            continue;
        }
        pending.extend(cfg.predecessors(node));
    }
    seen
}

/// Ensures the scoped CFG is reducible.
fn ensure_reducible(
    cfg_root: Node,
    cfg: &impl CfgNodeMap<Node>,
    scope: &BTreeSet<Node>,
    graph: &DiGraphMap<Node, ()>,
) -> Result<(), RvsdgBuildError<Node>> {
    for scc in kosaraju_scc(graph) {
        let cyclic = scc.len() > 1 || scc.iter().any(|node| graph.contains_edge(*node, *node));
        if !cyclic {
            continue;
        }
        let members = scc.into_iter().collect::<BTreeSet<_>>();
        let entries = members
            .iter()
            .copied()
            .filter(|node| {
                *node == cfg.entry_node()
                    || cfg
                        .predecessors(*node)
                        .any(|pred: Node| scope.contains(&pred) && !members.contains(&pred))
            })
            .collect_vec();
        if entries.len() > 1 {
            return Err(RvsdgBuildError::IrreducibleCfg {
                cfg: cfg_root,
                reason: format!("cyclic SCC has multiple entries: {:?}", entries),
            });
        }
    }
    Ok(())
}

/// Returns whether `dom` dominates `node`.
fn dominates(dom: Node, mut node: Node, idom: &BTreeMap<Node, Option<Node>>) -> bool {
    loop {
        if dom == node {
            return true;
        }
        match idom.get(&node).copied().flatten() {
            Some(parent) => node = parent,
            None => return false,
        }
    }
}

/// Computes the natural loop blocks for one loop header.
fn natural_loop_blocks(
    header: Node,
    sources: &[Node],
    preds: &BTreeMap<Node, Vec<Node>>,
    idom: &BTreeMap<Node, Option<Node>>,
    scope: &BTreeSet<Node>,
) -> BTreeSet<Node> {
    let mut blocks = BTreeSet::from([header]);
    let mut pending = sources.iter().copied().collect::<VecDeque<_>>();
    while let Some(node) = pending.pop_front() {
        if !scope.contains(&node) || !dominates(header, node, idom) || !blocks.insert(node) {
            continue;
        }
        pending.extend(
            preds
                .get(&node)
                .into_iter()
                .flatten()
                .copied()
                .filter(|pred| *pred != header),
        );
    }
    blocks
}
