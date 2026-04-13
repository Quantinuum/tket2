//! Shared CFG facts used by control-structure reconstruction strategies.
//!
//! Both the Beyond-Relooper and RVSDG implementations start from the same
//! deterministic graph facts: the entry-to-exit scope, ordered successors and
//! predecessors, dominators, postdominators, backedges, and natural-loop
//! membership. Keeping those facts in one place avoids subtle drift between
//! the strategy implementations and gives later preprocessing passes a common
//! boundary.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use hugr::core::HugrNode;
use itertools::Itertools;
use petgraph::algo::{dominators::simple_fast, kosaraju_scc};
use petgraph::graphmap::DiGraphMap;

use super::CfgNodeMap;

/// Reducibility or reachability mismatch discovered while analyzing a CFG.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum CfgFactsError<T> {
    /// No path exists from the CFG entry to the CFG exit.
    NoEntryExitPath,
    /// Some nodes reachable from the entry do not reach the exit.
    ReachableNodesDoNotReachExit {
        /// Reachable nodes excluded from the entry-to-exit scope.
        dropped: Vec<T>,
    },
    /// A cyclic SCC has more than one entry from outside the SCC.
    Irreducible {
        /// CFG root whose scope failed reducibility.
        cfg: T,
        /// Entry blocks into the cyclic SCC.
        entries: Vec<T>,
    },
}

/// Deterministic graph facts shared by structuralization strategies.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct CfgFacts<T> {
    /// Reachable nodes that also reach the exit.
    pub(crate) scope: BTreeSet<T>,
    /// Deterministic in-scope successor order per node.
    pub(crate) succs: BTreeMap<T, Vec<T>>,
    /// Deterministic in-scope predecessor order per node.
    pub(crate) preds: BTreeMap<T, Vec<T>>,
    /// Immediate dominator relation.
    pub(crate) idom: BTreeMap<T, Option<T>>,
    /// Immediate postdominator relation.
    pub(crate) ipostdom: BTreeMap<T, Option<T>>,
    /// Loop-header to backedge-source mapping.
    pub(crate) backedges: BTreeMap<T, Vec<T>>,
    /// Loop-header to natural-loop-block set mapping.
    pub(crate) loop_blocks: BTreeMap<T, BTreeSet<T>>,
}

impl<T> CfgFacts<T>
where
    T: HugrNode,
{
    /// Computes deterministic CFG facts over the entry-to-exit scope.
    ///
    /// # Errors
    ///
    /// Returns an error when no entry-to-exit path exists, when some reachable
    /// nodes cannot reach the exit, or when the scoped CFG is irreducible.
    pub(crate) fn new(cfg_root: T, cfg: &impl CfgNodeMap<T>) -> Result<Self, CfgFactsError<T>> {
        let reachable = forward_reachable(cfg);
        let can_reach_exit = backward_reachable_from_exit(cfg);
        let scope = reachable
            .intersection(&can_reach_exit)
            .copied()
            .collect::<BTreeSet<_>>();
        if scope.is_empty() {
            return Err(CfgFactsError::NoEntryExitPath);
        }

        let dropped = reachable.difference(&can_reach_exit).copied().collect_vec();
        if !dropped.is_empty() {
            return Err(CfgFactsError::ReachableNodesDoNotReachExit { dropped });
        }

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
                        .filter(|succ: &T| scope.contains(succ))
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
                        .filter(|pred: &T| scope.contains(pred))
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
            .fold(BTreeMap::<T, Vec<T>>::new(), |mut map, (src, dst)| {
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
            preds,
            idom,
            ipostdom,
            backedges,
            loop_blocks,
        })
    }

    /// Returns the in-scope successors visible from the current structured walk.
    ///
    /// When structuring a loop body, the active loop's closing backedge is
    /// suppressed so a single logical iteration can be traversed linearly.
    pub(crate) fn scope_successors(
        &self,
        node: T,
        scope: &BTreeSet<T>,
        active_loop: Option<T>,
    ) -> Vec<T> {
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

    /// Returns whether the specified edge is the active loop's backedge.
    pub(crate) fn is_loop_backedge(&self, src: T, dst: T, header: T) -> bool {
        dst == header
            && self
                .backedges
                .get(&header)
                .is_some_and(|sources| sources.contains(&src))
    }

    /// Returns whether the node should be structured as a nested loop in this scope.
    pub(crate) fn is_nested_loop_header(
        &self,
        node: T,
        scope: &BTreeSet<T>,
        active_loop: Option<T>,
    ) -> bool {
        active_loop != Some(node)
            && self
                .loop_blocks
                .get(&node)
                .is_some_and(|blocks| blocks.is_subset(scope))
    }

    /// Returns the first in-scope postdominator suitable as a branch join.
    ///
    /// If the postdominator chain leaves the current scope, an explicit stop
    /// node may still serve as the join for the branch arm currently being
    /// structured.
    pub(crate) fn branch_join(
        &self,
        split_node: T,
        scope: &BTreeSet<T>,
        stop: Option<T>,
    ) -> Result<T, String> {
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
        Err("branch has no in-scope join".into())
    }
}

/// Builds the scoped CFG graph used for dominance queries.
fn build_graph<T>(cfg: &impl CfgNodeMap<T>, scope: &BTreeSet<T>) -> DiGraphMap<T, ()>
where
    T: HugrNode,
{
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

/// Reverses a graph so postdominators can be computed as dominators.
fn reverse_graph<T>(graph: &DiGraphMap<T, ()>) -> DiGraphMap<T, ()>
where
    T: HugrNode,
{
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
fn forward_reachable<T>(cfg: &impl CfgNodeMap<T>) -> BTreeSet<T>
where
    T: HugrNode,
{
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
fn backward_reachable_from_exit<T>(cfg: &impl CfgNodeMap<T>) -> BTreeSet<T>
where
    T: HugrNode,
{
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

/// Ensures the entry-to-exit scope is reducible.
fn ensure_reducible<T>(
    cfg_root: T,
    cfg: &impl CfgNodeMap<T>,
    scope: &BTreeSet<T>,
    graph: &DiGraphMap<T, ()>,
) -> Result<(), CfgFactsError<T>>
where
    T: HugrNode,
{
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
                        .any(|pred: T| scope.contains(&pred) && !members.contains(&pred))
            })
            .collect_vec();
        if entries.len() > 1 {
            return Err(CfgFactsError::Irreducible {
                cfg: cfg_root,
                entries,
            });
        }
    }
    Ok(())
}

/// Returns whether `dom` dominates `node`.
fn dominates<T>(dom: T, mut node: T, idom: &BTreeMap<T, Option<T>>) -> bool
where
    T: HugrNode,
{
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
fn natural_loop_blocks<T>(
    header: T,
    sources: &[T],
    preds: &BTreeMap<T, Vec<T>>,
    idom: &BTreeMap<T, Option<T>>,
    scope: &BTreeSet<T>,
) -> BTreeSet<T>
where
    T: HugrNode,
{
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
