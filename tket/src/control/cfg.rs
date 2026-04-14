//! Shared CFG facts used by control-structure reconstruction strategies.
//!
//! Both the Beyond-Relooper and RVSDG implementations start from the same
//! deterministic graph facts: the entry-to-exit scope, ordered successors and
//! predecessors, dominators, postdominators, backedges, and natural-loop
//! membership. Keeping those facts in one place avoids subtle drift between
//! the strategy implementations and gives later preprocessing passes a common
//! boundary.

mod preprocess;
#[cfg(test)]
mod test;

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use hugr::core::HugrNode;
use itertools::Itertools;
use petgraph::algo::dominators::simple_fast;
use petgraph::graphmap::DiGraphMap;

use super::CfgNodeMap;
use preprocess::NormalizedCfg;
pub(crate) use preprocess::{PreprocessedCfg, PreprocessedNode};

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
        let normalized = NormalizedCfg::new(cfg_root, cfg)?;
        let scope = normalized.scope().clone();
        let graph = normalized.graph();

        let entry = normalized.entry_node();
        let exit = normalized.exit_node();
        let doms = simple_fast(&graph, entry);
        let reversed = reverse_graph(&graph);
        let postdoms = simple_fast(&reversed, exit);

        let succs = scope
            .iter()
            .copied()
            .map(|node| (node, normalized.successors(node).collect::<Vec<_>>()))
            .collect::<BTreeMap<_, _>>();
        let preds = scope
            .iter()
            .copied()
            .map(|node| (node, normalized.predecessors(node).collect::<Vec<_>>()))
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

    /// Returns the in-scope successors visible from the current structured walk
    /// together with their original successor indices.
    pub(crate) fn scope_successor_cases(
        &self,
        node: T,
        scope: &BTreeSet<T>,
        active_loop: Option<T>,
    ) -> Vec<(usize, T)> {
        self.succs
            .get(&node)
            .into_iter()
            .flatten()
            .copied()
            .enumerate()
            .filter(|(_, succ)| scope.contains(succ))
            .filter(|(_, succ)| {
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
        active_loop: Option<T>,
    ) -> Result<T, String> {
        if let Some(join) = self.visible_branch_join(split_node, scope, active_loop) {
            return Ok(join);
        }
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

    /// Returns the earliest common join visible inside the current scope.
    ///
    /// This induced-graph search is important for loop-local branches whose
    /// visible join is hidden by an out-of-scope postdominator in the full CFG.
    fn visible_branch_join(
        &self,
        split_node: T,
        scope: &BTreeSet<T>,
        active_loop: Option<T>,
    ) -> Option<T> {
        let successors = self.scope_successors(split_node, scope, active_loop);
        let mut reachable_sets = successors
            .into_iter()
            .map(|succ| self.reachable_in_scope(succ, scope, active_loop))
            .collect_vec();
        let mut common = reachable_sets.pop()?;
        for reachable in reachable_sets {
            common = common.intersection(&reachable).copied().collect();
        }
        common.remove(&split_node);
        let candidates = common.iter().copied().collect_vec();
        candidates
            .iter()
            .copied()
            .filter(|candidate| {
                !candidates.iter().copied().any(|other| {
                    other != *candidate
                        && self
                            .reachable_in_scope(other, scope, active_loop)
                            .contains(candidate)
                })
            })
            .min()
    }

    /// Returns all nodes reachable from `start` while staying inside `scope`.
    fn reachable_in_scope(
        &self,
        start: T,
        scope: &BTreeSet<T>,
        active_loop: Option<T>,
    ) -> BTreeSet<T> {
        let mut stack = vec![start];
        let mut visited = BTreeSet::new();
        while let Some(node) = stack.pop() {
            if !visited.insert(node) {
                continue;
            }
            stack.extend(self.scope_successors(node, scope, active_loop));
        }
        visited
    }
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
