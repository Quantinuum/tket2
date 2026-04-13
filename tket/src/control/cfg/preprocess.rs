//! Shared CFG preprocessing and normalization.
//!
//! Structuralization strategies operate on the entry-to-exit portion of a CFG.
//! This module owns that normalization boundary: it trims unreachable control
//! paths, rejects malformed CFGs whose reachable nodes do not all reach the
//! exit, and enforces the current reducibility requirement before later stages
//! derive dominance and loop facts.
//!
//! The current implementation returns a filtered view over the original CFG.
//! Keeping that view explicit gives future irreducibility preprocessing a
//! stable place to introduce synthetic graph nodes without changing the fact
//! builder's external contract again.

use std::collections::{BTreeSet, VecDeque};

use hugr::core::HugrNode;
use itertools::Itertools;
use petgraph::algo::kosaraju_scc;
use petgraph::graphmap::DiGraphMap;

use super::{CfgFactsError, CfgNodeMap};

/// Entry-to-exit normalized view of one CFG.
///
/// The view keeps only nodes that are reachable from the entry and can reach
/// the exit. Successor and predecessor iteration is filtered through that
/// scope, so later stages can treat the CFG as if dead control paths did not
/// exist.
pub(super) struct NormalizedCfg<'a, T, C> {
    /// Original CFG view.
    cfg: &'a C,
    /// Nodes preserved by normalization.
    scope: BTreeSet<T>,
}

impl<'a, T, C> NormalizedCfg<'a, T, C>
where
    T: HugrNode,
    C: CfgNodeMap<T>,
{
    /// Builds the normalized entry-to-exit view for one CFG.
    ///
    /// # Errors
    ///
    /// Returns an error when the CFG has no entry-to-exit path, when some
    /// reachable nodes cannot reach the exit, or when the scoped CFG is
    /// irreducible under the current preprocessing boundary.
    pub(super) fn new(cfg_root: T, cfg: &'a C) -> Result<Self, CfgFactsError<T>> {
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

        let normalized = Self { cfg, scope };
        ensure_reducible(cfg_root, &normalized)?;
        Ok(normalized)
    }

    /// Returns the nodes preserved by normalization.
    pub(super) fn scope(&self) -> &BTreeSet<T> {
        &self.scope
    }

    /// Builds the scoped graph used for dominance and SCC queries.
    pub(super) fn graph(&self) -> DiGraphMap<T, ()> {
        let mut graph = DiGraphMap::new();
        for &node in &self.scope {
            graph.add_node(node);
        }
        for &node in &self.scope {
            for succ in self.successors(node) {
                graph.add_edge(node, succ, ());
            }
        }
        graph
    }
}

impl<T, C> CfgNodeMap<T> for NormalizedCfg<'_, T, C>
where
    T: HugrNode,
    C: CfgNodeMap<T>,
{
    fn entry_node(&self) -> T {
        self.cfg.entry_node()
    }

    fn exit_node(&self) -> T {
        self.cfg.exit_node()
    }

    fn successors(&self, node: T) -> impl Iterator<Item = T> {
        self.cfg
            .successors(node)
            .filter(move |succ| self.scope.contains(succ))
    }

    fn predecessors(&self, node: T) -> impl Iterator<Item = T> {
        self.cfg
            .predecessors(node)
            .filter(move |pred| self.scope.contains(pred))
    }
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

/// Ensures the normalized CFG is reducible.
///
/// This is the current policy boundary. A later preprocessing pass can replace
/// this rejection with a graph rewrite that re-expresses irreducible control in
/// a reducible form before the strategies consume the normalized view.
fn ensure_reducible<T>(cfg_root: T, cfg: &impl CfgNodeMap<T>) -> Result<(), CfgFactsError<T>>
where
    T: HugrNode,
{
    let graph = cfg_graph(cfg);
    for scc in kosaraju_scc(&graph) {
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
                    || cfg.predecessors(*node).any(|pred| !members.contains(&pred))
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

/// Builds a graph from the nodes visible through a normalized CFG view.
fn cfg_graph<T>(cfg: &impl CfgNodeMap<T>) -> DiGraphMap<T, ()>
where
    T: HugrNode,
{
    let mut graph = DiGraphMap::new();
    let mut pending = VecDeque::from([cfg.entry_node()]);
    let mut seen = BTreeSet::new();
    while let Some(node) = pending.pop_front() {
        if !seen.insert(node) {
            continue;
        }
        graph.add_node(node);
        for succ in cfg.successors(node) {
            graph.add_node(succ);
            graph.add_edge(node, succ, ());
            pending.push_back(succ);
        }
    }
    graph
}
