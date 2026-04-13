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
#[cfg(test)]
use std::fmt;

use hugr::core::HugrNode;
use itertools::Itertools;
use petgraph::algo::kosaraju_scc;
use petgraph::graphmap::DiGraphMap;

use super::{CfgFactsError, CfgNodeMap};
#[cfg(test)]
use crate::control::CfgBlockMap;
#[cfg(test)]
use hugr::Node;
#[cfg(test)]
use std::collections::BTreeMap;

#[cfg(test)]
/// Graph node used by irreducibility preprocessing.
///
/// Preprocessing may duplicate part of an SCC to give each cyclic region a
/// unique entry. The duplicated graph nodes still refer back to one original
/// HUGR block so later stages can recover block summaries deterministically.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub(crate) enum PreprocessedNode<T> {
    /// One original CFG block.
    Original(T),
    /// One duplicated occurrence of an original CFG block.
    Duplicate {
        /// Original HUGR block represented by this duplicate.
        original: T,
        /// Deterministic duplicate identifier scoped to one preprocessed CFG.
        clone_id: usize,
    },
}

#[cfg(test)]
impl<T> fmt::Display for PreprocessedNode<T>
where
    T: HugrNode + fmt::Display,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Original(node) => fmt::Display::fmt(node, f),
            Self::Duplicate { original, clone_id } => write!(f, "{original}#{clone_id}"),
        }
    }
}

#[cfg(test)]
/// Graph-level preprocessing result for one CFG.
///
/// Unlike [`NormalizedCfg`], this view may contain duplicated graph nodes used
/// to split multi-entry SCCs into reducible regions. Each duplicated node still
/// maps back to an original HUGR block through [`CfgBlockMap`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct PreprocessedCfg<T> {
    /// Entry node of the preprocessed graph.
    entry: PreprocessedNode<T>,
    /// Exit node of the preprocessed graph.
    exit: PreprocessedNode<T>,
    /// Nodes preserved or introduced by preprocessing.
    scope: BTreeSet<PreprocessedNode<T>>,
    /// Deterministic outgoing edges.
    succs: BTreeMap<PreprocessedNode<T>, Vec<PreprocessedNode<T>>>,
    /// Deterministic incoming edges.
    preds: BTreeMap<PreprocessedNode<T>, Vec<PreprocessedNode<T>>>,
    /// Mapping back to original HUGR blocks.
    blocks: BTreeMap<PreprocessedNode<T>, T>,
}

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
        let scope = entry_to_exit_scope(cfg)?;
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

#[cfg(test)]
impl<T> PreprocessedCfg<T>
where
    T: HugrNode,
{
    /// Preprocesses one CFG into a reducible graph view.
    ///
    /// The current implementation uses deterministic node splitting on
    /// multi-entry SCCs: for every extra SCC entry, it duplicates the
    /// subgraph reachable from that entry until control rejoins another SCC
    /// entry. This is enough to express the resulting graph with unique-entry
    /// cyclic regions while keeping every graph node mapped back to one
    /// original HUGR block.
    ///
    /// # Errors
    ///
    /// Returns an error when the CFG has no entry-to-exit path, when reachable
    /// nodes do not all reach the exit, or when the current preprocessing
    /// strategy cannot eliminate all irreducible SCCs.
    pub(super) fn new(cfg_root: T, cfg: &impl CfgNodeMap<T>) -> Result<Self, CfgFactsError<T>> {
        let original_scope = entry_to_exit_scope(cfg)?;
        let entry = PreprocessedNode::Original(cfg.entry_node());
        let exit = PreprocessedNode::Original(cfg.exit_node());
        let mut scope = original_scope
            .iter()
            .copied()
            .map(PreprocessedNode::Original)
            .collect::<BTreeSet<_>>();
        let mut succs = scope
            .iter()
            .copied()
            .map(|node| {
                let original = original_node(node);
                (
                    node,
                    cfg.successors(original)
                        .filter(|succ| original_scope.contains(succ))
                        .map(PreprocessedNode::Original)
                        .collect::<Vec<_>>(),
                )
            })
            .collect::<BTreeMap<_, _>>();
        let mut blocks = scope
            .iter()
            .copied()
            .map(|node| (node, original_node(node)))
            .collect::<BTreeMap<_, _>>();
        let mut next_clone_id = 0;

        while let Some((members, entries)) = irreducible_scc(entry, &succs) {
            split_multi_entry_scc(
                &mut scope,
                &mut succs,
                &mut blocks,
                &mut next_clone_id,
                &members,
                &entries,
            );
        }

        if let Some((_, entries)) = irreducible_scc(entry, &succs) {
            return Err(CfgFactsError::Irreducible {
                cfg: cfg_root,
                entries: entries.into_iter().map(original_node).collect(),
            });
        }

        let preds = rebuild_predecessors(&scope, &succs);
        let preprocessed = Self {
            entry,
            exit,
            scope,
            succs,
            preds,
            blocks,
        };
        Ok(preprocessed)
    }

    /// Returns the nodes preserved or introduced by preprocessing.
    #[cfg(test)]
    pub(super) fn scope(&self) -> &BTreeSet<PreprocessedNode<T>> {
        &self.scope
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

#[cfg(test)]
impl<T> CfgNodeMap<PreprocessedNode<T>> for PreprocessedCfg<T>
where
    T: HugrNode,
{
    fn entry_node(&self) -> PreprocessedNode<T> {
        self.entry
    }

    fn exit_node(&self) -> PreprocessedNode<T> {
        self.exit
    }

    fn successors(&self, node: PreprocessedNode<T>) -> impl Iterator<Item = PreprocessedNode<T>> {
        self.succs.get(&node).into_iter().flatten().copied()
    }

    fn predecessors(&self, node: PreprocessedNode<T>) -> impl Iterator<Item = PreprocessedNode<T>> {
        self.preds.get(&node).into_iter().flatten().copied()
    }
}

#[cfg(test)]
impl CfgBlockMap<PreprocessedNode<Node>> for PreprocessedCfg<Node> {
    fn hugr_node(&self, node: PreprocessedNode<Node>) -> Node {
        self.blocks[&node]
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

/// Returns the entry-to-exit scope for a CFG view.
fn entry_to_exit_scope<T>(cfg: &impl CfgNodeMap<T>) -> Result<BTreeSet<T>, CfgFactsError<T>>
where
    T: HugrNode,
{
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

    Ok(scope)
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

/// Returns the first multi-entry cyclic SCC in deterministic node order.
#[cfg(test)]
fn irreducible_scc<T>(entry: T, succs: &BTreeMap<T, Vec<T>>) -> Option<(BTreeSet<T>, Vec<T>)>
where
    T: HugrNode,
{
    let graph = cfg_graph_from_successors(entry, succs);
    kosaraju_scc(&graph).into_iter().find_map(|scc| {
        let cyclic = scc.len() > 1 || scc.iter().any(|node| graph.contains_edge(*node, *node));
        if !cyclic {
            return None;
        }
        let members = scc.into_iter().collect::<BTreeSet<_>>();
        let entries = members
            .iter()
            .copied()
            .filter(|node| {
                *node == entry
                    || succs
                        .iter()
                        .filter(|(_, succs)| succs.contains(node))
                        .any(|(pred, _)| !members.contains(pred))
            })
            .collect_vec();
        (entries.len() > 1).then_some((members, entries))
    })
}

/// Splits a multi-entry SCC by cloning the subgraph owned by each extra entry.
#[cfg(test)]
fn split_multi_entry_scc<T>(
    scope: &mut BTreeSet<PreprocessedNode<T>>,
    succs: &mut BTreeMap<PreprocessedNode<T>, Vec<PreprocessedNode<T>>>,
    blocks: &mut BTreeMap<PreprocessedNode<T>, T>,
    next_clone_id: &mut usize,
    members: &BTreeSet<PreprocessedNode<T>>,
    entries: &[PreprocessedNode<T>],
) where
    T: HugrNode,
{
    let entry_set = entries.iter().copied().collect::<BTreeSet<_>>();
    for &extra_entry in &entries[1..] {
        let clone_set = clone_region(extra_entry, members, succs, &entry_set);
        let clones = clone_set
            .iter()
            .copied()
            .map(|node| {
                let clone = PreprocessedNode::Duplicate {
                    original: original_node(node),
                    clone_id: *next_clone_id,
                };
                *next_clone_id += 1;
                (node, clone)
            })
            .collect::<BTreeMap<_, _>>();

        for (&original, &clone) in &clones {
            scope.insert(clone);
            blocks.insert(clone, original_node(original));
        }

        for (&original, &clone) in &clones {
            let clone_succs = succs
                .get(&original)
                .into_iter()
                .flatten()
                .copied()
                .map(|succ| clones.get(&succ).copied().unwrap_or(succ))
                .collect::<Vec<_>>();
            succs.insert(clone, clone_succs);
        }

        for pred in predecessors_from_successors(extra_entry, succs) {
            if members.contains(&pred) {
                continue;
            }
            if let Some(pred_succs) = succs.get_mut(&pred) {
                for succ in pred_succs {
                    if *succ == extra_entry {
                        *succ = clones[&extra_entry];
                    }
                }
            }
        }
    }
}

/// Returns the region to clone for one extra SCC entry.
#[cfg(test)]
fn clone_region<T>(
    extra_entry: T,
    members: &BTreeSet<T>,
    succs: &BTreeMap<T, Vec<T>>,
    entries: &BTreeSet<T>,
) -> BTreeSet<T>
where
    T: HugrNode,
{
    let mut region = BTreeSet::new();
    let mut pending = VecDeque::from([extra_entry]);
    while let Some(node) = pending.pop_front() {
        if !members.contains(&node) || !region.insert(node) {
            continue;
        }
        for succ in succs.get(&node).into_iter().flatten().copied() {
            if members.contains(&succ) && (!entries.contains(&succ) || succ == extra_entry) {
                pending.push_back(succ);
            }
        }
    }
    region
}

/// Rebuilds predecessors from deterministic successors.
#[cfg(test)]
fn rebuild_predecessors<T>(scope: &BTreeSet<T>, succs: &BTreeMap<T, Vec<T>>) -> BTreeMap<T, Vec<T>>
where
    T: HugrNode,
{
    let mut preds = scope
        .iter()
        .copied()
        .map(|node| (node, Vec::new()))
        .collect::<BTreeMap<_, _>>();
    for (&src, dsts) in succs {
        for &dst in dsts {
            preds.entry(dst).or_default().push(src);
        }
    }
    preds
}

/// Returns predecessors of one node derived from the successor map.
#[cfg(test)]
fn predecessors_from_successors<T>(node: T, succs: &BTreeMap<T, Vec<T>>) -> Vec<T>
where
    T: HugrNode,
{
    succs
        .iter()
        .filter(|(_, succs)| succs.contains(&node))
        .map(|(&pred, _)| pred)
        .collect()
}

/// Returns the original HUGR block referenced by a preprocessed node.
#[cfg(test)]
fn original_node<T>(node: PreprocessedNode<T>) -> T
where
    T: HugrNode,
{
    match node {
        PreprocessedNode::Original(node) | PreprocessedNode::Duplicate { original: node, .. } => {
            node
        }
    }
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

/// Builds a graph from a deterministic successor map.
#[cfg(test)]
fn cfg_graph_from_successors<T>(entry: T, succs: &BTreeMap<T, Vec<T>>) -> DiGraphMap<T, ()>
where
    T: HugrNode,
{
    let mut graph = DiGraphMap::new();
    let mut pending = VecDeque::from([entry]);
    let mut seen = BTreeSet::new();
    while let Some(node) = pending.pop_front() {
        if !seen.insert(node) {
            continue;
        }
        graph.add_node(node);
        for &succ in succs.get(&node).into_iter().flatten() {
            graph.add_node(succ);
            graph.add_edge(node, succ, ());
            pending.push_back(succ);
        }
    }
    graph
}
