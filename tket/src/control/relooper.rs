//! Beyond-Relooper-style structural analysis for CFGs.
//!
//! This module implements the structured-control reconstruction algorithm
//! described in "Beyond Relooper" (OOPSLA 2022) and informed by GHC's
//! `FromCmm` implementation. The implementation here targets HUGR CFGs and
//! directly materializes the private shared lowering IR used by
//! [`crate::control::structuralize`], instead of first building a literal
//! WebAssembly AST.
//!
//! The important semantic difference from the original Emscripten-era
//! "relooper" is that this pass follows the paper's dominator/postdominator
//! driven reconstruction discipline for reducible CFGs. Irreducible CFGs still
//! require the Appendix A preprocessing step, which is intentionally out of
//! scope for this first implementation.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use hugr::HugrView;
use hugr::Node;
use itertools::Itertools;
use petgraph::algo::{dominators::simple_fast, kosaraju_scc};
use petgraph::graphmap::DiGraphMap;

use crate::control::structuralize::shared::{
    analyze_block, block_input_row, block_successor_payload, cfg_input_row, cfg_output_row,
};
use crate::control::structuralize::{
    RegionIo, StructuralizationError, StructuredBlock, StructuredBranchJoinKind,
    StructuredLoopKind, StructuredNode, StructuredRegion, StructuredRegionBody,
};
use crate::control::{CfgNodeMap, IdentityCfgMap};

/// Analyze one CFG using the Beyond-Relooper strategy.
///
/// The resulting structure is intentionally the same private lowering-oriented
/// region representation that the RVSDG path uses, so the HUGR rewrite and
/// lowering pipeline can be shared between strategies.
pub(crate) fn analyze_cfg<H: HugrView<Node = Node>>(
    cfg_view: &H,
    cfg: &IdentityCfgMap<H>,
) -> Result<StructuredRegion, StructuralizationError> {
    let info = CfgInfo::new(cfg_view.entrypoint(), cfg)?;
    let body = StructuredRegionBody::Sequence(info.build_scope(
        cfg_view,
        cfg.entry_node(),
        &info.scope,
        None,
        None,
    )?);
    Ok(StructuredRegion {
        io: RegionIo {
            inputs: cfg_input_row(cfg_view)?,
            outputs: cfg_output_row(cfg_view)?,
        },
        body,
    })
}

/// Cached CFG analyses used by the Beyond-Relooper traversal.
///
/// The traversal repeatedly queries the same successor order, dominance
/// relation, postdominator relation, and loop headers while recursively
/// structuring subscopes, so these facts are precomputed once and stored in a
/// deterministic form.
struct CfgInfo {
    scope: BTreeSet<Node>,
    succs: BTreeMap<Node, Vec<Node>>,
    ipostdom: BTreeMap<Node, Option<Node>>,
    backedges: BTreeMap<Node, Vec<Node>>,
    loop_blocks: BTreeMap<Node, BTreeSet<Node>>,
}

impl CfgInfo {
    /// Computes the deterministic graph facts needed by the Beyond traversal.
    ///
    /// # Errors
    ///
    /// Returns an error when the CFG contains reachable nodes that cannot reach
    /// the exit, when the CFG is irreducible and would need Appendix A
    /// preprocessing, or when a dominator/postdominator fact required by the
    /// reducible traversal cannot be derived.
    fn new(cfg_root: Node, cfg: &impl CfgNodeMap<Node>) -> Result<Self, StructuralizationError> {
        let reachable = forward_reachable(cfg);
        let can_reach_exit = backward_reachable_from_exit(cfg);
        let scope = reachable
            .intersection(&can_reach_exit)
            .copied()
            .collect::<BTreeSet<_>>();
        if scope.is_empty() {
            return Err(StructuralizationError::Relooper {
                reason: "cfg has no entry-to-exit path".into(),
            });
        }
        let dropped = reachable.difference(&can_reach_exit).copied().collect_vec();
        if !dropped.is_empty() {
            return Err(StructuralizationError::Relooper {
                reason: format!(
                    "reachable nodes do not all reach the CFG exit: {:?}",
                    dropped
                ),
            });
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
                let ordered = cfg
                    .successors(node)
                    .filter(|succ: &Node| scope.contains(succ))
                    .collect_vec();
                (node, ordered)
            })
            .collect::<BTreeMap<_, _>>();
        let preds = scope
            .iter()
            .copied()
            .map(|node| {
                let ordered = cfg
                    .predecessors(node)
                    .filter(|pred: &Node| scope.contains(pred))
                    .collect_vec();
                (node, ordered)
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

    /// Structures one linear scope until it reaches an explicit stop node.
    ///
    /// The walker emits straight-line blocks, nested branch regions, and nested
    /// loop regions while preserving CFG successor order. Nested loops are
    /// carved out before generic branching so loop headers are always lowered as
    /// loops rather than accidental multi-way branches.
    fn build_scope<H: HugrView<Node = Node>>(
        &self,
        cfg_view: &H,
        start: Node,
        scope: &BTreeSet<Node>,
        stop: Option<Node>,
        active_loop: Option<Node>,
    ) -> Result<Vec<StructuredNode>, StructuralizationError> {
        let mut items = Vec::new();
        let mut current = Some(start);
        let mut seen = BTreeSet::new();

        while let Some(node) = current {
            if Some(node) == stop || !scope.contains(&node) {
                break;
            }
            if !seen.insert(node) {
                return Err(StructuralizationError::Relooper {
                    reason: format!("scope walk revisited node {node}"),
                });
            }

            if self.is_nested_loop_header(node, scope, active_loop) {
                let (region, next) = self.build_loop_region(cfg_view, node, scope)?;
                items.push(StructuredNode::Region(Box::new(region)));
                current = next;
                continue;
            }

            let succs = self.scope_successors(node, scope, active_loop);
            if succs.len() > 1 {
                let (region, next) =
                    self.build_branch_region(cfg_view, node, scope, stop, active_loop)?;
                items.push(StructuredNode::Region(Box::new(region)));
                current = next;
                continue;
            }

            let block = analyze_block(cfg_view, node)?;
            let is_exit = matches!(block, StructuredBlock::Exit { .. });
            items.push(StructuredNode::Block(block));
            current = succs.into_iter().next();
            if is_exit {
                break;
            }
        }

        Ok(items)
    }

    /// Returns the in-scope successors visible from the current structured walk.
    ///
    /// When structuring a loop body, the loop-closing backedge is suppressed so
    /// one logical iteration can be materialized as a linear sequence ending at
    /// the latch or body tail.
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

    /// Tells whether the specified edge is the suppressed backedge of the active loop.
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

    /// Structures one reducible branch region rooted at a CFG split.
    ///
    /// The join node is taken from the postdominator chain, which is the
    /// Beyond-Relooper analogue of a block-followed-by-label target.
    fn build_branch_region<H: HugrView<Node = Node>>(
        &self,
        cfg_view: &H,
        split_node: Node,
        scope: &BTreeSet<Node>,
        stop: Option<Node>,
        active_loop: Option<Node>,
    ) -> Result<(StructuredRegion, Option<Node>), StructuralizationError> {
        let split = analyze_block(cfg_view, split_node)?;
        let join_node = self.branch_join(split_node, scope, stop)?;
        let join = analyze_block(cfg_view, join_node)?;
        let arms = self
            .scope_successors(split_node, scope, active_loop)
            .into_iter()
            .map(|succ| self.build_scope(cfg_view, succ, scope, Some(join_node), active_loop))
            .collect::<Result<Vec<_>, _>>()?;
        let (join_kind, next) = self.branch_continuation(join_node, scope, active_loop);

        Ok((
            StructuredRegion {
                io: RegionIo {
                    inputs: split.inputs().clone(),
                    outputs: join.inputs().clone(),
                },
                body: StructuredRegionBody::Branch {
                    split,
                    arms,
                    join,
                    join_kind,
                },
            },
            next,
        ))
    }

    /// Returns the join node for a structured branch.
    ///
    /// The first postdominator inside the current scope is the structured join
    /// used by the Beyond-Relooper traversal. When structuring an arm that is
    /// already bounded by an enclosing stop node, that stop node is allowed to
    /// act as the join.
    fn branch_join(
        &self,
        split_node: Node,
        scope: &BTreeSet<Node>,
        stop: Option<Node>,
    ) -> Result<Node, StructuralizationError> {
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
        Err(StructuralizationError::Relooper {
            reason: format!("branch at node {split_node} has no in-scope join"),
        })
    }

    /// Classifies how control should continue after a branch join.
    ///
    /// Most joins are plain blocks and can be lowered inside the branch region,
    /// after which the enclosing scope resumes with the join's single visible
    /// successor. If the join itself is the next split block, the branch region
    /// defers lowering that join and lets the enclosing scope resume directly
    /// at the join node.
    fn branch_continuation(
        &self,
        node: Node,
        scope: &BTreeSet<Node>,
        active_loop: Option<Node>,
    ) -> (StructuredBranchJoinKind, Option<Node>) {
        let successors = self.scope_successors(node, scope, active_loop);
        match successors.as_slice() {
            [] => (StructuredBranchJoinKind::Inline, None),
            [next] => (StructuredBranchJoinKind::Inline, Some(*next)),
            _ => (StructuredBranchJoinKind::Deferred, Some(node)),
        }
    }

    /// Structures one reducible loop rooted at its unique header.
    ///
    /// Tail-controlled and header-controlled loops are distinguished from the
    /// actual CFG edge pattern at the header. The body sequence itself is then
    /// built with the loop-closing backedge suppressed.
    fn build_loop_region<H: HugrView<Node = Node>>(
        &self,
        cfg_view: &H,
        header: Node,
        scope: &BTreeSet<Node>,
    ) -> Result<(StructuredRegion, Option<Node>), StructuralizationError> {
        let loop_blocks = self
            .loop_blocks
            .get(&header)
            .ok_or(StructuralizationError::Relooper {
                reason: format!("missing loop blocks for header {header}"),
            })?
            .intersection(scope)
            .copied()
            .collect::<BTreeSet<_>>();
        let exit_edges = loop_blocks
            .iter()
            .copied()
            .flat_map(|src| {
                self.succs
                    .get(&src)
                    .into_iter()
                    .flatten()
                    .copied()
                    .map(move |dst| (src, dst))
            })
            .filter(|(src, dst)| {
                !loop_blocks.contains(dst) && !self.is_loop_backedge(*src, *dst, header)
            })
            .collect_vec();
        let exit_target = exit_edges
            .iter()
            .map(|(_, dst)| *dst)
            .dedup()
            .exactly_one()
            .map_err(|_| StructuralizationError::Relooper {
                reason: format!("loop headed by {header} does not have a unique exit target"),
            })?;
        let backedge_source = self
            .backedges
            .get(&header)
            .into_iter()
            .flatten()
            .copied()
            .filter(|source| loop_blocks.contains(source))
            .exactly_one()
            .map_err(|_| StructuralizationError::Relooper {
                reason: format!("loop headed by {header} does not have a unique backedge source"),
            })?;
        let header_block = analyze_block(cfg_view, header)?;
        let header_succs = self.succs.get(&header).cloned().unwrap_or_default();
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

        let io = RegionIo {
            inputs: header_block.inputs().clone(),
            outputs: block_input_row(cfg_view, exit_target)?,
        };

        let body = if !out_of_loop_succs.is_empty() {
            let continue_target = in_loop_succs.into_iter().exactly_one().map_err(|_| {
                StructuralizationError::UnsupportedLoop {
                    reason: "header-controlled loop must have exactly one in-loop successor".into(),
                }
            })?;
            let break_target = out_of_loop_succs.into_iter().exactly_one().map_err(|_| {
                StructuralizationError::UnsupportedLoop {
                    reason: "header-controlled loop must have exactly one exit successor".into(),
                }
            })?;
            let continue_case = header_succs
                .iter()
                .position(|succ| *succ == continue_target)
                .ok_or(StructuralizationError::UnsupportedLoop {
                    reason: "header-controlled loop continue edge is missing".into(),
                })?;
            let break_case = header_succs
                .iter()
                .position(|succ| *succ == break_target)
                .ok_or(StructuralizationError::UnsupportedLoop {
                    reason: "header-controlled loop exit edge is missing".into(),
                })?;
            let continue_inputs = block_successor_payload(
                cfg_view,
                header,
                continue_case,
                "header-controlled loop continue case is out of range",
            )?;
            let break_outputs = block_successor_payload(
                cfg_view,
                header,
                break_case,
                "header-controlled loop exit case is out of range",
            )?;
            let body =
                self.build_scope(cfg_view, continue_target, &loop_blocks, None, Some(header))?;
            StructuredRegionBody::Loop {
                kind: StructuredLoopKind::HeaderControlled,
                header: header_block,
                body,
                backedge_source,
                continue_inputs,
                break_outputs,
                continue_case,
                break_case,
            }
        } else {
            let exit_source = exit_edges
                .iter()
                .map(|(src, _)| *src)
                .dedup()
                .exactly_one()
                .map_err(|_| StructuralizationError::UnsupportedLoop {
                    reason: "tail-controlled loop must have exactly one exit source".into(),
                })?;
            let latch_succs = self.succs.get(&exit_source).cloned().unwrap_or_default();
            let continue_case = latch_succs.iter().position(|succ| *succ == header).ok_or(
                StructuralizationError::UnsupportedLoop {
                    reason: "loop latch has no backedge to the header".into(),
                },
            )?;
            let break_case = latch_succs
                .iter()
                .position(|succ| *succ == exit_target)
                .ok_or(StructuralizationError::UnsupportedLoop {
                    reason: "loop latch has no exit edge".into(),
                })?;
            let continue_inputs = block_successor_payload(
                cfg_view,
                exit_source,
                continue_case,
                "tail-controlled loop continue case is out of range",
            )?;
            let break_outputs = block_successor_payload(
                cfg_view,
                exit_source,
                break_case,
                "tail-controlled loop exit case is out of range",
            )?;
            let body = self.build_scope(cfg_view, header, &loop_blocks, None, Some(header))?;
            StructuredRegionBody::Loop {
                kind: StructuredLoopKind::TailControlled,
                header: header_block,
                body,
                backedge_source,
                continue_inputs,
                break_outputs,
                continue_case,
                break_case,
            }
        };

        Ok((StructuredRegion { io, body }, Some(exit_target)))
    }
}

/// Builds a deterministic graph map over the scoped CFG nodes.
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

/// Builds the reversed graph used for postdominator computation.
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

/// Checks whether the scoped CFG is reducible.
///
/// Each cyclic SCC in a reducible CFG has a unique entry from outside the SCC.
/// If a cyclic SCC has multiple entries, the Beyond-Relooper implementation in
/// this module would require Appendix A preprocessing before translation.
fn ensure_reducible(
    cfg_root: Node,
    cfg: &impl CfgNodeMap<Node>,
    scope: &BTreeSet<Node>,
    graph: &DiGraphMap<Node, ()>,
) -> Result<(), StructuralizationError> {
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
            return Err(StructuralizationError::UnsupportedIrreducibleCfg {
                cfg: cfg_root,
                reason: format!("cyclic SCC has multiple entries: {:?}", entries),
            });
        }
    }
    Ok(())
}

/// Returns whether `dom` dominates `node` using an immediate-dominator chain.
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
