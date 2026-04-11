//! RVSDG-oriented structural analysis for CFGs.
//!
//! The Regionalized Value State Dependence Graph (RVSDG) is a region-based IR
//! that makes control flow explicit using nested branch and loop regions while
//! keeping data and state dependencies as graph edges. In practice, that means
//! we want a structural view of a CFG that answers questions like:
//!
//! - which blocks form a structured branch region?
//! - which blocks form a loop body?
//! - in what order do linear blocks and structured regions execute?
//!
//! This module provides a baseline for that first stage. It currently builds:
//!
//! - a nested SESE region graph over a reducible CFG
//! - an ordered control-region tree that preserves sequencing as well as
//!   nesting, and records the block set and control boundary of each region
//!
//! A later step can lower this structural form into HUGR `Conditional` and
//! `TailLoop` nodes, and from there into a fuller RVSDG-style representation.
//!
//! Relevant background:
//!
//! - H. Bahmann, N. Reissmann, M. Jahre, and J. C. Meyer, "Perfect
//!   Reconstructability of Control Flow from Demand Dependence Graphs",
//!   ACM TACO 12(4), 2015. <https://doi.org/10.1145/2693261>
//! - N. Reissmann, J. C. Meyer, H. Bahmann, and M. Sj\"alander, "RVSDG:
//!   An Intermediate Representation for Optimizing Compilers", ACM TECS 19(6),
//!   2020. <https://doi.org/10.1145/3391902>

use std::collections::{HashMap, HashSet, VecDeque};

use derive_more::{Display, Error};
use hugr::core::HugrNode;

use super::interface::RegionInterface;
use super::{CfgNodeMap, edge_classes, region_blocks};

/// A handle identifying a region in a [`RegionGraph`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct RegionId(usize);

/// A structural classification for a CFG region.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum RegionKind {
    /// The synthetic root region that covers the whole CFG.
    Root,
    /// A straight-line region with no internal control split or backedge.
    Linear,
    /// A branching region with a structured split/join.
    Branch,
    /// A looping region containing an internal backedge to its entry.
    Loop,
}

/// A node within an ordered structured-control tree.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ControlTree<T: HugrNode> {
    /// A single basic block in the control tree.
    Block(T),
    /// A nested structured control region.
    Region(Box<ControlRegion<T>>),
}

/// The ordered contents of a structured control region.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ControlRegionBody<T: HugrNode> {
    /// A linear sequence of control-tree nodes.
    Sequence(Vec<ControlTree<T>>),
    /// A structured branch with split blocks, per-arm bodies, and join blocks.
    Branch {
        /// Linear blocks executed before dispatching into the arms.
        split: Vec<T>,
        /// The branch arms.
        arms: Vec<Vec<ControlTree<T>>>,
        /// Linear blocks executed after the arms rejoin.
        join: Vec<T>,
    },
    /// A structured loop body.
    Loop {
        /// The control tree for one iteration of the loop body.
        body: Vec<ControlTree<T>>,
    },
}

/// A structured control region together with its control interface.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ControlRegion<T: HugrNode> {
    /// The structural kind of the region.
    pub kind: RegionKind,
    /// All CFG blocks covered by this region.
    pub blocks: HashSet<T>,
    /// The region's control boundary.
    pub boundary: RegionBoundary<T>,
    /// Canonicalized block-level interface derived from boundary edges.
    pub interface: RegionInterface<T>,
    /// The ordered contents of the region.
    pub body: ControlRegionBody<T>,
}

/// The control interface of a region, expressed as control-flow edges crossing
/// its boundary.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RegionBoundary<T> {
    /// Edges entering the region from outside it.
    pub incoming: Vec<(T, T)>,
    /// Edges leaving the region to nodes outside it.
    pub outgoing: Vec<(T, T)>,
}

/// Errors from building RVSDG structural analyses.
#[derive(Clone, Debug, Display, Error)]
#[non_exhaustive]
pub enum RvsdgBuildError<T: HugrNode> {
    /// Region boundaries did not define a valid SESE region.
    #[display("invalid SESE region boundaries for entry edge {entry:?} and exit edge {exit:?}")]
    InvalidRegionBlocks {
        /// Claimed entry edge for the SESE region.
        entry: (T, T),
        /// Claimed exit edge for the SESE region.
        exit: (T, T),
    },
    /// A branch region did not expose a join component.
    #[display("branch region {region:?} did not expose a join component")]
    MissingBranchJoin {
        /// Region whose branch join could not be identified.
        region: RegionId,
    },
    /// Structured traversal re-visited a component while building a path.
    #[display("control-tree path revisited component in region {region:?}")]
    CyclicPath {
        /// Region where component traversal re-visited a node.
        region: RegionId,
    },
    /// Expected a linearized path but found multiple successors.
    #[display("expected a linearized path in region {region:?}")]
    NonLinearPath {
        /// Region where linear path recovery found multiple successors.
        region: RegionId,
    },
    /// Expected a basic block component but found a nested region.
    #[display("expected a basic block component while building branch region {region:?}")]
    ExpectedBlock {
        /// Branch region where split/join component was unexpectedly nested.
        region: RegionId,
    },
}

/// A SESE region discovered in a CFG.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Region<T: HugrNode> {
    /// The kind of structural region.
    pub kind: RegionKind,
    /// The edge entering the region, if any.
    pub entry_edge: Option<(T, T)>,
    /// The edge leaving the region, if any.
    pub exit_edge: Option<(T, T)>,
    /// All CFG nodes contained in the region.
    pub blocks: HashSet<T>,
    /// The control boundary of the region.
    pub boundary: RegionBoundary<T>,
    /// The immediate child regions nested within this region.
    pub children: Vec<RegionId>,
}

/// A structural region hierarchy for a CFG.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RegionGraph<T: HugrNode> {
    root: RegionId,
    regions: Vec<Region<T>>,
}

impl<T: HugrNode> RegionGraph<T> {
    /// Returns the root region.
    pub fn root(&self) -> RegionId {
        self.root
    }

    /// Returns the region with the given identifier.
    pub fn region(&self, id: RegionId) -> &Region<T> {
        &self.regions[id.0]
    }

    /// Returns all regions in storage order.
    pub fn regions(&self) -> &[Region<T>] {
        &self.regions
    }
}

/// Build a structural region graph for a CFG view.
pub fn build_region_graph<T: HugrNode>(
    cfg: &impl CfgNodeMap<T>,
) -> Result<RegionGraph<T>, RvsdgBuildError<T>> {
    let mut descs = collect_regions(cfg)?;
    let root_blocks = root_blocks(cfg);

    let mut parents = vec![None; descs.len()];
    for i in 0..descs.len() {
        let mut best_parent = None;
        for j in 0..descs.len() {
            if i == j {
                continue;
            }
            if !descs[i].blocks.is_subset(&descs[j].blocks) || descs[i].blocks == descs[j].blocks {
                continue;
            }
            let candidate = descs[j].blocks.len();
            match best_parent {
                Some((_, best_size)) if best_size <= candidate => {}
                _ => best_parent = Some((j, candidate)),
            }
        }
        parents[i] = best_parent.map(|(j, _)| j);
    }

    let root = RegionId(0);
    let mut regions = Vec::with_capacity(descs.len() + 1);
    regions.push(Region {
        kind: RegionKind::Root,
        entry_edge: None,
        exit_edge: None,
        blocks: root_blocks,
        boundary: region_boundary(cfg, &regions_root_desc(cfg)),
        children: Vec::new(),
    });
    for desc in &mut descs {
        regions.push(Region {
            kind: classify_region(cfg, desc),
            entry_edge: Some(desc.entry_edge),
            exit_edge: Some(desc.exit_edge),
            boundary: region_boundary(cfg, desc),
            blocks: std::mem::take(&mut desc.blocks),
            children: Vec::new(),
        });
    }

    for (idx, parent) in parents.into_iter().enumerate() {
        let child_id = RegionId(idx + 1);
        let parent_id = parent.map_or(root, |p| RegionId(p + 1));
        regions[parent_id.0].children.push(child_id);
    }

    Ok(RegionGraph { root, regions })
}

/// Build an ordered control-region tree for a CFG view.
pub fn build_control_tree<T: HugrNode>(
    cfg: &impl CfgNodeMap<T>,
) -> Result<ControlRegion<T>, RvsdgBuildError<T>> {
    let regions = build_region_graph(cfg)?;
    build_region_tree(cfg, &regions, regions.root())
}

/// Minimal description of a SESE region while we are still discovering the
/// nesting structure.
///
/// `RegionGraph` needs parent/child relationships between regions, which in
/// turn depend on each region's block set. We therefore keep this lightweight
/// description around until nesting has been computed, and only then materialize
/// full [`Region`] values.
#[derive(Clone, Debug)]
struct RegionDesc<T> {
    entry_edge: (T, T),
    exit_edge: (T, T),
    blocks: HashSet<T>,
}

/// Collects the raw SESE regions implied by cycle-equivalent edge classes.
///
/// This is the "discovery" phase of the structural analysis. It intentionally
/// does not try to assign parents, classify loops vs. branches, or build an
/// ordered tree. Its job is only to answer: "which edge pairs delimit valid
/// SESE regions, and which blocks do those regions contain?"
fn collect_regions<T: HugrNode>(
    cfg: &impl CfgNodeMap<T>,
) -> Result<Vec<RegionDesc<T>>, RvsdgBuildError<T>> {
    let edge_classes = edge_classes(cfg);
    let mut rem_edges: HashMap<usize, HashSet<(T, T)>> = HashMap::new();
    let mut regions = Vec::new();
    for (edge, class) in &edge_classes {
        rem_edges.entry(*class).or_default().insert(*edge);
    }

    collect_regions_traverse(
        cfg,
        cfg.entry_node(),
        &edge_classes,
        &mut rem_edges,
        None,
        &mut regions,
    )?;
    Ok(regions)
}

/// Returns the unique exit edge found while traversing a candidate region.
///
/// The traversal logic is structured so a well-formed SESE subproblem either
/// has no exit edge yet or repeatedly rediscovers the same exit edge. Collapsing
/// those duplicates here keeps the traversal code simple while still asserting
/// the structural invariant we rely on.
fn unique_edge<T: HugrNode>(edges: Vec<(T, T)>) -> Option<(T, T)> {
    let first = *edges.first()?;
    assert!(edges.iter().all(|edge| *edge == first));
    Some(first)
}

/// Traverses the CFG in a SESE-respecting order and extracts nested regions.
///
/// `stop_at` is the cycle-equivalence class whose matching edge should terminate
/// the current recursive subproblem. Whenever traversal re-enters the same edge
/// class before it has been exhausted, the code has discovered a nested region
/// delimited by two edges from that class. Recording regions in this order means
/// all descendants are discovered before their ancestors are later nested.
fn collect_regions_traverse<T: HugrNode>(
    cfg: &impl CfgNodeMap<T>,
    n: T,
    edge_classes: &HashMap<(T, T), usize>,
    rem_edges: &mut HashMap<usize, HashSet<(T, T)>>,
    stop_at: Option<usize>,
    regions: &mut Vec<RegionDesc<T>>,
) -> Result<Option<(T, T)>, RvsdgBuildError<T>> {
    let mut seen = HashSet::new();
    let mut stack = vec![n];
    let mut exit_edges = Vec::new();

    while let Some(node) = stack.pop() {
        if !seen.insert(node) {
            continue;
        }

        let mut exits = Vec::new();
        let mut rest = Vec::new();
        for succ in cfg.successors(node) {
            let edge = (node, succ);
            if stop_at.is_some() && edge_classes.get(&edge).copied() == stop_at {
                exits.push(edge);
            } else {
                rest.push(edge);
            }
        }
        exit_edges.extend(exits);

        for mut edge in rest {
            if let Some(class) = edge_classes.get(&edge).copied() {
                assert!(rem_edges.get_mut(&class).unwrap().remove(&edge));
                while !rem_edges.get(&class).unwrap().is_empty() {
                    let prev_edge = edge;
                    edge = collect_regions_traverse(
                        cfg,
                        edge.1,
                        edge_classes,
                        rem_edges,
                        Some(class),
                        regions,
                    )?
                    .expect("region traversal should find an exit edge");
                    assert!(rem_edges.get_mut(&class).unwrap().remove(&edge));
                    let blocks = region_blocks(cfg, prev_edge, edge).map_err(|_| {
                        RvsdgBuildError::InvalidRegionBlocks {
                            entry: prev_edge,
                            exit: edge,
                        }
                    })?;
                    regions.push(RegionDesc {
                        entry_edge: prev_edge,
                        exit_edge: edge,
                        blocks,
                    });
                }
            }
            stack.push(edge.1);
        }
    }

    Ok(unique_edge(exit_edges))
}

/// Computes the block set for the synthetic root region.
///
/// The underlying SESE algorithm assumes every node lies on a path from entry to
/// exit. HUGR CFGs are only guaranteed to be reachable from entry, so we first
/// prune nodes that cannot reach the exit and then keep the forward-reachable
/// subset. That matches the restriction already used by the nesting code and
/// gives the root region a meaningful block set even in the presence of
/// exit-free loops.
fn root_blocks<T: HugrNode>(cfg: &impl CfgNodeMap<T>) -> HashSet<T> {
    let mut can_reach_exit = HashSet::new();
    let mut pending = VecDeque::from([cfg.exit_node()]);
    while let Some(node) = pending.pop_front() {
        if can_reach_exit.insert(node) {
            pending.extend(cfg.predecessors(node));
        }
    }

    let mut root = HashSet::new();
    let mut worklist = VecDeque::from([cfg.entry_node()]);
    while let Some(node) = worklist.pop_front() {
        if root.insert(node) {
            for succ in cfg.successors(node) {
                if can_reach_exit.contains(&succ) {
                    worklist.push_back(succ);
                }
            }
        }
    }
    root
}

/// Classifies a discovered SESE region by its internal control shape.
///
/// The classification is intentionally simple because it only needs to support
/// the current structural skeleton: backedges to the entry imply a loop, a
/// second in-region successor from the entry implies a branch, and everything
/// else is treated as linear. This is enough to drive the ordered tree shape we
/// later lower into more RVSDG-like structures.
fn classify_region<T: HugrNode>(cfg: &impl CfgNodeMap<T>, desc: &RegionDesc<T>) -> RegionKind {
    let entry = desc.entry_edge.1;
    let has_backedge = desc
        .blocks
        .iter()
        .copied()
        .any(|node| cfg.successors(node).any(|succ| succ == entry));
    if has_backedge {
        return RegionKind::Loop;
    }

    if cfg
        .successors(entry)
        .filter(|succ| desc.blocks.contains(succ))
        .nth(1)
        .is_some()
    {
        RegionKind::Branch
    } else {
        RegionKind::Linear
    }
}

/// Computes the control edges that cross a region boundary.
///
/// We record these explicitly because later RVSDG steps care about region
/// interfaces, not only nested block sets. Sorting and deduplicating the edges
/// gives deterministic results and avoids exposing repeated edges caused by
/// iterating over every block in the region.
fn region_boundary<T: HugrNode>(
    cfg: &impl CfgNodeMap<T>,
    desc: &RegionDesc<T>,
) -> RegionBoundary<T> {
    let mut incoming = desc
        .blocks
        .iter()
        .copied()
        .flat_map(|dst| {
            cfg.predecessors(dst)
                .filter(move |src| !desc.blocks.contains(src))
                .map(move |src| (src, dst))
        })
        .collect::<Vec<_>>();
    let mut outgoing = desc
        .blocks
        .iter()
        .copied()
        .flat_map(|src| {
            cfg.successors(src)
                .filter(move |dst| !desc.blocks.contains(dst))
                .map(move |dst| (src, dst))
        })
        .collect::<Vec<_>>();
    incoming.sort();
    incoming.dedup();
    outgoing.sort();
    outgoing.dedup();
    RegionBoundary { incoming, outgoing }
}

/// Synthesizes a descriptor for the root region so it can share the same
/// boundary-computation path as discovered SESE regions.
///
/// The root has no real entry/exit edges in the CFG, so we reuse the
/// exit-to-entry pseudo-edge that the SESE analysis already treats as the CFG's
/// enclosing outer boundary.
fn regions_root_desc<T: HugrNode>(cfg: &impl CfgNodeMap<T>) -> RegionDesc<T> {
    RegionDesc {
        entry_edge: (cfg.exit_node(), cfg.entry_node()),
        exit_edge: (cfg.exit_node(), cfg.entry_node()),
        blocks: root_blocks(cfg),
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum Component<T> {
    Block(T),
    Region(RegionId),
}

/// Converts a nested [`RegionGraph`] region into an ordered control region.
///
/// `RegionGraph` captures containment but not execution order. This function is
/// the bridge between those two representations: it chooses the appropriate
/// ordering strategy for each region kind, while also copying across the block
/// set and region boundary that later passes will need.
fn build_region_tree<T: HugrNode>(
    cfg: &impl CfgNodeMap<T>,
    graph: &RegionGraph<T>,
    region_id: RegionId,
) -> Result<ControlRegion<T>, RvsdgBuildError<T>> {
    let region = graph.region(region_id);
    match region.kind {
        RegionKind::Root | RegionKind::Linear => Ok(ControlRegion {
            kind: region.kind,
            blocks: region.blocks.clone(),
            boundary: region.boundary.clone(),
            interface: RegionInterface::from_boundary(&region.boundary),
            body: ControlRegionBody::Sequence(build_sequence_items(
                cfg,
                graph,
                region_id,
                sequence_start(cfg, graph, region_id),
                sequence_end(cfg, graph, region_id),
                None,
            )?),
        }),
        RegionKind::Loop => Ok(ControlRegion {
            kind: region.kind,
            blocks: region.blocks.clone(),
            boundary: region.boundary.clone(),
            interface: RegionInterface::from_boundary(&region.boundary),
            body: ControlRegionBody::Loop {
                body: build_sequence_items(
                    cfg,
                    graph,
                    region_id,
                    sequence_start(cfg, graph, region_id),
                    sequence_end(cfg, graph, region_id),
                    region
                        .entry_edge
                        .zip(region.exit_edge)
                        .map(|(entry, exit)| (exit.0, entry.1)),
                )?,
            },
        }),
        RegionKind::Branch => build_branch_tree(cfg, graph, region_id),
    }
}

/// Builds the ordered representation of a structured branch region.
///
/// Branch regions are the one place where containment alone is not enough:
/// reviewers need to see which nodes execute before the split, which nodes
/// belong to each arm, and which nodes execute after the join. The compressed
/// graph turns nested child regions into single components so each arm can be
/// recovered as a linear path from the split component to the join component.
fn build_branch_tree<T: HugrNode>(
    cfg: &impl CfgNodeMap<T>,
    graph: &RegionGraph<T>,
    region_id: RegionId,
) -> Result<ControlRegion<T>, RvsdgBuildError<T>> {
    let compressed = CompressedRegion::new(cfg, graph, region_id, None);
    let split = sequence_start(cfg, graph, region_id);
    let join = sequence_end(cfg, graph, region_id)
        .ok_or(RvsdgBuildError::MissingBranchJoin { region: region_id })?;

    let arms = compressed
        .successors(split)
        .into_iter()
        .map(|start| build_path_items(cfg, graph, &compressed, start, Some(join), false, region_id))
        .collect::<Result<Vec<_>, _>>()?;

    let region = graph.region(region_id);
    Ok(ControlRegion {
        kind: region.kind,
        blocks: region.blocks.clone(),
        boundary: region.boundary.clone(),
        interface: RegionInterface::from_boundary(&region.boundary),
        body: ControlRegionBody::Branch {
            split: vec![expect_block(split, region_id)?],
            arms,
            join: vec![expect_block(join, region_id)?],
        },
    })
}

/// Builds a linear sequence of control-tree items inside a region.
///
/// Root, linear, and loop bodies all boil down to "walk a single path through
/// this region." `ignored_edge` allows loop bodies to suppress the latch edge so
/// one iteration can be represented as a straight-line sequence rather than a
/// cyclic graph.
fn build_sequence_items<T: HugrNode>(
    cfg: &impl CfgNodeMap<T>,
    graph: &RegionGraph<T>,
    region_id: RegionId,
    start: Component<T>,
    end: Option<Component<T>>,
    ignored_edge: Option<(T, T)>,
) -> Result<Vec<ControlTree<T>>, RvsdgBuildError<T>> {
    let compressed = CompressedRegion::new(cfg, graph, region_id, ignored_edge);
    build_path_items(cfg, graph, &compressed, start, end, true, region_id)
}

/// Walks a linearized path through a compressed region and materializes tree items.
///
/// The key invariant is that after child regions have been collapsed to single
/// components, structured regions should expose at most one forward successor
/// along the path we are building. If that stops being true, the current
/// structural model is insufficient and we would rather fail loudly than emit an
/// ambiguous tree.
fn build_path_items<T: HugrNode>(
    cfg: &impl CfgNodeMap<T>,
    graph: &RegionGraph<T>,
    compressed: &CompressedRegion<T>,
    start: Component<T>,
    end: Option<Component<T>>,
    include_end: bool,
    region_id: RegionId,
) -> Result<Vec<ControlTree<T>>, RvsdgBuildError<T>> {
    let mut items = Vec::new();
    let mut current = Some(start);
    let mut seen = HashSet::new();

    while let Some(component) = current {
        if Some(component) == end && !include_end {
            break;
        }
        if !seen.insert(component) {
            return Err(RvsdgBuildError::CyclicPath { region: region_id });
        }
        items.push(component_to_tree(cfg, graph, component)?);
        if Some(component) == end {
            break;
        }
        let succs = compressed.successors(component);
        current = match succs.as_slice() {
            [] => None,
            [next] => Some(*next),
            _ => return Err(RvsdgBuildError::NonLinearPath { region: region_id }),
        };
    }

    Ok(items)
}

/// Converts a compressed-region component back into the public tree form.
///
/// The compressed traversal treats nested regions like single vertices so path
/// recovery stays simple. This helper is the point where those placeholders are
/// expanded back into recursively structured subtrees.
fn component_to_tree<T: HugrNode>(
    cfg: &impl CfgNodeMap<T>,
    graph: &RegionGraph<T>,
    component: Component<T>,
) -> Result<ControlTree<T>, RvsdgBuildError<T>> {
    match component {
        Component::Block(node) => Ok(ControlTree::Block(node)),
        Component::Region(region_id) => Ok(ControlTree::Region(Box::new(build_region_tree(
            cfg, graph, region_id,
        )?))),
    }
}

/// Returns the first component that executes within a region.
///
/// The root region starts at the CFG entry block. Every other region starts at
/// the destination of its entry edge, because that is the first block strictly
/// inside the region boundary.
fn sequence_start<T: HugrNode>(
    cfg: &impl CfgNodeMap<T>,
    graph: &RegionGraph<T>,
    region_id: RegionId,
) -> Component<T> {
    match graph.region(region_id).kind {
        RegionKind::Root => Component::Block(cfg.entry_node()),
        RegionKind::Linear | RegionKind::Loop | RegionKind::Branch => {
            Component::Block(graph.region(region_id).entry_edge.unwrap().1)
        }
    }
}

/// Returns the last component that executes within a region.
///
/// For non-root regions this is the source of the exit edge, which is the final
/// block still inside the region before control leaves it. We model the root
/// analogously using the CFG exit block.
fn sequence_end<T: HugrNode>(
    cfg: &impl CfgNodeMap<T>,
    graph: &RegionGraph<T>,
    region_id: RegionId,
) -> Option<Component<T>> {
    match graph.region(region_id).kind {
        RegionKind::Root => Some(Component::Block(cfg.exit_node())),
        RegionKind::Linear | RegionKind::Loop | RegionKind::Branch => Some(Component::Block(
            graph.region(region_id).exit_edge.unwrap().0,
        )),
    }
}

/// Extracts the underlying block from a component when the caller has already
/// established that the component cannot be a nested region.
///
/// Keeping this check centralized makes the branch-building code easier to read
/// and ensures we fail with a targeted message if a structural assumption stops
/// holding.
fn expect_block<T: HugrNode>(
    component: Component<T>,
    region: RegionId,
) -> Result<T, RvsdgBuildError<T>> {
    match component {
        Component::Block(node) => Ok(node),
        Component::Region(_) => Err(RvsdgBuildError::ExpectedBlock { region }),
    }
}

/// A region-local graph where each immediate child region is collapsed to a
/// single vertex.
///
/// This is the key helper for recovering execution order. The raw CFG contains
/// every block of every nested region, which makes "walk the top-level path
/// through this region" awkward. By compressing children away, the remaining
/// graph matches the granularity of the control tree we want to build.
struct CompressedRegion<T> {
    succs: HashMap<Component<T>, HashSet<Component<T>>>,
}

impl<T: HugrNode> CompressedRegion<T> {
    /// Builds the compressed view for one region.
    ///
    /// `ignored_edge` is used for loop bodies to omit the latch edge that closes
    /// the cycle. That converts the body of one iteration into an acyclic path
    /// that the sequence-building code can order.
    fn new(
        cfg: &impl CfgNodeMap<T>,
        graph: &RegionGraph<T>,
        region_id: RegionId,
        ignored_edge: Option<(T, T)>,
    ) -> Self {
        let region = graph.region(region_id);
        let child_owner = child_owner_map(graph, region);
        let mut succs: HashMap<Component<T>, HashSet<Component<T>>> = HashMap::new();

        for &block in &region.blocks {
            let src = component_for(block, &child_owner);
            succs.entry(src).or_default();
            for succ in cfg.successors(block) {
                if ignored_edge == Some((block, succ)) || !region.blocks.contains(&succ) {
                    continue;
                }
                let dst = component_for(succ, &child_owner);
                if src != dst {
                    succs.entry(src).or_default().insert(dst);
                    succs.entry(dst).or_default();
                }
            }
        }

        Self { succs }
    }

    /// Returns the successors of a compressed component.
    ///
    /// The result is intentionally a `Vec` rather than an iterator because most
    /// callers need to pattern-match on the number of successors to validate
    /// structural assumptions.
    fn successors(&self, component: Component<T>) -> Vec<Component<T>> {
        self.succs
            .get(&component)
            .into_iter()
            .flat_map(|set| set.iter().copied())
            .collect()
    }
}

/// Maps each block owned by an immediate child region to that child.
///
/// This lets the compression step collapse one level of nesting at a time while
/// still traversing the parent region's control flow. The invariant that
/// immediate child regions are disjoint is asserted here because many later
/// helpers rely on it implicitly.
fn child_owner_map<T: HugrNode>(
    graph: &RegionGraph<T>,
    region: &Region<T>,
) -> HashMap<T, RegionId> {
    let mut owners = HashMap::new();
    for &child_id in &region.children {
        for &block in &graph.region(child_id).blocks {
            let prev = owners.insert(block, child_id);
            assert!(prev.is_none(), "immediate child regions should be disjoint");
        }
    }
    owners
}

/// Reinterprets a concrete CFG block as either a plain block component or the
/// child region that owns it in the compressed view.
///
/// This is the small but important step that lets the ordered tree talk about
/// nested regions directly instead of leaking their internal blocks into the
/// parent sequence.
fn component_for<T: HugrNode>(block: T, child_owner: &HashMap<T, RegionId>) -> Component<T> {
    child_owner
        .get(&block)
        .copied()
        .map(Component::Region)
        .unwrap_or(Component::Block(block))
}

#[cfg(test)]
mod test {
    use super::{
        ControlRegionBody, ControlTree, RegionBoundary, RegionKind, build_control_tree,
        build_region_graph,
    };
    use crate::control::IdentityCfgMap;
    use crate::control::interface::RegionInterface;
    use crate::control::nest_cfgs::test::build_conditional_in_loop_cfg;
    use hugr::Hugr;
    use hugr::builder::{
        BuildError, CFGBuilder, Container, DataflowSubContainer, HugrBuilder, endo_sig,
    };
    use hugr::extension::prelude::usize_t;
    use hugr::ops::Value;
    use hugr::ops::handle::{BasicBlockID, ConstID};
    use hugr::types::Signature;

    fn n_identity<T: DataflowSubContainer>(
        mut dataflow_builder: T,
        pred_const: &ConstID,
    ) -> Result<T::ContainerHandle, BuildError> {
        let wires = dataflow_builder.input_wires();
        let unit = dataflow_builder.load_const(pred_const);
        dataflow_builder.finish_with_outputs([unit].into_iter().chain(wires))
    }

    fn build_then_else_merge_from_if<T: AsMut<Hugr> + AsRef<Hugr>>(
        cfg: &mut CFGBuilder<T>,
        unit_const: &ConstID,
        split: BasicBlockID,
    ) -> Result<BasicBlockID, BuildError> {
        let merge = n_identity(
            cfg.simple_block_builder(endo_sig([usize_t()]), 1)?,
            unit_const,
        )?;
        let left = n_identity(
            cfg.simple_block_builder(endo_sig([usize_t()]), 1)?,
            unit_const,
        )?;
        let right = n_identity(
            cfg.simple_block_builder(endo_sig([usize_t()]), 1)?,
            unit_const,
        )?;
        cfg.branch(&split, 0, &left)?;
        cfg.branch(&split, 1, &right)?;
        cfg.branch(&left, 0, &merge)?;
        cfg.branch(&right, 0, &merge)?;
        Ok(merge)
    }

    fn build_cond_then_loop_cfg() -> Result<Hugr, BuildError> {
        let mut cfg_builder = CFGBuilder::new(Signature::new_endo([usize_t()]))?;
        let pred_const = cfg_builder.add_constant(Value::unit_sum(0, 2).expect("0 < 2"));
        let const_unit = cfg_builder.add_constant(Value::unary_unit_sum());

        let entry = n_identity(
            cfg_builder.simple_entry_builder(vec![usize_t()].into(), 1)?,
            &const_unit,
        )?;
        let split = n_identity(
            cfg_builder.simple_block_builder(endo_sig([usize_t()]), 2)?,
            &pred_const,
        )?;
        cfg_builder.branch(&entry, 0, &split)?;
        let merge = build_then_else_merge_from_if(&mut cfg_builder, &const_unit, split)?;
        let head = n_identity(
            cfg_builder.simple_block_builder(endo_sig([usize_t()]), 1)?,
            &const_unit,
        )?;
        let tail = n_identity(
            cfg_builder.simple_block_builder(endo_sig([usize_t()]), 2)?,
            &pred_const,
        )?;
        cfg_builder.branch(&merge, 0, &head)?;
        cfg_builder.branch(&head, 0, &tail)?;
        cfg_builder.branch(&tail, 1, &head)?;
        let exit = cfg_builder.exit_block();
        cfg_builder.branch(&tail, 0, &exit)?;

        Ok(cfg_builder.finish_hugr()?)
    }

    #[test]
    fn detects_nested_branch_inside_loop() -> Result<(), BuildError> {
        let (h, _, _) = build_conditional_in_loop_cfg(true)?;
        let graph = build_region_graph(&IdentityCfgMap::new(h)).unwrap();

        let mut kind_counts =
            graph
                .regions()
                .iter()
                .fold(std::collections::HashMap::new(), |mut counts, region| {
                    *counts.entry(region.kind).or_insert(0usize) += 1;
                    counts
                });

        assert_eq!(kind_counts.remove(&RegionKind::Root), Some(1));
        assert_eq!(kind_counts.remove(&RegionKind::Loop), Some(1));
        assert_eq!(kind_counts.remove(&RegionKind::Branch), Some(1));
        assert_eq!(kind_counts.remove(&RegionKind::Linear), Some(2));
        assert!(kind_counts.is_empty());

        let root = graph.region(graph.root());
        assert_eq!(root.children.len(), 1);
        let loop_region = graph.region(root.children[0]);
        assert_eq!(loop_region.kind, RegionKind::Loop);
        assert_eq!(loop_region.children.len(), 1);
        let branch_region = graph.region(loop_region.children[0]);
        assert_eq!(branch_region.kind, RegionKind::Branch);
        assert_eq!(branch_region.children.len(), 2);
        for child in &branch_region.children {
            assert_eq!(graph.region(*child).kind, RegionKind::Linear);
        }

        Ok(())
    }

    #[test]
    fn detects_conditional_and_loop_as_siblings() -> Result<(), BuildError> {
        let h = build_cond_then_loop_cfg()?;
        let graph = build_region_graph(&IdentityCfgMap::new(h)).unwrap();
        let root = graph.region(graph.root());

        let root_child_kinds = root
            .children
            .iter()
            .map(|id| graph.region(*id).kind)
            .collect::<Vec<_>>();
        assert_eq!(root_child_kinds.len(), 2);
        assert!(root_child_kinds.contains(&RegionKind::Branch));
        assert!(root_child_kinds.contains(&RegionKind::Loop));

        Ok(())
    }

    #[test]
    fn region_boundaries_are_sese() -> Result<(), BuildError> {
        let (h, _, _) = build_conditional_in_loop_cfg(true)?;
        let graph = build_region_graph(&IdentityCfgMap::new(h)).unwrap();

        for region in graph.regions() {
            match region.kind {
                RegionKind::Root => {
                    assert_eq!(
                        region.boundary,
                        RegionBoundary {
                            incoming: vec![],
                            outgoing: vec![],
                        }
                    );
                }
                _ => {
                    assert_eq!(region.boundary.incoming.len(), 1);
                    assert_eq!(region.boundary.outgoing.len(), 1);
                }
            }
        }

        Ok(())
    }

    #[test]
    fn control_tree_orders_nested_loop_body() -> Result<(), BuildError> {
        let (h, _, _) = build_conditional_in_loop_cfg(true)?;
        let tree = build_control_tree(&IdentityCfgMap::new(h)).unwrap();

        assert_eq!(tree.kind, RegionKind::Root);
        assert_eq!(
            tree.boundary,
            RegionBoundary {
                incoming: vec![],
                outgoing: vec![],
            }
        );
        assert_eq!(
            tree.interface,
            RegionInterface {
                entry_blocks: vec![],
                exit_blocks: vec![],
            }
        );

        let ControlRegionBody::Sequence(root_items) = &tree.body else {
            panic!("root should be a sequence region");
        };
        assert_eq!(root_items.len(), 3);
        assert!(matches!(root_items[0], ControlTree::Block(_)));
        assert!(matches!(root_items[2], ControlTree::Block(_)));

        let ControlTree::Region(loop_region) = &root_items[1] else {
            panic!("middle root item should be a loop");
        };
        assert_eq!(loop_region.kind, RegionKind::Loop);
        assert_eq!(loop_region.boundary.incoming.len(), 1);
        assert_eq!(loop_region.boundary.outgoing.len(), 1);
        assert_eq!(loop_region.interface.entry_blocks.len(), 1);
        assert_eq!(loop_region.interface.exit_blocks.len(), 1);

        let ControlRegionBody::Loop { body: loop_items } = &loop_region.body else {
            panic!("loop region should expose a loop body");
        };
        assert_eq!(loop_items.len(), 3);
        assert!(matches!(loop_items[0], ControlTree::Block(_)));
        assert!(matches!(loop_items[2], ControlTree::Block(_)));

        let ControlTree::Region(branch_region) = &loop_items[1] else {
            panic!("middle loop item should be a branch");
        };
        assert_eq!(branch_region.kind, RegionKind::Branch);
        assert_eq!(branch_region.boundary.incoming.len(), 1);
        assert_eq!(branch_region.boundary.outgoing.len(), 1);
        assert_eq!(branch_region.interface.entry_blocks.len(), 1);
        assert_eq!(branch_region.interface.exit_blocks.len(), 1);

        let ControlRegionBody::Branch { split, arms, join } = &branch_region.body else {
            panic!("branch region should expose branch arms");
        };
        assert_eq!(split.len(), 1);
        assert_eq!(join.len(), 1);
        assert_eq!(arms.len(), 2);
        for arm in arms {
            assert_eq!(arm.len(), 1);
            assert!(matches!(arm[0], ControlTree::Region(_)));
        }

        Ok(())
    }

    #[test]
    fn control_tree_orders_conditional_then_loop() -> Result<(), BuildError> {
        let h = build_cond_then_loop_cfg()?;
        let tree = build_control_tree(&IdentityCfgMap::new(h)).unwrap();

        let ControlRegionBody::Sequence(root_items) = &tree.body else {
            panic!("root should be a sequence region");
        };
        assert_eq!(root_items.len(), 4);
        assert!(matches!(root_items[0], ControlTree::Block(_)));
        assert!(matches!(root_items[3], ControlTree::Block(_)));

        let ControlTree::Region(branch_region) = &root_items[1] else {
            panic!("second root item should be a branch");
        };
        let ControlRegionBody::Branch { split, arms, join } = &branch_region.body else {
            panic!("second root item should be a branch region");
        };
        assert_eq!(split.len(), 1);
        assert_eq!(join.len(), 1);
        assert_eq!(arms.len(), 2);
        assert!(arms.iter().all(|arm| arm.len() == 1));

        let ControlTree::Region(loop_region) = &root_items[2] else {
            panic!("third root item should be a loop");
        };
        let ControlRegionBody::Loop { body: loop_items } = &loop_region.body else {
            panic!("third root item should be a loop region");
        };
        assert_eq!(loop_items.len(), 2);
        assert!(matches!(loop_items[0], ControlTree::Block(_)));
        assert!(matches!(loop_items[1], ControlTree::Block(_)));

        Ok(())
    }

    #[test]
    fn control_tree_regions_preserve_region_blocks() -> Result<(), BuildError> {
        let (h, _, _) = build_conditional_in_loop_cfg(true)?;
        let graph = build_region_graph(&IdentityCfgMap::new(h.clone())).unwrap();
        let tree = build_control_tree(&IdentityCfgMap::new(h)).unwrap();

        let ControlRegionBody::Sequence(root_items) = &tree.body else {
            panic!("root should be a sequence region");
        };
        let ControlTree::Region(loop_region) = &root_items[1] else {
            panic!("expected loop region");
        };

        let loop_blocks = graph
            .regions()
            .iter()
            .find(|region| region.kind == RegionKind::Loop)
            .expect("expected a loop region in the graph")
            .blocks
            .clone();
        assert_eq!(loop_region.blocks, loop_blocks);

        let ControlRegionBody::Loop { body: loop_items } = &loop_region.body else {
            panic!("expected loop body");
        };
        let ControlTree::Region(branch_region) = &loop_items[1] else {
            panic!("expected nested branch region");
        };
        let branch_blocks = graph
            .regions()
            .iter()
            .find(|region| region.kind == RegionKind::Branch)
            .expect("expected a branch region in the graph")
            .blocks
            .clone();
        assert_eq!(branch_region.blocks, branch_blocks);

        Ok(())
    }
}
