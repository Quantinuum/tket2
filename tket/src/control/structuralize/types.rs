//! Shared data types for HUGR-specific structuralization.
//!
//! This module defines the typed representation passed between structural
//! analysis and lowering. Keeping these definitions in one place makes it
//! easier to understand what information is computed during analysis and what
//! invariants lowering relies on.

use std::collections::BTreeMap;

use derive_more::{Display, Error};
use hugr::Node;
use hugr::builder::BuildError;
use hugr::types::TypeRow;

use crate::control::rvsdg;
use crate::passes::normalize_cfgs::NormalizeCFGError;

/// Strategy selector for control-flow structuralization.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum StructuralizationStrategy {
    /// Build an RVSDG for the CFG and lower it.
    #[default]
    Rvsdg,
    /// Use the Beyond-Relooper strategy for reducible CFGs.
    BeyondRelooper,
}

/// Ordered HUGR-facing interface for a structured region.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct RegionIo {
    /// Values entering the region from outside it.
    pub inputs: TypeRow,
    /// Values leaving the region to the surrounding region.
    pub outputs: TypeRow,
}

/// HUGR-facing summary of a CFG block used during lowering.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum StructuredBlock {
    /// A normal CFG basic block.
    Dataflow {
        /// Original block node.
        node: Node,
        /// Dataflow inputs to the block body.
        inputs: TypeRow,
        /// Branching sum rows emitted by the block.
        sum_rows: Vec<TypeRow>,
        /// Non-control outputs emitted by the block.
        outputs: TypeRow,
    },
    /// The CFG exit block.
    Exit {
        /// Original exit block node.
        node: Node,
        /// Values consumed by the exit block.
        inputs: TypeRow,
    },
}

impl StructuredBlock {
    /// Returns the original CFG node represented by this structured block.
    pub(crate) fn node(&self) -> Node {
        match self {
            Self::Dataflow { node, .. } | Self::Exit { node, .. } => *node,
        }
    }

    /// Returns the value row that must be available before executing the block.
    pub(crate) fn inputs(&self) -> &TypeRow {
        match self {
            Self::Dataflow { inputs, .. } | Self::Exit { inputs, .. } => inputs,
        }
    }
}

/// A HUGR-specific analyzed control tree node.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum StructuredNode {
    /// A single CFG block.
    Block(StructuredBlock),
    /// A nested structured region.
    Region(Box<StructuredRegion>),
}

/// Lowering family chosen for a structured loop.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum StructuredLoopKind {
    /// The latch both re-enters the header and exits the loop.
    TailControlled,
    /// The header decides whether to enter the body or break immediately.
    HeaderControlled,
}

/// One analyzed loop edge that either continues or exits the loop.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct StructuredLoopEdge {
    /// CFG block producing the control decision.
    pub(crate) source: Node,
    /// Successor index selected by that control decision.
    pub(crate) case: usize,
    /// Payload row carried along that successor.
    pub(crate) payload: TypeRow,
}

/// One structured loop exit with its immediate payload and continuation.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct StructuredLoopExit {
    /// Break edges that select this exit variant.
    pub(crate) edges: Vec<StructuredLoopEdge>,
    /// Immediate payload emitted when the loop exits through this variant.
    pub(crate) outputs: TypeRow,
    /// Continuation lowered after the loop exits through this variant.
    pub(crate) continuation: Vec<StructuredNode>,
}

/// How a structured branch hands control to its join block.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum StructuredBranchJoinKind {
    /// The branch region lowers its join block internally and returns the
    /// continuation after that join.
    Inline,
    /// The branch region returns the join inputs to the surrounding sequence,
    /// which will lower the join block as the next structured node.
    Deferred,
}

/// HUGR-specific body information for a structured region.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum StructuredRegionBody {
    /// A linear sequence of blocks / nested regions.
    Sequence(Vec<StructuredNode>),
    /// A structured branch lowered via `Conditional`.
    Branch {
        /// Split block executed before the `Conditional`.
        split: StructuredBlock,
        /// Per-arm bodies.
        arms: Vec<Vec<StructuredNode>>,
        /// Join block executed after the `Conditional`.
        join: StructuredBlock,
        /// Whether the join block is lowered inside the branch or by the
        /// surrounding sequence.
        join_kind: StructuredBranchJoinKind,
    },
    /// A structured loop lowered via `TailLoop`.
    Loop {
        /// Loop-shape classification used during lowering.
        kind: StructuredLoopKind,
        /// The CFG block acting as the loop header.
        header: StructuredBlock,
        /// One-iteration loop body items.
        body: Vec<StructuredNode>,
        /// CFG block whose successor returns control to the header.
        backedge_source: Node,
        /// Continue edge routed back to the loop header.
        continue_edge: StructuredLoopEdge,
        /// Distinct loop exits selected by break edges in the loop body.
        exits: Vec<StructuredLoopExit>,
        /// Immediate payload row consumed by the `TailLoop` break path.
        ///
        /// Single-exit loops use the exit payload directly. Multi-exit loops
        /// use a tagged sum carrying the selected exit payload.
        break_outputs: TypeRow,
    },
}

/// HUGR-specific structural summary for one region.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct StructuredRegion {
    /// Ordered HUGR interface of the region.
    pub(crate) io: RegionIo,
    /// HUGR-specific lowering metadata.
    pub(crate) body: StructuredRegionBody,
}

impl StructuredRegion {
    /// Extracts the root sequence used as the entry point for whole-CFG lowering.
    ///
    /// The top-level structuralizer always lowers a CFG from a linear sequence.
    /// Nested regions may be branches or loops, but callers using this helper are
    /// specifically asking for the root lowering spine.
    pub(super) fn body_sequence(&self) -> Result<&[StructuredNode], StructuralizationError> {
        match &self.body {
            StructuredRegionBody::Sequence(items) => Ok(items),
            _ => Err(StructuralizationError::UnsupportedBranch {
                reason: "top-level CFG should lower from a sequence region".into(),
            }),
        }
    }
}

/// Structured CFG analysis output for one HUGR.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct StructuralizationAnalysisReport {
    /// Structured control tree plus HUGR-specific lowering metadata for each CFG.
    pub(crate) cfg_regions: BTreeMap<Node, StructuredRegion>,
}

/// Mutable rewrite report returned by the pass layer.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct StructuralizationRewriteReport {
    /// CFG nodes that were rewritten into structured dataflow.
    pub rewrites: Vec<Node>,
}

/// Errors returned by HUGR-level structuralization entry points.
#[derive(Debug, Display, Error)]
#[non_exhaustive]
pub enum StructuralizationError {
    /// Error while building an RVSDG structural tree for a CFG.
    #[display("failed to build RVSDG control tree: {_0}")]
    Rvsdg(rvsdg::RvsdgBuildError<Node>),
    /// A structured region required a basic block but found something else.
    #[display("expected a dataflow block while lowering node {node}")]
    ExpectedDataflowBlock {
        /// Node that was expected to be a dataflow block.
        node: Node,
    },
    /// A structured region expected the CFG exit block.
    #[display("expected CFG exit block at node {node}")]
    ExpectedExitBlock {
        /// Node that was expected to be an exit block.
        node: Node,
    },
    /// A branch region was structurally valid but not lowerable with the current assumptions.
    #[display("branch region could not be lowered because {reason}")]
    UnsupportedBranch {
        /// Short description of the mismatch.
        reason: String,
    },
    /// A loop region was structurally valid but not lowerable with the current assumptions.
    #[display("loop region could not be lowered because {reason}")]
    UnsupportedLoop {
        /// Short description of the mismatch.
        reason: String,
    },
    /// A reducible-CFG precondition for the Beyond-Relooper translation was not met.
    #[display("Beyond Relooper requires Appendix A preprocessing for CFG {cfg} because {reason}")]
    UnsupportedIrreducibleCfg {
        /// CFG root that would require the Appendix A preprocessing step.
        cfg: Node,
        /// Short description of the reducibility failure.
        reason: String,
    },
    /// The Beyond-Relooper translation could not derive a valid structured form.
    #[display("Beyond Relooper could not derive a structured form because {reason}")]
    Relooper {
        /// Short description of the mismatch or unsupported construct.
        reason: String,
    },
    /// The shared rewrite pipeline could not materialize a structured CFG replacement.
    #[display("failed to materialize a structured CFG rewrite because {reason}")]
    Materialization {
        /// Short description of the rewrite mismatch.
        reason: String,
    },
    /// The selected strategy is not implemented yet.
    #[display("structuralization strategy {strategy:?} is not implemented yet")]
    UnsupportedStrategy {
        /// Strategy requested by the caller.
        #[error(not(source))]
        strategy: StructuralizationStrategy,
    },
    /// Error while building structured HUGR nodes.
    #[display("failed to build structuralized HUGR: {_0}")]
    Build(BuildError),
    /// Error validating the constructed structured HUGR.
    #[display("structured HUGR validation failed: {_0}")]
    Validation(hugr::hugr::ValidationError<Node>),
    /// Error running local CFG normalization after rewriting.
    #[display("failed to normalize nested CFGs after structuralization: {_0}")]
    Normalize(NormalizeCFGError),
}

impl From<BuildError> for StructuralizationError {
    /// Converts builder failures into the public structuralization error type.
    fn from(value: BuildError) -> Self {
        Self::Build(value)
    }
}

impl From<hugr::hugr::ValidationError<Node>> for StructuralizationError {
    /// Converts HUGR validation failures into the public structuralization error type.
    fn from(value: hugr::hugr::ValidationError<Node>) -> Self {
        Self::Validation(value)
    }
}

impl From<NormalizeCFGError> for StructuralizationError {
    /// Converts post-rewrite normalization failures into the public error type.
    fn from(value: NormalizeCFGError) -> Self {
        Self::Normalize(value)
    }
}

/// Reports whether a structured subtree still contains a specific original CFG block.
///
/// Analysis and lowering both use this to sanity-check that inferred loop
/// metadata still lines up with the structured nodes being traversed.
pub(super) fn structured_node_contains_block(node: &StructuredNode, target: Node) -> bool {
    match node {
        StructuredNode::Block(StructuredBlock::Dataflow { node, .. })
        | StructuredNode::Block(StructuredBlock::Exit { node, .. }) => *node == target,
        StructuredNode::Region(region) => structured_region_contains_block(region, target),
    }
}

/// Recursively checks whether a structured region contains a specific CFG block.
fn structured_region_contains_block(region: &StructuredRegion, target: Node) -> bool {
    match &region.body {
        StructuredRegionBody::Sequence(items) => items
            .iter()
            .any(|item| structured_node_contains_block(item, target)),
        StructuredRegionBody::Branch {
            split,
            arms,
            join,
            join_kind: _,
        } => {
            split.node() == target
                || arms
                    .iter()
                    .flatten()
                    .any(|item| structured_node_contains_block(item, target))
                || join.node() == target
        }
        StructuredRegionBody::Loop { header, body, .. } => {
            header.node() == target
                || body
                    .iter()
                    .any(|item| structured_node_contains_block(item, target))
        }
    }
}
