//! Shared data types for HUGR-specific structuralization.
//!
//! This module defines the typed representation passed between structural
//! analysis and lowering. Keeping these definitions in one place makes it
//! easier to understand what information is computed during analysis and what
//! invariants lowering relies on.

use derive_more::{Display, Error};
use hugr::Node;
use hugr::builder::BuildError;
use hugr::types::TypeRow;

use crate::control::cfg::PreprocessedNode;
use crate::control::rvsdg;
use crate::passes::normalize_cfgs::NormalizeCFGError;

/// Strategy selector for control-flow structuralization.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum StructuralizationStrategy {
    /// Build an RVSDG for the CFG and lower it.
    #[default]
    Rvsdg,
    /// Use the Beyond-Relooper strategy for reducible CFGs.
    Relooper,
}

/// Ordered HUGR-facing interface for a structured region.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct RegionIo {
    /// Values entering the region from outside it.
    pub inputs: TypeRow,
    /// Values leaving the region to the surrounding region.
    pub outputs: TypeRow,
}

/// Stable identity for one CFG node occurrence used during lowering.
///
/// Structuralization may preprocess irreducible graphs by duplicating CFG
/// nodes. Lowering therefore needs a control-edge identity that remains unique
/// per CFG occurrence even when multiple occurrences map back to the same
/// original HUGR block.
pub(crate) type StructuredCfgNode = PreprocessedNode<Node>;

/// Converts one CFG graph node into the lowering identity used by structured blocks.
pub(crate) trait IntoStructuredCfgNode {
    /// Returns the stable CFG-node identity for this graph node.
    fn into_structured_cfg_node(self) -> StructuredCfgNode;
}

impl IntoStructuredCfgNode for Node {
    fn into_structured_cfg_node(self) -> StructuredCfgNode {
        PreprocessedNode::Original(self)
    }
}

impl IntoStructuredCfgNode for PreprocessedNode<Node> {
    fn into_structured_cfg_node(self) -> StructuredCfgNode {
        self
    }
}

/// HUGR-facing summary of a CFG block used during lowering.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum StructuredBlock {
    /// A normal CFG basic block.
    Dataflow {
        /// CFG-node identity used by structural analysis and lowering.
        cfg_node: StructuredCfgNode,
        /// Original block node.
        node: Node,
        /// Dataflow inputs to the block body.
        inputs: TypeRow,
        /// Branching sum rows emitted by the block.
        sum_rows: Vec<TypeRow>,
        /// Non-control outputs emitted by the block.
        outputs: TypeRow,
        /// Unique visible successor case when the block is lowered as straight-line code.
        ///
        /// Some structured walks suppress out-of-scope successors, such as
        /// loop-closing backedges or exits that belong to an enclosing region.
        /// Lowering needs to know which original CFG successor remains visible
        /// so it can forward that successor payload instead of blindly
        /// discarding the control sum.
        linear_successor: Option<usize>,
    },
    /// The CFG exit block.
    Exit {
        /// CFG-node identity used by structural analysis and lowering.
        cfg_node: StructuredCfgNode,
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

    /// Returns the CFG-node identity represented by this structured block.
    pub(crate) fn cfg_node(&self) -> StructuredCfgNode {
        match self {
            Self::Dataflow { cfg_node, .. } | Self::Exit { cfg_node, .. } => *cfg_node,
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

/// Kind of enclosing structured target selected by one propagated branch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum StructuredBranchTargetKind {
    /// Branch to the continuation after an enclosing labelled block.
    BlockFollowedBy,
    /// Branch to one enclosing loop header.
    LoopHeadedBy,
}

/// One analyzed loop edge that either continues or exits the loop.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct StructuredLoopEdge {
    /// CFG block producing the control decision.
    pub(crate) source: StructuredCfgNode,
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
    /// The output row that the continuation body actually produces.
    ///
    /// For empty continuations this equals `outputs`; for non-empty
    /// continuations it is the output row of the last node in the body.
    /// This may differ from `effect.outputs()` for Branch effects where
    /// the branch carries outer-scope values rather than the continuation's
    /// own outputs.
    pub(crate) continuation_outputs: TypeRow,
    /// Effect produced after the exit continuation, if any, or the local
    /// outputs visible when the continuation completes normally.
    pub(crate) effect: StructuredExitEffect,
}

/// Explicit control effect preserved after lowering one loop exit continuation.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum StructuredExitEffect {
    /// The loop exit continues normally inside the current enclosing sequence.
    Local(TypeRow),
    /// The loop exit returns from the enclosing region with the given row.
    Return(TypeRow),
    /// The loop exit branches to one enclosing structured target.
    ///
    /// The target is identified by the CFG-backed label node for the enclosing
    /// block or loop that should consume the propagated exit.
    Branch {
        /// Kind of enclosing target that should consume this exit.
        kind: StructuredBranchTargetKind,
        /// Enclosing CFG-backed target that should consume this exit.
        target: StructuredCfgNode,
        /// Ordered payload row visible when the branch effect is emitted.
        outputs: TypeRow,
    },
}

impl StructuredExitEffect {
    /// Returns the ordered row visible after this exit effect is produced.
    pub(crate) fn outputs(&self) -> &TypeRow {
        match self {
            Self::Local(outputs) | Self::Return(outputs) => outputs,
            Self::Branch { outputs, .. } => outputs,
        }
    }
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

/// One visible branch arm together with the split case that reaches it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct StructuredCaseArm {
    /// Successor case index selected by the split block.
    pub(crate) case: usize,
    /// Lowered body for that visible successor.
    pub(crate) body: Vec<StructuredNode>,
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
        /// Per-arm bodies keyed by their originating split case.
        arms: Vec<StructuredCaseArm>,
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
        /// CFG blocks whose successors return control to the header.
        backedge_sources: Vec<Node>,
        /// Continue edges routed back to the loop header.
        continue_edges: Vec<StructuredLoopEdge>,
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
    /// When present, the region's output is a Sum encoding which of several
    /// heterogeneous loop exits was taken.  The enclosing loop body must
    /// dispatch on this Sum: local variants continue the body, while non-local
    /// variants cause the enclosing loop to break or propagate the effect.
    pub(crate) multilevel_exit_dispatch: Option<MultilevelExitDispatch>,
}

/// Describes the exit dispatch that the enclosing loop must perform after a
/// region whose multi-exit inner loop produced a tagged Sum output.
///
/// The region output is `[Sum(variant_rows)]` where each variant corresponds
/// to one of the inner loop's structured exits.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct MultilevelExitDispatch {
    /// Per-variant description: which exit effect each Sum variant represents,
    /// and the ordered payload row carried by that variant.
    pub(crate) variants: Vec<MultilevelExitVariant>,
}

/// One variant of the multilevel-exit Sum dispatched by the enclosing loop.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct MultilevelExitVariant {
    /// Effect produced by this exit variant.
    pub(crate) effect: StructuredExitEffect,
    /// The actual output row produced by the inner loop's continuation for
    /// this variant. This is used as the Sum variant row in the dispatch
    /// Conditional.
    pub(crate) continuation_outputs: TypeRow,
    /// The exit edges associated with this variant, used to match against
    /// the enclosing loop's exits when building tagged break payloads.
    pub(crate) edges: Vec<StructuredLoopEdge>,
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
#[cfg(test)]
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub(crate) struct StructuralizationAnalysisReport {
    /// Structured control tree plus HUGR-specific lowering metadata for each CFG.
    pub(crate) cfg_regions: std::collections::BTreeMap<Node, StructuredRegion>,
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
                    .flat_map(|arm| arm.body.iter())
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
