//! RVSDG data types used by the structuralization pass.
//!
//! These types intentionally model only the control-oriented portion of RVSDG
//! needed for reducible CFG structuralization. They carry explicit region
//! arguments/results and `gamma`/`theta` variable bundles so later translation
//! into the shared structured-lowering IR can avoid CFG-shape heuristics.

use hugr::Node;
use hugr::types::{Type, TypeRow};

use crate::control::structuralize::StructuredCfgNode;

/// Identifier for one RVSDG variable.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub(super) struct VarId(pub(super) usize);

/// One ordered variable in a region or structural node interface.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct RegionVar {
    /// Stable identifier used by tests and translations.
    pub(crate) id: VarId,
    /// Value type carried by the variable.
    pub(crate) ty: Type,
}

/// Converts an ordered variable list back into a [`TypeRow`].
pub(super) fn vars_to_row(vars: &[RegionVar]) -> TypeRow {
    vars.iter()
        .map(|var| var.ty.clone())
        .collect::<Vec<_>>()
        .into()
}

/// Root RVSDG object for one CFG.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct Rvsdg {
    /// Root region covering the whole CFG.
    pub(crate) root: Region,
}

/// One RVSDG region with explicit ordered arguments and results.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct Region {
    /// Variables visible when entering the region.
    pub(crate) arguments: Vec<RegionVar>,
    /// Ordered body nodes in the region.
    pub(crate) body: Vec<RvsdgNode>,
    /// Variables produced when leaving the region.
    pub(crate) results: Vec<RegionVar>,
}

/// One node inside a region body.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) enum RvsdgNode {
    /// Reference to one original CFG block.
    Block(BlockNode),
    /// Structured branch.
    Gamma(Box<GammaNode>),
    /// Structured loop.
    Theta(Box<ThetaNode>),
}

/// HUGR-facing summary of one original CFG block.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) enum BlockNode {
    /// A dataflow basic block.
    Dataflow {
        /// CFG-node identity used by structural analysis and lowering.
        cfg_node: StructuredCfgNode,
        /// Original CFG block node.
        node: Node,
        /// Value inputs to the block.
        inputs: Vec<RegionVar>,
        /// Case-specific payload rows emitted by the block.
        sum_rows: Vec<Vec<RegionVar>>,
        /// Shared non-control outputs emitted by the block.
        outputs: Vec<RegionVar>,
        /// Unique visible successor case when the block is lowered as straight-line code.
        linear_successor: Option<usize>,
    },
    /// The CFG exit block.
    Exit {
        /// CFG-node identity used by structural analysis and lowering.
        cfg_node: StructuredCfgNode,
        /// Original CFG exit node.
        node: Node,
        /// Values consumed by the exit.
        inputs: Vec<RegionVar>,
    },
}

impl BlockNode {
    /// Returns the ordered value inputs required by the block.
    pub(super) fn inputs(&self) -> &[RegionVar] {
        match self {
            Self::Dataflow { inputs, .. } | Self::Exit { inputs, .. } => inputs,
        }
    }
}

/// How control continues after a `gamma` join.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum BranchJoinKind {
    /// The `gamma` lowers its join block internally.
    Inline,
    /// The enclosing region lowers the join block afterwards.
    Deferred,
}

/// Values routed into all `gamma` branches.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct GammaEntryVar {
    /// Shared input at the `gamma` boundary.
    pub(crate) input: RegionVar,
    /// Per-branch argument representing that shared input inside each branch.
    pub(crate) branch_arguments: Vec<RegionVar>,
}

/// Values routed out of all `gamma` branches.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct GammaOutputVar {
    /// Per-branch result reaching the join point.
    pub(crate) branch_results: Vec<RegionVar>,
    /// Shared output of the `gamma`.
    pub(crate) output: RegionVar,
}

/// RVSDG branch node corresponding to a structured conditional.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct GammaNode {
    /// Region inputs visible at the branch boundary.
    pub(crate) inputs: Vec<RegionVar>,
    /// Split block executed before dispatching to the branches.
    pub(crate) split: BlockNode,
    /// Shared values routed into every branch.
    pub(crate) entry_vars: Vec<GammaEntryVar>,
    /// Case-specific match payload rows.
    pub(crate) match_rows: Vec<Vec<RegionVar>>,
    /// Visible branch regions keyed by their originating split case.
    pub(crate) branches: Vec<GammaBranch>,
    /// Values produced by each branch and made available after the join.
    pub(crate) outputs: Vec<GammaOutputVar>,
    /// Join block executed after the branches merge.
    pub(crate) join: BlockNode,
    /// Whether the join belongs to the branch region or the enclosing sequence.
    pub(crate) join_kind: BranchJoinKind,
}

/// One visible `gamma` branch together with its match-case index.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct GammaBranch {
    /// Successor case index selected by the split block.
    pub(crate) case: usize,
    /// Region lowered for that visible successor.
    pub(crate) region: Region,
}

/// Loop families currently supported by the RVSDG builder.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum LoopKind {
    /// The loop break and backedge come from the same latch block.
    TailControlled,
    /// The header tests whether zero iterations are taken.
    HeaderControlled,
}

/// One loop-carried variable in a `theta` node.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct ThetaLoopVar {
    /// Value at loop entry.
    pub(crate) input: RegionVar,
    /// Value visible at the start of an iteration.
    pub(crate) pre: RegionVar,
    /// Value produced at the end of an iteration.
    pub(crate) post: RegionVar,
}

/// One loop edge classified during `theta` construction.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct ThetaEdge {
    /// CFG block producing the control decision.
    pub(crate) source: StructuredCfgNode,
    /// Successor index chosen by that control decision.
    pub(crate) case: usize,
    /// Variables carried along that successor.
    pub(crate) payload: Vec<RegionVar>,
}

/// One loop exit with its immediate payload and continuation region.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct ThetaExit {
    /// Break edges that select this exit variant.
    pub(crate) edges: Vec<ThetaEdge>,
    /// Immediate payload emitted when the loop exits through this variant.
    pub(crate) outputs: Vec<RegionVar>,
    /// Continuation lowered after the loop exits through this variant.
    pub(crate) continuation: Region,
}

/// RVSDG loop node corresponding to one structured loop.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct ThetaNode {
    /// Region inputs visible at the loop boundary.
    pub(crate) inputs: Vec<RegionVar>,
    /// Region outputs visible after the loop exits.
    pub(crate) outputs: Vec<RegionVar>,
    /// Loop kind selected during construction.
    pub(crate) kind: LoopKind,
    /// Block that acts as the loop header.
    pub(crate) header: BlockNode,
    /// Region representing one logical iteration of the loop body.
    pub(crate) body: Region,
    /// CFG block whose successor returns control to the header.
    pub(crate) backedge_source: Node,
    /// Continue edge routed back to the loop header.
    pub(crate) continue_edge: ThetaEdge,
    /// Distinct exits routed out of the loop to the enclosing continuation.
    pub(crate) exits: Vec<ThetaExit>,
    /// Explicit loop-carried variables.
    pub(crate) loop_vars: Vec<ThetaLoopVar>,
}
