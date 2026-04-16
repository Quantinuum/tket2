//! Private control AST for the Beyond-Relooper strategy.
//!
//! The Beyond Relooper paper phrases translation in terms of a target language
//! with straight-line code, sequencing, labelled blocks, loops, conditionals,
//! and multilevel exits interpreted relative to a surrounding context. This
//! module models that control language locally so the strategy can evolve
//! independently from the shared HUGR rewrite pipeline.
//!
//! One intentional deviation from the paper is that binary `if` is generalized
//! to [`RelooperStmt::Case`]. HUGR CFG blocks can branch over any ordered sum,
//! and the later HUGR lowering naturally consumes a multi-arm case split, so
//! keeping this construct n-ary avoids re-encoding an arbitrary fanout as a
//! tree of binary conditionals.
//!
//! A second intentional deviation is that some statements carry small
//! strategy-local lowering records. Those records are not part of the control
//! language itself; they preserve typed HUGR facts that the current lowerer
//! still needs while the strategy moves toward a fully context-driven lowering
//! of multilevel exits.

use hugr::{Node, types::TypeRow};

use crate::control::structuralize::{
    RegionIo, StructuredBlock, StructuredBranchJoinKind, StructuredLoopEdge, StructuredLoopKind,
};

/// One label referenced by the strategy-local AST.
///
/// Labels are backed by CFG nodes so debugging the reconstructed control is
/// straightforward. Preprocessing may duplicate CFG blocks, so duplicated
/// occurrences carry a deterministic clone identifier in addition to the
/// original HUGR node.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub(crate) enum RelooperLabel {
    /// One original CFG block.
    Original(Node),
    /// One duplicated occurrence created during preprocessing.
    Duplicate {
        /// Original HUGR block represented by the duplicate.
        original: Node,
        /// Deterministic duplicate identifier scoped to one preprocessed CFG.
        clone_id: usize,
    },
}

/// One frame in the context stack used by the paper's recursive translation.
///
/// The innermost enclosing construct is stored first, matching the paper's
/// presentation of context-sensitive branch depths.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum RelooperContextFrame {
    /// The current node is nested beneath a case split.
    Case,
    /// The current node is nested beneath a loop headed by `label`.
    LoopHeadedBy(RelooperLabel),
    /// The current node is nested within a block followed by `label`.
    BlockFollowedBy(RelooperLabel),
}

/// One explicit labelled branch target in the Beyond-Relooper AST.
///
/// Preserving whether a branch targets an enclosing block continuation or an
/// enclosing loop header is important for later multilevel-exit lowering.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum RelooperBranchTarget {
    /// Branch to the continuation after one enclosing labelled block.
    BlockFollowedBy(RelooperLabel),
    /// Branch to one enclosing loop header.
    LoopHeadedBy(RelooperLabel),
}

/// One explicit non-local branch in the Beyond-Relooper AST.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct RelooperBranch {
    /// Enclosing target that should consume the branch.
    pub(crate) target: RelooperBranchTarget,
    /// Ordered payload row carried to that enclosing target.
    pub(crate) outputs: TypeRow,
}

/// Ordered stack of enclosing structured constructs.
pub(crate) type RelooperContext = Vec<RelooperContextFrame>;

/// One analyzed region in the Beyond-Relooper AST.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct RelooperRegion {
    /// Ordered values crossing the region boundary.
    pub(super) io: RegionIo,
    /// Structured body of the region.
    pub(super) body: RelooperStmt,
}

/// HUGR-facing details for one labelled block.
///
/// The paper's target language only needs the label and body. The current
/// lowerer still requires whether the followed-by block lowers locally or in
/// the surrounding sequence, so that detail lives here instead of inside the
/// statement shape itself.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct RelooperBlockLowering {
    /// Whether the followed-by block lowers inside the block or in the
    /// surrounding scope.
    pub(super) join_kind: StructuredBranchJoinKind,
    /// Loop-exit selectors routed to this labelled block.
    ///
    /// Ordinary branch blocks leave this empty. Wrapped loop-exit blocks use it
    /// to record which CFG edges break to the block's label.
    pub(super) loop_exit_edges: Vec<StructuredLoopEdge>,
}

/// HUGR-facing details for one analysed loop.
///
/// The statement language only needs a labelled loop body. The current
/// structural HUGR lowerer also needs the loop family and the CFG edges that
/// feed continue and break paths, so those details are grouped here until exit
/// lowering becomes fully context-driven.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct RelooperLoopLowering {
    /// Loop family selected from the CFG.
    pub(super) kind: StructuredLoopKind,
    /// Header block that guards or enters the loop.
    pub(super) header: StructuredBlock,
    /// CFG blocks whose successors return control to the header.
    pub(super) backedge_sources: Vec<Node>,
    /// Continue edges routed back to the loop header.
    pub(super) continue_edges: Vec<StructuredLoopEdge>,
}

/// One statement in the Beyond-Relooper AST.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) enum RelooperStmt {
    /// A straight-line sequence of statements.
    Seq(Vec<RelooperStmt>),
    /// A nested structured region with its own interface.
    Region(Box<RelooperRegion>),
    /// One executable CFG block.
    Exec(StructuredBlock),
    /// A labelled block after which execution reaches the label's target.
    Block {
        /// Label naming the code immediately after the block.
        label: RelooperLabel,
        /// HUGR-facing lowering details for the block target.
        lowering: RelooperBlockLowering,
        /// Body governed by the labelled block.
        body: Box<RelooperStmt>,
    },
    /// An n-way case split driven by a CFG split block.
    Case {
        /// Split block executed before arm dispatch.
        split: StructuredBlock,
        /// Visible case arms keyed by their originating split case.
        arms: Vec<RelooperCaseArm>,
    },
    /// A labelled loop.
    Loop {
        /// Label used to refer to the loop header in the surrounding context.
        label: RelooperLabel,
        /// HUGR-facing lowering details for the loop.
        lowering: RelooperLoopLowering,
        /// One logical iteration of the loop body.
        body: Box<RelooperStmt>,
    },
    /// A branch to one enclosing structured construct.
    Br(RelooperBranch),
    /// Return from the enclosing region with the given payload row.
    Return(TypeRow),
}

/// One visible case arm together with the split case that reaches it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct RelooperCaseArm {
    /// Successor case index selected by the split block.
    pub(super) case: usize,
    /// Statement body lowered for that visible successor.
    pub(super) body: RelooperStmt,
}
