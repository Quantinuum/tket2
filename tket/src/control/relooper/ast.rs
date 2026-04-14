//! Private control AST for the Beyond-Relooper strategy.
//!
//! The current Beyond-Relooper implementation reconstructs structured control
//! before the shared HUGR rewrite phase. This module defines the strategy-local
//! AST used to represent that reconstructed control without exposing the shared
//! structuralization IR to the analysis phase.

use hugr::{Node, types::TypeRow};

use crate::control::structuralize::{
    RegionIo, StructuredBlock, StructuredBranchJoinKind, StructuredLoopEdge, StructuredLoopKind,
};

/// One analyzed region in the Beyond-Relooper AST.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct RelooperRegion {
    /// Ordered values crossing the region boundary.
    pub(super) io: RegionIo,
    /// Structured body of the region.
    pub(super) body: RelooperBody,
}

/// One structured body in the Beyond-Relooper AST.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) enum RelooperBody {
    /// A straight-line sequence of blocks and nested regions.
    Sequence(Vec<RelooperNode>),
    /// A structured branch rooted at one CFG split block.
    Branch {
        /// Split block executed before arm dispatch.
        split: StructuredBlock,
        /// Per-arm bodies.
        arms: Vec<Vec<RelooperNode>>,
        /// Join block merging the branch arms.
        join: StructuredBlock,
        /// Whether the join lowers inside the region or in the surrounding scope.
        join_kind: StructuredBranchJoinKind,
    },
    /// A structured loop rooted at one loop header.
    Loop {
        /// Loop family selected from the CFG.
        kind: StructuredLoopKind,
        /// Header block that guards or enters the loop.
        header: StructuredBlock,
        /// One logical iteration of the loop body.
        body: Vec<RelooperNode>,
        /// CFG block whose successor returns control to the header.
        backedge_source: Node,
        /// Continue edge routed back to the loop header.
        continue_edge: StructuredLoopEdge,
        /// Distinct exits routed out of the loop.
        exits: Vec<RelooperLoopExit>,
        /// Immediate payload row consumed by the `TailLoop` break path.
        break_outputs: TypeRow,
    },
}

/// One loop exit with its immediate payload and strategy-local continuation.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct RelooperLoopExit {
    /// Break edges that select this exit variant.
    pub(super) edges: Vec<StructuredLoopEdge>,
    /// Immediate payload emitted when the loop exits through this variant.
    pub(super) outputs: TypeRow,
    /// Continuation lowered after the loop exits through this variant.
    pub(super) continuation: Vec<RelooperNode>,
}

/// One item in a Beyond-Relooper sequence.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) enum RelooperNode {
    /// A leaf CFG block.
    Block(StructuredBlock),
    /// A nested structured region.
    Region(Box<RelooperRegion>),
}
