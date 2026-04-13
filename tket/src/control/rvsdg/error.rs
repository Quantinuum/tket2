//! Error types for RVSDG construction.
//!
//! The builder reports structural mismatches at the CFG/RVSDG boundary here so
//! the public structuralization layer can surface a stable high-level error
//! without leaking construction details into the pass-facing API.

use derive_more::{Display, Error};
use hugr::Node;

/// Errors returned while constructing the RVSDG.
#[derive(Clone, Debug, Display, Error, PartialEq, Eq)]
#[non_exhaustive]
pub enum RvsdgBuildError<T = Node> {
    /// The CFG is irreducible and needs preprocessing before RVSDG construction.
    #[display("cfg {cfg:?} is irreducible because {reason}")]
    IrreducibleCfg {
        /// CFG root that failed reducibility.
        cfg: T,
        /// Human-readable reason.
        reason: String,
    },
    /// The CFG does not expose the branch join needed by the reducible builder.
    #[display("branch rooted at {split:?} could not be structured because {reason}")]
    UnsupportedBranch {
        /// Split block that triggered the failure.
        split: T,
        /// Human-readable reason.
        reason: String,
    },
    /// The CFG does not expose the loop structure needed by the reducible builder.
    #[display("loop headed by {header:?} could not be structured because {reason}")]
    UnsupportedLoop {
        /// Loop header that triggered the failure.
        header: T,
        /// Human-readable reason.
        reason: String,
    },
    /// The builder expected a CFG block but found a different operation.
    #[display("expected a CFG block at node {node:?}")]
    ExpectedBlock {
        /// Unexpected node.
        node: T,
    },
    /// The scoped CFG walk failed to make forward progress.
    #[display("failed to structure scope starting at {start:?} because {reason}")]
    MalformedScope {
        /// First node in the malformed scope.
        start: T,
        /// Human-readable reason.
        reason: String,
    },
}
