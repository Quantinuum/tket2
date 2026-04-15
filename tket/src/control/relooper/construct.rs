//! CFG-to-AST construction for the Beyond-Relooper strategy.
//!
//! This module owns the control reconstruction step for Beyond Relooper. It
//! consumes deterministic CFG facts and produces the strategy-local AST used by
//! the lowering stage.

mod branch;
mod context;
mod r#loop;

use std::collections::BTreeSet;

use hugr::core::HugrNode;
use hugr::{HugrView, Node};

use crate::control::CfgBlockMap;
use crate::control::cfg::{CfgFacts, CfgFactsError, PreprocessedNode};
use crate::control::structuralize::{
    IntoStructuredCfgNode, RegionIo, StructuralizationError, StructuredBlock,
};

use super::ast::{RelooperContext, RelooperLabel, RelooperRegion, RelooperStmt};
use super::block::{analyze_block, cfg_input_row, cfg_output_row};

/// One recursive scope walk in the Beyond-Relooper constructor.
pub(super) struct ScopeBuild<'a, T> {
    start: T,
    scope: &'a BTreeSet<T>,
    stop: Option<T>,
    active_loop: Option<T>,
    context: &'a RelooperContext,
}

/// Shared recursive environment for nested branch and loop construction.
pub(super) struct ScopeFrame<'a, T> {
    scope: &'a BTreeSet<T>,
    stop: Option<T>,
    active_loop: Option<T>,
    context: &'a RelooperContext,
}

/// Converts one CFG-graph node into the label used by the Beyond-Relooper AST.
///
/// Labels follow CFG nodes so debugging the reconstructed control stays close
/// to the source graph. Preprocessing may duplicate nodes, so duplicated
/// entries carry their stable clone identifiers into the label.
pub(crate) trait IntoRelooperLabel: HugrNode {
    /// Returns the strategy-local label for this CFG node.
    fn into_relooper_label(self) -> RelooperLabel;
}

impl IntoRelooperLabel for Node {
    fn into_relooper_label(self) -> RelooperLabel {
        RelooperLabel::Original(self)
    }
}

impl IntoRelooperLabel for PreprocessedNode<Node> {
    fn into_relooper_label(self) -> RelooperLabel {
        match self {
            PreprocessedNode::Original(node) => RelooperLabel::Original(node),
            PreprocessedNode::Duplicate { original, clone_id } => {
                RelooperLabel::Duplicate { original, clone_id }
            }
        }
    }
}

/// Builds the Beyond-Relooper AST for one CFG-like graph view.
///
/// The graph view may introduce synthetic nodes during preprocessing, but it
/// must still map each graph node back to one original HUGR block so block
/// summaries and lowering metadata can be recovered from the immutable HUGR
/// view.
pub(super) fn build_cfg_program<H, T, C>(
    cfg_view: &H,
    cfg: &C,
) -> Result<RelooperRegion, StructuralizationError>
where
    H: HugrView<Node = Node>,
    T: IntoRelooperLabel + IntoStructuredCfgNode,
    C: CfgBlockMap<T>,
{
    let info = CfgFacts::<T>::new(cfg.entry_node(), cfg)
        .map_err(|err| map_cfg_facts_error(cfg_view.entrypoint(), err))?;
    let body = RelooperStmt::Seq(info.build_scope(
        cfg_view,
        cfg,
        ScopeBuild {
            start: cfg.entry_node(),
            scope: &info.scope,
            stop: None,
            active_loop: None,
            context: &RelooperContext::new(),
        },
    )?);
    Ok(RelooperRegion {
        io: RegionIo {
            inputs: cfg_input_row(cfg_view)?,
            outputs: cfg_output_row(cfg_view)?,
        },
        body,
    })
}

impl<T> CfgFacts<T>
where
    T: IntoRelooperLabel + IntoStructuredCfgNode,
{
    /// Structures one linear scope until it reaches an explicit stop node.
    ///
    /// The walker emits straight-line blocks, nested branch regions, and nested
    /// loop regions while preserving CFG successor order. Nested loops are
    /// carved out before generic branching so loop headers are always lowered as
    /// loops rather than accidental multi-way branches.
    fn build_scope<H, C>(
        &self,
        cfg_view: &H,
        cfg: &C,
        request: ScopeBuild<'_, T>,
    ) -> Result<Vec<RelooperStmt>, StructuralizationError>
    where
        H: HugrView<Node = Node>,
        C: CfgBlockMap<T>,
    {
        let mut items = Vec::new();
        let mut current = Some(request.start);
        let mut seen = BTreeSet::new();

        while let Some(node) = current {
            if Some(node) == request.stop || !request.scope.contains(&node) {
                break;
            }
            if !seen.insert(node) {
                return Err(StructuralizationError::Relooper {
                    reason: format!("scope walk revisited node {node}"),
                });
            }

            if self.is_nested_loop_header(node, request.scope, request.active_loop) {
                let (region, next) = self.build_loop_region(
                    cfg_view,
                    cfg,
                    node,
                    ScopeFrame {
                        scope: request.scope,
                        stop: request.stop,
                        active_loop: request.active_loop,
                        context: request.context,
                    },
                )?;
                items.push(RelooperStmt::Region(Box::new(region)));
                current = next;
                continue;
            }

            let succs = self.scope_successors(node, request.scope, request.active_loop);
            if succs.len() > 1 {
                let (region, next) = self.build_branch_region(
                    cfg_view,
                    cfg,
                    node,
                    ScopeFrame {
                        scope: request.scope,
                        stop: request.stop,
                        active_loop: request.active_loop,
                        context: request.context,
                    },
                )?;
                items.push(RelooperStmt::Region(Box::new(region)));
                current = next;
                continue;
            }

            let block = analyze_block(
                cfg_view,
                node.into_structured_cfg_node(),
                cfg.hugr_node(node),
            )?;
            match block {
                StructuredBlock::Exit { inputs, .. } => {
                    items.push(RelooperStmt::Return(inputs));
                    break;
                }
                block => {
                    items.push(RelooperStmt::Exec(block));
                    current = succs.into_iter().next();
                }
            }
        }

        Ok(items)
    }
}

/// Maps shared CFG-facts failures into Beyond-Relooper analysis errors.
fn map_cfg_facts_error<T: HugrNode>(
    cfg_root: Node,
    err: CfgFactsError<T>,
) -> StructuralizationError {
    match err {
        CfgFactsError::NoEntryExitPath => StructuralizationError::Relooper {
            reason: "cfg has no entry-to-exit path".into(),
        },
        CfgFactsError::ReachableNodesDoNotReachExit { dropped } => {
            StructuralizationError::Relooper {
                reason: format!(
                    "reachable nodes do not all reach the CFG exit: {:?}",
                    dropped
                ),
            }
        }
        CfgFactsError::Irreducible { entries, .. } => {
            StructuralizationError::UnsupportedIrreducibleCfg {
                cfg: cfg_root,
                reason: format!("cyclic SCC has multiple entries: {:?}", entries),
            }
        }
    }
}
