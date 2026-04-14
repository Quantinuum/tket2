//! Shared helpers for strategy-specific structural analyses.
//!
//! Both the RVSDG path and the Beyond-Relooper path need the same HUGR-facing
//! block summaries and row extraction logic before lowering can begin. Keeping
//! those helpers here avoids duplicating the typed CFG/HUGR boundary handling.

use hugr::ops::OpType;
use hugr::types::TypeRow;
use hugr::{HugrView, Node};

use crate::control::cfg::PreprocessedNode;

use super::types::{StructuralizationError, StructuredBlock, StructuredCfgNode};

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

/// Reclassifies a CFG node as either a typed dataflow block or exit block.
pub(crate) fn analyze_block<H: HugrView<Node = Node>>(
    cfg_view: &H,
    cfg_node: StructuredCfgNode,
    node: Node,
) -> Result<StructuredBlock, StructuralizationError> {
    match cfg_view.get_optype(node) {
        OpType::DataflowBlock(block) => Ok(StructuredBlock::Dataflow {
            cfg_node,
            node,
            inputs: block.inputs.clone(),
            sum_rows: block.sum_rows.clone(),
            outputs: block.other_outputs.clone(),
        }),
        OpType::ExitBlock(exit) => Ok(StructuredBlock::Exit {
            cfg_node,
            node,
            inputs: exit.cfg_outputs.clone(),
        }),
        _ => Err(StructuralizationError::ExpectedDataflowBlock { node }),
    }
}

/// Returns the input row required by a CFG block or exit block.
pub(crate) fn block_input_row<H: HugrView<Node = Node>>(
    cfg_view: &H,
    node: Node,
) -> Result<TypeRow, StructuralizationError> {
    match cfg_view.get_optype(node) {
        OpType::DataflowBlock(block) => Ok(block.inputs.clone()),
        OpType::ExitBlock(exit) => Ok(exit.cfg_outputs.clone()),
        _ => Err(StructuralizationError::ExpectedDataflowBlock { node }),
    }
}

/// Returns the payload row carried along one outgoing successor edge.
pub(crate) fn block_successor_payload<H: HugrView<Node = Node>>(
    cfg_view: &H,
    block: Node,
    case_idx: usize,
    reason: &str,
) -> Result<TypeRow, StructuralizationError> {
    cfg_view
        .get_optype(block)
        .as_dataflow_block()
        .ok_or(StructuralizationError::ExpectedDataflowBlock { node: block })?
        .successor_input(case_idx)
        .ok_or_else(|| StructuralizationError::Relooper {
            reason: reason.into(),
        })
}

/// Returns the entrypoint signature input row for a CFG view.
pub(crate) fn cfg_input_row<H: HugrView<Node = Node>>(
    cfg_view: &H,
) -> Result<TypeRow, StructuralizationError> {
    Ok(cfg_view
        .get_optype(cfg_view.entrypoint())
        .as_cfg()
        .ok_or(StructuralizationError::Relooper {
            reason: "cfg view is not rooted at a CFG node".into(),
        })?
        .signature
        .input
        .clone())
}

/// Returns the entrypoint signature output row for a CFG view.
pub(crate) fn cfg_output_row<H: HugrView<Node = Node>>(
    cfg_view: &H,
) -> Result<TypeRow, StructuralizationError> {
    Ok(cfg_view
        .get_optype(cfg_view.entrypoint())
        .as_cfg()
        .ok_or(StructuralizationError::Relooper {
            reason: "cfg view is not rooted at a CFG node".into(),
        })?
        .signature
        .output
        .clone())
}
