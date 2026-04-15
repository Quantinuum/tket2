//! HUGR-facing block summaries used by the Beyond-Relooper strategy.
//!
//! These helpers sit exactly at the boundary between CFG facts and the
//! strategy-local AST. They are intentionally local to `control::relooper`
//! because RVSDG no longer needs them and the shared lowering layer should not
//! own strategy-specific block analysis.

use hugr::ops::OpType;
use hugr::types::TypeRow;
use hugr::{HugrView, Node};

use crate::control::structuralize::{StructuralizationError, StructuredBlock, StructuredCfgNode};

/// Reclassifies a CFG node as either a typed dataflow block or exit block.
pub(super) fn analyze_block<H: HugrView<Node = Node>>(
    cfg_view: &H,
    cfg_node: StructuredCfgNode,
    node: Node,
) -> Result<StructuredBlock, StructuralizationError> {
    analyze_block_with_linear_successor(cfg_view, cfg_node, node, None)
}

/// Reclassifies a CFG node while recording its unique visible successor case.
pub(super) fn analyze_block_with_linear_successor<H: HugrView<Node = Node>>(
    cfg_view: &H,
    cfg_node: StructuredCfgNode,
    node: Node,
    linear_successor: Option<usize>,
) -> Result<StructuredBlock, StructuralizationError> {
    match cfg_view.get_optype(node) {
        OpType::DataflowBlock(block) => Ok(StructuredBlock::Dataflow {
            cfg_node,
            node,
            inputs: block.inputs.clone(),
            sum_rows: block.sum_rows.clone(),
            outputs: block.other_outputs.clone(),
            linear_successor,
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
pub(super) fn block_input_row<H: HugrView<Node = Node>>(
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
pub(super) fn block_successor_payload<H: HugrView<Node = Node>>(
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
pub(super) fn cfg_input_row<H: HugrView<Node = Node>>(
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
pub(super) fn cfg_output_row<H: HugrView<Node = Node>>(
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
