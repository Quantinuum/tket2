//! Block-level lowering for detached structuralization templates.

use hugr::builder::{Container, Dataflow, DataflowSubContainer, SubContainer};
use hugr::extension::prelude::ConstError;
use hugr::extension::prelude::UnwrapBuilder;
use hugr::ops::OpParent;
use hugr::ops::handle::NodeHandle;
use hugr::types::TypeRow;
use hugr::{Direction, Node, Wire};

use super::{
    BlockMaterialization, BlockPlaceholder, LoweredFragment, StructuredBlock, TemplateLowerer,
};
use crate::control::structuralize::StructuralizationError;

/// One straight-line successor selection for a lowered block.
struct LinearSuccessorSpec<'a> {
    /// Original CFG block being lowered.
    node: Node,
    /// Ordered successor payload rows emitted by the block.
    sum_rows: &'a [TypeRow],
    /// Shared non-control outputs emitted by every successor.
    outputs: &'a TypeRow,
    /// Unique visible successor case in the current structured scope.
    case_idx: usize,
}

impl<'a, H: hugr::HugrView<Node = Node>> TemplateLowerer<'a, H> {
    /// Lowers a node that is expected to behave like straight-line dataflow.
    pub(super) fn lower_linear_block<B>(
        &mut self,
        builder: &mut B,
        block: &StructuredBlock,
        current: Vec<Wire>,
    ) -> Result<LoweredFragment, StructuralizationError>
    where
        B: Dataflow + Container,
    {
        match block {
            StructuredBlock::Dataflow {
                linear_successor, ..
            } => {
                let (out, node) = self.lower_block(builder, block, current)?;
                let outputs = self.lower_linear_outputs(builder, block, out, *linear_successor)?;
                Ok(LoweredFragment::ordered(outputs, node))
            }
            StructuredBlock::Exit { inputs, .. } => {
                let actual = current
                    .iter()
                    .map(|&wire| builder.get_wire_type(wire))
                    .collect::<Result<Vec<_>, _>>()?;
                if hugr::types::TypeRow::from(actual) != *inputs {
                    return Err(StructuralizationError::ExpectedExitBlock { node: block.node() });
                }
                Ok(LoweredFragment::passthrough(current))
            }
        }
    }

    /// Resolves the outputs of a straight-line block for its unique visible successor.
    ///
    /// Dataflow blocks expose control first, followed by shared non-control
    /// outputs. When only one CFG successor remains visible in the current
    /// structured scope, lowering must unwrap that successor's payload and
    /// concatenate it with the shared outputs instead of dropping the control
    /// sum outright.
    fn lower_linear_outputs<B>(
        &mut self,
        builder: &mut B,
        block: &StructuredBlock,
        block_outputs: Vec<Wire>,
        linear_successor: Option<usize>,
    ) -> Result<Vec<Wire>, StructuralizationError>
    where
        B: Dataflow + Container,
    {
        let StructuredBlock::Dataflow {
            node,
            sum_rows,
            outputs,
            ..
        } = block
        else {
            return Err(StructuralizationError::ExpectedDataflowBlock { node: block.node() });
        };

        let [control, rest @ ..] = block_outputs.as_slice() else {
            return Err(StructuralizationError::UnsupportedBranch {
                reason: format!("block {node} did not produce a control output"),
            });
        };

        if rest.len() != outputs.len() {
            return Err(StructuralizationError::UnsupportedBranch {
                reason: format!(
                    "block {node} output arity does not match analyzed non-control outputs"
                ),
            });
        }

        let shared_outputs = rest.to_vec();
        match linear_successor {
            None => Ok(shared_outputs),
            Some(case_idx) => self.unwrap_linear_successor_payload(
                builder,
                *control,
                shared_outputs,
                LinearSuccessorSpec {
                    node: *node,
                    sum_rows,
                    outputs,
                    case_idx,
                },
            ),
        }
    }

    /// Unwraps the payload for one known successor case and appends shared outputs.
    fn unwrap_linear_successor_payload<B>(
        &mut self,
        builder: &mut B,
        control: Wire,
        shared_outputs: Vec<Wire>,
        spec: LinearSuccessorSpec<'_>,
    ) -> Result<Vec<Wire>, StructuralizationError>
    where
        B: Dataflow + Container,
    {
        let Some(payload_row) = spec.sum_rows.get(spec.case_idx) else {
            return Err(StructuralizationError::UnsupportedBranch {
                reason: format!(
                    "block {} linear successor case {} is out of range",
                    spec.node, spec.case_idx
                ),
            });
        };
        if payload_row.is_empty() {
            return Ok(shared_outputs);
        }

        let output_row = payload_row
            .iter()
            .cloned()
            .chain(spec.outputs.iter().cloned())
            .collect::<Vec<_>>()
            .into();
        let mut cond = builder.conditional_builder(
            (spec.sum_rows.to_vec(), control),
            spec.outputs
                .iter()
                .cloned()
                .zip(shared_outputs.iter().copied())
                .collect::<Vec<_>>(),
            output_row,
        )?;

        for (current_case, current_row) in spec.sum_rows.iter().enumerate() {
            let mut case = cond.case_builder(current_case)?;
            let case_inputs = case.input_wires().collect::<Vec<_>>();
            if current_case == spec.case_idx {
                case.finish_with_outputs(case_inputs)?;
                continue;
            }
            let panic = case.add_panic(
                ConstError::new(
                    1,
                    format!(
                        "linear block {} reached unexpected successor case {current_case}",
                        spec.node
                    ),
                ),
                payload_row
                    .iter()
                    .cloned()
                    .chain(spec.outputs.iter().cloned()),
                case_inputs.into_iter().zip(
                    current_row
                        .iter()
                        .cloned()
                        .chain(spec.outputs.iter().cloned()),
                ),
            )?;
            case.finish_with_outputs(panic.outputs())?;
        }

        Ok(cond.finish_sub_container()?.outputs().collect())
    }

    /// Builds a valid placeholder `DFG` for one original CFG dataflow block.
    pub(super) fn lower_block<B>(
        &mut self,
        builder: &mut B,
        block: &StructuredBlock,
        block_inputs: Vec<Wire>,
    ) -> Result<(Vec<Wire>, Node), StructuralizationError>
    where
        B: Dataflow + Container,
    {
        let StructuredBlock::Dataflow { node, .. } = block else {
            return Err(StructuralizationError::ExpectedDataflowBlock { node: block.node() });
        };
        if block_inputs.len() != block.inputs().len() {
            return Err(StructuralizationError::Build(
                hugr::builder::BuildError::InvalidHUGR(
                    hugr::hugr::ValidationError::WrongNumberOfPorts {
                        node: *node,
                        optype: Box::new(self.cfg_view.get_optype(*node).clone()),
                        expected: block.inputs().len(),
                        actual: block_inputs.len(),
                        dir: Direction::Incoming,
                    },
                ),
            ));
        }

        let signature = self
            .cfg_view
            .get_optype(*node)
            .inner_function_type()
            .ok_or(StructuralizationError::ExpectedDataflowBlock { node: *node })?
            .into_owned();
        let mut placeholder = builder.dfg_builder(signature.clone(), block_inputs)?;
        let placeholder_node = placeholder.container_node();
        let placeholder_inputs = placeholder
            .input_wires()
            .zip(signature.input().iter().cloned())
            .collect::<Vec<_>>();
        let panic = placeholder.add_panic(
            ConstError::new(
                1,
                format!("unmaterialized structuralization placeholder for {node}"),
            ),
            signature.output().iter().cloned(),
            placeholder_inputs,
        )?;
        let handle = placeholder.finish_with_outputs(panic.outputs())?;
        let materialization = if self.seen_blocks.insert(*node) {
            BlockMaterialization::Move
        } else {
            BlockMaterialization::Clone
        };
        self.placeholders.push(BlockPlaceholder {
            original: *node,
            placeholder: placeholder_node,
            materialization,
        });
        Ok((handle.outputs().collect(), handle.node()))
    }
}
