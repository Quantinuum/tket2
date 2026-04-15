//! Block-level lowering for detached structuralization templates.

use hugr::builder::{Container, Dataflow, DataflowSubContainer};
use hugr::extension::prelude::ConstError;
use hugr::extension::prelude::UnwrapBuilder;
use hugr::ops::OpParent;
use hugr::ops::handle::NodeHandle;
use hugr::{Direction, Node, Wire};

use super::{
    BlockMaterialization, BlockPlaceholder, LoweredFragment, StructuredBlock, TemplateLowerer,
};
use crate::control::structuralize::StructuralizationError;

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
            StructuredBlock::Dataflow { .. } => {
                let (out, node) = self.lower_block(builder, block, current)?;
                Ok(LoweredFragment::ordered(
                    out.into_iter().skip(1).collect(),
                    node,
                ))
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
