//! Detached template construction for structured CFG rewrites.
//!
//! The template phase builds a standalone structured HUGR using the shared
//! `Conditional` / `TailLoop` lowering logic. Original CFG blocks are not
//! cloned into this template. Instead, each block becomes a valid placeholder
//! `DFG` whose body immediately panics if it is ever executed before the
//! materialization phase swaps the real block subtree back in.

mod block;
mod r#loop;

use hugr::builder::{
    Container, DFGBuilder, Dataflow, DataflowHugr, DataflowSubContainer, SubContainer,
};
use hugr::ops::handle::NodeHandle;
use hugr::types::{Signature, TypeRow};
use hugr::{Hugr, HugrView, Node, Wire};
use itertools::Itertools;
use std::collections::BTreeSet;

use super::super::types::{
    StructuralizationError, StructuredBlock, StructuredBranchJoinKind, StructuredCfgNode,
    StructuredLoopEdge, StructuredLoopExit, StructuredLoopKind, StructuredNode, StructuredRegion,
    StructuredRegionBody, structured_node_contains_block,
};

/// Detached replacement template for one CFG rewrite.
///
/// The `hugr` field holds the structured skeleton. Each entry in `placeholders`
/// records which placeholder `DFG` should be replaced with which original CFG
/// block once the template is inserted into the destination HUGR.
#[derive(Debug)]
pub(crate) struct LoweredCfgTemplate {
    /// Structured replacement skeleton built in isolation.
    pub(super) hugr: Hugr,
    /// Placeholder-to-original block mapping for materialization.
    pub(super) placeholders: Vec<BlockPlaceholder>,
}

/// Mapping from a template placeholder node back to the original CFG block.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct BlockPlaceholder {
    /// Original CFG basic block node.
    pub(super) original: Node,
    /// Placeholder `DFG` node inside the detached template.
    pub(super) placeholder: Node,
    /// Whether this placeholder should move the source subtree or clone it.
    pub(super) materialization: BlockMaterialization,
}

/// How one placeholder should obtain its CFG block subtree.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum BlockMaterialization {
    /// Move the original CFG block subtree into the structured replacement.
    Move,
    /// Clone the already-existing block subtree for a duplicated CFG node.
    Clone,
}

/// Borrowed lowering state for one analyzed loop region.
///
/// The lowering code for tail-controlled and header-controlled loops shares the
/// same typed metadata, so this struct keeps the dispatch and helper signatures
/// compact without losing meaning.
struct LoopLowering<'a> {
    /// The analyzed loop header block.
    header: &'a StructuredBlock,
    /// One-iteration body items.
    body: &'a [StructuredNode],
    /// CFG blocks that return control to the header.
    backedge_sources: &'a [Node],
    /// Continue edges routed back to the loop header.
    continue_edges: &'a [StructuredLoopEdge],
    /// Distinct exits routed out of the loop.
    exits: &'a [StructuredLoopExit],
    /// Payload row consumed by the `TailLoop` break path.
    break_outputs: &'a TypeRow,
}

/// Lowered fragment together with the sibling nodes that carry its order
/// dependencies inside the enclosing container.
struct LoweredFragment {
    /// Value wires produced by the fragment.
    outputs: Vec<Wire>,
    /// Nodes in the fragment that should be ordered after the enclosing
    /// container input or a previous sibling.
    entry_order_edges: Vec<Node>,
    /// Nodes in the fragment that should be ordered before a following sibling
    /// or the enclosing container output.
    exit_order_edges: Vec<Node>,
}

impl LoweredFragment {
    /// Creates a fragment for one lowered sibling node.
    fn ordered(outputs: Vec<Wire>, node: Node) -> Self {
        Self {
            outputs,
            entry_order_edges: vec![node],
            exit_order_edges: vec![node],
        }
    }

    /// Creates an order-less fragment, used for exit blocks that only validate
    /// an existing row of wires.
    fn passthrough(outputs: Vec<Wire>) -> Self {
        Self {
            outputs,
            entry_order_edges: Vec::new(),
            exit_order_edges: Vec::new(),
        }
    }
}

/// Builds one detached template from the analyzed structured region.
///
/// # Errors
///
/// Returns an error when the analyzed region cannot be lowered into the shared
/// structured HUGR template.
pub(crate) fn prepare_cfg_replacement<H: HugrView<Node = Node>>(
    cfg_view: &H,
    region: &StructuredRegion,
) -> Result<LoweredCfgTemplate, StructuralizationError> {
    TemplateLowerer::new(cfg_view).lower_cfg_region(region)
}

/// Stateful detached lowerer for one CFG template.
///
/// The lowerer holds the source CFG view plus the placeholder mapping that will
/// later drive in-place block materialization after the template is inserted.
struct TemplateLowerer<'a, H> {
    /// Immutable view of the original CFG being rewritten.
    cfg_view: &'a H,
    /// Placeholder mapping accumulated while lowering blocks.
    placeholders: Vec<BlockPlaceholder>,
    /// Original blocks already assigned a move-based placeholder.
    seen_blocks: BTreeSet<Node>,
}

impl<'a, H: HugrView<Node = Node>> TemplateLowerer<'a, H> {
    /// Creates a lowerer for one CFG template.
    fn new(cfg_view: &'a H) -> Self {
        Self {
            cfg_view,
            placeholders: Vec::new(),
            seen_blocks: BTreeSet::new(),
        }
    }

    /// Lowers one analyzed CFG root into a detached template HUGR.
    fn lower_cfg_region(
        mut self,
        region: &StructuredRegion,
    ) -> Result<LoweredCfgTemplate, StructuralizationError> {
        let mut builder = DFGBuilder::new(Signature::new(
            region.io.inputs.clone(),
            region.io.outputs.clone(),
        ))?;
        let inputs = builder.input_wires().collect_vec();
        let fragment = self.lower_sequence(&mut builder, region.body_sequence()?, inputs)?;
        self.close_fragment(&mut builder, &fragment, None);
        let hugr = builder.finish_hugr_with_outputs(fragment.outputs)?;
        Ok(LoweredCfgTemplate {
            hugr,
            placeholders: self.placeholders,
        })
    }

    /// Lowers a linear sequence of structured nodes left-to-right.
    fn lower_sequence<B>(
        &mut self,
        builder: &mut B,
        items: &[StructuredNode],
        mut current: Vec<Wire>,
    ) -> Result<LoweredFragment, StructuralizationError>
    where
        B: Dataflow + Container,
    {
        let mut entry_order_edges = Vec::new();
        let mut previous_exit_order_edges = Vec::new();
        for item in items {
            let fragment = self.lower_node(builder, item, current)?;
            self.connect_order_edges(
                builder,
                &previous_exit_order_edges,
                &fragment.entry_order_edges,
            );
            if entry_order_edges.is_empty() {
                entry_order_edges = fragment.entry_order_edges.clone();
            }
            if !fragment.exit_order_edges.is_empty() {
                previous_exit_order_edges = fragment.exit_order_edges.clone();
            }
            current = fragment.outputs;
        }
        Ok(LoweredFragment {
            outputs: current,
            entry_order_edges,
            exit_order_edges: previous_exit_order_edges,
        })
    }

    /// Dispatches lowering for either a single block or a nested region.
    fn lower_node<B>(
        &mut self,
        builder: &mut B,
        node: &StructuredNode,
        current: Vec<Wire>,
    ) -> Result<LoweredFragment, StructuralizationError>
    where
        B: Dataflow + Container,
    {
        match node {
            StructuredNode::Block(block) => self.lower_linear_block(builder, block, current),
            StructuredNode::Region(region) => self.lower_region(builder, region, current),
        }
    }

    /// Lowers one structured region according to its region kind.
    ///
    /// Sequence regions recurse directly, branch regions become a
    /// `Conditional` plus join block, and loop regions delegate to the
    /// specialized loop lowering helpers below.
    fn lower_region<B>(
        &mut self,
        builder: &mut B,
        region: &StructuredRegion,
        current: Vec<Wire>,
    ) -> Result<LoweredFragment, StructuralizationError>
    where
        B: Dataflow + Container,
    {
        match &region.body {
            StructuredRegionBody::Sequence(items) => self.lower_sequence(builder, items, current),
            StructuredRegionBody::Branch {
                split,
                arms,
                join,
                join_kind,
            } => {
                let (split_out, _) = self.lower_block(builder, split, current)?;
                let StructuredBlock::Dataflow {
                    sum_rows, outputs, ..
                } = split
                else {
                    return Err(StructuralizationError::ExpectedDataflowBlock {
                        node: split.node(),
                    });
                };
                let [control, rest @ ..] = split_out.as_slice() else {
                    return Err(StructuralizationError::UnsupportedBranch {
                        reason: "split block did not produce a control output".into(),
                    });
                };
                let mut cond = builder.conditional_builder(
                    (sum_rows.clone(), *control),
                    outputs
                        .iter()
                        .cloned()
                        .zip(rest.iter().copied())
                        .collect_vec(),
                    join.inputs().clone(),
                )?;

                if arms.len() != sum_rows.len()
                    || arms
                        .iter()
                        .map(|arm| arm.case)
                        .sorted()
                        .ne(0..sum_rows.len())
                {
                    return Err(StructuralizationError::UnsupportedBranch {
                        reason: format!(
                            "split block exposes {} cases but region has {} visible arms",
                            sum_rows.len(),
                            arms.len()
                        ),
                    });
                }

                for arm in arms {
                    let mut case = cond.case_builder(arm.case)?;
                    let case_inputs = case.input_wires().collect_vec();
                    let case_fragment = self.lower_sequence(&mut case, &arm.body, case_inputs)?;
                    self.close_fragment(&mut case, &case_fragment, None);
                    case.finish_with_outputs(case_fragment.outputs)?;
                }

                let cond_handle = cond.finish_sub_container()?;
                let cond_node = cond_handle.node();
                let cond_out = cond_handle.outputs().collect_vec();
                match join_kind {
                    StructuredBranchJoinKind::Inline => match join {
                        StructuredBlock::Dataflow { .. } => {
                            let join_fragment = self.lower_linear_block(builder, join, cond_out)?;
                            self.connect_order_edges(
                                builder,
                                &[cond_node],
                                &join_fragment.entry_order_edges,
                            );
                            Ok(LoweredFragment {
                                outputs: join_fragment.outputs,
                                entry_order_edges: vec![cond_node],
                                exit_order_edges: if join_fragment.exit_order_edges.is_empty() {
                                    vec![cond_node]
                                } else {
                                    join_fragment.exit_order_edges
                                },
                            })
                        }
                        StructuredBlock::Exit { .. } => {
                            let join_fragment = self.lower_linear_block(builder, join, cond_out)?;
                            Ok(LoweredFragment {
                                outputs: join_fragment.outputs,
                                entry_order_edges: vec![cond_node],
                                exit_order_edges: vec![cond_node],
                            })
                        }
                    },
                    StructuredBranchJoinKind::Deferred => {
                        Ok(LoweredFragment::ordered(cond_out, cond_node))
                    }
                }
            }
            StructuredRegionBody::Loop {
                kind,
                header,
                body,
                backedge_sources,
                continue_edges,
                exits,
                break_outputs,
            } => {
                let loop_lowering = LoopLowering::new(
                    header,
                    body,
                    backedge_sources,
                    continue_edges,
                    exits,
                    break_outputs,
                );
                match kind {
                    StructuredLoopKind::TailControlled => {
                        self.lower_tail_controlled_loop(builder, loop_lowering, current)
                    }
                    StructuredLoopKind::HeaderControlled => {
                        self.lower_header_controlled_loop(builder, loop_lowering, current)
                    }
                }
            }
        }
    }

    /// Closes a fragment within the current container by connecting its
    /// explicit order edges to the container `Input`/`Output` nodes.
    fn close_fragment<B>(
        &mut self,
        builder: &mut B,
        fragment: &LoweredFragment,
        start_order_edge: Option<Node>,
    ) where
        B: Dataflow + Container,
    {
        let start = start_order_edge.unwrap_or(builder.input().node());
        self.connect_order_edges(builder, &[start], &fragment.entry_order_edges);
        self.connect_order_edges(
            builder,
            &fragment.exit_order_edges,
            &[builder.output().node()],
        );
    }

    /// Connects each source order edge to each destination order edge.
    fn connect_order_edges<B>(&mut self, builder: &mut B, sources: &[Node], targets: &[Node])
    where
        B: Dataflow + Container,
    {
        for &source in sources {
            for &target in targets {
                builder.add_other_wire(source, target);
            }
        }
    }
}
