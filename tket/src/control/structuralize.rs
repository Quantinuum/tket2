//! HUGR-specific entry points for CFG structuralization analysis and lowering.
//!
//! This module keeps HUGR-facing concerns out of `control::rvsdg`: typed block
//! interfaces, region I/O summaries, and lowering from the structural control
//! tree into nested `DFG` / `Conditional` / `TailLoop` nodes.

use std::collections::{HashMap, HashSet};

use derive_more::{Display, Error};
use hugr::builder::{
    BuildError, Container, DFGBuilder, Dataflow, DataflowHugr, DataflowSubContainer, SubContainer,
};
use hugr::hugr::hugrmut::HugrMut;
use hugr::ops::{DFG, OpParent, OpTrait, OpType};
use hugr::types::{Signature, Type, TypeRow};
use hugr::{Hugr, HugrView, Node, Wire};
use hugr_core::hugr::internal::HugrMutInternals;
use hugr_core::ops::OpTag;
use itertools::Itertools;

use crate::control::{CfgNodeMap, IdentityCfgMap, rvsdg};
use crate::passes::composable::ComposablePass;
use crate::passes::{NormalizeCFGPass, normalize_cfgs::NormalizeCFGError};

/// Strategy selector for control-flow structuralization.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum StructuralizationStrategy {
    /// Build the current RVSDG-style structured control tree and lower it.
    #[default]
    Rvsdg,
    /// Placeholder for a future Beyond-Relooper implementation.
    BeyondRelooper,
}

/// Ordered HUGR-facing interface for a structured region.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RegionIo {
    /// Values entering the region from outside it.
    pub inputs: TypeRow,
    /// Values leaving the region to the surrounding region.
    pub outputs: TypeRow,
}

/// HUGR-facing summary of a CFG block used during lowering.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum StructuredBlock {
    /// A normal CFG basic block.
    Dataflow {
        /// Original block node.
        node: Node,
        /// Dataflow inputs to the block body.
        inputs: TypeRow,
        /// Branching sum rows emitted by the block.
        sum_rows: Vec<TypeRow>,
        /// Non-control outputs emitted by the block.
        outputs: TypeRow,
    },
    /// The CFG exit block.
    Exit {
        /// Original exit block node.
        node: Node,
        /// Values consumed by the exit block.
        inputs: TypeRow,
    },
}

impl StructuredBlock {
    fn node(&self) -> Node {
        match self {
            Self::Dataflow { node, .. } | Self::Exit { node, .. } => *node,
        }
    }

    fn inputs(&self) -> &TypeRow {
        match self {
            Self::Dataflow { inputs, .. } | Self::Exit { inputs, .. } => inputs,
        }
    }
}

/// A HUGR-specific analyzed control tree node.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum StructuredNode {
    /// A single CFG block.
    Block(StructuredBlock),
    /// A nested structured region.
    Region(Box<StructuredRegion>),
}

/// Lowering family chosen for a structured loop.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StructuredLoopKind {
    /// The latch both re-enters the header and exits the loop.
    TailControlled,
    /// The header decides whether to enter the body or break immediately.
    HeaderControlled,
}

/// HUGR-specific body information for a structured region.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum StructuredRegionBody {
    /// A linear sequence of blocks / nested regions.
    Sequence(Vec<StructuredNode>),
    /// A structured branch lowered via `Conditional`.
    Branch {
        /// Split block executed before the `Conditional`.
        split: StructuredBlock,
        /// Per-arm bodies.
        arms: Vec<Vec<StructuredNode>>,
        /// Join block executed after the `Conditional`.
        join: StructuredBlock,
    },
    /// A structured loop lowered via `TailLoop`.
    Loop {
        /// Loop-shape classification used during lowering.
        kind: StructuredLoopKind,
        /// The CFG block acting as the loop header.
        header: StructuredBlock,
        /// One-iteration loop body items.
        body: Vec<StructuredNode>,
        /// CFG block whose successor returns control to the header.
        backedge_source: Node,
        /// Payload row for the continue edge.
        continue_inputs: TypeRow,
        /// Payload row for the break edge.
        break_outputs: TypeRow,
        /// Index of the latch successor that continues the loop.
        continue_case: usize,
        /// Index of the latch successor that exits the loop.
        break_case: usize,
    },
}

/// HUGR-specific structural summary for one region.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StructuredRegion {
    /// Generic RVSDG control region.
    pub control: rvsdg::ControlRegion<Node>,
    /// Ordered HUGR interface of the region.
    pub io: RegionIo,
    /// HUGR-specific lowering metadata.
    pub body: StructuredRegionBody,
}

/// Structured CFG analysis output for one HUGR.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct StructuralizationAnalysisReport {
    /// Structured control tree plus HUGR-specific lowering metadata for each CFG.
    pub cfg_regions: HashMap<Node, StructuredRegion>,
}

/// Per-CFG rewrite result.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StructuralizationRewrite {
    /// Rewritten CFG node.
    pub cfg: Node,
    /// Whether the CFG was rewritten.
    pub rewritten: bool,
}

/// Mutable rewrite report returned by the pass layer.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct StructuralizationRewriteReport {
    /// All CFG rewrites performed by the pass.
    pub rewrites: Vec<StructuralizationRewrite>,
}

/// Errors returned by HUGR-level structuralization entry points.
#[derive(Debug, Display, Error)]
#[non_exhaustive]
pub enum StructuralizationError {
    /// Error while building an RVSDG structural tree for a CFG.
    #[display("failed to build RVSDG control tree: {_0}")]
    Rvsdg(rvsdg::RvsdgBuildError<Node>),
    /// A structured region required a basic block but found something else.
    #[display("expected a dataflow block while lowering node {node}")]
    ExpectedDataflowBlock {
        /// Node that was expected to be a dataflow block.
        node: Node,
    },
    /// A structured region expected the CFG exit block.
    #[display("expected CFG exit block at node {node}")]
    ExpectedExitBlock {
        /// Node that was expected to be an exit block.
        node: Node,
    },
    /// A branch region was structurally valid but not lowerable with the current assumptions.
    #[display("branch region could not be lowered because {reason}")]
    UnsupportedBranch {
        /// Short description of the mismatch.
        reason: String,
    },
    /// A loop region was structurally valid but not lowerable with the current assumptions.
    #[display("loop region could not be lowered because {reason}")]
    UnsupportedLoop {
        /// Short description of the mismatch.
        reason: String,
    },
    /// The selected strategy is not implemented yet.
    #[display("structuralization strategy {strategy:?} is not implemented yet")]
    UnsupportedStrategy {
        /// Strategy requested by the caller.
        #[error(not(source))]
        strategy: StructuralizationStrategy,
    },
    /// Error while building structured HUGR nodes.
    #[display("failed to build structuralized HUGR: {_0}")]
    Build(BuildError),
    /// Error validating the constructed structured HUGR.
    #[display("structured HUGR validation failed: {_0}")]
    Validation(hugr::hugr::ValidationError<Node>),
    /// Error running local CFG normalization after rewriting.
    #[display("failed to normalize nested CFGs after structuralization: {_0}")]
    Normalize(NormalizeCFGError),
}

impl From<BuildError> for StructuralizationError {
    fn from(value: BuildError) -> Self {
        Self::Build(value)
    }
}

impl From<hugr::hugr::ValidationError<Node>> for StructuralizationError {
    fn from(value: hugr::hugr::ValidationError<Node>) -> Self {
        Self::Validation(value)
    }
}

impl From<NormalizeCFGError> for StructuralizationError {
    fn from(value: NormalizeCFGError) -> Self {
        Self::Normalize(value)
    }
}

/// Analyze all CFGs in a HUGR using the requested strategy.
pub fn analyze_hugr_cfgs<H: HugrView<Node = Node>>(
    hugr: &H,
    strategy: StructuralizationStrategy,
) -> Result<StructuralizationAnalysisReport, StructuralizationError> {
    match strategy {
        StructuralizationStrategy::Rvsdg => {
            let mut cfg_regions = HashMap::new();
            for cfg in hugr
                .nodes()
                .filter(|n| hugr.get_optype(*n).tag() == OpTag::Cfg)
            {
                let cfg_view = hugr.with_entrypoint(cfg);
                let id_cfg = IdentityCfgMap::new(cfg_view.clone());
                let analyzed = analyze_cfg_region(
                    &cfg_view,
                    &id_cfg,
                    rvsdg::build_control_tree(&id_cfg).map_err(StructuralizationError::Rvsdg)?,
                )?;
                cfg_regions.insert(cfg, analyzed);
            }
            Ok(StructuralizationAnalysisReport { cfg_regions })
        }
        StructuralizationStrategy::BeyondRelooper => {
            Err(StructuralizationError::UnsupportedStrategy { strategy })
        }
    }
}

/// Structuralize the specified CFGs and rewrite them in place.
pub fn structurize_cfgs<H: HugrMut<Node = Node>>(
    hugr: &mut H,
    cfgs: &[Node],
    strategy: StructuralizationStrategy,
) -> Result<StructuralizationRewriteReport, StructuralizationError> {
    match strategy {
        StructuralizationStrategy::Rvsdg => {}
        StructuralizationStrategy::BeyondRelooper => {
            return Err(StructuralizationError::UnsupportedStrategy { strategy });
        }
    }

    let cfgs = outermost_cfgs(hugr, cfgs);
    let mut prepared = Vec::with_capacity(cfgs.len());

    for &cfg in &cfgs {
        let cfg_view = hugr.with_entrypoint(cfg);
        let id_cfg = IdentityCfgMap::new(cfg_view.clone());
        let analyzed = analyze_cfg_region(
            &cfg_view,
            &id_cfg,
            rvsdg::build_control_tree(&id_cfg).map_err(StructuralizationError::Rvsdg)?,
        )?;
        let replacement = lower_cfg_region(&cfg_view, &analyzed)?;
        prepared.push((cfg, replacement));
    }

    let mut rewrites = Vec::with_capacity(prepared.len());
    for (cfg, replacement) in prepared {
        rewrite_cfg_as_dfg(hugr, cfg, replacement);
        NormalizeCFGPass::default().run(&mut hugr.with_entrypoint_mut(cfg))?;
        rewrites.push(StructuralizationRewrite {
            cfg,
            rewritten: true,
        });
    }

    Ok(StructuralizationRewriteReport { rewrites })
}

fn outermost_cfgs<H: HugrView<Node = Node>>(hugr: &H, cfgs: &[Node]) -> Vec<Node> {
    let cfg_set = cfgs.iter().copied().collect::<HashSet<_>>();
    cfgs.iter()
        .copied()
        .filter(|cfg| {
            let mut parent = hugr.get_parent(*cfg);
            while let Some(node) = parent {
                if cfg_set.contains(&node) {
                    return false;
                }
                parent = hugr.get_parent(node);
            }
            true
        })
        .collect()
}

fn analyze_cfg_region<H: HugrView<Node = Node>>(
    cfg_view: &H,
    cfg: &IdentityCfgMap<H>,
    region: rvsdg::ControlRegion<Node>,
) -> Result<StructuredRegion, StructuralizationError> {
    let io = RegionIo {
        inputs: region_input_row(cfg_view, &region)?,
        outputs: region_output_row(cfg_view, &region)?,
    };

    let body = match &region.body {
        rvsdg::ControlRegionBody::Sequence(items) => StructuredRegionBody::Sequence(
            items
                .iter()
                .cloned()
                .map(|item| analyze_node(cfg_view, cfg, item))
                .collect::<Result<Vec<_>, _>>()?,
        ),
        rvsdg::ControlRegionBody::Branch { split, arms, join } => {
            let [split] = split.as_slice() else {
                return Err(StructuralizationError::UnsupportedBranch {
                    reason: format!("expected exactly one split block, found {}", split.len()),
                });
            };
            let [join] = join.as_slice() else {
                return Err(StructuralizationError::UnsupportedBranch {
                    reason: format!("expected exactly one join block, found {}", join.len()),
                });
            };
            StructuredRegionBody::Branch {
                split: analyze_block(cfg_view, *split)?,
                arms: arms
                    .iter()
                    .map(|arm| {
                        arm.iter()
                            .cloned()
                            .map(|item| analyze_node(cfg_view, cfg, item))
                            .collect::<Result<Vec<_>, _>>()
                    })
                    .collect::<Result<Vec<_>, _>>()?,
                join: analyze_block(cfg_view, *join)?,
            }
        }
        rvsdg::ControlRegionBody::Loop { body } => {
            let structured_body = body
                .iter()
                .cloned()
                .map(|item| analyze_node(cfg_view, cfg, item))
                .collect::<Result<Vec<_>, _>>()?;

            let header = region
                .boundary
                .incoming
                .first()
                .map(|(_, dst)| *dst)
                .ok_or_else(|| StructuralizationError::UnsupportedLoop {
                    reason: "loop has no entry boundary".into(),
                })?;
            let header_block = analyze_block(cfg_view, header)?;
            let exit_target = region
                .boundary
                .outgoing
                .first()
                .map(|(_, dst)| *dst)
                .ok_or_else(|| StructuralizationError::UnsupportedLoop {
                    reason: "loop has no exit boundary".into(),
                })?;
            let exit_source = region
                .boundary
                .outgoing
                .first()
                .map(|(src, _)| *src)
                .ok_or_else(|| StructuralizationError::UnsupportedLoop {
                    reason: "loop has no exit boundary".into(),
                })?;
            let backedge_source = unique_backedge_source(cfg, &region, header)?;

            let loop_body = analyze_loop_body(
                cfg_view,
                cfg,
                LoopAnalysisInput {
                    region: &region,
                    boundary: LoopBoundary {
                        header,
                        backedge_source,
                        exit_source,
                        exit_target,
                    },
                    header: header_block,
                    body: structured_body,
                    io: &io,
                },
            )?;

            StructuredRegionBody::Loop {
                kind: loop_body.kind,
                header: loop_body.header,
                body: loop_body.body,
                backedge_source: loop_body.backedge_source,
                continue_inputs: loop_body.continue_inputs,
                break_outputs: loop_body.break_outputs,
                continue_case: loop_body.continue_case,
                break_case: loop_body.break_case,
            }
        }
    };

    Ok(StructuredRegion {
        control: region,
        io,
        body,
    })
}

fn analyze_node<H: HugrView<Node = Node>>(
    cfg_view: &H,
    cfg: &IdentityCfgMap<H>,
    node: rvsdg::ControlTree<Node>,
) -> Result<StructuredNode, StructuralizationError> {
    match node {
        rvsdg::ControlTree::Block(block) => {
            Ok(StructuredNode::Block(analyze_block(cfg_view, block)?))
        }
        rvsdg::ControlTree::Region(region) => Ok(StructuredNode::Region(Box::new(
            analyze_cfg_region(cfg_view, cfg, *region)?,
        ))),
    }
}

fn analyze_block<H: HugrView<Node = Node>>(
    cfg_view: &H,
    node: Node,
) -> Result<StructuredBlock, StructuralizationError> {
    match cfg_view.get_optype(node) {
        OpType::DataflowBlock(block) => Ok(StructuredBlock::Dataflow {
            node,
            inputs: block.inputs.clone(),
            sum_rows: block.sum_rows.clone(),
            outputs: block.other_outputs.clone(),
        }),
        OpType::ExitBlock(exit) => Ok(StructuredBlock::Exit {
            node,
            inputs: exit.cfg_outputs.clone(),
        }),
        _ => Err(StructuralizationError::ExpectedDataflowBlock { node }),
    }
}

struct LoopAnalysis {
    kind: StructuredLoopKind,
    header: StructuredBlock,
    body: Vec<StructuredNode>,
    backedge_source: Node,
    continue_inputs: TypeRow,
    break_outputs: TypeRow,
    continue_case: usize,
    break_case: usize,
}

#[derive(Clone, Copy)]
struct LoopBoundary {
    header: Node,
    backedge_source: Node,
    exit_source: Node,
    exit_target: Node,
}

struct LoopAnalysisInput<'a> {
    region: &'a rvsdg::ControlRegion<Node>,
    boundary: LoopBoundary,
    header: StructuredBlock,
    body: Vec<StructuredNode>,
    io: &'a RegionIo,
}

struct LoopLowering<'a> {
    header: &'a StructuredBlock,
    body: &'a [StructuredNode],
    backedge_source: Node,
    continue_inputs: &'a TypeRow,
    break_outputs: &'a TypeRow,
    continue_case: usize,
    break_case: usize,
}

impl<'a> LoopLowering<'a> {
    fn new(
        header: &'a StructuredBlock,
        body: &'a [StructuredNode],
        backedge_source: Node,
        continue_inputs: &'a TypeRow,
        break_outputs: &'a TypeRow,
        continue_case: usize,
        break_case: usize,
    ) -> Self {
        Self {
            header,
            body,
            backedge_source,
            continue_inputs,
            break_outputs,
            continue_case,
            break_case,
        }
    }

    fn lower_tail_controlled<B, H>(
        &self,
        builder: &mut B,
        cfg_view: &H,
        current: Vec<Wire>,
    ) -> Result<Vec<Wire>, StructuralizationError>
    where
        B: Dataflow + Container,
        H: HugrView<Node = Node>,
    {
        let (latch, prefix) =
            self.body
                .split_last()
                .ok_or_else(|| StructuralizationError::UnsupportedLoop {
                    reason: "loop body is empty".into(),
                })?;
        let StructuredNode::Block(latch_block @ StructuredBlock::Dataflow { node, sum_rows, .. }) =
            latch
        else {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "loop latch is not a basic block".into(),
            });
        };
        if *node != self.backedge_source {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "loop latch does not match the analyzed backedge source".into(),
            });
        }

        let mut loop_builder = builder.tail_loop_builder(
            self.continue_inputs
                .iter()
                .cloned()
                .zip(current.iter().copied())
                .collect_vec(),
            [],
            self.break_outputs.clone(),
        )?;
        let loop_sig = loop_builder.loop_signature()?.clone();
        let loop_inputs = loop_builder.input_wires().collect_vec();
        let prefix_out = lower_sequence(&mut loop_builder, cfg_view, prefix, loop_inputs)?;
        let latch_out = lower_block(&mut loop_builder, cfg_view, latch_block, prefix_out)?;
        let [latch_control, latch_rest @ ..] = latch_out.as_slice() else {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "loop latch did not produce a control value".into(),
            });
        };

        if sum_rows.len() != 2 {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: format!(
                    "loop latch must have exactly two successors, found {}",
                    sum_rows.len()
                ),
            });
        }
        if self.continue_case >= sum_rows.len() || self.break_case >= sum_rows.len() {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "loop case index is out of range".into(),
            });
        }

        let mut cond = loop_builder.conditional_builder(
            (sum_rows.clone(), *latch_control),
            self.continue_inputs
                .iter()
                .cloned()
                .zip(latch_rest.iter().copied())
                .collect_vec(),
            [Type::new_sum([
                self.continue_inputs.clone(),
                self.break_outputs.clone(),
            ])]
            .into(),
        )?;
        for case_idx in 0..sum_rows.len() {
            let mut case = cond.case_builder(case_idx)?;
            let case_inputs = case.input_wires().collect_vec();
            let result = if case_idx == self.continue_case {
                case.make_continue(loop_sig.clone(), case_inputs)?
            } else if case_idx == self.break_case {
                case.make_break(loop_sig.clone(), case_inputs)?
            } else {
                return Err(StructuralizationError::UnsupportedLoop {
                    reason: format!("unexpected loop successor case {case_idx}"),
                });
            };
            case.finish_with_outputs([result])?;
        }
        let [loop_control] = cond.finish_sub_container()?.outputs_arr();
        let loop_handle = loop_builder.finish_with_outputs(loop_control, [])?;
        Ok(loop_handle.outputs().collect())
    }

    fn lower_header_controlled<B, H>(
        &self,
        builder: &mut B,
        cfg_view: &H,
        current: Vec<Wire>,
    ) -> Result<Vec<Wire>, StructuralizationError>
    where
        B: Dataflow + Container,
        H: HugrView<Node = Node>,
    {
        let StructuredBlock::Dataflow {
            sum_rows, outputs, ..
        } = self.header
        else {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "header-controlled loop header is not a basic block".into(),
            });
        };
        if self.continue_case >= sum_rows.len() || self.break_case >= sum_rows.len() {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "loop case index is out of range".into(),
            });
        }

        let header_out = lower_block(builder, cfg_view, self.header, current)?;
        let [header_control, header_rest @ ..] = header_out.as_slice() else {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "loop header did not produce a control value".into(),
            });
        };
        if header_rest.len() != outputs.len() {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "loop header output arity does not match its analyzed row".into(),
            });
        }

        let mut cond = builder.conditional_builder(
            (sum_rows.clone(), *header_control),
            outputs
                .iter()
                .cloned()
                .zip(header_rest.iter().copied())
                .collect_vec(),
            self.break_outputs.clone(),
        )?;

        for case_idx in 0..sum_rows.len() {
            let mut case = cond.case_builder(case_idx)?;
            let case_inputs = case.input_wires().collect_vec();
            if case_idx == self.break_case {
                case.finish_with_outputs(case_inputs)?;
                continue;
            }
            if case_idx != self.continue_case {
                return Err(StructuralizationError::UnsupportedLoop {
                    reason: format!("unexpected loop successor case {case_idx}"),
                });
            }

            let mut loop_builder = case.tail_loop_builder(
                self.continue_inputs
                    .iter()
                    .cloned()
                    .zip(case_inputs.iter().copied())
                    .collect_vec(),
                [],
                self.break_outputs.clone(),
            )?;
            let loop_sig = loop_builder.loop_signature()?.clone();
            let loop_inputs = loop_builder.input_wires().collect_vec();
            let body_out = lower_sequence(&mut loop_builder, cfg_view, self.body, loop_inputs)?;
            let reevaluated = lower_block(&mut loop_builder, cfg_view, self.header, body_out)?;
            let [loop_control, loop_rest @ ..] = reevaluated.as_slice() else {
                return Err(StructuralizationError::UnsupportedLoop {
                    reason: "loop header reevaluation did not produce a control value".into(),
                });
            };
            if !self
                .body
                .iter()
                .any(|item| structured_node_contains_block(item, self.backedge_source))
            {
                return Err(StructuralizationError::UnsupportedLoop {
                    reason: "header-controlled loop body does not contain the backedge source"
                        .into(),
                });
            }

            let mut loop_cond = loop_builder.conditional_builder(
                (sum_rows.clone(), *loop_control),
                outputs
                    .iter()
                    .cloned()
                    .zip(loop_rest.iter().copied())
                    .collect_vec(),
                [Type::new_sum([
                    self.continue_inputs.clone(),
                    self.break_outputs.clone(),
                ])]
                .into(),
            )?;
            for inner_case_idx in 0..sum_rows.len() {
                let mut inner_case = loop_cond.case_builder(inner_case_idx)?;
                let inner_inputs = inner_case.input_wires().collect_vec();
                let result = if inner_case_idx == self.continue_case {
                    inner_case.make_continue(loop_sig.clone(), inner_inputs)?
                } else if inner_case_idx == self.break_case {
                    inner_case.make_break(loop_sig.clone(), inner_inputs)?
                } else {
                    return Err(StructuralizationError::UnsupportedLoop {
                        reason: format!("unexpected loop successor case {inner_case_idx}"),
                    });
                };
                inner_case.finish_with_outputs([result])?;
            }
            let [loop_control] = loop_cond.finish_sub_container()?.outputs_arr();
            let loop_handle = loop_builder.finish_with_outputs(loop_control, [])?;
            case.finish_with_outputs(loop_handle.outputs())?;
        }

        let cond_handle = cond.finish_sub_container()?;
        Ok(cond_handle.outputs().collect_vec())
    }
}

fn unique_backedge_source<H: HugrView<Node = Node>>(
    cfg: &IdentityCfgMap<H>,
    region: &rvsdg::ControlRegion<Node>,
    header: Node,
) -> Result<Node, StructuralizationError> {
    cfg.predecessors(header)
        .filter(|pred| region.blocks.contains(pred))
        .exactly_one()
        .map_err(|_| StructuralizationError::UnsupportedLoop {
            reason: "loop does not have a unique in-region backedge to the header".into(),
        })
}

fn block_successor_payload<H: HugrView<Node = Node>>(
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
        .ok_or_else(|| StructuralizationError::UnsupportedLoop {
            reason: reason.into(),
        })
}

fn analyze_loop_body<H: HugrView<Node = Node>>(
    cfg_view: &H,
    cfg: &IdentityCfgMap<H>,
    input: LoopAnalysisInput<'_>,
) -> Result<LoopAnalysis, StructuralizationError> {
    if input.body.is_empty() {
        return Err(StructuralizationError::UnsupportedLoop {
            reason: "loop body is empty".into(),
        });
    }

    if input.boundary.backedge_source == input.boundary.exit_source {
        let structured_tail =
            input
                .body
                .last()
                .ok_or_else(|| StructuralizationError::UnsupportedLoop {
                    reason: "loop body is empty".into(),
                })?;
        let StructuredNode::Block(StructuredBlock::Dataflow {
            node: tail_node, ..
        }) = structured_tail
        else {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "tail-controlled loop body does not end in a basic block".into(),
            });
        };
        if *tail_node != input.boundary.backedge_source {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "tail-controlled loop body does not end at the backedge source".into(),
            });
        }

        let successors = cfg.successors(input.boundary.backedge_source).collect_vec();
        let continue_case = successors
            .iter()
            .position(|&succ| succ == input.boundary.header)
            .ok_or_else(|| StructuralizationError::UnsupportedLoop {
                reason: "loop latch has no backedge to the header".into(),
            })?;
        let break_case = successors
            .iter()
            .position(|&succ| succ == input.boundary.exit_target)
            .ok_or_else(|| StructuralizationError::UnsupportedLoop {
                reason: "loop latch has no exit edge".into(),
            })?;
        let continue_inputs = block_successor_payload(
            cfg_view,
            input.boundary.backedge_source,
            continue_case,
            "loop continue case is out of range",
        )?;
        let break_outputs = block_successor_payload(
            cfg_view,
            input.boundary.backedge_source,
            break_case,
            "loop break case is out of range",
        )?;

        if continue_inputs != input.io.inputs {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "loop continue payload does not match the loop entry row".into(),
            });
        }
        if break_outputs != input.io.outputs {
            return Err(StructuralizationError::UnsupportedLoop {
                reason: "loop break payload does not match the loop exit row".into(),
            });
        }

        return Ok(LoopAnalysis {
            kind: StructuredLoopKind::TailControlled,
            header: input.header,
            body: input.body,
            backedge_source: input.boundary.backedge_source,
            continue_inputs,
            break_outputs,
            continue_case,
            break_case,
        });
    }

    if input.boundary.exit_source != input.boundary.header {
        return Err(StructuralizationError::UnsupportedLoop {
            reason: "header-controlled loops must exit directly from the header".into(),
        });
    }

    let StructuredNode::Block(StructuredBlock::Dataflow {
        node: first_node,
        sum_rows,
        outputs,
        ..
    }) = input
        .body
        .first()
        .ok_or_else(|| StructuralizationError::UnsupportedLoop {
            reason: "loop body is empty".into(),
        })?
    else {
        return Err(StructuralizationError::UnsupportedLoop {
            reason: "header-controlled loop header is not a basic block".into(),
        });
    };
    if *first_node != input.boundary.header {
        return Err(StructuralizationError::UnsupportedLoop {
            reason: "header-controlled loop body does not begin at the header".into(),
        });
    }

    let successors = cfg.successors(input.boundary.header).collect_vec();
    let continue_case = successors
        .iter()
        .position(|succ| input.region.blocks.contains(succ) && *succ != input.boundary.header)
        .ok_or_else(|| StructuralizationError::UnsupportedLoop {
            reason: "header-controlled loop header has no in-region continue edge".into(),
        })?;
    let break_case = successors
        .iter()
        .position(|&succ| succ == input.boundary.exit_target)
        .ok_or_else(|| StructuralizationError::UnsupportedLoop {
            reason: "header-controlled loop header has no exit edge".into(),
        })?;
    let continue_inputs = block_successor_payload(
        cfg_view,
        input.boundary.header,
        continue_case,
        "header-controlled loop continue case is out of range",
    )?;
    let break_outputs = block_successor_payload(
        cfg_view,
        input.boundary.header,
        break_case,
        "header-controlled loop break case is out of range",
    )?;

    if break_outputs != input.io.outputs {
        return Err(StructuralizationError::UnsupportedLoop {
            reason: "loop break payload does not match the loop exit row".into(),
        });
    }
    if sum_rows.len() != 2 {
        return Err(StructuralizationError::UnsupportedLoop {
            reason: format!(
                "header-controlled loop header must have exactly two successors, found {}",
                sum_rows.len()
            ),
        });
    }
    if outputs != &continue_inputs || continue_inputs != break_outputs {
        return Err(StructuralizationError::UnsupportedLoop {
            reason:
                "header-controlled loop requires identical continue, break, and header output rows"
                    .into(),
        });
    }
    if !input
        .body
        .iter()
        .skip(1)
        .any(|node| structured_node_contains_block(node, input.boundary.backedge_source))
    {
        return Err(StructuralizationError::UnsupportedLoop {
            reason: "header-controlled loop body does not contain the backedge source".into(),
        });
    }

    Ok(LoopAnalysis {
        kind: StructuredLoopKind::HeaderControlled,
        header: input.header,
        body: input.body.into_iter().skip(1).collect(),
        backedge_source: input.boundary.backedge_source,
        continue_inputs,
        break_outputs,
        continue_case,
        break_case,
    })
}

fn region_input_row<H: HugrView<Node = Node>>(
    cfg_view: &H,
    region: &rvsdg::ControlRegion<Node>,
) -> Result<TypeRow, StructuralizationError> {
    if region.kind == rvsdg::RegionKind::Root {
        return Ok(cfg_view
            .get_optype(cfg_view.entrypoint())
            .as_cfg()
            .expect("cfg view should be rooted at CFG")
            .signature
            .input
            .clone());
    }

    let entry_block = region
        .boundary
        .incoming
        .first()
        .map(|(_, dst)| *dst)
        .ok_or_else(|| StructuralizationError::UnsupportedBranch {
            reason: "region has no entry boundary".into(),
        })?;
    match cfg_view.get_optype(entry_block) {
        OpType::DataflowBlock(block) => Ok(block.inputs.clone()),
        OpType::ExitBlock(exit) => Ok(exit.cfg_outputs.clone()),
        _ => Err(StructuralizationError::ExpectedDataflowBlock { node: entry_block }),
    }
}

fn structured_node_contains_block(node: &StructuredNode, target: Node) -> bool {
    match node {
        StructuredNode::Block(StructuredBlock::Dataflow { node, .. })
        | StructuredNode::Block(StructuredBlock::Exit { node, .. }) => *node == target,
        StructuredNode::Region(region) => structured_region_contains_block(region, target),
    }
}

fn structured_region_contains_block(region: &StructuredRegion, target: Node) -> bool {
    match &region.body {
        StructuredRegionBody::Sequence(items) => items
            .iter()
            .any(|item| structured_node_contains_block(item, target)),
        StructuredRegionBody::Branch { split, arms, join } => {
            split.node() == target
                || arms
                    .iter()
                    .flatten()
                    .any(|item| structured_node_contains_block(item, target))
                || join.node() == target
        }
        StructuredRegionBody::Loop { header, body, .. } => {
            header.node() == target
                || body
                    .iter()
                    .any(|item| structured_node_contains_block(item, target))
        }
    }
}

fn region_output_row<H: HugrView<Node = Node>>(
    cfg_view: &H,
    region: &rvsdg::ControlRegion<Node>,
) -> Result<TypeRow, StructuralizationError> {
    if region.kind == rvsdg::RegionKind::Root {
        return Ok(cfg_view
            .get_optype(cfg_view.entrypoint())
            .as_cfg()
            .expect("cfg view should be rooted at CFG")
            .signature
            .output
            .clone());
    }

    let exit_target = region
        .boundary
        .outgoing
        .first()
        .map(|(_, dst)| *dst)
        .ok_or_else(|| StructuralizationError::UnsupportedBranch {
            reason: "region has no exit boundary".into(),
        })?;
    match cfg_view.get_optype(exit_target) {
        OpType::DataflowBlock(block) => Ok(block.inputs.clone()),
        OpType::ExitBlock(exit) => Ok(exit.cfg_outputs.clone()),
        _ => Err(StructuralizationError::ExpectedExitBlock { node: exit_target }),
    }
}

fn lower_cfg_region<H: HugrView<Node = Node>>(
    cfg_view: &H,
    region: &StructuredRegion,
) -> Result<Hugr, StructuralizationError> {
    let mut builder = DFGBuilder::new(Signature::new(
        region.io.inputs.clone(),
        region.io.outputs.clone(),
    ))?;
    let inputs = builder.input_wires().collect_vec();
    let state = lower_sequence(&mut builder, cfg_view, region.body_sequence()?, inputs)?;
    Ok(builder.finish_hugr_with_outputs(state)?)
}

impl StructuredRegion {
    fn body_sequence(&self) -> Result<&[StructuredNode], StructuralizationError> {
        match &self.body {
            StructuredRegionBody::Sequence(items) => Ok(items),
            _ => Err(StructuralizationError::UnsupportedBranch {
                reason: "top-level CFG should lower from a sequence region".into(),
            }),
        }
    }
}

fn lower_sequence<B, H>(
    builder: &mut B,
    cfg_view: &H,
    items: &[StructuredNode],
    mut current: Vec<Wire>,
) -> Result<Vec<Wire>, StructuralizationError>
where
    B: Dataflow + Container,
    H: HugrView<Node = Node>,
{
    for item in items {
        current = lower_node(builder, cfg_view, item, current)?;
    }
    Ok(current)
}

fn lower_node<B, H>(
    builder: &mut B,
    cfg_view: &H,
    node: &StructuredNode,
    current: Vec<Wire>,
) -> Result<Vec<Wire>, StructuralizationError>
where
    B: Dataflow + Container,
    H: HugrView<Node = Node>,
{
    match node {
        StructuredNode::Block(block) => lower_linear_block(builder, cfg_view, block, current),
        StructuredNode::Region(region) => lower_region(builder, cfg_view, region, current),
    }
}

fn lower_region<B, H>(
    builder: &mut B,
    cfg_view: &H,
    region: &StructuredRegion,
    current: Vec<Wire>,
) -> Result<Vec<Wire>, StructuralizationError>
where
    B: Dataflow + Container,
    H: HugrView<Node = Node>,
{
    match &region.body {
        StructuredRegionBody::Sequence(items) => lower_sequence(builder, cfg_view, items, current),
        StructuredRegionBody::Branch { split, arms, join } => {
            let split_out = lower_block(builder, cfg_view, split, current)?;
            let StructuredBlock::Dataflow {
                sum_rows, outputs, ..
            } = split
            else {
                return Err(StructuralizationError::ExpectedDataflowBlock { node: split.node() });
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

            if arms.len() != sum_rows.len() {
                return Err(StructuralizationError::UnsupportedBranch {
                    reason: format!(
                        "split block exposes {} cases but region has {} arms",
                        sum_rows.len(),
                        arms.len()
                    ),
                });
            }

            for (case_idx, arm) in arms.iter().enumerate() {
                let mut case = cond.case_builder(case_idx)?;
                let case_inputs = case.input_wires().collect_vec();
                let case_outputs = lower_sequence(&mut case, cfg_view, arm, case_inputs)?;
                case.finish_with_outputs(case_outputs)?;
            }

            let cond_out = cond.finish_sub_container()?.outputs().collect_vec();
            let join_out = lower_block(builder, cfg_view, join, cond_out)?;
            Ok(join_out.into_iter().skip(1).collect())
        }
        StructuredRegionBody::Loop {
            kind,
            header,
            body,
            backedge_source,
            continue_inputs,
            break_outputs,
            continue_case,
            break_case,
        } => {
            let loop_lowering = LoopLowering::new(
                header,
                body,
                *backedge_source,
                continue_inputs,
                break_outputs,
                *continue_case,
                *break_case,
            );
            match kind {
                StructuredLoopKind::TailControlled => {
                    loop_lowering.lower_tail_controlled(builder, cfg_view, current)
                }
                StructuredLoopKind::HeaderControlled => {
                    loop_lowering.lower_header_controlled(builder, cfg_view, current)
                }
            }
        }
    }
}

fn lower_linear_block<B, H>(
    builder: &mut B,
    cfg_view: &H,
    block: &StructuredBlock,
    current: Vec<Wire>,
) -> Result<Vec<Wire>, StructuralizationError>
where
    B: Dataflow + Container,
    H: HugrView<Node = Node>,
{
    match block {
        StructuredBlock::Dataflow { .. } => {
            let out = lower_block(builder, cfg_view, block, current)?;
            Ok(out.into_iter().skip(1).collect())
        }
        StructuredBlock::Exit { inputs, .. } => {
            let actual = current
                .iter()
                .map(|&wire| builder.get_wire_type(wire))
                .collect::<Result<Vec<_>, _>>()?;
            if TypeRow::from(actual) != *inputs {
                return Err(StructuralizationError::ExpectedExitBlock { node: block.node() });
            }
            Ok(current)
        }
    }
}

fn lower_block<B, H>(
    builder: &mut B,
    cfg_view: &H,
    block: &StructuredBlock,
    block_inputs: Vec<Wire>,
) -> Result<Vec<Wire>, StructuralizationError>
where
    B: Dataflow + Container,
    H: HugrView<Node = Node>,
{
    let StructuredBlock::Dataflow { node, .. } = block else {
        return Err(StructuralizationError::ExpectedDataflowBlock { node: block.node() });
    };
    if block_inputs.len() != block.inputs().len() {
        return Err(StructuralizationError::Build(BuildError::InvalidHUGR(
            hugr::hugr::ValidationError::WrongNumberOfPorts {
                node: *node,
                optype: Box::new(cfg_view.get_optype(*node).clone()),
                expected: block.inputs().len(),
                actual: block_inputs.len(),
                dir: hugr::Direction::Incoming,
            },
        )));
    }
    let mut copied_nodes = cfg_view.descendants(*node).collect_vec();
    let mut copied_roots = vec![(*node, builder.container_node())];
    let mut copied_set = copied_nodes.iter().copied().collect::<HashSet<_>>();
    let mut extra_roots = Vec::new();
    for copied in copied_nodes.iter().copied() {
        if let Some(source) = cfg_view.static_source(copied) {
            if copied_set.contains(&source) {
                continue;
            }
            match cfg_view.get_optype(source) {
                OpType::Const(_) => {
                    extra_roots.push(source);
                }
                optype => {
                    return Err(StructuralizationError::Build(
                        BuildError::HugrViewInsertionError(format!(
                            "unsupported static dependency {source} ({optype}) while lowering block {node}"
                        )),
                    ));
                }
            }
        }
    }
    for root in extra_roots {
        for descendant in cfg_view.descendants(root) {
            if copied_set.insert(descendant) {
                copied_nodes.push(descendant);
            }
        }
        copied_roots.push((root, builder.container_node()));
    }
    let insertion = builder
        .hugr_mut()
        .insert_view_forest(cfg_view, copied_nodes.iter().copied(), copied_roots)
        .map_err(|err| BuildError::HugrViewInsertionError(err.to_string()))?;
    let root = insertion.node_map[node];
    let signature = builder
        .hugr()
        .get_optype(root)
        .inner_function_type()
        .expect("inserted block should be dataflow")
        .into_owned();
    let value_output_count = signature.output_count();
    builder.hugr_mut().replace_op(root, DFG { signature });
    let (input_count, output_count) = {
        let new_op = builder.hugr().get_optype(root);
        (new_op.input_count(), new_op.output_count())
    };
    builder
        .hugr_mut()
        .set_num_ports(root, input_count, output_count);
    let root_input_count = builder.hugr().get_optype(root).input_count();
    if block_inputs.len() > root_input_count {
        return Err(StructuralizationError::Build(BuildError::InvalidHUGR(
            hugr::hugr::ValidationError::WrongNumberOfPorts {
                node: *node,
                optype: Box::new(cfg_view.get_optype(*node).clone()),
                expected: root_input_count,
                actual: block_inputs.len(),
                dir: hugr::Direction::Incoming,
            },
        )));
    }
    for (dst_port, wire) in block_inputs.into_iter().enumerate() {
        builder
            .hugr_mut()
            .connect(wire.node(), wire.source(), root, dst_port);
    }
    Ok((0..value_output_count)
        .map(|offset| Wire::new(root, offset))
        .collect())
}

fn rewrite_cfg_as_dfg<H: HugrMut<Node = Node>>(hugr: &mut H, cfg: Node, replacement: Hugr) {
    let signature = replacement
        .get_optype(replacement.entrypoint())
        .inner_function_type()
        .expect("replacement should be dataflow")
        .into_owned();

    for child in hugr.children(cfg).collect_vec() {
        hugr.remove_subtree(child);
    }
    hugr.replace_op(cfg, DFG { signature });
    let inserted = hugr.insert_hugr(cfg, replacement).inserted_entrypoint;
    while let Some(child) = hugr.first_child(inserted) {
        hugr.set_parent(child, cfg);
    }
    hugr.remove_node(inserted);
}

#[cfg(test)]
mod test {
    use super::{
        StructuralizationAnalysisReport, StructuralizationStrategy, StructuredLoopKind,
        StructuredNode, StructuredRegionBody, analyze_hugr_cfgs, structurize_cfgs,
    };
    use crate::control::nest_cfgs::test::build_conditional_in_loop_cfg;
    use crate::passes::composable::WithScope;
    use crate::passes::structuralize_cfgs::StructuralizeCfgsPass;
    use crate::passes::{ComposablePass, PassScope};
    use hugr::builder::{
        BuildError, CFGBuilder, Container, Dataflow, DataflowSubContainer, HugrBuilder,
        ModuleBuilder, endo_sig,
    };
    use hugr::extension::prelude::usize_t;
    use hugr::hugr::hugrmut::HugrMut;
    use hugr::ops::handle::{BasicBlockID, ConstID, NodeHandle};
    use hugr::ops::{OpTrait, OpType, Value};
    use hugr::types::Signature;
    use hugr::{Hugr, HugrView};
    use hugr_core::ops::OpTag;
    use itertools::Itertools;
    use std::io::BufReader;

    fn n_identity<T: DataflowSubContainer>(
        mut dataflow_builder: T,
        pred_const: &ConstID,
    ) -> Result<T::ContainerHandle, BuildError> {
        let wires = dataflow_builder.input_wires();
        let unit = dataflow_builder.load_const(pred_const);
        dataflow_builder.finish_with_outputs([unit].into_iter().chain(wires))
    }

    fn build_then_else_merge_from_if<T: AsMut<Hugr> + AsRef<Hugr>>(
        cfg: &mut CFGBuilder<T>,
        unit_const: &ConstID,
        split: BasicBlockID,
    ) -> Result<BasicBlockID, BuildError> {
        let merge = n_identity(
            cfg.simple_block_builder(endo_sig([usize_t()]), 1)?,
            unit_const,
        )?;
        let left = n_identity(
            cfg.simple_block_builder(endo_sig([usize_t()]), 1)?,
            unit_const,
        )?;
        let right = n_identity(
            cfg.simple_block_builder(endo_sig([usize_t()]), 1)?,
            unit_const,
        )?;
        cfg.branch(&split, 0, &left)?;
        cfg.branch(&split, 1, &right)?;
        cfg.branch(&left, 0, &merge)?;
        cfg.branch(&right, 0, &merge)?;
        Ok(merge)
    }

    fn build_cond_then_loop_cfg() -> Result<Hugr, BuildError> {
        let mut cfg_builder = CFGBuilder::new(Signature::new_endo([usize_t()]))?;
        let pred_const = cfg_builder.add_constant(Value::unit_sum(0, 2).expect("0 < 2"));
        let const_unit = cfg_builder.add_constant(Value::unary_unit_sum());

        let entry = n_identity(
            cfg_builder.simple_entry_builder(vec![usize_t()].into(), 1)?,
            &const_unit,
        )?;
        let split = n_identity(
            cfg_builder.simple_block_builder(endo_sig([usize_t()]), 2)?,
            &pred_const,
        )?;
        cfg_builder.branch(&entry, 0, &split)?;
        let merge = build_then_else_merge_from_if(&mut cfg_builder, &const_unit, split)?;
        let head = n_identity(
            cfg_builder.simple_block_builder(endo_sig([usize_t()]), 1)?,
            &const_unit,
        )?;
        let tail = n_identity(
            cfg_builder.simple_block_builder(endo_sig([usize_t()]), 2)?,
            &pred_const,
        )?;
        cfg_builder.branch(&merge, 0, &head)?;
        cfg_builder.branch(&head, 0, &tail)?;
        cfg_builder.branch(&tail, 1, &head)?;
        let exit = cfg_builder.exit_block();
        cfg_builder.branch(&tail, 0, &exit)?;

        Ok(cfg_builder.finish_hugr()?)
    }

    fn build_header_controlled_loop_cfg() -> Result<Hugr, BuildError> {
        let mut cfg_builder = CFGBuilder::new(Signature::new_endo([usize_t()]))?;
        let pred_const = cfg_builder.add_constant(Value::unit_sum(0, 2).expect("0 < 2"));
        let const_unit = cfg_builder.add_constant(Value::unary_unit_sum());

        let entry = n_identity(
            cfg_builder.simple_entry_builder(vec![usize_t()].into(), 1)?,
            &const_unit,
        )?;
        let header = n_identity(
            cfg_builder.simple_block_builder(endo_sig([usize_t()]), 2)?,
            &pred_const,
        )?;
        let body = n_identity(
            cfg_builder.simple_block_builder(endo_sig([usize_t()]), 1)?,
            &const_unit,
        )?;
        let after = n_identity(
            cfg_builder.simple_block_builder(endo_sig([usize_t()]), 1)?,
            &const_unit,
        )?;
        let exit = cfg_builder.exit_block();

        cfg_builder.branch(&entry, 0, &header)?;
        cfg_builder.branch(&header, 0, &after)?;
        cfg_builder.branch(&header, 1, &body)?;
        cfg_builder.branch(&body, 0, &header)?;
        cfg_builder.branch(&after, 0, &exit)?;

        Ok(cfg_builder.finish_hugr()?)
    }

    fn single_cfg_analysis(h: &Hugr) -> StructuredRegionBody {
        let report = analyze_hugr_cfgs(h, StructuralizationStrategy::Rvsdg).unwrap();
        let StructuralizationAnalysisReport { mut cfg_regions } = report;
        let (_, region) = cfg_regions.drain().exactly_one().unwrap();
        region.body
    }

    #[test]
    fn analyzes_branch_then_loop_io() -> Result<(), BuildError> {
        let h = build_cond_then_loop_cfg()?;
        let body = single_cfg_analysis(&h);
        let StructuredRegionBody::Sequence(items) = body else {
            panic!("expected root sequence");
        };
        assert_eq!(items.len(), 4);
        let StructuredNode::Region(branch) = &items[1] else {
            panic!("expected branch region");
        };
        let StructuredRegionBody::Branch { split, join, .. } = &branch.body else {
            panic!("expected branch body");
        };
        assert_eq!(split.inputs().len(), 1);
        assert_eq!(join.inputs().len(), 1);

        let StructuredNode::Region(loop_region) = &items[2] else {
            panic!("expected loop region");
        };
        let StructuredRegionBody::Loop {
            continue_inputs,
            break_outputs,
            ..
        } = &loop_region.body
        else {
            panic!("expected loop body");
        };
        assert_eq!(continue_inputs.len(), 1);
        assert_eq!(break_outputs.len(), 1);
        Ok(())
    }

    #[test]
    fn analyzes_nested_branch_inside_loop_io() -> Result<(), BuildError> {
        let (h, _, _) = build_conditional_in_loop_cfg(true)?;
        let body = single_cfg_analysis(&h);
        let StructuredRegionBody::Sequence(items) = body else {
            panic!("expected root sequence");
        };
        let StructuredNode::Region(loop_region) = &items[1] else {
            panic!("expected loop region");
        };
        let StructuredRegionBody::Loop { body, .. } = &loop_region.body else {
            panic!("expected loop body");
        };
        let StructuredNode::Region(branch_region) = &body[1] else {
            panic!("expected nested branch");
        };
        let StructuredRegionBody::Branch { arms, .. } = &branch_region.body else {
            panic!("expected branch body");
        };
        assert_eq!(arms.len(), 2);
        Ok(())
    }

    #[test]
    fn analyzes_header_controlled_loop_io() -> Result<(), BuildError> {
        let h = build_header_controlled_loop_cfg()?;
        let body = single_cfg_analysis(&h);
        let StructuredRegionBody::Sequence(items) = body else {
            panic!("expected root sequence");
        };
        let StructuredNode::Region(loop_region) = &items[1] else {
            panic!("expected loop region");
        };
        let StructuredRegionBody::Loop {
            kind,
            body,
            continue_inputs,
            break_outputs,
            ..
        } = &loop_region.body
        else {
            panic!("expected loop body");
        };
        assert_eq!(*kind, StructuredLoopKind::HeaderControlled);
        assert_eq!(body.len(), 1);
        assert_eq!(continue_inputs.len(), 1);
        assert_eq!(break_outputs.len(), 1);
        Ok(())
    }

    #[test]
    fn lowers_conditional_and_loop_cfgs() -> Result<(), BuildError> {
        let mut h = build_cond_then_loop_cfg()?;
        let cfgs = h
            .nodes()
            .filter(|n| h.get_optype(*n).tag() == OpTag::Cfg)
            .collect_vec();
        let report = structurize_cfgs(&mut h, &cfgs, StructuralizationStrategy::Rvsdg).unwrap();
        assert_eq!(report.rewrites.len(), 1);
        assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_cfg()).count(), 0);
        assert_eq!(
            h.nodes()
                .filter(|n| matches!(h.get_optype(*n), OpType::Conditional(_)))
                .count(),
            2
        );
        assert_eq!(
            h.nodes()
                .filter(|n| matches!(h.get_optype(*n), OpType::TailLoop(_)))
                .count(),
            1
        );
        Ok(())
    }

    #[test]
    fn lowers_nested_branch_inside_loop() -> Result<(), BuildError> {
        let (mut h, _, _) = build_conditional_in_loop_cfg(true)?;
        let cfgs = h
            .nodes()
            .filter(|n| h.get_optype(*n).tag() == OpTag::Cfg)
            .collect_vec();
        structurize_cfgs(&mut h, &cfgs, StructuralizationStrategy::Rvsdg).unwrap();
        assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_cfg()).count(), 0);
        assert_eq!(
            h.nodes()
                .filter(|n| matches!(h.get_optype(*n), OpType::Conditional(_)))
                .count(),
            2
        );
        assert_eq!(
            h.nodes()
                .filter(|n| matches!(h.get_optype(*n), OpType::TailLoop(_)))
                .count(),
            1
        );
        Ok(())
    }

    #[test]
    fn lowers_header_controlled_loop() -> Result<(), BuildError> {
        let mut h = build_header_controlled_loop_cfg()?;
        let cfgs = h
            .nodes()
            .filter(|n| h.get_optype(*n).tag() == OpTag::Cfg)
            .collect_vec();
        structurize_cfgs(&mut h, &cfgs, StructuralizationStrategy::Rvsdg).unwrap();
        assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_cfg()).count(), 0);
        assert_eq!(
            h.nodes()
                .filter(|n| matches!(h.get_optype(*n), OpType::Conditional(_)))
                .count(),
            2
        );
        assert_eq!(
            h.nodes()
                .filter(|n| matches!(h.get_optype(*n), OpType::TailLoop(_)))
                .count(),
            1
        );
        Ok(())
    }

    #[test]
    fn pass_rewrites_cfgs_and_keeps_strategy_error() -> Result<(), BuildError> {
        let (mut h, _, _) = build_conditional_in_loop_cfg(true)?;
        let report = StructuralizeCfgsPass::default().run(&mut h).unwrap();
        assert_eq!(report.rewrites.len(), 1);

        let (mut h, _, _) = build_conditional_in_loop_cfg(true)?;
        let err = StructuralizeCfgsPass::default()
            .with_strategy(StructuralizationStrategy::BeyondRelooper)
            .run(&mut h)
            .unwrap_err();
        assert!(matches!(
            err,
            super::StructuralizationError::UnsupportedStrategy { .. }
        ));
        Ok(())
    }

    #[test]
    fn pass_inlines_helper_dfgs_by_default() -> Result<(), BuildError> {
        let mut h = build_header_controlled_loop_cfg()?;
        StructuralizeCfgsPass::default().run(&mut h).unwrap();
        assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_cfg()).count(), 0);
        assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_dfg()).count(), 1);

        let mut h = build_header_controlled_loop_cfg()?;
        StructuralizeCfgsPass::default()
            .inline_dfgs(false)
            .run(&mut h)
            .unwrap();
        assert!(h.nodes().filter(|n| h.get_optype(*n).is_dfg()).count() > 1);
        Ok(())
    }

    #[test]
    fn pass_respects_scope_without_touching_other_cfgs() -> Result<(), BuildError> {
        let cfg_a = build_cond_then_loop_cfg()?;
        let cfg_b = build_cond_then_loop_cfg()?;
        let mut module = ModuleBuilder::new();
        let mut func_a = module.define_function("a", Signature::new_endo([usize_t()]))?;
        let [a_in] = func_a.input_wires_arr();
        let ins_a = func_a.add_hugr_with_wires(cfg_a, [a_in])?;
        func_a.finish_with_outputs(ins_a.outputs())?;
        let mut func_b = module.define_function("b", Signature::new_endo([usize_t()]))?;
        let [b_in] = func_b.input_wires_arr();
        let ins_b = func_b.add_hugr_with_wires(cfg_b, [b_in])?;
        func_b.finish_with_outputs(ins_b.outputs())?;
        let mut h = module.finish_hugr()?;

        let report = StructuralizeCfgsPass::default()
            .with_scope(PassScope::EntrypointRecursive)
            .run(&mut h.with_entrypoint_mut(ins_a.node()))
            .unwrap();
        assert_eq!(report.rewrites.len(), 1);
        assert_eq!(h.get_optype(ins_a.node()).tag(), OpTag::Dfg);
        assert!(h.get_optype(ins_b.node()).is_cfg());
        assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_cfg()).count(), 1);
        Ok(())
    }

    #[test]
    fn structurizes_guppy_complex_control_fixture() {
        let reader = BufReader::new(
            include_bytes!(
                "../../../test_files/guppy_optimization/complex_control/complex_control.hugr"
            )
            .as_slice(),
        );
        let mut h = Hugr::load(reader, None).unwrap();

        let report = StructuralizeCfgsPass::default().run(&mut h).unwrap();
        assert!(!report.rewrites.is_empty());
        assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_cfg()).count(), 0);
        assert!(
            h.nodes()
                .any(|n| matches!(h.get_optype(n), OpType::TailLoop(_)))
        );
        assert!(
            h.nodes()
                .filter(|n| matches!(h.get_optype(*n), OpType::Conditional(_)))
                .count()
                >= 1
        );
    }
}
