//! Reducible-CFG to RVSDG construction.
//!
//! The builder uses shared CFG facts to structure a reducible CFG into nested
//! RVSDG regions with explicit `gamma` and `theta` nodes, ordered
//! arguments/results, and bundled control variables.

mod branch;
mod tail_loop;

use std::collections::BTreeSet;

use hugr::core::HugrNode;
use hugr::ops::OpType;
use hugr::types::{Type, TypeRow};
use hugr::{HugrView, Node};

use crate::control::CfgBlockMap;
use crate::control::cfg::{CfgFacts, CfgFactsError};
use crate::control::structuralize::IntoStructuredCfgNode;

use super::error::RvsdgBuildError;
use super::ir::{BlockNode, Region, RegionVar, Rvsdg, RvsdgNode, VarId};

/// Builds an RVSDG for one reducible CFG.
///
/// # Errors
///
/// Returns an error when the CFG is irreducible, when a required branch/loop
/// fact cannot be derived, or when a node expected to be a CFG block is not a
/// dataflow/exit block.
///
/// Preprocessing may introduce synthetic graph nodes that still map back to
/// original HUGR blocks. This helper keeps the graph walk generic over that
/// node type while preserving the RVSDG's references to original block nodes.
pub(super) fn build_cfg_rvsdg_with_map<H, T, C>(
    cfg_view: &H,
    cfg: &C,
) -> Result<Rvsdg, RvsdgBuildError<Node>>
where
    H: HugrView<Node = Node>,
    T: HugrNode + IntoStructuredCfgNode,
    C: CfgBlockMap<T>,
{
    let cfg_root = cfg_view.entrypoint();
    let info = CfgFacts::<T>::new(cfg.entry_node(), cfg)
        .map_err(|err| map_cfg_facts_error(cfg_root, err))?;
    let mut builder = RvsdgBuilder::new(cfg_view);
    let root = builder.build_root(cfg.entry_node(), &info.scope, &info, cfg)?;
    Ok(Rvsdg { root })
}

/// Stateful builder allocating deterministic RVSDG variables.
struct RvsdgBuilder<'a, H> {
    /// Immutable view of the CFG being structured.
    cfg_view: &'a H,
    /// Monotonic variable allocator.
    next_var: usize,
}

impl<'a, H: HugrView<Node = Node>> RvsdgBuilder<'a, H> {
    /// Creates a builder for one CFG.
    fn new(cfg_view: &'a H) -> Self {
        Self {
            cfg_view,
            next_var: 0,
        }
    }

    /// Allocates one ordered variable of the given type.
    fn fresh_var(&mut self, ty: Type) -> RegionVar {
        let id = VarId(self.next_var);
        self.next_var += 1;
        RegionVar { id, ty }
    }

    /// Allocates one ordered variable per element in a row.
    fn fresh_vars(&mut self, row: &TypeRow) -> Vec<RegionVar> {
        row.iter().cloned().map(|ty| self.fresh_var(ty)).collect()
    }

    /// Builds the root region for the whole CFG.
    fn build_root<T, C>(
        &mut self,
        start: T,
        scope: &BTreeSet<T>,
        info: &CfgFacts<T>,
        cfg: &C,
    ) -> Result<Region, RvsdgBuildError<Node>>
    where
        T: HugrNode + IntoStructuredCfgNode,
        C: CfgBlockMap<T>,
    {
        let arguments = self.cfg_signature_inputs()?;
        let results = self.cfg_signature_outputs()?;
        let body = self.build_scope(start, scope, None, None, info, cfg)?;
        Ok(Region {
            arguments,
            body,
            results,
        })
    }

    /// Returns the ordered CFG signature inputs as region arguments.
    fn cfg_signature_inputs(&mut self) -> Result<Vec<RegionVar>, RvsdgBuildError<Node>> {
        let cfg = self
            .cfg_view
            .get_optype(self.cfg_view.entrypoint())
            .as_cfg()
            .ok_or(RvsdgBuildError::ExpectedBlock {
                node: self.cfg_view.entrypoint(),
            })?;
        Ok(self.fresh_vars(&cfg.signature.input))
    }

    /// Returns the ordered CFG signature outputs as region results.
    fn cfg_signature_outputs(&mut self) -> Result<Vec<RegionVar>, RvsdgBuildError<Node>> {
        let cfg = self
            .cfg_view
            .get_optype(self.cfg_view.entrypoint())
            .as_cfg()
            .ok_or(RvsdgBuildError::ExpectedBlock {
                node: self.cfg_view.entrypoint(),
            })?;
        Ok(self.fresh_vars(&cfg.signature.output))
    }

    /// Structures one linear scope until an explicit stop node is reached.
    fn build_scope<T, C>(
        &mut self,
        start: T,
        scope: &BTreeSet<T>,
        stop: Option<T>,
        active_loop: Option<T>,
        info: &CfgFacts<T>,
        cfg: &C,
    ) -> Result<Vec<RvsdgNode>, RvsdgBuildError<Node>>
    where
        T: HugrNode + IntoStructuredCfgNode,
        C: CfgBlockMap<T>,
    {
        let mut items = Vec::new();
        let mut current = Some(start);
        let mut seen = BTreeSet::new();

        while let Some(node) = current {
            if Some(node) == stop || !scope.contains(&node) {
                break;
            }
            if !seen.insert(node) {
                return Err(RvsdgBuildError::MalformedScope {
                    start: cfg.hugr_node(start),
                    reason: format!("scope walk revisited node {node}"),
                });
            }

            if info.is_nested_loop_header(node, scope, active_loop) {
                let (theta, next) = self.build_theta(node, scope, stop, active_loop, info, cfg)?;
                items.push(RvsdgNode::Theta(Box::new(theta)));
                current = next;
                continue;
            }

            let succs = info.scope_successors(node, scope, active_loop);
            if succs.len() > 1 {
                let (gamma, next) = self.build_gamma(node, scope, stop, active_loop, info, cfg)?;
                items.push(RvsdgNode::Gamma(Box::new(gamma)));
                current = next;
                continue;
            }

            let block = self.build_block_with_linear_successor(
                node,
                cfg.hugr_node(node),
                info.scope_linear_successor_case(node, scope, active_loop),
            )?;
            let is_exit = matches!(block, BlockNode::Exit { .. });
            items.push(RvsdgNode::Block(block));
            current = succs.into_iter().next();
            if is_exit {
                break;
            }
        }

        Ok(items)
    }

    /// Converts one CFG block into a typed RVSDG leaf.
    fn build_block<T: IntoStructuredCfgNode>(
        &mut self,
        cfg_node: T,
        node: Node,
    ) -> Result<BlockNode, RvsdgBuildError<Node>> {
        self.build_block_with_linear_successor(cfg_node, node, None)
    }

    /// Converts one CFG block into a typed RVSDG leaf with straight-line successor metadata.
    fn build_block_with_linear_successor<T: IntoStructuredCfgNode>(
        &mut self,
        cfg_node: T,
        node: Node,
        linear_successor: Option<usize>,
    ) -> Result<BlockNode, RvsdgBuildError<Node>> {
        match self.cfg_view.get_optype(node) {
            OpType::DataflowBlock(block) => Ok(BlockNode::Dataflow {
                cfg_node: cfg_node.into_structured_cfg_node(),
                node,
                inputs: self.fresh_vars(&block.inputs),
                sum_rows: block
                    .sum_rows
                    .iter()
                    .map(|row| self.fresh_vars(row))
                    .collect(),
                outputs: self.fresh_vars(&block.other_outputs),
                linear_successor,
            }),
            OpType::ExitBlock(exit) => Ok(BlockNode::Exit {
                cfg_node: cfg_node.into_structured_cfg_node(),
                node,
                inputs: self.fresh_vars(&exit.cfg_outputs),
            }),
            _ => Err(RvsdgBuildError::ExpectedBlock { node }),
        }
    }
}

/// Maps shared CFG-facts failures into RVSDG construction errors.
fn map_cfg_facts_error<T: HugrNode>(
    cfg_root: Node,
    err: CfgFactsError<T>,
) -> RvsdgBuildError<Node> {
    match err {
        CfgFactsError::NoEntryExitPath => RvsdgBuildError::MalformedScope {
            start: cfg_root,
            reason: "cfg has no entry-to-exit path".into(),
        },
        CfgFactsError::ReachableNodesDoNotReachExit { dropped } => {
            RvsdgBuildError::MalformedScope {
                start: cfg_root,
                reason: format!(
                    "reachable nodes do not all reach the CFG exit: {:?}",
                    dropped
                ),
            }
        }
        CfgFactsError::Irreducible { entries, .. } => RvsdgBuildError::IrreducibleCfg {
            cfg: cfg_root,
            reason: format!("cyclic SCC has multiple entries: {:?}", entries),
        },
    }
}
