//! Thin pass wrapper for CFG structuralization.
//!
//! The pass delegates analysis/extraction to `crate::control::structuralize` and
//! keeps strategy/scope selection in pass-land, so future RVSDG and
//! Beyond-Relooper implementations can share one integration point.

use std::collections::VecDeque;

use hugr::Node;
use hugr::hugr::hugrmut::HugrMut;
use hugr::ops::{OpTag, OpTrait, OpType};

use crate::control::structuralize::{
    StructuralizationError, StructuralizationRewriteReport, StructuralizationStrategy,
    structurize_cfg,
};
use crate::passes::composable::WithScope;
use crate::passes::{ComposablePass, PassScope};

/// Pass that structuralizes CFG regions.
#[derive(Clone, Debug)]
pub struct StructuralizeCfgsPass {
    strategy: StructuralizationStrategy,
    scope: PassScope,
    skip_unstructuralizable_cfgs: bool,
}

impl Default for StructuralizeCfgsPass {
    fn default() -> Self {
        Self {
            strategy: StructuralizationStrategy::default(),
            scope: PassScope::default(),
            skip_unstructuralizable_cfgs: true,
        }
    }
}

impl StructuralizeCfgsPass {
    /// Sets the structuralization strategy.
    pub fn with_strategy(mut self, strategy: StructuralizationStrategy) -> Self {
        self.strategy = strategy;
        self
    }

    /// Sets whether CFGs that cannot be structuralized should be left unchanged.
    ///
    /// When enabled, unsupported CFGs are skipped instead of returning an
    /// error from the pass. This defaults to `true`.
    pub fn with_skip_unstructuralizable_cfgs(mut self, skip_unstructuralizable_cfgs: bool) -> Self {
        self.skip_unstructuralizable_cfgs = skip_unstructuralizable_cfgs;
        self
    }
}

impl WithScope for StructuralizeCfgsPass {
    fn with_scope(mut self, scope: impl Into<PassScope>) -> Self {
        self.scope = scope.into();
        self
    }
}

impl<H: HugrMut<Node = Node>> ComposablePass<H> for StructuralizeCfgsPass {
    type Error = StructuralizationError;
    type Result = StructuralizationRewriteReport;

    fn run(&self, hugr: &mut H) -> Result<Self::Result, Self::Error> {
        let mut rewrites = Vec::new();
        let mut queue = VecDeque::new();

        let try_structurize =
            |hugr: &mut H, cfg: Node| match structurize_cfg(hugr, cfg, self.strategy) {
                Ok(rewritten) => Ok(rewritten),
                Err(err) if self.skip_unstructuralizable_cfgs && err.is_unstructuralizable() => {
                    Ok(false)
                }
                Err(err) => Err(err),
            };

        fn region_children<H: HugrMut<Node = Node>>(
            n: Node,
            hugr: &mut H,
        ) -> impl Iterator<Item = Node> + '_ {
            hugr.children(n)
                .filter(|child| hugr.children(*child).next().is_some())
        }

        let Some(root) = self.scope.root(hugr) else {
            return Ok(StructuralizationRewriteReport {
                rewrites: Vec::new(),
            });
        };
        queue.push_back(root);

        while let Some(region) = queue.pop_front() {
            if hugr.get_optype(region).is_cfg() && try_structurize(hugr, region)? {
                rewrites.push(region);
            } else if is_dataflow_region(hugr.get_optype(region)) {
                let local_cfgs = hugr
                    .children(region)
                    .filter(|node| hugr.get_optype(*node).is_cfg())
                    .collect::<Vec<_>>();
                for cfg in local_cfgs {
                    if try_structurize(hugr, cfg)? {
                        rewrites.push(cfg);
                    }
                }
            }

            if self.scope.recursive() {
                queue.extend(region_children(region, hugr));
            }
        }

        Ok(StructuralizationRewriteReport { rewrites })
    }
}

/// Returns whether one node owns a dataflow region whose direct CFG children
/// should be structuralized locally.
fn is_dataflow_region(op: &OpType) -> bool {
    OpTag::DataflowParent.is_superset(op.tag())
}
