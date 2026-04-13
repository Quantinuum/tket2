//! Thin pass wrapper for CFG structuralization.
//!
//! The pass delegates analysis/extraction to `crate::control::structuralize` and
//! keeps strategy/scope selection in pass-land, so future RVSDG and
//! Beyond-Relooper implementations can share one integration point.

use hugr::Node;
use hugr::hugr::hugrmut::HugrMut;
use itertools::Either;

use crate::control::structuralize::{
    StructuralizationError, StructuralizationRewriteReport, StructuralizationStrategy,
    structurize_cfgs,
};
use crate::passes::composable::WithScope;
use crate::passes::{ComposablePass, InlineDFGsPass, PassScope};

/// Pass that structuralizes CFG regions, optionally inlining helper DFGs.
#[derive(Clone, Debug)]
pub struct StructuralizeCfgsPass {
    strategy: StructuralizationStrategy,
    scope: PassScope,
    inline_dfgs: bool,
}

impl Default for StructuralizeCfgsPass {
    fn default() -> Self {
        Self {
            strategy: StructuralizationStrategy::default(),
            scope: PassScope::default(),
            inline_dfgs: true,
        }
    }
}

impl StructuralizeCfgsPass {
    /// Sets the structuralization strategy.
    pub fn with_strategy(mut self, strategy: StructuralizationStrategy) -> Self {
        self.strategy = strategy;
        self
    }

    /// Sets whether helper DFG wrappers emitted during lowering should be inlined.
    pub fn inline_dfgs(mut self, inline: bool) -> Self {
        self.inline_dfgs = inline;
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
        let ctrs = {
            let r = self.scope.root(hugr);
            if let Some(r) = r.filter(|_| self.scope.recursive()) {
                Either::Right(hugr.descendants(r))
            } else {
                Either::Left(r.into_iter())
            }
        };

        let scoped_cfgs = ctrs
            .filter(|n| hugr.get_optype(*n).is_cfg())
            .collect::<Vec<_>>();
        let report = structurize_cfgs(hugr, &scoped_cfgs, self.strategy)?;
        if self.inline_dfgs {
            for rewrite in &report.rewrites {
                if !rewrite.rewritten {
                    continue;
                }
                InlineDFGsPass::default_with_scope(PassScope::EntrypointRecursive)
                    .run(&mut hugr.with_entrypoint_mut(rewrite.cfg))
                    .unwrap();
            }
        }
        Ok(report)
    }
}
