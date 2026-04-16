//! Thin pass wrapper for CFG structuralization.
//!
//! The pass delegates analysis/extraction to `crate::control::structuralize` and
//! keeps strategy/scope selection in pass-land, so future RVSDG and
//! Beyond-Relooper implementations can share one integration point.

use std::collections::VecDeque;

use hugr::Node;
use hugr::hugr::hugrmut::HugrMut;

use crate::control::structuralize::{
    StructuralizationError, StructuralizationRewriteReport, StructuralizationStrategy,
    structurize_cfg,
};
use crate::passes::composable::WithScope;
use crate::passes::{ComposablePass, PassScope};

/// Pass that structuralizes CFG regions.
#[derive(Clone, Debug, Default)]
pub struct StructuralizeCfgsPass {
    strategy: StructuralizationStrategy,
    scope: PassScope,
}

impl StructuralizeCfgsPass {
    /// Sets the structuralization strategy.
    pub fn with_strategy(mut self, strategy: StructuralizationStrategy) -> Self {
        self.strategy = strategy;
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
        queue.extend(region_children(root, hugr));

        while let Some(region) = queue.pop_front() {
            if hugr.get_optype(region).is_cfg() {
                if structurize_cfg(hugr, region, self.strategy)? {
                    rewrites.push(region);
                }
            };

            if self.scope.recursive() {
                queue.extend(region_children(region, hugr));
            }
        }

        Ok(StructuralizationRewriteReport { rewrites })
    }
}
