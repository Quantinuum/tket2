//! Thin pass wrapper for CFG structuralization.
//!
//! The pass delegates analysis/extraction to `crate::control::structuralize` and
//! keeps strategy/scope selection in pass-land, so future RVSDG and
//! Beyond-Relooper implementations can share one integration point.

use std::collections::HashMap;

use hugr::Node;
use hugr::hugr::hugrmut::HugrMut;
use itertools::Either;

use crate::control::structuralize::{
    StructuralizationError, StructuralizationStrategy, analyze_hugr_cfgs,
};
use crate::passes::composable::WithScope;
use crate::passes::{ComposablePass, PassScope};

/// Pass that extracts structured CFG regions from CFG nodes.
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
    type Error = StructuralizationError<Node>;
    type Result = HashMap<Node, crate::control::rvsdg::ControlRegion<Node>>;

    fn run(&self, hugr: &mut H) -> Result<Self::Result, Self::Error> {
        let ctrs = {
            let r = self.scope.root(hugr);
            if let Some(r) = r.filter(|_| self.scope.recursive()) {
                Either::Right(hugr.descendants(r))
            } else {
                Either::Left(r.into_iter())
            }
        };

        let scoped_nodes = ctrs.collect::<std::collections::HashSet<_>>();
        let report = analyze_hugr_cfgs(hugr, self.strategy)?;
        Ok(report
            .cfg_regions
            .into_iter()
            .filter(|(cfg, _)| scoped_nodes.contains(cfg))
            .collect())
    }
}
