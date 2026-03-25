//! Pass to resolve modifiers (control/dagger/power) in a Hugr.
use hugr::Node;
use hugr::hugr::hugrmut::HugrMut;
use hugr_passes::composable::WithScope;
use hugr_passes::{ComposablePass, PassScope};

use crate::modifier::modifier_resolver::ModifierResolverErrors;

use super::modifier_resolver::resolve_modifier_with_entrypoints;

/// A pass to resolve modifiers (control/dagger/power) in a Hugr.
#[derive(Default)]
pub struct ModifierResolverPass {
    /// Where to apply the pass.
    scope: PassScope,
}

impl WithScope for ModifierResolverPass {
    fn with_scope(mut self, scope: impl Into<hugr_passes::PassScope>) -> Self {
        self.scope = scope.into();
        self
    }
}

impl<H: HugrMut<Node = Node>> ComposablePass<H> for ModifierResolverPass {
    type Error = ModifierResolverErrors<H::Node>;

    /// Returns whether any drops were lowered
    type Result = ();

    fn run(&self, hugr: &mut H) -> Result<Self::Result, Self::Error> {
        let Some(root) = self.scope.root(hugr) else {
            return Ok(());
        };
        resolve_modifier_with_entrypoints(hugr, [root])
    }
}
