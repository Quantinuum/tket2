//! Pass to resolve modifiers (control/dagger/power) in a Hugr.
use hugr::hugr::hugrmut::HugrMut;
use hugr::{HugrView, Node};
use hugr_passes::ComposablePass;
use hugr_passes::composable::WithScope;

use crate::modifier::modifier_resolver::ModifierResolverErrors;

use super::modifier_resolver::resolve_modifier_with_entrypoints;

/// A pass to resolve modifiers (control/dagger/power) in a Hugr.
#[derive(Default)]
pub struct ModifierResolverPass;

impl WithScope for ModifierResolverPass {
    fn with_scope(self, _scope: impl Into<hugr_passes::PassScope>) -> Self {
        // TODO: Follow scope configuration
        // <https://github.com/Quantinuum/tket2/pull/1429>
        self
    }
}

impl<H: HugrMut<Node = Node>> ComposablePass<H> for ModifierResolverPass {
    type Error = ModifierResolverErrors<H::Node>;

    /// Returns whether any drops were lowered
    type Result = ();

    fn run(&self, hugr: &mut H) -> Result<Self::Result, Self::Error> {
        resolve_modifier_with_entrypoints(hugr, [hugr.entrypoint()])
    }
}
