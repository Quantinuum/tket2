//! Pass to resolve modifiers (control/dagger/power) in a Hugr.
use hugr::hugr::hugrmut::HugrMut;
use hugr::{HugrView, Node};
use hugr_passes::ComposablePass;

use crate::modifier::modifier_resolver::ModifierResolverErrors;

use super::modifier_resolver::resolve_modifier_with_entrypoints;

/// A pass to resolve modifiers (control/dagger/power) in a Hugr.
#[derive(Default)]
pub struct ModifierResolverPass;

impl<H: HugrMut<Node = Node>> ComposablePass<H> for ModifierResolverPass {
    type Error = ModifierResolverErrors<H::Node>;

    /// Returns whether any drops were lowered
    type Result = ();

    fn run(&self, hugr: &mut H) -> Result<Self::Result, Self::Error> {
        resolve_modifier_with_entrypoints(hugr, [hugr.entrypoint()])
    }
}
