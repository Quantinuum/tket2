//! Pass to resolve modifiers (control/dagger/power) in a Hugr.
//!
//! Solved original function nodes may be removed after resolution when they
//! are no longer needed and the configured pass scope allows removing them.
use crate::modifier::modifier_resolver::resolve_modifier_with_entrypoints_and_scope;
use crate::passes::{ComposablePass, PassScope, WithScope};
use hugr::Node;
use hugr::hugr::hugrmut::HugrMut;

pub use crate::modifier::modifier_resolver::ModifierResolverErrors;

/// A pass to resolve modifiers (control/dagger/power) in a Hugr.
///
/// Solved original function nodes may be removed after resolution when they
/// are no longer needed and [`PassScope::in_scope`] allows them to be modified
/// freely. Nodes whose interface is preserved by the scope are kept.
#[derive(Default)]
pub struct ModifierResolverPass {
    /// Where to apply the pass.
    scope: PassScope,
}

impl WithScope for ModifierResolverPass {
    fn with_scope(mut self, scope: impl Into<crate::passes::PassScope>) -> Self {
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
        resolve_modifier_with_entrypoints_and_scope(hugr, [root], &self.scope)
    }
}
