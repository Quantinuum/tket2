//! Lowering to native QSystem operations.

use std::collections::HashSet;

use derive_more::{Display, Error, From};
use hugr::hugr::hugrmut::HugrMut;
use hugr::{HugrView, Node, core::Visibility, ops::OpType};
use itertools::Itertools as _;
use tket::passes::composable::WithScope;
use tket::passes::modifier_resolver::ModifierResolverErrors;
use tket::passes::replace_types::ReplaceTypesError;
use tket::passes::{ComposablePass, ModifierResolverPass, PassScope};

use crate::extension::qsystem::{LowerTk2Error, LowerTketToQSystemPass, QSystemPlatform};
use crate::lower_drops::LowerDropsPass;

/// Errors reported while lowering a HUGR to native QSystem operations.
#[derive(Error, Debug, Display, From)]
#[non_exhaustive]
pub enum QSystemRebasePassError {
    /// Error while resolving modifier operations.
    ModifierResolver(ModifierResolverErrors),
    /// An error from the component [`LowerTketToQSystemPass`] pass.
    LowerTk2Error(LowerTk2Error),
    /// An error from the component [`LowerDropsPass`] pass.
    LowerDropsError(ReplaceTypesError),
}

/// Lower a HUGR to operations supported by a concrete QSystem platform.
///
/// This pass performs the target-aware lowering part of the old
/// [`crate::QSystemPass`] pipeline. By default it resolves modifier operations,
/// lowers `tket.quantum` operations to platform-native QSystem operations, lowers
/// Guppy `drop` operations, and marks helper functions introduced during
/// lowering as private.
///
/// The pass supports both local and global scopes. Some lowerings require adding
/// helper functions at module scope and are therefore only performed for global
/// scopes by the underlying [`LowerTketToQSystemPass`].
#[derive(Debug, Clone)]
pub struct QSystemRebasePass {
    resolve_modifiers: bool,
    lower_drops: bool,
    hide_funcs: bool,
    /// Where to apply the pass.
    ///
    /// Configurable via [`WithScope::with_scope`].
    scope: PassScope,
    /// Target platform, which may affect how certain operations are lowered.
    platform: QSystemPlatform,
}

impl QSystemRebasePass {
    /// Load default settings for [`QSystemRebasePass`] given the target QSystem
    /// platform.
    pub fn defaults(platform: QSystemPlatform) -> Self {
        Self {
            resolve_modifiers: true,
            lower_drops: true,
            hide_funcs: true,
            scope: PassScope::default(),
            platform,
        }
    }

    /// Returns a new pass with modifier resolution enabled according to
    /// `resolve_modifiers`.
    ///
    /// On by default.
    pub fn with_resolve_modifiers(mut self, resolve_modifiers: bool) -> Self {
        self.resolve_modifiers = resolve_modifiers;
        self
    }

    /// Returns a new pass with Guppy `drop` lowering enabled according to
    /// `lower_drops`.
    ///
    /// On by default.
    pub fn with_lower_drops(mut self, lower_drops: bool) -> Self {
        self.lower_drops = lower_drops;
        self
    }

    /// Changes whether helper functions introduced by lowering are marked as
    /// private.
    ///
    /// On by default.
    pub fn with_hide_funcs(mut self, hide_funcs: bool) -> Self {
        self.hide_funcs = hide_funcs;
        self
    }

    /// Collect the public function definitions present before lowering.
    ///
    /// Lowering can introduce reusable helper functions with public visibility.
    /// The collected set lets us keep pre-existing public functions public while
    /// hiding newly introduced helpers before LLVM sees the HUGR.
    fn collect_pub_funcs(&self, hugr: &impl HugrView<Node = Node>) -> Option<HashSet<Node>> {
        self.hide_funcs.then(|| {
            hugr.children(hugr.module_root())
                .filter(|n| {
                    hugr.get_optype(*n)
                        .as_func_defn()
                        .is_some_and(|fd| fd.visibility() == &Visibility::Public)
                })
                .collect::<HashSet<_>>()
        })
    }

    /// Mark non-whitelisted function definitions as private.
    fn hide_non_pub_funcs(&self, hugr: &mut impl HugrMut<Node = Node>, pub_funcs: HashSet<Node>) {
        for n in hugr.children(hugr.module_root()).collect_vec() {
            if !pub_funcs.contains(&n)
                && let OpType::FuncDefn(fd) = hugr.optype_mut(n)
            {
                *fd.visibility_mut() = Visibility::Private;
            }
        }
    }
}

impl WithScope for QSystemRebasePass {
    fn with_scope(mut self, scope: impl Into<PassScope>) -> Self {
        self.scope = scope.into();
        self
    }
}

impl<H: HugrMut<Node = Node> + 'static> ComposablePass<H> for QSystemRebasePass {
    type Error = QSystemRebasePassError;
    type Result = ();

    fn run(&self, hugr: &mut H) -> Result<Self::Result, Self::Error> {
        if self.resolve_modifiers {
            ModifierResolverPass::default()
                .with_scope(self.scope.clone())
                .run(hugr)?;
        }

        let pub_funcs = self.collect_pub_funcs(hugr);

        LowerTketToQSystemPass::new(self.platform)
            .with_scope(self.scope.clone())
            .run(hugr)?;

        if self.lower_drops {
            LowerDropsPass::default_with_scope(self.scope.clone()).run(hugr)?;
        }

        if let Some(pub_funcs) = pub_funcs {
            self.hide_non_pub_funcs(hugr, pub_funcs);
        }

        Ok(())
    }
}
