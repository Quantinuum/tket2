//! Compatibility pass that runs the full QSystem preparation pipeline.
//!
//! This module will be removed in a future release. New callers should use [`QSystemRebasePass`] and [`QSystemLLVMPass`] directly.
#![expect(deprecated)]

use derive_more::{Display, Error};
use hugr::Node;
use hugr::hugr::{HugrError, hugrmut::HugrMut};
use tket::passes::composable::WithScope;
use tket::passes::const_fold::ConstFoldError;
use tket::passes::modifier_resolver::ModifierResolverErrors;
use tket::passes::replace_types::ReplaceTypesError;
use tket::passes::{ComposablePass, PassScope, RemoveDeadFuncsError};

use crate::extension::qsystem::{LowerTk2Error, QSystemPlatform};

use super::llvm::{QSystemLLVMPass, QSystemLLVMPassError};
use super::rebase::{QSystemRebasePass, QSystemRebasePassError};

/// Errors reported from [`QSystemPass`].
#[derive(Error, Debug, Display)]
#[non_exhaustive]
#[deprecated(
    since = "0.27.0",
    note = "QSystemPass and its error type have been split into QSystemRebasePass and QSystemLLVMPass."
)]
pub enum QSystemPassError {
    /// An error from the component [`tket::passes::force_order`] pass.
    ForceOrderError(HugrError),
    /// An error from the component
    /// [`LowerTketToQSystemPass`][crate::extension::qsystem::LowerTketToQSystemPass]
    /// pass.
    LowerTk2Error(LowerTk2Error),
    /// An error from the component [`ConstantFoldPass`] pass.
    ///
    /// [`ConstantFoldPass`]: tket::passes::ConstantFoldPass
    ConstantFoldError(ConstFoldError),
    /// An error from the component
    /// [`LowerDropsPass`][crate::lower_drops::LowerDropsPass] pass.
    LinearizeArrayError(ReplaceTypesError),
    /// An error when running [`RemoveDeadFuncsPass`] after monomorphisation.
    ///
    /// [`RemoveDeadFuncsPass`]: tket::passes::RemoveDeadFuncsPass
    DCEError(RemoveDeadFuncsError),
    /// Error while resolving modifier operations.
    ModifierResolver(ModifierResolverErrors),
    /// No [FuncDefn] named "main" in [Module].
    ///
    /// [FuncDefn]: hugr::ops::FuncDefn
    /// [Module]: hugr::ops::Module
    #[display("No function named 'main' in module.")]
    NoMain,
    /// QSystemPass was applied with a local scope.
    #[display("QSystemPass was applied with a local scope {scope}")]
    LocalScopeError {
        /// The scope that was applied.
        scope: PassScope,
    },
}

impl From<QSystemRebasePassError> for QSystemPassError {
    fn from(error: QSystemRebasePassError) -> Self {
        match error {
            QSystemRebasePassError::ModifierResolver(error) => Self::ModifierResolver(error),
            QSystemRebasePassError::LowerTk2Error(error) => Self::LowerTk2Error(error),
            QSystemRebasePassError::LowerDropsError(error) => Self::LinearizeArrayError(error),
        }
    }
}

impl From<QSystemLLVMPassError> for QSystemPassError {
    fn from(error: QSystemLLVMPassError) -> Self {
        match error {
            QSystemLLVMPassError::ForceOrderError(error) => Self::ForceOrderError(error),
            QSystemLLVMPassError::ConstantFoldError(error) => Self::ConstantFoldError(error),
            QSystemLLVMPassError::DCEError(error) => Self::DCEError(error),
            QSystemLLVMPassError::InvalidEntrypoint { .. } => Self::NoMain,
            QSystemLLVMPassError::LocalScopeError { scope } => Self::LocalScopeError { scope },
        }
    }
}

/// Modify a HUGR into a form that is acceptable for ingress into a Q-System.
///
/// This is a compatibility wrapper for the historical `QSystemPass` API. New
/// callers that need a pass boundary for native-gate optimization should run
/// [`QSystemRebasePass`], perform any native optimizations, and then run
/// [`QSystemLLVMPass`].
///
/// This pass should only be applied with [`PassScope::Global`] scopes on HUGRs
/// with function entrypoints. An error will be returned if this is not the case.
#[derive(Debug, Clone)]
#[deprecated(
    since = "0.27.0",
    note = "QSystemPass has been split into QSystemRebasePass and QSystemLLVMPass."
)]
pub struct QSystemPass {
    native: QSystemRebasePass,
    llvm: QSystemLLVMPass,
}

impl QSystemPass {
    /// Load default settings for [`QSystemPass`] given the target QSystem
    /// platform.
    pub fn defaults(platform: QSystemPlatform) -> Self {
        Self {
            native: QSystemRebasePass::defaults(platform),
            llvm: QSystemLLVMPass::default(),
        }
    }

    /// Returns a new pass with constant folding enabled according to
    /// `constant_fold`.
    ///
    /// On by default.
    pub fn with_constant_fold(mut self, constant_fold: bool) -> Self {
        self.llvm = self.llvm.with_constant_fold(constant_fold);
        self
    }

    /// Returns a new pass with monomorphization enabled according to
    /// `monomorphize`.
    ///
    /// On by default.
    pub fn with_monomorphize(mut self, monomorphize: bool) -> Self {
        self.llvm = self.llvm.with_monomorphize(monomorphize);
        self
    }

    /// Changes whether we force a total ordering on all ops in the HUGR.
    ///
    /// On by default.
    pub fn with_force_order(mut self, force_order: bool) -> Self {
        self.llvm = self.llvm.with_force_order(force_order);
        self
    }

    /// Changes whether helper functions introduced by QSystem lowering are
    /// marked as private.
    ///
    /// On by default.
    pub fn with_hide_funcs(mut self, hide_funcs: bool) -> Self {
        self.native = self.native.with_hide_funcs(hide_funcs);
        self
    }

    /// Returns a new pass with modifier resolution enabled according to
    /// `resolve_modifiers`.
    ///
    /// On by default.
    pub fn with_resolve_modifiers(mut self, resolve_modifiers: bool) -> Self {
        self.native = self.native.with_resolve_modifiers(resolve_modifiers);
        self
    }

    /// Returns a new pass with Guppy `drop` lowering enabled according to
    /// `lower_drops`.
    ///
    /// On by default.
    pub fn with_lower_drops(mut self, lower_drops: bool) -> Self {
        self.native = self.native.with_lower_drops(lower_drops);
        self
    }
}

impl WithScope for QSystemPass {
    fn with_scope(mut self, scope: impl Into<PassScope>) -> Self {
        let scope = scope.into();
        self.native = self.native.with_scope(scope.clone());
        self.llvm = self.llvm.with_scope(scope);
        self
    }
}

impl<H: HugrMut<Node = Node> + 'static> ComposablePass<H> for QSystemPass {
    type Error = QSystemPassError;
    type Result = ();

    fn run(&self, hugr: &mut H) -> Result<Self::Result, Self::Error> {
        self.llvm.check_global_scope()?;
        self.native.run(hugr)?;
        self.llvm.run(hugr)?;
        Ok(())
    }
}
