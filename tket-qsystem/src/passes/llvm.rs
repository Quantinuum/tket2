//! Final preparation for lowering QSystem HUGRs to LLVM.

use derive_more::{Display, Error, From};
use hugr::hugr::{HugrError, hugrmut::HugrMut};
use hugr::{HugrView, Node};
use tket::TketOp;
use tket::passes::composable::WithScope;
use tket::passes::const_fold::{ConstFoldError, ConstantFoldPass};
use tket::passes::{
    ComposablePass, MonomorphizePass, PassScope, RemoveDeadFuncsError, RemoveDeadFuncsPass,
    force_order,
};

use crate::extension::futures::FutureOpDef;
use crate::extension::qsystem::{helios::HeliosOp, sol::SolOp};

/// Errors reported while preparing a QSystem HUGR for LLVM lowering.
#[derive(Error, Debug, Display, From)]
#[non_exhaustive]
pub enum QSystemLLVMPassError {
    /// An error from the component [`force_order`] pass.
    ForceOrderError(HugrError),
    /// An error from the component [`ConstantFoldPass`] pass.
    ConstantFoldError(ConstFoldError),
    /// An error when running [`RemoveDeadFuncsPass`] after monomorphisation.
    DCEError(RemoveDeadFuncsError),
    /// The entrypoint of the HUGR is not a function.
    #[display("Expected the HUGR entrypoint to be a function, but found {entrypoint_optype}.")]
    InvalidEntrypoint {
        /// The optype of the entrypoint node.
        entrypoint_optype: String,
    },
    /// QSystemLLVMPass was applied with a local scope.
    #[display("QSystemLLVMPass was applied with a local scope {scope}")]
    LocalScopeError {
        /// The scope that was applied.
        scope: PassScope,
    },
}

/// Prepare a QSystem-only HUGR for LLVM lowering.
///
/// This pass is intended to run after [`super::rebase::QSystemRebasePass`] and
/// any native-gate optimizations. It performs the final global cleanups required
/// by the LLVM lowering path. No further HUGR-level optimizations are expected
/// to run after this pass.
///
/// The pass currently requires a global scope, matching the final preparation
/// behavior of the old [`crate::QSystemPass`].
#[derive(Debug, Clone)]
pub struct QSystemLLVMPass {
    constant_fold: bool,
    monomorphize: bool,
    force_order: bool,
    /// Where to apply the pass.
    ///
    /// Configurable via [`WithScope::with_scope`].
    scope: PassScope,
}

impl Default for QSystemLLVMPass {
    fn default() -> Self {
        Self {
            constant_fold: true,
            monomorphize: true,
            force_order: true,
            scope: PassScope::default(),
        }
    }
}

impl QSystemLLVMPass {
    /// Returns a new pass with constant folding enabled according to
    /// `constant_fold`.
    ///
    /// On by default.
    pub fn with_constant_fold(mut self, constant_fold: bool) -> Self {
        self.constant_fold = constant_fold;
        self
    }

    /// Returns a new pass with monomorphization enabled according to
    /// `monomorphize`.
    ///
    /// On by default.
    pub fn with_monomorphize(mut self, monomorphize: bool) -> Self {
        self.monomorphize = monomorphize;
        self
    }

    /// Changes whether we force a total ordering on all ops in the HUGR.
    ///
    /// On by default.
    ///
    /// When enabled, we push quantum ops as early as possible, and we push
    /// `tket.futures.read` ops as late as possible.
    pub fn with_force_order(mut self, force_order: bool) -> Self {
        self.force_order = force_order;
        self
    }

    /// Check that this pass is configured with a global scope.
    pub(crate) fn check_global_scope(&self) -> Result<(), QSystemLLVMPassError> {
        if matches!(self.scope, PassScope::Global(_)) {
            Ok(())
        } else {
            Err(QSystemLLVMPassError::LocalScopeError {
                scope: self.scope.clone(),
            })
        }
    }

    /// Add order edges in the HUGR regions to force qubit frees to be as early
    /// as possible, quantum ops to be as early as possible, and Future::Reads
    /// to be as late as possible.
    fn force_order(
        &self,
        hugr: &mut impl HugrMut<Node = Node>,
    ) -> Result<(), QSystemLLVMPassError> {
        let Some(root) = self.scope.root(hugr) else {
            return Ok(());
        };

        force_order::force_order(hugr, root, |hugr, node| {
            let optype = hugr.get_optype(node);

            let is_quantum = optype.cast::<TketOp>().is_some()
                || optype.cast::<HeliosOp>().is_some()
                || optype.cast::<SolOp>().is_some();
            let is_qalloc = matches!(
                optype.cast(),
                Some(TketOp::QAlloc) | Some(TketOp::TryQAlloc)
            ) || optype.cast() == Some(HeliosOp::TryQAlloc)
                || optype.cast() == Some(SolOp::TryQAlloc);
            let is_qfree = optype.cast() == Some(TketOp::QFree)
                || optype.cast() == Some(HeliosOp::QFree)
                || optype.cast() == Some(SolOp::QFree);
            let is_read = optype.cast() == Some(FutureOpDef::Read);

            // For now qallocs and qfrees are not adequately ordered; see
            // <https://github.com/quantinuum/guppylang/issues/778>. To
            // mitigate this we push qfrees as early as possible and qallocs as
            // late as possible. To maximise laziness we push quantum ops
            // (including LazyMeasure) as early as possible and Future::Read as
            // late as possible.
            if is_qfree {
                -3
            } else if is_quantum && !is_qalloc {
                -2
            } else if is_qalloc {
                -1
            } else if !is_read {
                0
            } else {
                1
            }
        })?;
        Ok(())
    }
}

impl WithScope for QSystemLLVMPass {
    fn with_scope(mut self, scope: impl Into<PassScope>) -> Self {
        self.scope = scope.into();
        self
    }
}

impl<H: HugrMut<Node = Node> + 'static> ComposablePass<H> for QSystemLLVMPass {
    type Error = QSystemLLVMPassError;
    type Result = ();

    fn run(&self, hugr: &mut H) -> Result<Self::Result, Self::Error> {
        // The entrypoint must be a function.
        if !hugr.entrypoint_optype().is_func_defn() {
            return Err(QSystemLLVMPassError::InvalidEntrypoint {
                entrypoint_optype: hugr.entrypoint_optype().to_string(),
            });
        }

        self.check_global_scope()?;

        if self.monomorphize {
            MonomorphizePass::default_with_scope(self.scope.clone())
                .run(hugr)
                .unwrap_or_else(|never| match never {});
            RemoveDeadFuncsPass::default_with_scope(self.scope.clone()).run(hugr)?;
        }

        if self.constant_fold {
            ConstantFoldPass::default().run(hugr)?;
        }
        if self.force_order {
            self.force_order(hugr)?;
        }

        Ok(())
    }
}
