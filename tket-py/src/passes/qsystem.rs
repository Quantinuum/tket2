//! QSystem pass bindings
use pyo3::prelude::*;
use tket::passes::{ComposablePass, WithScope};

use crate::passes::PyPassScope;
use crate::state::CompilationState;
use crate::utils::ConvertPyErr;

#[pyfunction]
#[pyo3(signature=(circ, *, resolve_modifiers = true, lower_drops = true, hide_funcs = true, scope = None))]
pub(super) fn qsystem_rebase_pass(
    circ: &mut CompilationState,
    resolve_modifiers: bool,
    lower_drops: bool,
    hide_funcs: bool,
    scope: Option<PyPassScope>,
) -> PyResult<()> {
    let py_scope = scope.unwrap_or_default();
    let qsystem_pass =
        tket_qsystem::QSystemRebasePass::defaults(tket_qsystem::QSystemPlatform::Helios)
            .with_scope(py_scope.scope)
            .with_resolve_modifiers(resolve_modifiers)
            .with_lower_drops(lower_drops)
            .with_hide_funcs(hide_funcs);

    qsystem_pass.run(&mut circ.hugr).convert_pyerrs()?;
    Ok(())
}

#[pyfunction]
#[pyo3(signature=(circ, *, constant_fold = true, monomorphize = true, force_order = true, scope = None))]
pub(super) fn qsystem_llvm_pass(
    circ: &mut CompilationState,
    constant_fold: bool,
    monomorphize: bool,
    force_order: bool,
    scope: Option<PyPassScope>,
) -> PyResult<()> {
    let py_scope = scope.unwrap_or_default();
    let qsystem_pass = tket_qsystem::QSystemLLVMPass::default()
        .with_scope(py_scope.scope)
        .with_constant_fold(constant_fold)
        .with_monomorphize(monomorphize)
        .with_force_order(force_order);

    qsystem_pass.run(&mut circ.hugr).convert_pyerrs()?;
    Ok(())
}
