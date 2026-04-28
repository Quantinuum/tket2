//! Python bindings for the `InlineFunctions` pass.

use pyo3::prelude::*;

use tket::passes::{ComposablePass, WithScope};

use super::PyPassScope;
use crate::state::CompilationState;
use crate::utils::ConvertPyErr;

/// Inline functions marked with the `inline="always"` decorator below the selected scope.
#[pyfunction]
#[pyo3(signature = (circ, *, scope = None))]
pub(super) fn inline_always(
    circ: &mut CompilationState,
    scope: Option<PyPassScope>,
) -> PyResult<()> {
    let py_scope = scope.unwrap_or_default();
    let pass = tket::passes::InlineAlwaysPass::default_with_scope(py_scope.scope);
    pass.run(&mut circ.hugr).convert_pyerrs()?;
    Ok(())
}
