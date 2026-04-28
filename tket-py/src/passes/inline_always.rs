//! Python bindings for the `InlineFunctions` pass.

use pyo3::prelude::*;

use tket::passes::{ComposablePass, WithScope};

use super::PyPassScope;
use crate::state::CompilationState;
use crate::utils::ConvertPyErr;

/// Inline acyclic function calls below the selected scope.
///
/// Parameters:
/// - heuristic: Heuristic used to choose which non-recursive functions to
///   inline. Defaults to `tket.passes.MaxSize(64)`.
/// - follow_inline_hints: Whether to follow compiler hints for inlining
///   functions.
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
