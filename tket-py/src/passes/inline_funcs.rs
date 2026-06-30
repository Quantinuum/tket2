//! Python bindings for the `InlineFunctions` pass.

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyAnyMethods;
use tket::passes::inline_funcs::InlineFuncsHeuristic;
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
#[pyo3(signature = (circ, *, heuristic = None, scope = None))]
pub(super) fn inline_functions(
    circ: &mut CompilationState,
    heuristic: Option<PyInlineFuncsHeuristic>,
    scope: Option<PyPassScope>,
) -> PyResult<()> {
    let py_scope = scope.unwrap_or_default();
    let heuristic = heuristic.unwrap_or_default().0;
    let pass = tket::passes::InlineFunctionsPass::default_with_scope(py_scope.scope)
        .with_heuristic(heuristic);
    pass.run(&mut circ.hugr).convert_pyerrs()?;
    Ok(())
}

#[derive(Clone, Debug, Default)]
pub(super) struct PyInlineFuncsHeuristic(pub(super) InlineFuncsHeuristic);

impl<'a, 'py> FromPyObject<'a, 'py> for PyInlineFuncsHeuristic {
    type Error = PyErr;

    fn extract(ob: Borrowed<'a, 'py, PyAny>) -> Result<Self, Self::Error> {
        let py = ob.py();
        let inline_mad = py.import("tket.passes.inline_funcs")?;
        let max_size_ty = inline_mad.getattr("MaxSize")?;
        if ob.is_instance(&max_size_ty)? {
            let size = ob.getattr("size")?.extract()?;
            return Ok(Self(InlineFuncsHeuristic::MaxSize(size)));
        }

        let all_ty = inline_mad.getattr("All")?;
        if ob.is_instance(&all_ty)? {
            return Ok(Self(InlineFuncsHeuristic::All));
        }

        Err(PyErr::new::<PyValueError, _>(
            "Unknown InlineFuncsHeuristic instance. Expected tket.passes.MaxSize(...) or tket.passes.All().",
        ))
    }
}
