//! Bindings to allow users to access the gridsynth pass from Python.
//! The definitions here should be reflected in the
//! `tket-py/tket/_tket/passes.pyi` type stubs file
use pyo3::prelude::*;
use tket::passes::ComposablePass;
use tket::passes::gridsynth::GridSynthPass;

use super::super::state::CompilationState;
use super::super::utils::ConvertPyErr;

/// Binding to a python function called gridsynth that runs the gridsynth pass
/// behind the scenes
#[pyfunction]
pub(super) fn gridsynth(
    circ: &mut CompilationState,
    epsilon: f64,
    simplify: bool,
) -> PyResult<()> {
    // TODO: thread `simplify` through to the pass once the simplification
    // is implemented (it is currently the identity function).
    let _ = simplify;

    GridSynthPass::default()
        .with_epsilon(epsilon)
        .run(&mut circ.hugr)
        .convert_pyerrs()?;
    Ok(())
}
