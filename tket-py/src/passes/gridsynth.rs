//! Pass to apply GridSynth
use pyo3::prelude::*;
use tket::passes::ComposablePass;
use tket::passes::gridsynth::GridSynthPass;

use super::super::state::CompilationState;
use super::super::utils::ConvertPyErr;

#[pyfunction]
pub(super) fn gridsynth(
    circ: &mut CompilationState,
    epsilon: f64,
) -> PyResult<()> {
    GridSynthPass::default()
        .with_epsilon(epsilon)
        .run(&mut circ.hugr)
        .convert_pyerrs()?;
    Ok(())
}
