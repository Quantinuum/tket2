//! Bindings to allow users to access the gridsynth pass from Python.
//! The definitions here should be reflected in the
//! `tket-py/tket/_tket/passes.pyi` type stubs file
use crate::circuit::CircuitType;
use crate::circuit::try_with_circ;
use crate::utils::{ConvertPyErr, create_py_exception};
use pyo3::prelude::*;
use tket::passes::gridsynth::apply_gridsynth_pass;

create_py_exception!(
    tket::passes::gridsynth::GridsynthError,
    PyGridsynthError,
    "Errors from the gridsynth pass."
);

/// Binding to a python function called gridsynth that runs the rust function called
/// apply_gridsynth pass behind the scenes
#[pyfunction]
pub fn gridsynth<'py>(
    circ: &Bound<'py, PyAny>,
    epsilon: f64,
    simplify: bool,
) -> PyResult<Bound<'py, PyAny>> {
    let py = circ.py();

    try_with_circ(circ, |mut circ: tket::Circuit, typ: CircuitType| {
        apply_gridsynth_pass(circ.hugr_mut(), epsilon, simplify).convert_pyerrs()?;

        let circ = typ.convert(py, circ)?;
        PyResult::Ok(circ)
    })
}
