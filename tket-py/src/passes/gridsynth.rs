//! Bindings to allow users to access the gridsynth pass from Python.
//! The definitions here should be reflected in the
//! `tket-py/tket/_tket/passes.pyi` type stubs file
use crate::circuit::try_with_circ;
use crate::circuit::CircuitType;
use pyo3::prelude::*;
use tket::passes::gridsynth::apply_gridsynth_pass;

/// Binding to a python function called gridsynth that runs the rust function called
/// apply_gridsynth pass behind the scenes
#[pyfunction]
pub fn gridsynth<'py>(circ: &Bound<'py, PyAny>, epsilon: f64) -> PyResult<Bound<'py, PyAny>> {
    let py = circ.py();

    try_with_circ(circ, |mut circ: tket::Circuit, typ: CircuitType| {
        apply_gridsynth_pass(circ.hugr_mut(), epsilon);

        let circ = typ.convert(py, circ)?;
        PyResult::Ok(circ)
    })
}
// TO DO: change everything that refers to this as
// inplace to be not inplace as this implementation
// is not inplace
