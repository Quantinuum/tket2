//! Bindings to allow users to access the gridsynth pass from Python


/// The definitions here should be reflected in the
/// `tket-py/tket/_tket/passes.pyi` type stubs file

use pyo3::prelude::*;
use pyo3::types::PyFloat;
use tket::passes::gridsynth::apply_gridsynth_pass;
use crate::circuit::CircuitType;
use crate::utils::{create_py_exception, ConvertPyErr};
use crate::circuit::{try_update_circ, try_with_circ};

// use tket::passes::gridsynth;


// TO DO: add bindings here similarly to tket1.rs
#[pyfunction]
pub fn gridsynth<'py>(
    circ: &Bound<'py, PyAny>,
    epsilon: f64
) -> PyResult<Bound<'py, PyAny>> {
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