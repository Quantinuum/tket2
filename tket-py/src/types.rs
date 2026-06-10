//! Hugr types

use derive_more::{From, Into};
use hugr::extension::prelude::{bool_t, qb_t};
use hugr::types::{Type, TypeBound};
use pyo3::prelude::*;
use std::fmt;

/// The module definition
pub fn module(py: Python<'_>) -> PyResult<Bound<'_, PyModule>> {
    let m = PyModule::new(py, "types")?;
    m.add_class::<PyHugrType>()?;
    m.add_class::<PyTypeBound>()?;

    Ok(m)
}

/// Bounds on the valid operations on a type in a HUGR program.
#[pyclass(eq, eq_int, skip_from_py_object)]
#[pyo3(name = "TypeBound")]
#[derive(PartialEq, Clone, Debug)]
pub enum PyTypeBound {
    /// No bound on the type.
    Linear,
    /// The type can be copied in the program.
    Copyable,
}

impl From<PyTypeBound> for TypeBound {
    fn from(bound: PyTypeBound) -> Self {
        match bound {
            PyTypeBound::Linear => TypeBound::Linear,
            PyTypeBound::Copyable => TypeBound::Copyable,
        }
    }
}

impl From<TypeBound> for PyTypeBound {
    fn from(bound: TypeBound) -> Self {
        match bound {
            TypeBound::Linear => PyTypeBound::Linear,
            TypeBound::Copyable => PyTypeBound::Copyable,
        }
    }
}

/// A HUGR type.
//
// TODO: These can no longer be constructed from Python. Since `hugr-rs 0.14`
// we need an extension to defines these.
// If fixing this, make sure to fix `PyExtensionOp` too.
#[pyclass(from_py_object)]
#[pyo3(name = "HugrType")]
#[repr(transparent)]
#[derive(From, Into, PartialEq, Clone)]
pub struct PyHugrType(Type);

impl fmt::Debug for PyHugrType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

#[pymethods]
impl PyHugrType {
    #[staticmethod]
    fn qubit() -> Self {
        Self(qb_t())
    }

    #[staticmethod]
    fn bool() -> Self {
        Self(bool_t())
    }

    /// A string representation of the type.
    pub fn __repr__(&self) -> String {
        format!("{}", self.0)
    }
}
