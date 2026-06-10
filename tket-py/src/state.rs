//! Program state definition.
//!
//! This module defines [`CompilationState`], a wrapper around a rust-defined
//! [`hugr::Hugr`] that is optimised for compilation and rewriting.

mod base;
mod cost;

use derive_more::{From, Into};
use pyo3::prelude::*;
use std::fmt;

use hugr::{Node, PortIndex};

use crate::utils::create_py_exception;

pub use self::cost::PyCircuitCost;
pub use base::{CompilationState, embedded_extensions};
pub use tket::{Pauli, TketOp};

/// The module definition
pub fn module(py: Python<'_>) -> PyResult<Bound<'_, PyModule>> {
    let m = PyModule::new(py, "state")?;
    m.add_class::<CompilationState>()?;
    m.add_class::<PyNode>()?;
    m.add_class::<PyWire>()?;
    m.add_class::<PyCircuitCost>()?;

    m.add_function(wrap_pyfunction!(embedded_extensions, &m)?)?;

    m.add("HugrError", py.get_type::<PyHugrError>())?;
    m.add("BuildError", py.get_type::<PyBuildError>())?;
    m.add("ValidationError", py.get_type::<PyValidationError>())?;
    m.add(
        "HUGRSerializationError",
        py.get_type::<PyHUGRSerializationError>(),
    )?;
    m.add("TK1EncodeError", py.get_type::<PyTk1EncodeError>())?;
    m.add("TK1DecodeError", py.get_type::<PyTK1DecodeError>())?;

    Ok(m)
}

create_py_exception!(
    hugr::hugr::HugrError,
    PyHugrError,
    "Errors that can occur while manipulating a HUGR."
);

create_py_exception!(
    hugr::builder::BuildError,
    PyBuildError,
    "Error while building the HUGR."
);

create_py_exception!(
    hugr::hugr::validate::ValidationError<Node>,
    PyValidationError,
    "Errors that can occur while validating a Hugr."
);

create_py_exception!(
    hugr::hugr::serialize::HUGRSerializationError,
    PyHUGRSerializationError,
    "Errors that can occur while serializing a HUGR."
);

create_py_exception!(
    tket::serialize::pytket::PytketEncodeError,
    PyTk1EncodeError,
    "Error encoding a HUGR region into a pytket circuit."
);

create_py_exception!(
    tket::serialize::pytket::PytketDecodeError,
    PyTK1DecodeError,
    "Error decoding a HUGR region from a pytket circuit."
);

/// A [`hugr::Node`] wrapper for Python.
#[pyclass(from_py_object)]
#[pyo3(name = "Node")]
#[repr(transparent)]
#[derive(From, Into, PartialEq, Eq, Hash, Clone, Copy)]
pub struct PyNode {
    /// Rust representation of the node
    pub node: hugr::Node,
}

impl fmt::Display for PyNode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.node.fmt(f)
    }
}

impl fmt::Debug for PyNode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.node.fmt(f)
    }
}

#[pymethods]
impl PyNode {
    #[new]
    fn new(index: usize) -> Self {
        Self {
            node: serde_json::from_value(serde_json::Value::Number(index.into())).unwrap(),
        }
    }
    /// A string representation of the pattern.
    pub fn __repr__(&self) -> String {
        format!("{self:?}")
    }
}

/// A [`hugr::Node`] wrapper for Python.
#[pyclass(from_py_object)]
#[pyo3(name = "Wire")]
#[repr(transparent)]
#[derive(From, Into, PartialEq, Eq, Hash, Clone, Copy)]
pub struct PyWire {
    /// Rust representation of the node
    pub wire: hugr::Wire,
}

impl fmt::Display for PyWire {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.wire.fmt(f)
    }
}

impl fmt::Debug for PyWire {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.wire.fmt(f)
    }
}

#[pymethods]
impl PyWire {
    /// A string representation of the pattern.
    pub fn __repr__(&self) -> String {
        format!("{self:?}")
    }

    fn node(&self) -> PyNode {
        self.wire.node().into()
    }

    fn port(&self) -> usize {
        self.wire.source().index()
    }
}
