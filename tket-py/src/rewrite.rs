//! PyO3 wrapper for rewriters.

use derive_more::From;
use hugr::{HugrView, hugr::views::SiblingSubgraph};
use itertools::Itertools;
use pyo3::prelude::*;
use std::path::PathBuf;
use tket::{
    Circuit,
    rewrite::{CircuitRewrite, ECCRewriter, Rewriter},
};

use crate::state::{CompilationState, PyNode};

/// The module definition
pub fn module(py: Python<'_>) -> PyResult<Bound<'_, PyModule>> {
    let m = PyModule::new(py, "rewrite")?;
    m.add_class::<PyECCRewriter>()?;
    m.add_class::<PyCircuitRewrite>()?;
    m.add_class::<PySubcircuit>()?;
    Ok(m)
}

/// A rewrite rule for circuits.
///
/// Python equivalent of [`CircuitRewrite`].
///
/// [`CircuitRewrite`]: tket::rewrite::CircuitRewrite
#[pyclass(from_py_object)]
#[pyo3(name = "CircuitRewrite")]
#[derive(Debug, Clone, From)]
#[repr(transparent)]
pub struct PyCircuitRewrite {
    /// Rust representation of the circuit chunks.
    pub rewrite: CircuitRewrite,
}

#[pymethods]
impl PyCircuitRewrite {
    /// Number of nodes added or removed by the rewrite.
    ///
    /// The difference between the new number of nodes minus the old. A positive
    /// number is an increase in node count, a negative number is a decrease.
    pub fn node_count_delta(&self) -> isize {
        self.rewrite.node_count_delta()
    }

    /// The replacement subcircuit.
    pub fn replacement(&self) -> CompilationState {
        self.rewrite.replacement().to_owned().into_hugr().into()
    }

    #[new]
    fn try_new(
        source_position: PySubcircuit,
        source_circ: PyRef<CompilationState>,
        replacement: PyRef<CompilationState>,
    ) -> PyResult<Self> {
        Ok(Self {
            rewrite: CircuitRewrite::try_new(
                &source_position.0,
                &source_circ.hugr,
                Circuit::new(replacement.hugr.clone()),
            )
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyValueError, _>(e.to_string()))?,
        })
    }
}

/// An enum of all rewriters exposed to the Python API.
///
/// This type is not exposed to Python, but instead corresponds to the Python
/// type union in `rewrite.py`.
#[derive(Clone, FromPyObject)]
pub enum PyRewriter {
    /// A rewriter based on circuit equivalence classes.
    ECC(PyECCRewriter),
    /// A rewriter based on a list of rewriters.
    Vec(Vec<PyRewriter>),
}

impl Rewriter for PyRewriter {
    fn get_rewrites(
        &self,
        circ: &Circuit<impl HugrView<Node = hugr::Node>>,
    ) -> Vec<CircuitRewrite> {
        match self {
            Self::ECC(ecc) => ecc.0.get_rewrites(circ),
            Self::Vec(rewriters) => rewriters
                .iter()
                .flat_map(|r| r.get_rewrites(circ))
                .collect(),
        }
    }
}

/// A subcircuit specification.
///
/// Python equivalent of [`Subcircuit`].
///
/// [`Subcircuit`]: tket::rewrite::Subcircuit
#[pyclass(from_py_object)]
#[pyo3(name = "Subcircuit")]
#[derive(Debug, Clone, From)]
#[repr(transparent)]
pub struct PySubcircuit(SiblingSubgraph);

#[pymethods]
impl PySubcircuit {
    #[new]
    fn from_nodes(nodes: Vec<PyNode>, circ: &CompilationState) -> PyResult<Self> {
        let nodes: Vec<_> = nodes.into_iter().map_into().collect();
        Ok(Self(
            SiblingSubgraph::try_from_nodes(nodes, &circ.hugr)
                .map_err(|e| PyErr::new::<pyo3::exceptions::PyValueError, _>(e.to_string()))?,
        ))
    }
}

/// A rewriter based on circuit equivalence classes.
///
/// In every equivalence class, one circuit is chosen as the representative.
/// Valid rewrites turn a non-representative circuit into its representative,
/// or a representative circuit into any of the equivalent non-representative
#[pyclass(name = "ECCRewriter", from_py_object)]
#[derive(Clone, From)]
pub struct PyECCRewriter(ECCRewriter);

#[pymethods]
impl PyECCRewriter {
    /// Load a precompiled ecc rewriter from a file.
    #[staticmethod]
    pub fn load_precompiled(path: PathBuf) -> PyResult<Self> {
        Ok(Self(ECCRewriter::load_binary(path).map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(e.to_string())
        })?))
    }

    /// Compile an ECC rewriter from a JSON file.
    #[staticmethod]
    pub fn compile_eccs(path: &str) -> PyResult<Self> {
        Ok(Self(ECCRewriter::try_from_eccs_json_file(path).map_err(
            |e| PyErr::new::<pyo3::exceptions::PyValueError, _>(e.to_string()),
        )?))
    }

    /// Returns a list of circuit rewrites that can be applied to the given CompilationState.
    pub fn get_rewrites(&self, circ: &CompilationState) -> Vec<PyCircuitRewrite> {
        let c = Circuit::new(circ.hugr.clone());
        self.0.get_rewrites(&c).into_iter().map_into().collect()
    }
}
