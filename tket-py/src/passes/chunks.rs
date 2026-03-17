//! Circuit chunking utilities.

use derive_more::From;
use pyo3::exceptions::PyAttributeError;
use pyo3::prelude::*;
use tket::passes::CircuitChunks;

use crate::circuit::CircuitType;
use crate::circuit::{try_with_circ, with_circ};
use crate::utils::ConvertPyErr;

/// Split a circuit into chunks of a given size.
#[pyfunction]
pub fn chunks(c: &Bound<PyAny>, max_chunk_size: usize) -> PyResult<PyCircuitChunks> {
    with_circ(c, |hugr, typ| {
        // TODO: Detect if the circuit is in tket1 format or Tk2Circuit.
        let chunks = CircuitChunks::split(&hugr, max_chunk_size);
        (chunks, typ).into()
    })
}

/// A pattern that match a circuit exactly
///
/// Python equivalent of [`CircuitChunks`].
///
/// [`CircuitChunks`]: tket::passes::chunks::CircuitChunks
#[pyclass(from_py_object)]
#[pyo3(name = "CircuitChunks")]
#[derive(Debug, Clone, From)]
pub struct PyCircuitChunks {
    /// Rust representation of the circuit chunks.
    pub chunks: CircuitChunks,
    /// Whether to reassemble the circuit in the tket1 or tket format.
    pub original_type: CircuitType,
}

#[pymethods]
impl PyCircuitChunks {
    /// Reassemble the chunks into a circuit.
    fn reassemble<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let circ = self.clone().chunks.reassemble().convert_pyerrs()?;
        self.original_type.convert(py, circ)
    }

    /// Returns clones of the split circuits.
    fn circuits<'py>(&self, py: Python<'py>) -> PyResult<Vec<Bound<'py, PyAny>>> {
        self.chunks
            .iter()
            .map(|circ| self.original_type.convert(py, circ.clone()))
            .collect()
    }

    /// Replaces a chunk's circuit with an updated version.
    fn update_circuit(&mut self, index: usize, new_circ: &Bound<PyAny>) -> PyResult<()> {
        try_with_circ(new_circ, |circ, _| {
            let circuit_sig = circ.circuit_signature();
            let chunk_sig = self.chunks[index].circuit_signature();
            if circuit_sig.input() != chunk_sig.input()
                || circuit_sig.output() != chunk_sig.output()
            {
                return Err(PyAttributeError::new_err(
                    "The new circuit has a different signature.",
                ));
            }
            self.chunks[index] = circ;
            Ok(())
        })
    }
}
