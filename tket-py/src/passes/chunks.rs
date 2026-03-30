//! Circuit chunking utilities.

use derive_more::From;
use pyo3::exceptions::PyAttributeError;
use pyo3::prelude::*;
use tket::Circuit;
use tket::passes::utils::CircuitChunks;

use crate::state::CompilationState;
use crate::utils::ConvertPyErr;

/// Split a circuit into chunks of a given size.
#[pyfunction]
pub fn chunks(c: &CompilationState, max_chunk_size: usize) -> PyResult<PyCircuitChunks> {
    let circ = Circuit::new(c.hugr.clone());
    let chunks = CircuitChunks::split(&circ, max_chunk_size);
    Ok(PyCircuitChunks { chunks })
}

/// A pattern that match a circuit exactly
///
/// Python equivalent of [`CircuitChunks`].
///
/// [`CircuitChunks`]: tket::passes::utils::CircuitChunks
#[pyclass(from_py_object)]
#[pyo3(name = "CircuitChunks")]
#[derive(Debug, Clone, From)]
pub struct PyCircuitChunks {
    /// Rust representation of the circuit chunks.
    pub chunks: CircuitChunks,
}

#[pymethods]
impl PyCircuitChunks {
    /// Reassemble the chunks into a circuit.
    fn reassemble(&self) -> PyResult<CompilationState> {
        let circ = self.clone().chunks.reassemble().convert_pyerrs()?;
        Ok(CompilationState {
            hugr: circ.into_hugr(),
        })
    }

    /// Returns clones of the split circuits.
    fn circuits(&self) -> PyResult<Vec<CompilationState>> {
        self.chunks
            .iter()
            .map(|circ| {
                Ok(CompilationState {
                    hugr: circ.clone().into_hugr(),
                })
            })
            .collect()
    }

    /// Replaces a chunk's circuit with an updated version.
    fn update_circuit(&mut self, index: usize, new_circ: &CompilationState) -> PyResult<()> {
        let circ = Circuit::new(&new_circ.hugr);
        let circuit_sig = circ.circuit_signature();
        let chunk_sig = self.chunks[index].circuit_signature();
        if circuit_sig.input() != chunk_sig.input() || circuit_sig.output() != chunk_sig.output() {
            return Err(PyAttributeError::new_err(
                "The new circuit has a different signature.",
            ));
        }
        self.chunks[index] = circ.to_owned();
        Ok(())
    }
}
