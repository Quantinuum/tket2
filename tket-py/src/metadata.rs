//! Bindings for metadata keys defined in the `tket` crate.

use hugr::metadata::Metadata;
use pyo3::prelude::*;
use tket::metadata::{
    BitRegisters, CircuitRewriteTraces, InputParameters, MaxQubits, OpGroup, Phase, QubitRegisters,
    Unitary,
};

/// The module definition.
pub fn module(py: Python<'_>) -> PyResult<Bound<'_, PyModule>> {
    let m = PyModule::new(py, "metadata")?;
    m.add("MAX_QUBITS", MaxQubits::KEY)?;
    m.add("CIRCUIT_REWRITE_TRACES", CircuitRewriteTraces::KEY)?;
    m.add("UNITARY", Unitary::KEY)?;
    m.add("INPUT_PARAMETERS", InputParameters::KEY)?;
    m.add("OP_GROUP", OpGroup::KEY)?;
    m.add("BIT_REGISTERS", BitRegisters::KEY)?;
    m.add("QUBIT_REGISTERS", QubitRegisters::KEY)?;
    m.add("PHASE", Phase::KEY)?;
    Ok(m)
}
