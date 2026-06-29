//! Bindings for metadata keys defined in the `tket` crate.

use hugr::metadata::Metadata;
use pyo3::prelude::*;
use tket::metadata::{
    CircuitRewriteTraces, ExpectedQubitsHint, InlineAnnotation, PytketBitRegisterNames,
    PytketInputParameters, PytketOpGroup, PytketPhaseExpr, PytketQubitRegisterNames, UnitaryFlags,
};

/// The module definition.
pub fn module(py: Python<'_>) -> PyResult<Bound<'_, PyModule>> {
    let m = PyModule::new(py, "metadata")?;
    m.add("EXPECTED_QUBITS_HINT", ExpectedQubitsHint::KEY)?;
    m.add("EXPECTED_QUBITS_HINT_ALIASES", ExpectedQubitsHint::ALIASES)?;
    m.add("INLINE_ANNOTATION", InlineAnnotation::KEY)?;
    m.add("CIRCUIT_REWRITE_TRACES", CircuitRewriteTraces::KEY)?;
    m.add("UNITARY_FLAGS", UnitaryFlags::KEY)?;
    m.add("UNITARY_FLAGS_ALIAS", UnitaryFlags::ALIASES)?;
    m.add("PYTKET_INPUT_PARAMETERS", PytketInputParameters::KEY)?;
    m.add("PYTKET_OP_GROUP", PytketOpGroup::KEY)?;
    m.add("PYTKET_BIT_REGISTER_NAMES", PytketBitRegisterNames::KEY)?;
    m.add("PYTKET_QUBIT_REGISTER_NAMES", PytketQubitRegisterNames::KEY)?;
    m.add("PYTKET_PHASE_EXPR", PytketPhaseExpr::KEY)?;
    Ok(m)
}
