//! Bindings for metadata keys defined in the `tket` crate.

use hugr::metadata::Metadata;
use pyo3::prelude::*;
#[expect(deprecated)]
use tket::metadata::PytketPhaseExpr;
use tket::metadata::{
    CircuitRewriteTraces, ControlledImplementations, CtrlDaggeredImplementations,
    DaggeredImplementation, ExpectedQubitsHint, InlineAnnotation, NumControlQubits,
    PytketBitRegisterNames, PytketInputParameters, PytketOpGroup, PytketQubitRegisterNames,
    UnitaryFlags,
};
use tket_qsystem::extension::qsystem::helios::HeliosPlatformConfig;

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
    #[expect(deprecated)]
    m.add("PYTKET_PHASE_EXPR", PytketPhaseExpr::KEY)?;
    m.add("HELIOS_PLATFORM_CONFIG", HeliosPlatformConfig::KEY)?;
    m.add("CONTROLLED_IMPLEMENTATIONS", ControlledImplementations::KEY)?;
    m.add(
        "CTRL_DAGGERED_IMPLEMENTATIONS",
        CtrlDaggeredImplementations::KEY,
    )?;
    m.add("DAGGERED_IMPLEMENTATION", DaggeredImplementation::KEY)?;
    m.add("NUM_CONTROL_QUBITS", NumControlQubits::KEY)?;
    Ok(m)
}
