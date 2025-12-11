//! Utilities for calling tket Circuit functions on generic python objects.

use std::borrow::Borrow;

use hugr::builder::{CircuitBuilder, DFGBuilder, Dataflow, DataflowHugr};
use hugr::extension::prelude::qb_t;
use hugr::ops::OpType;
use hugr::ops::handle::NodeHandle;
use hugr::types::Type;
use itertools::Itertools;
use pyo3::exceptions::{PyAttributeError, PyValueError};
use pyo3::types::{PyAnyMethods, PyModule, PyString, PyTypeMethods};
use pyo3::{
    Bound, FromPyObject, IntoPyObject, PyAny, PyErr, PyRefMut, PyResult, PyTypeInfo, Python,
    pyclass, pymethods,
};

use derive_more::From;
use hugr::{Hugr, HugrView, Wire};
use serde::Serialize;
use tket::circuit::CircuitHash;
use tket::passes::CircuitChunks;
use tket::serialize::TKETDecode;
use tket::serialize::pytket::{DecodeOptions, EncodeOptions};
use tket::{Circuit, TketOp};
use tket_json_rs::circuit_json::SerialCircuit;

use crate::rewrite::PyCircuitRewrite;
use crate::utils::ConvertPyErr;

use super::{PyCircuitCost, PyNode, PyWire, Tk2Circuit, cost};

/// A flag to indicate the encoding of a circuit.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum CircuitType {
    /// A `pytket` `Circuit`.
    Tket1,
    /// A tket `Tk2Circuit`, represented as a HUGR.
    Tket,
}

impl CircuitType {
    /// Converts a circuit into the format indicated by the flag.
    pub fn convert(self, py: Python, circ: Circuit) -> PyResult<Bound<PyAny>> {
        match self {
            CircuitType::Tket1 => SerialCircuit::encode(
                &circ,
                EncodeOptions::new().with_config(tket_qsystem::pytket::qsystem_encoder_config()),
            )
            .convert_pyerrs()?
            .to_tket1(py),
            CircuitType::Tket => Ok(Bound::new(py, Tk2Circuit { circ })?.into_any()),
        }
    }
}

/// Apply a fallible function expecting a tket circuit on a python object.
///
/// This method supports both `pytket.Circuit` and `Tk2Circuit` python objects.
pub fn try_with_circ<T, E, F>(circ: &Bound<PyAny>, f: F) -> PyResult<T>
where
    E: ConvertPyErr<Output = PyErr>,
    F: FnOnce(Circuit, CircuitType) -> Result<T, E>,
{
    let (circ, typ) = match Tk2Circuit::extract_bound(circ) {
        // tket circuit
        Ok(t2circ) => (t2circ.circ, CircuitType::Tket),
        // tket1 circuit
        Err(_) => (
            SerialCircuit::from_tket1(circ)?
                .decode(
                    DecodeOptions::new()
                        .with_config(tket_qsystem::pytket::qsystem_decoder_config()),
                )
                .convert_pyerrs()?,
            CircuitType::Tket1,
        ),
    };
    (f)(circ, typ).map_err(|e| e.convert_pyerrs())
}

/// Apply a function expecting a tket circuit on a python object.
///
/// This method supports both `pytket.Circuit` and `Tk2Circuit` python objects.
pub fn with_circ<T, F>(circ: &Bound<PyAny>, f: F) -> PyResult<T>
where
    F: FnOnce(Circuit, CircuitType) -> T,
{
    try_with_circ(circ, |circ, typ| Ok::<T, PyErr>((f)(circ, typ)))
}

/// Apply a fallible circuit-to-circuit function on a python object, and return the modified result.
///
/// This method supports both `pytket.Circuit` and `Tk2Circuit` python objects.
/// The returned [`Circuit`] is converted back into the circuit representation used in the input.
pub fn try_update_circ<'py, E, F>(circ: &Bound<'py, PyAny>, f: F) -> PyResult<Bound<'py, PyAny>>
where
    E: ConvertPyErr<Output = PyErr>,
    F: FnOnce(Circuit, CircuitType) -> Result<Circuit, E>,
{
    let py = circ.py();
    try_with_circ(circ, |circ, typ| {
        let circ = f(circ, typ).map_err(|e| e.convert_pyerrs())?;
        typ.convert(py, circ)
    })
}

/// Apply a circuit-to-circuit function on a python object, and return the modified result.
///
/// This method supports both `pytket.Circuit` and `Tk2Circuit` python objects.
/// The returned [`Circuit`] is converted back into the circuit representation used in the input.
pub fn update_circ<'py, F>(circ: &Bound<'py, PyAny>, f: F) -> PyResult<Bound<'py, PyAny>>
where
    F: FnOnce(Circuit, CircuitType) -> Circuit,
{
    let py = circ.py();
    try_with_circ(circ, |circ, typ| {
        let circ = f(circ, typ);
        typ.convert(py, circ)
    })
}

#[cfg(test)]
mod test {
    use crate::utils::test::make_module_tk2_circuit;

    use super::*;
    use cool_asserts::assert_matches;
    use pyo3::prelude::*;
    use rstest::rstest;

    #[rstest]
    fn test_with_circ() -> PyResult<()> {
        pyo3::prepare_freethreaded_python();
        Python::with_gil(|py| {
            let circ = make_module_tk2_circuit(py)?;

            with_circ(&circ, |circ, typ| {
                assert_eq!(typ, CircuitType::Tket);

                let parent_optype = circ.hugr().get_optype(circ.parent());
                assert_matches!(parent_optype, OpType::FuncDefn(_));
            })
        })?;
        Ok(())
    }
}
