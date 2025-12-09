//! Rust-backed representation of circuits

use std::borrow::{Borrow, Cow};
use std::fmt::Display;
use std::mem;
use std::num::{NonZero, NonZeroU8};
use std::sync::LazyLock;

use hugr::builder::{CircuitBuilder, DFGBuilder, Dataflow, DataflowHugr};
use hugr::envelope::{EnvelopeConfig, EnvelopeFormat, ZstdConfig};
use hugr::extension::prelude::qb_t;
use hugr::extension::{ExtensionRegistry, EMPTY_REG};
use hugr::ops::handle::NodeHandle;
use hugr::ops::{ExtensionOp, OpType};
use hugr::package::Package;
use hugr::types::Type;
use itertools::Itertools;
use pyo3::exceptions::{PyAttributeError, PyValueError};
use pyo3::types::{PyAnyMethods, PyModule, PyString, PyTypeMethods};
use pyo3::{
    pyclass, pymethods, Bound, FromPyObject, IntoPyObject, PyAny, PyErr, PyRef, PyRefMut, PyResult,
    PyTypeInfo, Python,
};

use derive_more::From;
use hugr::{Hugr, HugrView, Wire};
use serde::Serialize;
use tket::circuit::CircuitHash;
use tket::passes::pytket::lower_to_pytket;
use tket::passes::CircuitChunks;
use tket::serialize::pytket::{DecodeOptions, EncodeOptions};
use tket::serialize::TKETDecode;
use tket::{Circuit, TketOp};
use tket_json_rs::circuit_json::SerialCircuit;

use crate::ops::PyTketOp;
use crate::rewrite::PyCircuitRewrite;
use crate::types::PyHugrType;
use crate::utils::{into_vec, ConvertPyErr};

use super::{cost, with_circ, PyCircuitCost, PyNode, PyWire};

/// A circuit in tket format.
///
/// This can be freely converted to and from a `pytket.Circuit`. Prefer using
/// this class when applying multiple tket operations on a circuit, as it
/// avoids the overhead of converting to and from a `pytket.Circuit` each time.
///
/// Node indices returned by this class are not stable across conversion to and
/// from a `pytket.Circuit`.
///
/// # Examples
///
/// Convert between `pytket.Circuit`s and `Tk2Circuit`s:
/// ```python
/// from pytket import Circuit
/// c = Circuit(2).H(0).CX(0, 1)
/// # Convert to a Tk2Circuit
/// t2c = Tk2Circuit(c)
/// # Convert back to a pytket.Circuit
/// c2 = t2c.to_tket1()
/// ```
#[pyclass]
#[derive(Clone, Debug, PartialEq, From)]
pub struct Tk2Circuit {
    /// Rust representation of the circuit.
    pub circ: Circuit,
}

#[pymethods]
impl Tk2Circuit {
    /// Initialize a `Tk2Circuit` from a `pytket.Circuit` or `guppy.Hugr`.
    ///
    /// Converts the input circuit to a `Hugr` if required via its serialisation
    /// interface.
    #[new]
    pub fn new(circ: &Bound<PyAny>) -> PyResult<Self> {
        Ok(Self {
            circ: with_circ(circ, |hugr, _| hugr)?,
        })
    }

    /// Convert the [`Tk2Circuit`] to a tket1 circuit.
    pub fn to_tket1<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let circ = lower_to_pytket(&self.circ).convert_pyerrs()?;
        SerialCircuit::encode(
            &circ,
            EncodeOptions::new().with_config(tket_qsystem::pytket::qsystem_encoder_config()),
        )
        .convert_pyerrs()?
        .to_tket1(py)
    }

    /// Apply a rewrite on the circuit.
    pub fn apply_rewrite(&mut self, rw: PyCircuitRewrite) {
        rw.rewrite.apply(&mut self.circ).expect("Apply error.");
    }

    /// Encode the circuit as a HUGR envelope.
    ///
    /// If no config is given, it defaults to the default binary envelope.
    #[pyo3(signature = (config = None))]
    pub fn to_bytes(&self, config: Option<Bound<'_, PyAny>>) -> PyResult<Vec<u8>> {
        fn err(e: impl Display) -> PyErr {
            PyErr::new::<PyAttributeError, _>(format!("Could not encode circuit: {e}"))
        };
        let config = match config {
            Some(cfg) => envelope_config_from_py(cfg)?,
            None => EnvelopeConfig::binary(),
        };
        let mut buf = Vec::new();
        self.circ.store(&mut buf, config).map_err(err)?;
        Ok(buf)
    }

    /// Encode the circuit as a HUGR envelope.
    ///
    /// If no config is given, it defaults to the default text envelope.
    #[pyo3(signature = (config = None))]
    pub fn to_str(&self, config: Option<Bound<'_, PyAny>>) -> PyResult<String> {
        fn err(e: impl Display) -> PyErr {
            PyErr::new::<PyAttributeError, _>(format!("Could not encode circuit: {e}"))
        };
        let config = match config {
            Some(cfg) => envelope_config_from_py(cfg)?,
            None => EnvelopeConfig::text(),
        };
        self.circ.store_str(config).map_err(err)
    }

    /// Loads a circuit from a HUGR envelope.
    ///
    /// If the name is not given, uses the encoded entrypoint.
    //
    // TODO(deprecated): Drop the `function_name` parameter in a breaking change.
    #[staticmethod]
    #[pyo3(signature = (bytes, function_name = None))]
    pub fn from_bytes(bytes: &[u8], function_name: Option<String>) -> PyResult<Self> {
        fn err(e: impl Display) -> PyErr {
            PyErr::new::<PyAttributeError, _>(format!("Could not read envelope: {e}"))
        };
        let circ = match function_name {
            // NOTE: `load_function` uses the default REGISTRY which does not contain tket-qsystem extensions.
            Some(name) => Circuit::load_function(bytes, name).map_err(err)?,
            None => Circuit::load(bytes, Some(&REGISTRY)).map_err(err)?,
        };
        Ok(Tk2Circuit { circ })
    }

    /// Loads a circuit from a HUGR envelope string.
    ///
    /// If the name is not given, uses the encoded entrypoint.
    //
    // TODO(deprecated): Drop the `function_name` parameter in a breaking change.
    #[staticmethod]
    #[pyo3(signature = (envelope, function_name = None))]
    pub fn from_str(envelope: &str, function_name: Option<String>) -> PyResult<Self> {
        fn err(e: impl Display) -> PyErr {
            PyErr::new::<PyAttributeError, _>(format!("Could not read envelope: {e}"))
        };
        let circ = match function_name {
            // NOTE: `load_function_str` uses the default REGISTRY which does not contain tket-qsystem extensions.
            Some(name) => Circuit::load_function_str(envelope, name).map_err(err)?,
            None => Circuit::load_str(envelope, Some(&REGISTRY)).map_err(err)?,
        };
        Ok(Tk2Circuit { circ })
    }

    /// Encode the circuit as a tket1 json string.
    pub fn to_tket1_json(&self) -> PyResult<String> {
        // Try to simplify tuple pack-unpack pairs, and other operations not supported by pytket.
        let circ = lower_to_pytket(&self.circ).convert_pyerrs()?;
        serde_json::to_string(
            &SerialCircuit::encode(
                &circ,
                EncodeOptions::new().with_config(tket_qsystem::pytket::qsystem_encoder_config()),
            )
            .convert_pyerrs()?,
        )
        .map_err(|e| {
            PyErr::new::<PyValueError, _>(format!("Could not encode pytket circuit to str: {e}"))
        })
    }

    /// Decode a tket1 json string to a circuit.
    #[staticmethod]
    pub fn from_tket1_json(json: &str) -> PyResult<Self> {
        let circ = tket::serialize::load_tk1_json_str(
            json,
            DecodeOptions::new().with_config(tket_qsystem::pytket::qsystem_decoder_config()),
        )
        .map_err(|e| {
            PyErr::new::<PyAttributeError, _>(format!("Could not load pytket circuit: {e}"))
        })?;
        Ok(Tk2Circuit { circ })
    }

    /// Encode the circuit as a tket1 json utf8 bytes.
    pub fn to_tket1_json_bytes(&self) -> PyResult<Vec<u8>> {
        // Try to simplify tuple pack-unpack pairs, and other operations not supported by pytket.
        let circ = lower_to_pytket(&self.circ).convert_pyerrs()?;
        serde_json::to_vec(
            &SerialCircuit::encode(
                &circ,
                EncodeOptions::new().with_config(tket_qsystem::pytket::qsystem_encoder_config()),
            )
            .convert_pyerrs()?,
        )
        .map_err(|e| {
            PyErr::new::<PyValueError, _>(format!("Could not encode pytket circuit to bytes: {e}"))
        })
    }

    /// Decode a tket1 json utf8 bytes to a circuit.
    #[staticmethod]
    pub fn from_tket1_json_bytes(json: &[u8]) -> PyResult<Self> {
        let circ = tket::serialize::load_tk1_json_reader(
            json,
            DecodeOptions::new().with_config(tket_qsystem::pytket::qsystem_decoder_config()),
        )
        .map_err(|e| {
            PyErr::new::<PyAttributeError, _>(format!("Could not load pytket circuit: {e}"))
        })?;
        Ok(Tk2Circuit { circ })
    }

    /// Compute the cost of the circuit based on a per-operation cost function.
    ///
    /// :param cost_fn: A function that takes a `TketOp` and returns an arbitrary cost.
    ///     The cost must implement `__add__`, `__sub__`, `__lt__`,
    ///     `__eq__`, `__int__`, and integer `__div__`.
    ///
    /// :returns: The sum of all operation costs.
    pub fn circuit_cost<'py>(&self, cost_fn: &Bound<'py, PyAny>) -> PyResult<Bound<'py, PyAny>> {
        let py = cost_fn.py();
        let cost_fn = |op: &OpType| -> PyResult<PyCircuitCost> {
            // TODO: We should ignore non-tket operations instead.
            let Some(tk2_op) = op.cast::<TketOp>() else {
                let op_name = op.to_string();
                return Err(PyErr::new::<PyValueError, _>(format!(
                    "Could not convert circuit operation to a `TketOp`: {op_name}"
                )));
            };
            let tk2_py_op = PyTketOp::from(tk2_op);
            let cost = cost_fn.call1((tk2_py_op,))?;
            Ok(PyCircuitCost { cost: cost.into() })
        };
        let circ_cost = self.circ.circuit_cost(cost_fn)?;
        Ok(circ_cost.cost.into_bound(py))
    }

    /// Returns the number of operations in the circuit.
    ///
    /// This includes [`TketOp`]s, pytket ops, and any other custom operations.
    ///
    /// Nested circuits are traversed to count their operations.
    pub fn num_operations(&self) -> usize {
        self.circ.num_operations()
    }

    /// Returns a hash of the circuit.
    pub fn hash(&self) -> u64 {
        self.circ.circuit_hash(self.circ.parent()).unwrap()
    }

    /// Hash the circuit
    pub fn __hash__(&self) -> isize {
        self.hash() as isize
    }

    /// Copy the circuit.
    pub fn __copy__(&self) -> PyResult<Self> {
        Ok(self.clone())
    }

    /// Copy the circuit.
    pub fn __deepcopy__(&self, _memo: Bound<PyAny>) -> PyResult<Self> {
        Ok(self.clone())
    }

    fn node_op(&self, node: PyNode) -> PyResult<Cow<'_, [u8]>> {
        let custom: ExtensionOp = self
            .circ
            .hugr()
            .get_optype(node.node)
            .clone()
            .try_into()
            .map_err(|e| {
                PyErr::new::<PyValueError, _>(format!(
                    "Could not convert circuit operation to an `ExtensionOp`: {e}"
                ))
            })?;

        Ok(serde_json::to_vec(&custom).unwrap().into())
    }

    fn node_inputs(&self, node: PyNode) -> Vec<PyWire> {
        self.circ
            .hugr()
            .all_linked_outputs(node.node)
            .map(|(n, p)| Wire::new(n, p).into())
            .collect()
    }

    fn node_outputs(&self, node: PyNode) -> Vec<PyWire> {
        self.circ
            .hugr()
            .node_outputs(node.node)
            .map(|p| Wire::new(node.node, p).into())
            .collect()
    }

    fn input_node(&self) -> PyNode {
        self.circ.input_node().into()
    }

    fn output_node(&self) -> PyNode {
        self.circ.output_node().into()
    }

    fn render_mermaid(&self) -> String {
        self.circ.mermaid_string()
    }
}
impl Tk2Circuit {
    /// Tries to extract a Tk2Circuit from a python object.
    ///
    /// Returns an error if the py object is not a Tk2Circuit.
    pub fn try_extract(circ: &Bound<PyAny>) -> PyResult<Self> {
        circ.extract::<Tk2Circuit>()
    }
}

/// Converts a python `hugr.envelope.EnvelopeConfig` into a rust-based [`EnvelopeConfig`].
pub fn envelope_config_from_py(config: Bound<'_, PyAny>) -> PyResult<EnvelopeConfig> {
    let mut res = EnvelopeConfig::default();

    let format = config.getattr("format")?;
    let format_ident: usize = format.getattr("value")?.extract()?;
    res.format = EnvelopeFormat::from_repr(format_ident).ok_or_else(|| {
        PyErr::new::<PyValueError, _>(format!("Invalid envelope format: {format_ident}"))
    })?;

    let zstd: Option<usize> = config.getattr("zstd")?.extract()?;
    res.zstd = zstd.map(|level| {
        let mut z = ZstdConfig::default();
        // Compression level 0 means default compression.
        // We represent that as `None` on the rust struct.
        if level > 0 && level < u8::MAX as usize {
            z.level = Some(NonZeroU8::new(level as u8).unwrap());
        }
        z
    });

    Ok(res)
}

/// Extension registry used for loading circuits.
pub static REGISTRY: LazyLock<ExtensionRegistry> = LazyLock::new(|| {
    let mut registry = hugr::std_extensions::std_reg();
    registry.extend([
        // tket extensions
        tket::extension::TKET_EXTENSION.to_owned(),
        tket::extension::rotation::ROTATION_EXTENSION.to_owned(),
        tket::extension::bool::BOOL_EXTENSION.to_owned(),
        tket::extension::debug::DEBUG_EXTENSION.to_owned(),
        tket::extension::guppy::GUPPY_EXTENSION.to_owned(),
        tket::extension::global_phase::GLOBAL_PHASE_EXTENSION.to_owned(),
        tket::extension::modifier::MODIFIER_EXTENSION.to_owned(),
        // tket-qsystem extensions
        tket_qsystem::extension::gpu::EXTENSION.to_owned(),
        tket_qsystem::extension::qsystem::EXTENSION.to_owned(),
        tket_qsystem::extension::futures::EXTENSION.to_owned(),
        tket_qsystem::extension::random::EXTENSION.to_owned(),
        tket_qsystem::extension::result::EXTENSION.to_owned(),
        tket_qsystem::extension::utils::EXTENSION.to_owned(),
        tket_qsystem::extension::wasm::EXTENSION.to_owned(),
    ]);
    registry
});
