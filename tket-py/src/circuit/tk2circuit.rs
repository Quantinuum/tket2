//! Rust-backed representation of circuits

use std::any::Any;
use std::borrow::{Borrow, Cow};
use std::fmt::Display;
use std::mem;
use std::num::{NonZero, NonZeroU8};
use std::sync::LazyLock;

use anyhow::Context;
use hugr::builder::{CircuitBuilder, DFGBuilder, Dataflow, DataflowHugr};
use hugr::envelope::{EnvelopeConfig, EnvelopeFormat, ZstdConfig};
use hugr::extension::prelude::qb_t;
use hugr::extension::{EMPTY_REG, ExtensionRegistry};
use hugr::ops::handle::NodeHandle;
use hugr::ops::{ExtensionOp, OpType};
use hugr::package::Package;
use hugr::types::Type;
use hugr_passes::composable::ComposablePass;
use itertools::Itertools;
use pyo3::exceptions::{PyAttributeError, PyValueError};
use pyo3::types::{PyAnyMethods, PyModule, PyString, PyTypeMethods};
use pyo3::{
    Bound, FromPyObject, IntoPyObject, PyAny, PyErr, PyRef, PyRefMut, PyResult, PyTypeInfo, Python,
    pyclass, pyfunction, pymethods,
};

use derive_more::From;
use hugr::{Hugr, HugrView, Wire};
use serde::Serialize;
use tket::passes::NormalizeGuppy;
use tket::passes::utils::CircuitChunks;
use tket::serialize::TKETDecode;
use tket::serialize::pytket::{DecodeOptions, EncodeOptions};
use tket::{Circuit, TketOp};
use tket_json_rs::circuit_json::SerialCircuit;

use crate::ops::PyTketOp;
use crate::rewrite::PyCircuitRewrite;
use crate::types::PyHugrType;
use crate::utils::{ConvertPyErr, into_vec};

use super::{PyCircuitCost, PyNode, PyWire, cost, with_circ};

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
#[pyclass(from_py_object)]
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
    pub fn new(circ: &Bound<PyAny>) -> anyhow::Result<Self> {
        Ok(Self {
            circ: with_circ(circ, |hugr, _| hugr)?,
        })
    }

    /// Convert the [`Tk2Circuit`] to a tket1 circuit.
    pub fn to_tket1<'py>(&self, py: Python<'py>) -> anyhow::Result<Bound<'py, PyAny>> {
        let mut hugr = self.circ.hugr().clone();
        NormalizeGuppy::default().run(&mut hugr)?;
        let serial = SerialCircuit::encode(
            &hugr,
            EncodeOptions::new().with_config(tket_qsystem::pytket::qsystem_encoder_config()),
        )?;
        serial
            .to_tket1(py)
            .context("Could not convert tk2circuit to tket1")
    }

    /// Apply a rewrite on the circuit.
    pub fn apply_rewrite(&mut self, rw: PyCircuitRewrite) {
        rw.rewrite.apply(&mut self.circ).expect("Apply error.");
    }

    /// Encode the circuit as a HUGR envelope.
    ///
    /// If no config is given, it defaults to the default binary envelope.
    #[pyo3(signature = (config = None))]
    pub fn to_bytes(&self, config: Option<Bound<'_, PyAny>>) -> anyhow::Result<Vec<u8>> {
        let config = match config {
            Some(cfg) => envelope_config_from_py(cfg)?,
            None => EnvelopeConfig::binary(),
        };
        let mut buf = Vec::new();
        self.circ
            .store(&mut buf, config)
            .context("Could not encode tk2circuit to bytes")?;
        Ok(buf)
    }

    /// Encode the circuit as a HUGR envelope.
    ///
    /// If no config is given, it defaults to the default text envelope.
    #[pyo3(signature = (config = None))]
    pub fn to_str(&self, config: Option<Bound<'_, PyAny>>) -> anyhow::Result<String> {
        let config = match config {
            Some(cfg) => envelope_config_from_py(cfg)?,
            None => EnvelopeConfig::text(),
        };
        self.circ
            .store_str(config)
            .context("Could not encode tk2circuit to string")
    }

    /// Loads a circuit from a HUGR envelope.
    ///
    /// If the name is not given, uses the encoded entrypoint.
    //
    // TODO(deprecated): Drop the `function_name` parameter in a breaking change.
    #[staticmethod]
    #[pyo3(signature = (bytes, function_name = None))]
    pub fn from_bytes(bytes: &[u8], function_name: Option<String>) -> anyhow::Result<Self> {
        let circ = match function_name {
            // NOTE: `load_function` uses the default REGISTRY which does not contain tket-qsystem extensions.
            Some(name) => Circuit::load_function(bytes, name),
            None => Circuit::load(bytes, Some(&REGISTRY)),
        }
        .context("Could not read tk2circuit from bytes")?;
        Ok(Tk2Circuit { circ })
    }

    /// Loads a circuit from a HUGR envelope string.
    ///
    /// If the name is not given, uses the encoded entrypoint.
    //
    // TODO(deprecated): Drop the `function_name` parameter in a breaking change.
    #[staticmethod]
    #[pyo3(signature = (envelope, function_name = None))]
    pub fn from_str(envelope: &str, function_name: Option<String>) -> anyhow::Result<Self> {
        let circ = match function_name {
            // NOTE: `load_function_str` uses the default REGISTRY which does not contain tket-qsystem extensions.
            Some(name) => Circuit::load_function_str(envelope, name),
            None => Circuit::load_str(envelope, Some(&REGISTRY)),
        }
        .context("Could not read tk2circuit from string")?;
        Ok(Tk2Circuit { circ })
    }

    /// Encode the circuit as a tket1 json string.
    pub fn to_tket1_json(&self) -> anyhow::Result<String> {
        // Try to simplify tuple pack-unpack pairs, and other operations not supported by pytket.
        let mut hugr = self.circ.hugr().clone();
        NormalizeGuppy::default().run(&mut hugr)?;
        let serial = SerialCircuit::encode(
            &hugr,
            EncodeOptions::new().with_config(tket_qsystem::pytket::qsystem_encoder_config()),
        )?;
        serde_json::to_string(&serial).context("Could not encode pytket circuit to str")
    }

    /// Decode a tket1 json string to a circuit.
    #[staticmethod]
    pub fn from_tket1_json(json: &str) -> anyhow::Result<Self> {
        let hugr = tket::serialize::load_tk1_json_str(
            json,
            DecodeOptions::new().with_config(tket_qsystem::pytket::qsystem_decoder_config()),
        )
        .context("Could not load pytket circuit")?;
        Ok(Tk2Circuit { circ: hugr.into() })
    }

    /// Encode the circuit as a tket1 json utf8 bytes.
    pub fn to_tket1_json_bytes(&self) -> anyhow::Result<Vec<u8>> {
        // Try to simplify tuple pack-unpack pairs, and other operations not supported by pytket.
        let mut hugr = self.circ.hugr().clone();
        NormalizeGuppy::default().run(&mut hugr)?;
        let serial = SerialCircuit::encode(
            &hugr,
            EncodeOptions::new().with_config(tket_qsystem::pytket::qsystem_encoder_config()),
        )?;
        serde_json::to_vec(&serial).context("Could not encode pytket circuit to bytes")
    }

    /// Decode a tket1 json utf8 bytes to a circuit.
    #[staticmethod]
    pub fn from_tket1_json_bytes(json: &[u8]) -> anyhow::Result<Self> {
        let hugr = tket::serialize::load_tk1_json_reader(
            json,
            DecodeOptions::new().with_config(tket_qsystem::pytket::qsystem_decoder_config()),
        )
        .context("Could not load pytket circuit")?;
        Ok(Tk2Circuit { circ: hugr.into() })
    }

    /// Compute the cost of the circuit based on a per-operation cost function.
    ///
    /// :param cost_fn: A function that takes a `TketOp` and returns an arbitrary cost.
    ///     The cost must implement `__add__`, `__sub__`, `__lt__`,
    ///     `__eq__`, `__int__`, and integer `__div__`.
    ///
    /// :returns: The sum of all operation costs.
    pub fn circuit_cost<'py>(
        &self,
        cost_fn: &Bound<'py, PyAny>,
    ) -> anyhow::Result<Bound<'py, PyAny>> {
        let py = cost_fn.py();
        let cost_fn = |op: &OpType| -> anyhow::Result<PyCircuitCost> {
            // TODO: We should ignore non-tket operations instead.
            let Some(tk2_op) = op.cast::<TketOp>() else {
                let op_name = op.to_string();
                anyhow::bail!("Could not convert circuit operation to a `TketOp`: {op_name}");
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
    pub fn __copy__(&self) -> anyhow::Result<Self> {
        Ok(self.clone())
    }

    /// Copy the circuit.
    pub fn __deepcopy__(&self, _memo: Bound<PyAny>) -> anyhow::Result<Self> {
        Ok(self.clone())
    }

    fn node_op(&self, node: PyNode) -> anyhow::Result<Cow<'_, [u8]>> {
        let optype: OpType = self.circ.hugr().get_optype(node.node).clone();
        let custom: ExtensionOp = optype.try_into().map_err(|_| {
            anyhow::anyhow!("Could not convert circuit operation to an `ExtensionOp`")
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
        circ.extract::<Tk2Circuit>().map_err(|e| e.into())
    }
}

/// Converts a python `hugr.envelope.EnvelopeConfig` into a rust-based [`EnvelopeConfig`].
pub fn envelope_config_from_py(config: Bound<'_, PyAny>) -> anyhow::Result<EnvelopeConfig> {
    let mut res = EnvelopeConfig::default();

    let format = config.getattr("format")?;
    let format_ident: usize = format.getattr("value")?.extract()?;
    res.format = EnvelopeFormat::from_repr(format_ident)
        .ok_or_else(|| anyhow::anyhow!("Invalid envelope format: {format_ident}"))?;

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

/// Returns a list of extension ids supported by the Tk2Circuit loader.
///
/// Extensions not in this list must be included in the package when
/// loading a Tk2Circuit.
#[pyfunction]
pub fn embedded_extensions() -> Vec<String> {
    REGISTRY.iter().map(|e| e.name.to_string()).collect()
}
