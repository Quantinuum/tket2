//! Rust-backed representation of circuits

use std::any::Any;
use std::borrow::{Borrow, Cow};
use std::fmt::Display;
use std::mem;
use std::num::{NonZero, NonZeroU8};
use std::sync::{Arc, LazyLock};

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
use pyo3::types::{IntoPyDict, PyAnyMethods, PyModule, PyString, PyTypeMethods};
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

use super::{PyCircuitCost, PyNode, PyWire, cost};

/// A quantum program represented as a HUGR.
///
/// This representation is optimized for compilation and rewriting. For building
/// and direct manipulation of programs, the `hugr.Hugr` python class should be
/// used instead.
#[pyclass(skip_from_py_object)]
#[derive(Clone, Debug, Default, PartialEq, From)]
pub struct TkProgram {
    /// Rust representation of the Hugr.
    pub hugr: Hugr,
}

#[pymethods]
impl TkProgram {
    /// Create a new empty program.
    #[new]
    pub fn new() -> Self {
        TkProgram { hugr: Hugr::new() }
    }

    /// Load a program from a legacy `pytket.Circuit`.
    #[staticmethod]
    pub fn from_tket1(circ: &Bound<PyAny>) -> anyhow::Result<Self> {
        let hugr = SerialCircuit::from_tket1(circ)?
            .decode(
                DecodeOptions::new().with_config(tket_qsystem::pytket::qsystem_decoder_config()),
            )
            .context("Could not decode a TkProgram from a pytket circuit")?;
        Ok(TkProgram { hugr })
    }

    /// Convert the program back to a legacy `pytket.Circuit`.
    pub fn to_tket1<'py>(&self, py: Python<'py>) -> anyhow::Result<Bound<'py, PyAny>> {
        let serial = SerialCircuit::encode(
            &self.hugr,
            EncodeOptions::new().with_config(tket_qsystem::pytket::qsystem_encoder_config()),
        )?;
        let pytket = serial.to_tket1(py)?;
        Ok(pytket.into_any())
    }

    /// Apply a rewrite on the circuit.
    pub fn apply_rewrite(&mut self, rw: PyCircuitRewrite) -> anyhow::Result<()> {
        let mut circ = Circuit::new(&mut self.hugr);
        rw.rewrite
            .apply(&mut circ)
            .context("Could not apply rewrite")?;
        Ok(())
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
        self.hugr
            .store(&mut buf, config)
            .context("Could not encode TkProgram to bytes")?;
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
        self.hugr
            .store_str(config)
            .context("Could not encode TkProgram to string")
    }

    /// Loads a HUGR envelope from envelope bytes.
    #[staticmethod]
    #[pyo3(signature = (bytes))]
    pub fn from_bytes(bytes: &[u8]) -> anyhow::Result<Self> {
        let hugr =
            Hugr::load(bytes, Some(&REGISTRY)).context("Could not read TkProgram from bytes")?;
        Ok(TkProgram { hugr })
    }

    /// Loads a HUGR from an envelope string.
    #[staticmethod]
    #[pyo3(signature = (envelope))]
    #[expect(clippy::should_implement_trait)] // Cannot use AsRef<str> with pyo3 methods.
    pub fn from_str(envelope: &str) -> anyhow::Result<Self> {
        let hugr = Hugr::load_str(envelope, Some(&REGISTRY))
            .context("Could not read TkProgram from string")?;
        Ok(TkProgram { hugr })
    }

    /// Compute the cost of the circuit based on a per-operation cost function.
    ///
    /// :param cost_fn: A function that takes a `TketOp` and returns an arbitrary cost.
    ///     The cost type must implement `__add__`, `__sub__`, `__lt__`,
    ///     `__eq__`, `__int__`, and integer `__div__`.
    ///
    /// :returns: The sum of all operation costs.
    //
    // TODO: This needs to be updated to handle non-tket operations, passing a `hugr.ops.Op` to the cost function.
    pub fn _circuit_cost<'py>(
        &self,
        cost_fn: &Bound<'py, PyAny>,
    ) -> anyhow::Result<Bound<'py, PyAny>> {
        let py = cost_fn.py();
        let cost_fn = |op: &OpType| -> anyhow::Result<PyCircuitCost> {
            let Some(tk2_op) = op.cast::<TketOp>() else {
                return Ok(PyCircuitCost::default());
            };
            let tk2_py_op = PyTketOp::from(tk2_op);
            let cost = cost_fn.call1((tk2_py_op,))?;
            Ok(PyCircuitCost { cost: cost.into() })
        };
        let circ_cost = Circuit::new(&self.hugr).circuit_cost(cost_fn)?;
        Ok(circ_cost.cost.into_bound(py))
    }

    /// Returns the number of operations in the circuit.
    ///
    /// This includes [`TketOp`]s, pytket ops, and any other custom operations.
    ///
    /// Nested circuits are traversed to count their operations.
    pub fn num_operations(&self) -> anyhow::Result<usize> {
        let ops = Circuit::try_new(&self.hugr)
            .context("Could not count circuit operations")?
            .num_operations();
        Ok(ops)
    }

    /// Returns a hash of the circuit.
    pub fn hash(&self) -> anyhow::Result<u64> {
        let hash = Circuit::try_new(&self.hugr)
            .context("Could not create circuit for hashing")?
            .circuit_hash(self.hugr.entrypoint())?;
        Ok(hash)
    }

    /// Copy the circuit.
    pub fn __copy__(&self) -> anyhow::Result<Self> {
        Ok(self.clone())
    }

    /// Copy the circuit.
    pub fn __deepcopy__(&self, _memo: Bound<PyAny>) -> anyhow::Result<Self> {
        Ok(self.clone())
    }

    /// Return the mermaid representation of the program.
    pub fn render_mermaid(&self) -> String {
        self.hugr.mermaid_string()
    }

    /// Validate the program, checking for structural issues.
    ///
    /// Returns `Ok(())` if the program is valid, and raises an exception with details if not.
    pub fn validate(&self) -> anyhow::Result<()> {
        self.hugr.validate().context("TkProgram validation failed")
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

/// Returns a list of extension ids supported by the TkProgram loader.
///
/// Extensions not in this list must be included in the package when
/// loading a TkProgram.
#[pyfunction]
pub fn embedded_extensions() -> Vec<String> {
    REGISTRY.iter().map(|e| e.name.to_string()).collect()
}
