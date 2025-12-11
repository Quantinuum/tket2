//! PyO3 wrapper for the Badger circuit optimiser.

use std::io::BufWriter;
use std::{fs, num::NonZeroUsize, path::PathBuf};

use derive_more::derive::From;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use tket::Circuit;
use tket::optimiser::badger::{BadgerOptions, DefaultBadgerStrategy};
use tket::optimiser::{BadgerLogger, BadgerOptimiser};

use crate::circuit::update_circ;
use crate::rewrite::{PyECCRewriter, PyRewriter};

/// The module definition
pub fn module(py: Python<'_>) -> PyResult<Bound<'_, PyModule>> {
    let m = PyModule::new(py, "optimiser")?;
    m.add_class::<PyBadgerOptimiser>()?;
    Ok(m)
}

/// Wrapped [`DefaultBadgerOptimiser`].
///
/// Currently only exposes loading from an ECC file using the constructor
/// and optimising using default logging settings.
#[pyclass(name = "BadgerOptimiser")]
#[derive(Clone, From)]
pub struct PyBadgerOptimiser(BadgerOptimiser<PyRewriter, DefaultBadgerStrategy>);

/// The cost function to use for the Badger optimiser.
#[derive(Debug, Clone, Copy, Default)]
pub enum BadgerCostFunction {
    /// Minimise CX count.
    #[default]
    CXCount,
    /// Minimise Rz count.
    RzCount,
}

impl BadgerCostFunction {
    fn into_strategy(self) -> DefaultBadgerStrategy {
        match self {
            BadgerCostFunction::CXCount => DefaultBadgerStrategy::cx_count(),
            BadgerCostFunction::RzCount => DefaultBadgerStrategy::rz_count(),
        }
    }
}

impl<'py> FromPyObject<'py> for BadgerCostFunction {
    fn extract_bound(ob: &Bound<'py, PyAny>) -> PyResult<Self> {
        let str = ob.extract::<&str>()?;
        match str {
            "cx" => Ok(BadgerCostFunction::CXCount),
            "rz" => Ok(BadgerCostFunction::RzCount),
            _ => Err(PyErr::new::<PyValueError, _>(format!(
                "Invalid cost function: {str}. Expected 'cx' or 'rz'."
            ))),
        }
    }
}

#[pymethods]
impl PyBadgerOptimiser {
    /// Create a new [`PyBadgerOptimiser`] from a rewriter and cost function.
    #[new]
    #[pyo3(signature = (rewriter, cost_fn=None))]
    pub fn new(rewriter: PyRewriter, cost_fn: Option<BadgerCostFunction>) -> Self {
        BadgerOptimiser::new(rewriter, cost_fn.unwrap_or_default().into_strategy()).into()
    }

    /// Create a new [`PyDefaultBadgerOptimiser`] from a precompiled rewriter.
    #[staticmethod]
    #[pyo3(signature = (path, cost_fn=None))]
    pub fn load_precompiled(path: PathBuf, cost_fn: Option<BadgerCostFunction>) -> Self {
        let rewriter = PyECCRewriter::load_precompiled(path).unwrap();
        let strategy = cost_fn.unwrap_or_default().into_strategy();
        BadgerOptimiser::new(PyRewriter::ECC(rewriter), strategy).into()
    }

    /// Create a new [`PyDefaultBadgerOptimiser`] from ECC sets.
    ///
    /// This will compile the rewriter from the provided ECC JSON file.
    #[staticmethod]
    #[pyo3(signature = (path, cost_fn=None))]
    pub fn compile_eccs(path: &str, cost_fn: Option<BadgerCostFunction>) -> Self {
        let rewriter = PyECCRewriter::compile_eccs(path).unwrap();
        let strategy = cost_fn.unwrap_or_default().into_strategy();
        BadgerOptimiser::new(PyRewriter::ECC(rewriter), strategy).into()
    }

    /// Run the optimiser on a circuit.
    ///
    /// Returns an optimised circuit and optionally log the progress to a CSV
    /// file.
    ///
    /// # Parameters
    ///
    /// * `circ`: The circuit to optimise.
    ///
    /// * `timeout`: The maximum time (in seconds) to run the optimiser.
    ///
    ///     If `None` the optimiser will run indefinitely, or until
    ///     `progress_timeout` is reached.
    ///
    /// * `progress_timeout`: The maximum time (in seconds) to search for new
    ///     improvements to the circuit. If no progress is made in this time,
    ///     the optimiser will stop.
    ///
    ///     If `None` the optimiser will run indefinitely, or until `timeout` is
    ///     reached.
    ///
    /// * `max_circuit_count`: The maximum number of circuits to process before
    ///     stopping the optimisation.
    ///
    ///
    ///     For data parallel multi-threading, (split_circuit=true), applies on
    ///     a per-thread basis, otherwise applies globally.
    ///
    ///     If `None` the optimiser will run indefinitely, or until `timeout` is
    ///     reached.
    ///
    /// * `n_threads`: The number of threads to use. Defaults to `1`.
    ///
    /// * `split_circ`: Whether to split the circuit into chunks and process
    ///     each in a separate thread.
    ///
    ///     If this option is set to `true`, the optimiser will split the
    ///     circuit into `n_threads` chunks.
    ///
    ///     If this option is set to `false`, the optimiser will run `n_threads`
    ///     parallel searches on the whole circuit (default).
    ///
    /// * `queue_size`: The maximum size of the circuit candidates priority
    ///     queue. Defaults to `20`.
    ///
    /// * `log_progress`: The path to a CSV file to log progress to.
    ///
    #[pyo3(name = "optimise")]
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (circ, timeout=None, progress_timeout=None, max_circuit_count=None, n_threads=None, split_circ=None, queue_size=None, log_progress=None))]
    pub fn py_optimise<'py>(
        &self,
        circ: &Bound<'py, PyAny>,
        timeout: Option<u64>,
        progress_timeout: Option<u64>,
        max_circuit_count: Option<usize>,
        n_threads: Option<NonZeroUsize>,
        split_circ: Option<bool>,
        queue_size: Option<usize>,
        log_progress: Option<PathBuf>,
    ) -> PyResult<Bound<'py, PyAny>> {
        let options = BadgerOptions {
            timeout,
            progress_timeout,
            max_circuit_count,
            n_threads: n_threads.unwrap_or(NonZeroUsize::new(1).unwrap()),
            split_circuit: split_circ.unwrap_or(false),
            queue_size: queue_size.unwrap_or(100),
        };
        update_circ(circ, |circ, _| self.optimise(circ, log_progress, options))
    }
}

impl PyBadgerOptimiser {
    /// The Python optimise method, but on Hugrs.
    pub(super) fn optimise(
        &self,
        circ: Circuit,
        log_progress: Option<PathBuf>,
        options: BadgerOptions,
    ) -> Circuit {
        let badger_logger = log_progress
            .map(|file_name| {
                let log_file = fs::File::create(file_name).unwrap();
                let log_file = BufWriter::new(log_file);
                BadgerLogger::new(log_file)
            })
            .unwrap_or_default();
        self.0.optimise_with_log(&circ, badger_logger, options)
    }
}
