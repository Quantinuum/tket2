//! Passes for optimising circuits.

pub mod chunks;
pub mod tket1;

use std::{cmp::min, convert::TryInto, fs, num::NonZeroUsize, path::PathBuf};

use hugr::algorithms::ComposablePass;
use pyo3::{prelude::*, types::IntoPyDict};
use tket::optimiser::badger::BadgerOptions;
use tket::passes;
use tket::{TketOp, op_matches};

use crate::circuit::CircuitType;
use crate::utils::{ConvertPyErr, create_py_exception};
use crate::{
    circuit::{try_update_circ, try_with_circ},
    optimiser::PyBadgerOptimiser,
};

/// The module definition
///
/// This module is re-exported from the python module with the same name.
pub fn module(py: Python<'_>) -> PyResult<Bound<'_, PyModule>> {
    let m = PyModule::new(py, "passes")?;
    m.add_function(wrap_pyfunction!(greedy_depth_reduce, &m)?)?;
    m.add_function(wrap_pyfunction!(lower_to_pytket, &m)?)?;
    m.add_function(wrap_pyfunction!(badger_optimise, &m)?)?;
    m.add_function(wrap_pyfunction!(normalize_guppy, &m)?)?;
    m.add_class::<self::chunks::PyCircuitChunks>()?;
    m.add_function(wrap_pyfunction!(self::chunks::chunks, &m)?)?;
    m.add_function(wrap_pyfunction!(self::tket1::tket1_pass, &m)?)?;
    m.add("PullForwardError", py.get_type::<PyPullForwardError>())?;
    m.add("TK1PassError", py.get_type::<tket1::PytketPassError>())?;
    Ok(m)
}

create_py_exception!(
    tket::passes::PullForwardError,
    PyPullForwardError,
    "Error from a `PullForward` operation"
);

create_py_exception!(
    tket::passes::pytket::PytketLoweringError,
    PyPytketLoweringError,
    "Errors that can occur while removing high-level operations from HUGR intended to be encoded as a pytket circuit."
);

create_py_exception!(
    tket::passes::guppy::NormalizeGuppyErrors,
    PyNormalizeGuppyError,
    "Errors from the Guppy normalization pass."
);

/// Flatten the structure of a Guppy-generated program to enable additional optimisations.
///
/// This should normally be called first before other optimisations.
///
/// Parameters:
/// - simplify_cfgs: Whether to simplify CFG control flow.
/// - remove_tuple_untuple: Whether to remove tuple/untuple operations.
/// - constant_folding: Whether to constant fold the program.
/// - remove_dead_funcs: Whether to remove dead functions.
/// - inline_dfgs: Whether to inline DFG operations.
/// - remove_redundant_order_edges: Whether to remove redundant order edges.
#[pyfunction]
#[pyo3(signature = (circ, *, simplify_cfgs = true, remove_tuple_untuple = true, constant_folding = true, remove_dead_funcs = true, inline_dfgs = true, remove_redundant_order_edges = true))]
fn normalize_guppy<'py>(
    circ: &Bound<'py, PyAny>,
    simplify_cfgs: bool,
    remove_tuple_untuple: bool,
    constant_folding: bool,
    remove_dead_funcs: bool,
    inline_dfgs: bool,
    remove_redundant_order_edges: bool,
) -> PyResult<Bound<'py, PyAny>> {
    let py = circ.py();
    try_with_circ(circ, |mut circ, typ| {
        let mut pass = tket::passes::NormalizeGuppy::default();

        pass.simplify_cfgs(simplify_cfgs)
            .remove_tuple_untuple(remove_tuple_untuple)
            .constant_folding(constant_folding)
            .remove_dead_funcs(remove_dead_funcs)
            .inline_dfgs(inline_dfgs)
            .remove_redundant_order_edges(remove_redundant_order_edges);

        pass.run(circ.hugr_mut()).convert_pyerrs()?;

        let circ = typ.convert(py, circ)?;
        PyResult::Ok(circ)
    })
}

/// Pass which greedily commutes operations forwards in order to reduce depth.
#[pyfunction]
fn greedy_depth_reduce<'py>(circ: &Bound<'py, PyAny>) -> PyResult<(Bound<'py, PyAny>, u32)> {
    let py = circ.py();
    try_with_circ(circ, |mut circ, typ| {
        let n_moves = passes::apply_greedy_commutation(&mut circ).convert_pyerrs()?;
        let circ = typ.convert(py, circ)?;
        PyResult::Ok((circ, n_moves))
    })
}

/// Rebase a circuit to the Nam gate set (CX, Rz, H) using TKET1.
///
/// Equivalent to running the following code:
/// ```python
/// from pytket.passes import AutoRebase
/// from pytket import OpType
/// AutoRebase({OpType.CX, OpType.Rz, OpType.H}).apply(circ)"
// ```
fn rebase_nam(circ: &Bound<PyAny>) -> PyResult<()> {
    let py = circ.py();
    let auto_rebase = py.import("pytket.passes")?.getattr("AutoRebase")?;
    let optype = py.import("pytket")?.getattr("OpType")?;
    let locals = [("OpType", &optype)].into_py_dict(py)?;
    let op_set = py.eval(c"{OpType.CX, OpType.Rz, OpType.H}", None, Some(&locals))?;
    let rebase_pass = auto_rebase.call1((op_set,))?.getattr("apply")?;
    rebase_pass.call1((circ,)).map(|_| ())
}

/// A pass that removes high-level control flow from a HUGR, so it can be used in pytket.
#[pyfunction]
fn lower_to_pytket<'py>(circ: &Bound<'py, PyAny>) -> PyResult<Bound<'py, PyAny>> {
    let py = circ.py();
    try_with_circ(circ, |circ, typ| match typ {
        CircuitType::Tket1 => {
            // If the circuit is already in tket1 format, just return it.
            let circ = typ.convert(py, circ)?;
            PyResult::Ok(circ)
        }
        CircuitType::Tket => {
            let circ = passes::lower_to_pytket(&circ).convert_pyerrs()?;
            let circ = typ.convert(py, circ)?;
            PyResult::Ok(circ)
        }
    })
}

/// Badger optimisation pass.
///
/// HyperTKET's best attempt at optimising a circuit using circuit rewriting
/// and the given Badger optimiser.
///
/// By default, the input circuit will be rebased to Nam, i.e. CX + Rz + H before
/// optimising. This can be deactivated by setting `rebase` to `false`, in which
/// case the circuit is expected to be in the Nam gate set.
///
/// Will use at most `max_threads` threads (plus a constant). Defaults to the
/// number of CPUs available.
///
/// The optimisation will terminate at the first of the following timeout
/// criteria, if set:
/// - `timeout` seconds (default: 15min) have elapsed since the start of the
///    optimisation
/// - `progress_timeout` (default: None) seconds have elapsed since progress
///    in the cost function was last made
/// - `max_circuit_count` (default: None) circuits have been explored.
///
/// Log files will be written to the directory `log_dir` if specified.
#[pyfunction]
#[expect(clippy::too_many_arguments)]
#[pyo3(signature = (circ, optimiser, max_threads=None, timeout=None, progress_timeout=None, max_circuit_count=None, log_dir=None, rebase=None))]
fn badger_optimise<'py>(
    circ: &Bound<'py, PyAny>,
    optimiser: &PyBadgerOptimiser,
    max_threads: Option<NonZeroUsize>,
    timeout: Option<u64>,
    progress_timeout: Option<u64>,
    max_circuit_count: Option<usize>,
    log_dir: Option<PathBuf>,
    rebase: Option<bool>,
) -> PyResult<Bound<'py, PyAny>> {
    // Default parameter values
    let rebase = rebase.unwrap_or(true);
    let max_threads = max_threads.unwrap_or(num_cpus::get().try_into().unwrap());
    let timeout = timeout.unwrap_or(30);
    // Create log directory if necessary
    if let Some(log_dir) = log_dir.as_ref() {
        fs::create_dir_all(log_dir)?;
    }
    // Rebase circuit
    if rebase {
        rebase_nam(circ)?;
    }
    // Logic to choose how to split the circuit
    let badger_splits = |n_threads: NonZeroUsize| match n_threads.get() {
        n if n >= 7 => (
            vec![n, 3, 1],
            vec![timeout / 2, timeout / 10 * 3, timeout / 10 * 2],
        ),
        n if n >= 4 => (
            vec![n, 2, 1],
            vec![timeout / 2, timeout / 10 * 3, timeout / 10 * 2],
        ),
        n if n > 1 => (vec![n, 1], vec![timeout / 2, timeout / 2]),
        1 => (vec![1], vec![timeout]),
        _ => unreachable!(),
    };
    // Optimise
    try_update_circ(circ, |mut circ, _| {
        let n_cx = circ
            .commands()
            .filter(|c| op_matches(c.optype(), TketOp::CX))
            .count();
        let n_threads = min(
            (n_cx / 50).try_into().unwrap_or(1.try_into().unwrap()),
            max_threads,
        );
        let (split_threads, split_timeouts) = badger_splits(n_threads);
        for (i, (n_threads, timeout)) in split_threads.into_iter().zip(split_timeouts).enumerate() {
            let log_file = log_dir.as_ref().map(|log_dir| {
                let mut log_file = log_dir.clone();
                log_file.push(format!("cycle-{i}.log"));
                log_file
            });
            let options = BadgerOptions {
                timeout: Some(timeout),
                progress_timeout,
                n_threads: n_threads.try_into().unwrap(),
                split_circuit: true,
                max_circuit_count,
                ..Default::default()
            };
            circ = optimiser.optimise(circ, log_file, options);
        }
        PyResult::Ok(circ)
    })
}
