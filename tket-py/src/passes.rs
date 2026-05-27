//! Passes for optimising circuits.

pub mod chunks;
mod inline_funcs;
mod scope;
pub mod tket1;

use hugr::HugrView;
pub(crate) use scope::PyPassScope;

use std::{cmp::min, convert::TryInto, fs, num::NonZeroUsize, path::PathBuf};

use pyo3::prelude::*;
use tket::optimiser::badger::BadgerOptions;
use tket::passes::composable::{ComposablePass, WithScope};
use tket::{Circuit, TketOp};

use tket::passes;

use crate::optimiser::PyBadgerOptimiser;
use crate::state::CompilationState;
use crate::utils::{ConvertPyErr, create_py_exception};

/// The module definition
///
/// This module is re-exported from the python module with the same name.
pub fn module(py: Python<'_>) -> PyResult<Bound<'_, PyModule>> {
    let m = PyModule::new(py, "passes")?;
    m.add_function(wrap_pyfunction!(greedy_depth_reduce, &m)?)?;
    m.add_function(wrap_pyfunction!(badger_optimise, &m)?)?;
    m.add_function(wrap_pyfunction!(normalize_guppy, &m)?)?;
    m.add_function(wrap_pyfunction!(self::inline_funcs::inline_functions, &m)?)?;
    m.add_class::<self::chunks::PyCircuitChunks>()?;
    m.add_function(wrap_pyfunction!(self::chunks::chunks, &m)?)?;
    m.add_function(wrap_pyfunction!(self::tket1::tket1_pass, &m)?)?;
    m.add_function(wrap_pyfunction!(resolve_modifiers, &m)?)?;
    m.add_function(wrap_pyfunction!(qsystem_rebase_pass, &m)?)?;
    m.add("PullForwardError", py.get_type::<PyPullForwardError>())?;
    m.add(
        "InlineFunctionsError",
        py.get_type::<PyInlineFunctionsError>(),
    )?;
    m.add("TK1PassError", py.get_type::<tket1::PytketPassError>())?;
    Ok(m)
}

create_py_exception!(
    tket::passes::commutation::PullForwardError,
    PyPullForwardError,
    "Error from a `PullForward` operation"
);

create_py_exception!(
    tket::passes::guppy::NormalizeGuppyErrors,
    PyNormalizeGuppyError,
    "Errors from the Guppy normalization pass."
);

create_py_exception!(
    tket::passes::modifier_resolver::ModifierResolverErrors,
    PyModifierResolverError,
    "Errors from the modifer resolver pass."
);

create_py_exception!(
    tket::passes::inline_funcs::InlineFuncsError,
    PyInlineFunctionsError,
    "Errors from the function inlining pass."
);

create_py_exception!(
    tket_qsystem::QSystemPassError,
    PyQSystemPassError,
    "Errors from the QSystem rebase pass."
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
/// - squash_borrows: Whether to squash return-borrow pairs on BorrowArrays.
/// - remove_redundant_order_edges: Whether to remove redundant order edges.
#[pyfunction]
#[pyo3(signature = (circ, *, simplify_cfgs = true, remove_tuple_untuple = true, constant_folding = true, remove_dead_funcs = true, inline_dfgs = true, remove_redundant_order_edges = true, squash_borrows = true, scope = None))]
#[expect(clippy::too_many_arguments)]
fn normalize_guppy(
    circ: &mut CompilationState,
    simplify_cfgs: bool,
    remove_tuple_untuple: bool,
    constant_folding: bool,
    remove_dead_funcs: bool,
    inline_dfgs: bool,
    remove_redundant_order_edges: bool,
    squash_borrows: bool,
    scope: Option<PyPassScope>,
) -> PyResult<()> {
    let py_scope = scope.unwrap_or_default();
    let mut pass = tket::passes::NormalizeGuppy::default_with_scope(py_scope.scope);

    pass.simplify_cfgs(simplify_cfgs)
        .remove_tuple_untuple(remove_tuple_untuple)
        .constant_folding(constant_folding)
        .remove_dead_funcs(remove_dead_funcs)
        .inline_dfgs(inline_dfgs)
        .remove_redundant_order_edges(remove_redundant_order_edges)
        .squash_borrows(squash_borrows);

    pass.run(&mut circ.hugr).convert_pyerrs()?;
    Ok(())
}

/// Pass which greedily commutes operations forwards in order to reduce depth.
#[pyfunction]
fn greedy_depth_reduce(circ: &mut CompilationState) -> PyResult<u32> {
    let mut c = Circuit::new(circ.hugr.clone());
    let n_moves = passes::apply_greedy_commutation(&mut c).convert_pyerrs()?;
    circ.hugr = c.into_hugr();
    Ok(n_moves)
}

/// Badger optimisation pass.
///
/// HyperTKET's best attempt at optimising a circuit using circuit rewriting
/// and the given Badger optimiser.
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
#[pyo3(signature = (circ, optimiser, max_threads=None, timeout=None, progress_timeout=None, max_circuit_count=None, log_dir=None))]
fn badger_optimise(
    circ: &mut CompilationState,
    optimiser: &PyBadgerOptimiser,
    max_threads: Option<NonZeroUsize>,
    timeout: Option<u64>,
    progress_timeout: Option<u64>,
    max_circuit_count: Option<usize>,
    log_dir: Option<PathBuf>,
) -> PyResult<()> {
    // Default parameter values
    let max_threads = max_threads.unwrap_or(num_cpus::get().try_into().unwrap());
    let timeout = timeout.unwrap_or(30);
    // Create log directory if necessary
    if let Some(log_dir) = log_dir.as_ref() {
        fs::create_dir_all(log_dir)?;
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
    let c = Circuit::new(&circ.hugr);
    let n_cx = c
        .toposorted_children(c.parent())
        .expect("circuit entrypoint should be dataflow region")
        .filter(|&n| c.hugr().get_optype(n).cast::<TketOp>() == Some(TketOp::CX))
        .count();
    let n_threads = min(
        (n_cx / 50).try_into().unwrap_or(1.try_into().unwrap()),
        max_threads,
    );
    let (split_threads, split_timeouts) = badger_splits(n_threads);
    let mut optimised = Circuit::new(circ.hugr.clone());
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
        optimised = optimiser.optimise(optimised, log_file, options);
    }
    circ.hugr = optimised.into_hugr();
    Ok(())
}

#[pyfunction]
#[pyo3(signature = (circ, scope = None))]
fn resolve_modifiers(circ: &mut CompilationState, scope: Option<PyPassScope>) -> PyResult<()> {
    let py_scope = scope.unwrap_or_default();
    let pass = tket::passes::ModifierResolverPass::default_with_scope(py_scope.scope);
    pass.run(&mut circ.hugr).convert_pyerrs()?;
    Ok(())
}

#[pyfunction]
#[pyo3(signature=(circ, *, constant_fold = true, monomorphize = true, force_order = true, lazify = true, hide_funcs = true, scope = None))]
fn qsystem_rebase_pass(
    circ: &mut CompilationState,
    constant_fold: bool,
    monomorphize: bool,
    force_order: bool,
    lazify: bool,
    hide_funcs: bool,
    scope: Option<PyPassScope>,
) -> PyResult<()> {
    let py_scope = scope.unwrap_or_default();
    let qsystem_pass = tket_qsystem::QSystemPass::default_with_scope(py_scope.scope)
        .with_constant_fold(constant_fold)
        .with_monomorphize(monomorphize)
        .with_force_order(force_order)
        .with_lazify(lazify)
        .with_hide_funcs(hide_funcs);

    qsystem_pass.run(&mut circ.hugr).convert_pyerrs()?;
    Ok(())
}
