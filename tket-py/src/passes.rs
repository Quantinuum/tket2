//! Passes for optimising circuits.

pub mod chunks;
mod scope;
pub mod tket1;

pub(crate) use scope::PyPassScope;
use tket::control::structuralize::StructuralizationStrategy;

use std::{cmp::min, convert::TryInto, fs, num::NonZeroUsize, path::PathBuf};

use pyo3::prelude::*;
use tket::optimiser::badger::BadgerOptions;
use tket::passes;
use tket::passes::composable::{ComposablePass, WithScope};
use tket::{Circuit, TketOp, op_matches};

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
    m.add_function(wrap_pyfunction!(case_of_case, &m)?)?;
    m.add_function(wrap_pyfunction!(sink_conditional_inputs, &m)?)?;
    m.add_function(wrap_pyfunction!(structuralize_cfgs, &m)?)?;
    m.add_function(wrap_pyfunction!(inline_functions, &m)?)?;
    m.add_class::<self::chunks::PyCircuitChunks>()?;
    m.add_function(wrap_pyfunction!(self::chunks::chunks, &m)?)?;
    m.add_function(wrap_pyfunction!(self::tket1::tket1_pass, &m)?)?;
    m.add_function(wrap_pyfunction!(resolve_modifiers, &m)?)?;
    m.add("PullForwardError", py.get_type::<PyPullForwardError>())?;
    m.add("CaseOfCaseError", py.get_type::<PyCaseOfCaseError>())?;
    m.add(
        "StructuralizeCfgsError",
        py.get_type::<PyStructuralizeCfgsError>(),
    )?;
    m.add(
        "SinkConditionalInputsError",
        py.get_type::<PySinkConditionalInputsError>(),
    )?;
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
    tket::passes::case_of_case::CaseOfCaseError,
    PyCaseOfCaseError,
    "Errors from the case-of-case pass."
);

create_py_exception!(
    tket::control::structuralize::StructuralizationError,
    PyStructuralizeCfgsError,
    "Errors from the CFG structuralization pass."
);

create_py_exception!(
    tket::passes::sink_conditional_inputs::SinkConditionalInputsError,
    PySinkConditionalInputsError,
    "Errors from the conditional-input sinking pass."
);

create_py_exception!(
    tket::passes::inline_funcs::InlineFuncsError,
    PyInlineFunctionsError,
    "Errors from the function inlining pass."
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

/// Fuse direct sibling conditional chains with a conservative case-of-case rewrite.
///
/// This pass looks for direct sibling `Conditional A -> Conditional B` chains
/// in one region at a time and duplicates small selected consumer cases into
/// the producer when the selector is known branch-locally.
#[pyfunction]
#[pyo3(signature = (circ, *, max_duplicated_nodes = 32, scope = None))]
fn case_of_case(
    circ: &mut CompilationState,
    max_duplicated_nodes: usize,
    scope: Option<PyPassScope>,
) -> PyResult<()> {
    let py_scope = scope.unwrap_or_default();
    let pass = tket::passes::CaseOfCasePass::default_with_scope(py_scope.scope)
        .with_max_duplicated_nodes(max_duplicated_nodes);
    pass.run(&mut circ.hugr).convert_pyerrs()?;
    Ok(())
}

/// Sink branch-local shared conditional inputs into the case that uses them.
///
/// Shared `other_inputs` to a conditional that are used by at most one case
/// are rebuilt inside that case, or removed entirely if they are unused.
#[pyfunction]
#[pyo3(signature = (circ, *, scope = None))]
fn sink_conditional_inputs(
    circ: &mut CompilationState,
    scope: Option<PyPassScope>,
) -> PyResult<()> {
    let py_scope = scope.unwrap_or_default();
    let pass = tket::passes::SinkConditionalInputsPass::default_with_scope(py_scope.scope);
    pass.run(&mut circ.hugr).convert_pyerrs()?;
    Ok(())
}

/// Structuralize CFG regions into nested `Conditional` and `TailLoop` nodes.
///
/// Parameters:
/// - strategy: Structuralization strategy to use. One of `"rvsdg"` or
///   `"beyond_relooper"`.
/// - skip_unstructuralizable_cfgs: When true, CFGs that cannot be structuralized
///   are left unchanged instead of causing the pass to fail.
#[pyfunction]
#[pyo3(signature = (
    circ,
    *,
    strategy = "rvsdg",
    skip_unstructuralizable_cfgs = true,
    scope = None
))]
fn structuralize_cfgs(
    circ: &mut CompilationState,
    strategy: &str,
    skip_unstructuralizable_cfgs: bool,
    scope: Option<PyPassScope>,
) -> PyResult<()> {
    let py_scope = scope.unwrap_or_default();

    let strategy = match strategy {
        "rvsdg" => StructuralizationStrategy::Rvsdg,
        "relooper" => StructuralizationStrategy::Relooper,
        _ => Err(pyo3::exceptions::PyValueError::new_err(format!(
            "unknown structuralization strategy {strategy:?}; expected 'rvsdg' or 'relooper'"
        )))?,
    };

    let pass = tket::passes::StructuralizeCfgsPass::default_with_scope(py_scope.scope)
        .with_strategy(strategy)
        .with_skip_unstructuralizable_cfgs(skip_unstructuralizable_cfgs);
    pass.run(&mut circ.hugr).convert_pyerrs()?;
    Ok(())
}

/// Inline acyclic function calls below the selected scope.
///
/// Parameters:
/// - max_inline_size: Maximum number of descendants allowed in a callee for
///   its call sites to be inlined.
#[pyfunction]
#[pyo3(signature = (circ, *, max_inline_size = 64, scope = None))]
fn inline_functions(
    circ: &mut CompilationState,
    max_inline_size: usize,
    scope: Option<PyPassScope>,
) -> PyResult<()> {
    let py_scope = scope.unwrap_or_default();
    let pass = tket::passes::InlineFunctionsPass::default_with_scope(py_scope.scope)
        .with_max_inline_size(max_inline_size);
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
        .commands()
        .filter(|c| op_matches(c.optype(), TketOp::CX))
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
