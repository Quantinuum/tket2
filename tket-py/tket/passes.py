from pathlib import Path
from typing import Optional, Literal
import json
from dataclasses import dataclass

from hugr import Hugr
from pytket.passes import (
    CustomPass,
    BasePass,
)

from tket import optimiser
from tket._tket.optimiser import BadgerOptimiser
from tket.program import TkProgram

from hugr.passes._composable_pass import (
    ComposablePass,
    ComposedPass,
    implement_pass_run,
    PassResult,
)


# Import the native bindings (used internally).
from ._tket.passes import (
    CircuitChunks,
    greedy_depth_reduce as _greedy_depth_reduce,
    badger_optimise as _badger_optimise,
    chunks as _chunks,
    tket1_pass as _tket1_pass,
    normalize_guppy as _normalize_guppy,
    PullForwardError,
)


__all__ = [
    "badger_pass",
    # Bindings.
    # TODO: Wrap these in Python classes.
    "CircuitChunks",
    "greedy_depth_reduce",
    "badger_optimise",
    "chunks",
    "PullForwardError",
    "PytketHugrPass",
    "PassResult",
    "NormalizeGuppy",
]


def badger_pass(
    rewriter: Optional[Path] = None,
    max_threads: Optional[int] = None,
    timeout: Optional[int] = None,
    progress_timeout: Optional[int] = None,
    max_circuit_count: Optional[int] = None,
    log_dir: Optional[Path] = None,
    cost_fn: Literal["cx", "rz"] | None = None,
) -> BasePass:
    """Construct a Badger pass.

    The Badger optimiser requires a pre-compiled rewriter produced by the
    `compile-rewriter <https://github.com/quantinuum/tket2/tree/main/badger-optimiser>`_
    utility. If `rewriter` is not specified, a default one will be used.

    The cost function to minimise can be specified by passing `cost_fn` as `'cx'`
    or `'rz'`. If not specified, the default is `'cx'`.

    The arguments `max_threads`, `timeout`, `progress_timeout`, `max_circuit_count`,
    and `log_dir` are optional and will be passed on to the Badger
    optimiser if provided."""
    if rewriter is None:
        try:
            import tket_eccs
        except ImportError:
            raise ValueError(
                "The default rewriter is not available. Please specify a path to a rewriter or install tket-eccs."
            )

        rewriter = tket_eccs.nam_6_3()
    opt = optimiser.BadgerOptimiser.load_precompiled(rewriter, cost_fn=cost_fn)

    def apply(circuit):
        """Apply Badger optimisation to the circuit."""
        from tket._tket.program import TkProgram as RustTkProgram

        tk = RustTkProgram.from_tket1(circuit)
        badger_optimise(
            tk,
            optimiser=opt,
            max_threads=max_threads,
            timeout=timeout,
            progress_timeout=progress_timeout,
            max_circuit_count=max_circuit_count,
            log_dir=log_dir,
        )
        return tk

    return CustomPass(apply, label="tket.badger_pass")


@dataclass
class PytketHugrPass(ComposablePass):
    pytket_passes: list[BasePass]

    """
    A class which provides an interface to apply pytket passes to Hugr programs.

    The user can create a :py:class:`PytketHugrPass` object from any serializable member of `pytket.passes`.
    """

    def __init__(self, *pytket_passes: BasePass) -> None:
        """Initialize a PytketHugrPass from a :py:class:`~pytket.passes.BasePass` instance."""
        self.pytket_passes = list(pytket_passes)

    def run(self, hugr: Hugr, *, inplace: bool = True) -> PassResult:
        """Run the pytket pass as a HUGR transform returning a PassResult."""
        return implement_pass_run(
            self,
            hugr=hugr,
            inplace=inplace,
            copy_call=lambda h: self._run_pytket_pass_on_hugr(h, inplace),
        )

    def then(self, other: ComposablePass) -> ComposablePass:
        """Perform another composable pass after this pass."""
        if isinstance(other, PytketHugrPass):
            return PytketHugrPass(*self.pytket_passes, *other.pytket_passes)
        else:
            return ComposedPass(self, other)

    def _run_pytket_pass_on_hugr(self, hugr: Hugr, inplace: bool) -> PassResult:
        tk_program = TkProgram.from_python(hugr)
        for py_pass in self.pytket_passes:
            pass_json = json.dumps(py_pass.to_dict())
            _tket1_pass(tk_program._inner, pass_json, traverse_subcircuits=True)

        package = tk_program.to_python()
        new_hugr = package.modules[0]
        if tk_program._py_extensions is not None:
            new_hugr.resolve_extensions(tk_program._py_extensions)
        return PassResult.for_pass(self, hugr=new_hugr, inplace=inplace, result=None)


@dataclass
class NormalizeGuppy(ComposablePass):
    simplify_cfgs: bool = True
    remove_tuple_untuple: bool = True
    constant_folding: bool = True
    remove_dead_funcs: bool = True
    inline_dfgs: bool = True
    remove_redundant_order_edges: bool = True
    squash_borrows: bool = True

    """Flatten the structure of a Guppy-generated program to enable additional optimisations.

    This should normally be called first before other optimisations.

    Parameters:
    - simplify_cfgs: Whether to simplify CFG control flow.
    - remove_tuple_untuple: Whether to remove tuple/untuple operations.
    - constant_folding: Whether to constant fold the program.
    - remove_dead_funcs: Whether to remove dead functions.
    - inline_dfgs: Whether to inline DFG operations.
    - remove_redundant_order_edges: Whether to remove redundant order edges.
    - squash_borrows: Whether to squash return-borrow pairs on BorrowArrays.
    """

    def run(self, hugr: Hugr, *, inplace: bool = True) -> PassResult:
        return implement_pass_run(
            self,
            hugr=hugr,
            inplace=inplace,
            copy_call=lambda h: self._normalize(h, inplace),
        )

    def _normalize(self, hugr: Hugr, inplace: bool) -> PassResult:
        tk_program = TkProgram.from_python(hugr)

        self._run_tk(tk_program)

        package = tk_program.to_python()
        return PassResult.for_pass(
            self, hugr=package.modules[0], inplace=inplace, result=None
        )

    def _run_tk(self, program: TkProgram) -> TkProgram:
        """Run the pass in the TkProgram

        TODO: This should be part of a protocol."""
        _normalize_guppy(
            program._inner,
            simplify_cfgs=self.simplify_cfgs,
            remove_tuple_untuple=self.remove_tuple_untuple,
            constant_folding=self.constant_folding,
            remove_dead_funcs=self.remove_dead_funcs,
            inline_dfgs=self.inline_dfgs,
            remove_redundant_order_edges=self.remove_redundant_order_edges,
            squash_borrows=self.squash_borrows,
        )
        return program


def greedy_depth_reduce(program: TkProgram) -> int:
    return _greedy_depth_reduce(program._inner)


def badger_optimise(
    program: TkProgram,
    optimiser: BadgerOptimiser,
    max_threads: int | None = None,
    timeout: int | None = None,
    progress_timeout: int | None = None,
    max_circuit_count: int | None = None,
    log_dir: Path | None = None,
) -> None:
    """Optimise a circuit using the Badger optimiser.

    HyperTKET's best attempt at optimising a circuit using circuit rewriting
    and the given Badger optimiser.

    The input circuit is expected to be in the Nam gate set, i.e. CX + Rz + H.

    Mutates the circuit in place.

    Will use at most `max_threads` threads (plus a constant). Defaults to the
    number of CPUs available.

    The optimisation will terminate at the first of the following timeout
    criteria, if set:
    - `timeout` seconds (default: 15min) have elapsed since the start of the
      optimisation
    - `progress_timeout` (default: None) seconds have elapsed since progress
      in the cost function was last made
    - `max_circuit_count` (default: None) circuits have been explored.

    Log files will be written to the directory `log_dir` if specified.
    """
    _badger_optimise(
        program._inner,
        optimiser=optimiser,
        max_threads=max_threads,
        timeout=timeout,
        progress_timeout=progress_timeout,
        max_circuit_count=max_circuit_count,
        log_dir=log_dir,
    )


def chunks(program: TkProgram, max_chunk_size: int) -> CircuitChunks:
    """Split a circuit into chunks of at most `max_chunk_size` gates."""
    return _chunks(program._inner, max_chunk_size)
