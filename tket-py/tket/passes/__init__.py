from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import TYPE_CHECKING

from hugr import Hugr

from tket import _state
from . import inline_funcs
from .._tket import passes as _passes, optimiser as _optimiser

from hugr.passes.composable import (
    ComposablePass,
    ComposedPass,
    implement_pass_run,
    PassResult,
)
from hugr.passes.scope import PassScope, GlobalScope

if TYPE_CHECKING:
    from tket.util import PytketPassProto as PytketPass


__all__ = [
    "PytketHugrPass",
    "PassResult",
    "InlineFuncsHeuristic",
    "InlineFunctions",
    "NormalizeGuppy",
    "ReplaceNonCliffordWithClifford",
    "ModifierResolverPass",
    "QSystemPass",
]


@dataclass
class PytketHugrPass(ComposablePass):
    pytket_passes: list[PytketPass]
    _scope: PassScope = GlobalScope.PRESERVE_PUBLIC

    """
    A class which provides an interface to apply pytket passes to Hugr programs.

    The user can create a :py:class:`PytketHugrPass` object from any serializable member of `pytket.passes`.
    """

    def __init__(self, *pytket_passes: PytketPass) -> None:
        """Initialize a PytketHugrPass from a :py:class:`~pytket.passes.BasePass` instance."""
        self.pytket_passes = list(pytket_passes)

    def with_scope(self, scope: PassScope) -> PytketHugrPass:
        """Set the scope configuration for the composed pass."""
        self._scope = scope
        return self

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
            return PytketHugrPass(*self.pytket_passes, *other.pytket_passes).with_scope(
                self._scope
            )
        else:
            return ComposedPass(self, other)

    def _run_pytket_pass_on_hugr(self, hugr: Hugr, inplace: bool) -> PassResult:
        tk_program = _state.CompilationState.from_python(hugr)
        for py_pass in self.pytket_passes:
            pass_json = json.dumps(py_pass.to_dict())
            _passes.tket1_pass(tk_program._inner, pass_json, scope=self._scope)

        package = tk_program.to_python()
        new_hugr = package.modules[0]
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
    _scope: PassScope = GlobalScope.PRESERVE_PUBLIC

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

    def with_scope(self, _scope: PassScope) -> NormalizeGuppy:
        """Set the scope of this pass and return self."""
        self._scope = _scope
        return self

    def _normalize(self, hugr: Hugr, inplace: bool) -> PassResult:
        tk_program = _state.CompilationState.from_python(hugr)

        self._run_tk(tk_program)

        package = tk_program.to_python()
        return PassResult.for_pass(
            self, hugr=package.modules[0], inplace=inplace, result=None
        )

    def _run_tk(self, program: _state.CompilationState) -> _state.CompilationState:
        """Run the pass in the CompilationState

        TODO: This should be part of a protocol."""
        _passes.normalize_guppy(
            program._inner,
            simplify_cfgs=self.simplify_cfgs,
            remove_tuple_untuple=self.remove_tuple_untuple,
            constant_folding=self.constant_folding,
            remove_dead_funcs=self.remove_dead_funcs,
            inline_dfgs=self.inline_dfgs,
            remove_redundant_order_edges=self.remove_redundant_order_edges,
            squash_borrows=self.squash_borrows,
            scope=self._scope,
        )
        return program


@dataclass
class InlineFunctions(ComposablePass):
    """Inline acyclic function calls below the selected scope.

    Parameters:
    - heuristic: Heuristic used to choose which non-recursive functions to
      inline. Defaults to `MaxSize(64)`.
    """

    heuristic: inline_funcs.InlineFuncsHeuristic = inline_funcs.MaxSize(64)
    _scope: PassScope = GlobalScope.PRESERVE_PUBLIC

    def run(self, hugr: Hugr, *, inplace: bool = True) -> PassResult:
        return implement_pass_run(
            self,
            hugr=hugr,
            inplace=inplace,
            copy_call=lambda h: self._inline_functions(h, inplace),
        )

    def with_scope(self, _scope: PassScope) -> InlineFunctions:
        """Set the scope of this pass and return self."""
        self._scope = _scope
        return self

    def _inline_functions(self, hugr: Hugr, inplace: bool) -> PassResult:
        tk_program = _state.CompilationState.from_python(hugr)

        _passes.inline_functions(
            tk_program._inner,
            heuristic=self.heuristic,
            scope=self._scope,
        )

        package = tk_program.to_python()
        return PassResult.for_pass(
            self, hugr=package.modules[0], inplace=inplace, result=None
        )


def _greedy_depth_reduce(program: _state.CompilationState) -> int:
    return _passes.greedy_depth_reduce(program._inner)


def _badger_optimise(
    program: _state.CompilationState,
    optimiser: _optimiser.BadgerOptimiser | Path | None = None,
    *,
    max_threads: int | None = None,
    timeout: int | None = None,
    progress_timeout: int | None = None,
    max_circuit_count: int | None = None,
    log_dir: Path | None = None,
) -> None:
    """Optimise a circuit using the Badger optimiser.

    HyperTKET's best attempt at optimising a circuit using circuit rewriting.


    If `optimiser` is a path, it should point to a file containing a Badger ECC
    set. If `optimiser` is None, the default ECC set will be used. Otherwise, the
    provided BadgerOptimiser instance will be used.

    The input circuit is expected to be in the Nam gate set, i.e. CX + Rz + H.

    Mutates the circuit in place.

    Will use at most `max_threads` threads (plus a constant). Defaults to the
    number of CPUs available.

    The optimisation will terminate at the first of the following timeout
    criteria, if set: - `timeout` seconds (default: 15min) have elapsed since
    the start of the
      optimisation
    - `progress_timeout` (default: None) seconds have elapsed since progress in
      the cost function was last made
    - `max_circuit_count` (default: None) circuits have been explored.

    Log files will be written to the directory `log_dir` if specified.
    """
    badger_optimiser: _optimiser.BadgerOptimiser
    if optimiser is None:
        try:
            import tket_eccs
        except ImportError:
            raise ValueError(
                "The default rewriter is not available. Please specify a path to a rewriter or install tket-eccs."
            )

        ecc = tket_eccs.nam_6_3()
        badger_optimiser = _optimiser.BadgerOptimiser.load_precompiled(ecc)
    elif isinstance(optimiser, Path):
        badger_optimiser = _optimiser.BadgerOptimiser.load_precompiled(optimiser)
    else:
        badger_optimiser = optimiser

    _passes.badger_optimise(
        program._inner,
        optimiser=badger_optimiser,
        max_threads=max_threads,
        timeout=timeout,
        progress_timeout=progress_timeout,
        max_circuit_count=max_circuit_count,
        log_dir=log_dir,
    )


_CLIFFORD_OP_NAMES = {
    "H",
    "S",
    "Sdg",
    "X",
    "Y",
    "Z",
    "V",
    "Vdg",
    "CX",
    "CY",
    "CZ",
    "SWAP",
}

_NON_UNITARY_OP_NAMES = {
    "Barrier",
    "Measure",
    "MeasureFree",
    "noop",
    "Reset",
}


def _op_name(op: object) -> str:
    return getattr(getattr(op, "type", op), "name", str(getattr(op, "type", op)))


def _empty_like_pytket_circuit(circuit):
    from pytket import Circuit

    replacement = Circuit()
    for qubit in circuit.qubits:
        replacement.add_qubit(qubit)
    for bit in circuit.bits:
        replacement.add_bit(bit)
    return replacement


def _add_clifford_placeholder(circuit, qubits, *, one_qubit: str, two_qubit: str) -> None:
    from pytket import OpType

    if len(qubits) == 1:
        circuit.add_gate(getattr(OpType, one_qubit), qubits)
        return

    if len(qubits) == 2:
        circuit.add_gate(getattr(OpType, two_qubit), qubits)
        return

    # For wider non-Clifford operations, insert a deterministic Clifford scaffold
    # over the same qubits instead of leaving an unsupported operation behind.
    for target in qubits[1:]:
        circuit.add_gate(getattr(OpType, two_qubit), [qubits[0], target])
    for target in reversed(qubits[1:]):
        circuit.add_gate(getattr(OpType, two_qubit), [qubits[0], target])


def _replace_non_clifford_ops(circuit, *, one_qubit: str, two_qubit: str):
    replacement = _empty_like_pytket_circuit(circuit)
    qubit_set = set(circuit.qubits)

    for command in circuit.get_commands():
        name = _op_name(command.op)
        command_qubits = [arg for arg in command.args if arg in qubit_set]

        if not command_qubits or name in _CLIFFORD_OP_NAMES or name in _NON_UNITARY_OP_NAMES:
            replacement.add_gate(command.op, command.args)
            continue

        _add_clifford_placeholder(
            replacement,
            command_qubits,
            one_qubit=one_qubit,
            two_qubit=two_qubit,
        )

    return replacement


@dataclass
class ReplaceNonCliffordWithClifford(ComposablePass):
    """Replace non-Clifford quantum operations with Clifford stand-ins.

    This is intended for generating Clifford-only HUGRs that can be sent through
    Clifford simulators such as Stim for testing and debugging.
    """

    one_qubit_replacement: str = "S"
    two_qubit_replacement: str = "CX"
    _scope: PassScope = GlobalScope.PRESERVE_PUBLIC

    def run(self, hugr: Hugr, *, inplace: bool = True) -> PassResult:
        return implement_pass_run(
            self,
            hugr=hugr,
            inplace=inplace,
            copy_call=lambda h: self._replace(h, inplace),
        )

    def with_scope(self, scope: PassScope) -> ReplaceNonCliffordWithClifford:
        self._scope = scope
        return self

    def _replace(self, hugr: Hugr, inplace: bool) -> PassResult:
        tk_program = _state.CompilationState.from_python(hugr)
        self._run_tk(tk_program)
        package = tk_program.to_python()
        return PassResult.for_pass(
            self, hugr=package.modules[0], inplace=inplace, result=None
        )

    def _run_tk(self, program: _state.CompilationState) -> _state.CompilationState:
        circuit = program.to_tket1()
        replacement = _replace_non_clifford_ops(
            circuit,
            one_qubit=self.one_qubit_replacement,
            two_qubit=self.two_qubit_replacement,
        )
        program._inner = _state.CompilationState.from_tket1(replacement)
        return program


@dataclass
class ModifierResolverPass(ComposablePass):
    """A pass to resolve Guppy modifiers (control, dagger, power).

    Original function nodes replaced by solved modified versions may be removed
    when no longer needed and allowed by the pass scope. Nodes whose interface
    is preserved by the scope are kept.
    """

    _scope: PassScope = GlobalScope.PRESERVE_PUBLIC

    def run(self, hugr: Hugr, *, inplace: bool = True) -> PassResult:
        return implement_pass_run(
            self,
            hugr=hugr,
            inplace=inplace,
            copy_call=lambda h: self._resolve(h, inplace),
        )

    def with_scope(self, scope: PassScope) -> ModifierResolverPass:
        """Set the scope of this pass and return self."""
        self._scope = scope
        return self

    def _resolve(self, hugr: Hugr, inplace: bool) -> PassResult:
        tk_program = _state.CompilationState.from_python(hugr)

        self._run_tk(tk_program)

        package = tk_program.to_python()
        return PassResult.for_pass(
            self, hugr=package.modules[0], inplace=inplace, result=None
        )

    def _run_tk(self, program: _state.CompilationState) -> _state.CompilationState:
        """Run the pass in the CompilationState"""
        _passes.resolve_modifiers(
            program._inner,
            scope=self._scope,
        )
        return program


@dataclass(kw_only=True)
class QSystemPass(ComposablePass):
    """A pass to convert quantum ops to qsystem ops.

     Parameters:
    - constant_fold: Whether to perform constant folding.
    - monomorphize: Whether to monomorphize generic functions.
    - force_order: Whether to enforce total ordering of all HUGR operations.
    - hide_funcs: Whether to mark all functions as private.
    """

    constant_fold: bool = True
    monomorphize: bool = True
    force_order: bool = True
    hide_funcs: bool = True
    _scope: PassScope = GlobalScope.PRESERVE_PUBLIC

    def run(self, hugr: Hugr, *, inplace: bool = True) -> PassResult:
        return implement_pass_run(
            self,
            hugr=hugr,
            inplace=inplace,
            copy_call=lambda h: self._qsystem_rebase(h, inplace),
        )

    def with_scope(self, scope: PassScope) -> QSystemPass:
        """Set the scope of this pass and return self."""
        self._scope = scope
        return self

    def _qsystem_rebase(self, hugr: Hugr, inplace: bool) -> PassResult:
        tk_program = _state.CompilationState.from_python(hugr)

        self._run_tk(tk_program)

        package = tk_program.to_python()
        return PassResult.for_pass(
            self, hugr=package.modules[0], inplace=inplace, result=None
        )

    def _run_tk(self, program: _state.CompilationState) -> _state.CompilationState:
        """Run the pass in the CompilationState"""
        _passes.qsystem_rebase_pass(
            program._inner,
            constant_fold=self.constant_fold,
            monomorphize=self.monomorphize,
            force_order=self.force_order,
            hide_funcs=self.hide_funcs,
            scope=self._scope,
        )
        return program
