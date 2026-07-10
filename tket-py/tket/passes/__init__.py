from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from functools import cache
import json
from pathlib import Path
from typing import TYPE_CHECKING
from typing_extensions import deprecated

from hugr import Hugr

from tket import _state
from . import inline_funcs
from .._pattern import Rule, RuleMatcher
from .._state.build import OneQbGate, from_coms
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
    "PlatformTarget",
    "PassResult",
    "InlineFuncsHeuristic",
    "InlineFunctions",
    "Normalize",
    "NormalizeGuppy",
    "ModifierResolverPass",
    "QSystemRebasePass",
    "_QSystemLLVMPass",
    "Cliffordize",
]


class PlatformTarget(Enum):
    """A hardware platform that passes can target.

    Passes use this to decide which platform-specific gate sets and extensions
    to produce or accept. See the individual passes (e.g.
    :py:class:`PytketHugrPass`) for the behaviour associated with each target.
    """

    Tket = "tket"  # Platform-agnostic target using only the base `tket` extensions.
    Sol = "sol"  # Quantinuum Sol platform (base tket + Sol operations).
    Helios = "helios"  # Quantinuum Helios platform (base tket + Helios operations).


@dataclass
class PytketHugrPass(ComposablePass):
    pytket_passes: list[PytketPass]
    target: PlatformTarget = PlatformTarget.Tket
    _scope: PassScope = GlobalScope.PRESERVE_PUBLIC

    """
    A class which provides an interface to apply pytket passes to Hugr programs.

    The user can create a :py:class:`PytketHugrPass` object from any
    serializable member of `pytket.passes`.

    The ``target`` selects which set of encoder/decoder extensions is used when
    translating between HUGRs and pytket circuits, controlling which operations
    get encoded as pytket commands and which pytket commands get decoded back
    into HUGR operations:

    - :py:attr:`PlatformTarget.Tket` (default):
        Only base ``tket`` operations are encoded.
        When decoding, pytket commands are translated into base ``tket.quantum``
        operations, falling back to Helios qsystem operations for
        commands without a base counterpart (e.g. ``ZZPhase``).
    - :py:attr:`PlatformTarget.Sol`:
        Base ``tket`` and native Sol operations are encoded.
        When decoding, commands are translated into native Sol qsystem operations
        where possible, falling back to base ``tket.quantum`` operations.
    - :py:attr:`PlatformTarget.Helios`:
        Base ``tket`` and native Helios operations are encoded.
        When decoding, commands are translated into native Helios qsystem operations
        where possible, falling back to base ``tket.quantum`` operations.

    Operations without a valid encoder are kept as-is on a pytket roundtrip.
    Pytket commands without a valid decoder produce an unsupported
    ``TKET1.tk1op`` operation in the decoded HUGR.

    Parameters:
    - pytket_passes: The pytket passes to run.
    - target: The platform target selecting which encoder/decoder extension set
      to use when translating between HUGRs and pytket circuits. Defaults to the
      platform-agnostic :py:attr:`PlatformTarget.Tket`.
    """

    def __init__(
        self,
        *pytket_passes: PytketPass,
        target: PlatformTarget = PlatformTarget.Tket,
    ) -> None:
        """Initialize a PytketHugrPass from a :py:class:`~pytket.passes.BasePass` instance."""
        self.pytket_passes = list(pytket_passes)
        self.target = target

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
        if isinstance(other, PytketHugrPass) and self.target == other.target:
            combined = PytketHugrPass(
                *self.pytket_passes, *other.pytket_passes, target=self.target
            )
            return combined.with_scope(self._scope)
        else:
            return ComposedPass(self, other)

    def _run_pytket_pass_on_hugr(self, hugr: Hugr, inplace: bool) -> PassResult:
        tk_program = _state.CompilationState.from_python(hugr)
        for py_pass in self.pytket_passes:
            pass_json = json.dumps(py_pass.to_dict())
            _passes.tket1_pass(
                tk_program._inner,
                pass_json,
                scope=self._scope,
                target=self.target.value,
            )

        package = tk_program.to_python()
        new_hugr = package.modules[0]
        return PassResult.for_pass(self, hugr=new_hugr, inplace=inplace, result=None)


@dataclass
class Normalize(ComposablePass):
    resolve_modifiers: bool = True
    simplify_cfgs: bool = True
    remove_tuple_untuple: bool = True
    constant_folding: bool = True
    remove_dead_funcs: bool = True
    inline_funcs: inline_funcs.InlineFuncsHeuristic | bool = True
    inline_dfgs: bool = True
    remove_redundant_order_edges: bool = True
    squash_borrows: bool = True
    _scope: PassScope = GlobalScope.PRESERVE_PUBLIC

    """Flatten the structure of a program to enable additional optimisations.

    This should normally be called first before other optimisations.

    Parameters:
    - resolve_modifiers: Whether to resolve modifier operations.
    - simplify_cfgs: Whether to simplify CFG control flow.
    - remove_tuple_untuple: Whether to remove tuple/untuple operations.
    - constant_folding: Whether to constant fold the program.
    - remove_dead_funcs: Whether to remove dead functions.
    - inline_dfgs: Whether to inline DFG operations.
    - inline_funcs: Heuristic for inlining function calls, or True for default heuristic,
      or False to disable inlining.
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

    def with_scope(self, _scope: PassScope) -> Normalize:
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
        inline_funcs_heuristic: inline_funcs.InlineFuncsHeuristic | None
        match self.inline_funcs:
            case True:
                inline_funcs_heuristic = inline_funcs.MaxSize(128)
            case False:
                inline_funcs_heuristic = None
            case _:
                inline_funcs_heuristic = self.inline_funcs
        _passes.normalize_guppy(
            program._inner,
            resolve_modifiers=self.resolve_modifiers,
            simplify_cfgs=self.simplify_cfgs,
            remove_tuple_untuple=self.remove_tuple_untuple,
            constant_folding=self.constant_folding,
            remove_dead_funcs=self.remove_dead_funcs,
            inline_dfgs=self.inline_dfgs,
            inline_funcs=inline_funcs_heuristic,
            remove_redundant_order_edges=self.remove_redundant_order_edges,
            squash_borrows=self.squash_borrows,
            scope=self._scope,
        )
        return program


@deprecated("Use `Normalize` instead.")
class NormalizeGuppy(Normalize):
    """Deprecated alias for :py:class:`Normalize`."""


@cache
def _cliffordize_matcher() -> RuleMatcher:
    """Build the matcher containing the supported Cliffordize rules."""
    replacements = [
        ("T", "S"),
        ("Tdg", "Sdg"),
    ]

    rules = [
        Rule(
            from_coms(OneQbGate(source)(0))._inner,
            from_coms(OneQbGate(replacement)(0))._inner,
        )
        for source, replacement in replacements
    ]

    return RuleMatcher(rules)


@dataclass
class Cliffordize(ComposablePass):
    """Replace supported non-Clifford operations with Clifford operations.

    This pass is intended for debugging and workflows that require Clifford-only
    circuits. It is not semantics-preserving.

    The currently supported replacements are:

    - `T` with `S`
    - `Tdg` with `Sdg`

    Other non-Clifford operations, including arbitrary rotations, symbolic
    rotations, `PhasedX`, and `ZZPhase`, are left unchanged.
    """

    _scope: PassScope = GlobalScope.PRESERVE_PUBLIC

    def run(self, hugr: Hugr, *, inplace: bool = True) -> PassResult:
        """Run the pass and return the transformed HUGR and rewrite count."""
        return implement_pass_run(
            self,
            hugr=hugr,
            inplace=inplace,
            copy_call=lambda h: self._cliffordize(h, inplace),
        )

    def with_scope(self, scope: PassScope) -> Cliffordize:
        """Set the scope of this pass and return self."""
        self._scope = scope
        return self

    def _cliffordize(self, hugr: Hugr, inplace: bool) -> PassResult:
        tk_program = _state.CompilationState.from_python(hugr)

        rewrite_count = self._run_tk(tk_program)

        package = tk_program.to_python()
        return PassResult.for_pass(
            self,
            hugr=package.modules[0],
            inplace=inplace,
            result=rewrite_count,
        )

    def _run_tk(self, program: _state.CompilationState) -> int:
        """Run the pass on a CompilationState and return the rewrite count."""
        return _cliffordize_matcher().apply_exhaustive(
            program._inner,
            scope=self._scope,
        )


@dataclass
class InlineFunctions(ComposablePass):
    """Inline acyclic function calls below the selected scope.

    Parameters:
    - heuristic: Heuristic used to choose which non-recursive functions to
      inline. Defaults to `MaxSize(128)`.
    """

    heuristic: inline_funcs.InlineFuncsHeuristic = inline_funcs.MaxSize(128)
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
class QSystemRebasePass(ComposablePass):
    """Convert quantum operations to QSystem operations.

    Parameters:
    - resolve_modifiers: Whether to resolve Guppy modifiers.
    - lower_drops: Whether to lower qubit drops to QSystem operations.
    - hide_funcs: Whether to mark generated helper functions as private.
    """

    resolve_modifiers: bool = True
    lower_drops: bool = True
    hide_funcs: bool = True
    _scope: PassScope = GlobalScope.PRESERVE_PUBLIC

    def run(self, hugr: Hugr, *, inplace: bool = True) -> PassResult:
        return implement_pass_run(
            self,
            hugr=hugr,
            inplace=inplace,
            copy_call=lambda h: self._qsystem_rebase(h, inplace),
        )

    def with_scope(self, scope: PassScope) -> QSystemRebasePass:
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
            resolve_modifiers=self.resolve_modifiers,
            lower_drops=self.lower_drops,
            hide_funcs=self.hide_funcs,
            scope=self._scope,
        )
        return program


@dataclass(kw_only=True)
class _QSystemLLVMPass(ComposablePass):
    """Prepare a QSystem program for LLVM lowering.

    This is normally called automatically by the tools before LLVM lowering.

    Parameters:
    - constant_fold: Whether to perform constant folding.
    - monomorphize: Whether to monomorphize generic functions.
    - force_order: Whether to enforce total ordering of all HUGR operations.
    """

    constant_fold: bool = True
    monomorphize: bool = True
    force_order: bool = True
    _scope: PassScope = GlobalScope.PRESERVE_PUBLIC

    def run(self, hugr: Hugr, *, inplace: bool = True) -> PassResult:
        return implement_pass_run(
            self,
            hugr=hugr,
            inplace=inplace,
            copy_call=lambda h: self._qsystem_llvm(h, inplace),
        )

    def with_scope(self, scope: PassScope) -> _QSystemLLVMPass:
        """Set the scope of this pass and return self."""
        self._scope = scope
        return self

    def _qsystem_llvm(self, hugr: Hugr, inplace: bool) -> PassResult:
        tk_program = _state.CompilationState.from_python(hugr)

        self._run_tk(tk_program)

        package = tk_program.to_python()
        return PassResult.for_pass(
            self, hugr=package.modules[0], inplace=inplace, result=None
        )

    def _run_tk(self, program: _state.CompilationState) -> _state.CompilationState:
        """Run the pass in the CompilationState"""
        _passes.qsystem_llvm_pass(
            program._inner,
            constant_fold=self.constant_fold,
            monomorphize=self.monomorphize,
            force_order=self.force_order,
            scope=self._scope,
        )
        return program
