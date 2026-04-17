from __future__ import annotations

from pathlib import Path
import json
from dataclasses import dataclass
from enum import Enum

from hugr import Hugr
from pytket.passes import (
    BasePass,
)

from tket import _state
from ._tket import passes as _passes, optimiser as _optimiser

from hugr.passes.composable import (
    ComposablePass,
    ComposedPass,
    implement_pass_run,
    PassResult,
)
from hugr.passes.scope import PassScope, GlobalScope


__all__ = [
    "PytketHugrPass",
    "PassResult",
    "CaseOfCase",
    "InlineFunctions",
    "NormalizeGuppy",
    "SinkConditionalInputs",
    "StructuralizationStrategy",
    "StructuralizeCfgs",
    "ModifierResolverPass",
]


@dataclass
class PytketHugrPass(ComposablePass):
    pytket_passes: list[BasePass]
    _scope: PassScope = GlobalScope.PRESERVE_PUBLIC

    """
    A class which provides an interface to apply pytket passes to Hugr programs.

    The user can create a :py:class:`PytketHugrPass` object from any serializable member of `pytket.passes`.
    """

    def __init__(self, *pytket_passes: BasePass) -> None:
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
class CaseOfCase(ComposablePass):
    """Fuse direct sibling conditional chains with a conservative case-of-case rewrite.

    Parameters:
    - max_duplicated_nodes: Maximum number of consumer-case internal nodes that
      may be duplicated by one rewrite.
    """

    max_duplicated_nodes: int = 32
    _scope: PassScope = GlobalScope.PRESERVE_PUBLIC

    def run(self, hugr: Hugr, *, inplace: bool = True) -> PassResult:
        return implement_pass_run(
            self,
            hugr=hugr,
            inplace=inplace,
            copy_call=lambda h: self._case_of_case(h, inplace),
        )

    def with_scope(self, _scope: PassScope) -> CaseOfCase:
        """Set the scope of this pass and return self."""
        self._scope = _scope
        return self

    def _case_of_case(self, hugr: Hugr, inplace: bool) -> PassResult:
        tk_program = _state.CompilationState.from_python(hugr)

        _passes.case_of_case(
            tk_program._inner,
            max_duplicated_nodes=self.max_duplicated_nodes,
            scope=self._scope,
        )

        package = tk_program.to_python()
        return PassResult.for_pass(
            self, hugr=package.modules[0], inplace=inplace, result=None
        )


@dataclass
class NormalizeGuppy(ComposablePass):
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

    simplify_cfgs: bool = True
    remove_tuple_untuple: bool = True
    constant_folding: bool = True
    remove_dead_funcs: bool = True
    inline_dfgs: bool = True
    remove_redundant_order_edges: bool = True
    squash_borrows: bool = True
    _scope: PassScope = GlobalScope.PRESERVE_PUBLIC

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
    - max_inline_size: Maximum number of descendants allowed in a callee for
      its call sites to be inlined.
    """

    max_inline_size: int = 64
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
            max_inline_size=self.max_inline_size,
            scope=self._scope,
        )

        package = tk_program.to_python()
        return PassResult.for_pass(
            self, hugr=package.modules[0], inplace=inplace, result=None
        )


class StructuralizationStrategy(str, Enum):
    """Configuration for Structuralization strategy to use in the StructuralizeCfgs pass.

    The strategies available are:
    - RVSDG: Use the RVSDG-based structuralization
        strategy. Based in the approach described in "Structural Analysis of
        Reducible and Irreducible Control Flow Graphs" by Banerjee et al.
        <https://arxiv.org/abs/2210.03652>.
    - Relooper: Use the Beyond-Relooper structuralization strategy <https://zenodo.org/records/6727752>

    Both strategies are currently being tested. We may keep one or both of them in the long term
    depending on their performance and the quality of the resulting circuits.
    """

    RVSDG = "rvsdg"
    RELOOPER = "relooper"


@dataclass
class SinkConditionalInputs(ComposablePass):
    """Sink branch-local shared conditional inputs into individual cases.

    Shared `other_inputs` of a `Conditional` that are used by at most one case
    are moved into that case, or removed entirely when dead.
    """

    _scope: PassScope = GlobalScope.PRESERVE_PUBLIC

    def run(self, hugr: Hugr, *, inplace: bool = True) -> PassResult:
        return implement_pass_run(
            self,
            hugr=hugr,
            inplace=inplace,
            copy_call=lambda h: self._sink_inputs(h, inplace),
        )

    def with_scope(self, _scope: PassScope) -> SinkConditionalInputs:
        """Set the scope of this pass and return self."""
        self._scope = _scope
        return self

    def _sink_inputs(self, hugr: Hugr, inplace: bool) -> PassResult:
        tk_program = _state.CompilationState.from_python(hugr)

        _passes.sink_conditional_inputs(
            tk_program._inner,
            scope=self._scope,
        )

        package = tk_program.to_python()
        return PassResult.for_pass(
            self, hugr=package.modules[0], inplace=inplace, result=None
        )


@dataclass
class StructuralizeCfgs(ComposablePass):
    """Structuralize CFG regions into nested `Conditional` and `TailLoop` nodes.

    Parameters:
    - strategy: Structuralization strategy.
    - skip_unstructuralizable_cfgs: Leave CFGs unchanged when they cannot be
      structuralized instead of raising an error.
    """

    strategy: StructuralizationStrategy = StructuralizationStrategy.RVSDG
    skip_unstructuralizable_cfgs: bool = True
    _scope: PassScope = GlobalScope.PRESERVE_PUBLIC

    def run(self, hugr: Hugr, *, inplace: bool = True) -> PassResult:
        return implement_pass_run(
            self,
            hugr=hugr,
            inplace=inplace,
            copy_call=lambda h: self._structuralize(h, inplace),
        )

    def with_scope(self, _scope: PassScope) -> StructuralizeCfgs:
        """Set the scope of this pass and return self."""
        self._scope = _scope
        return self

    def _structuralize(self, hugr: Hugr, inplace: bool) -> PassResult:
        tk_program = _state.CompilationState.from_python(hugr)

        self._run_tk(tk_program)

        package = tk_program.to_python()
        return PassResult.for_pass(
            self, hugr=package.modules[0], inplace=inplace, result=None
        )

    def _run_tk(self, program: _state.CompilationState) -> _state.CompilationState:
        """Run the pass in the CompilationState."""
        _passes.structuralize_cfgs(
            program._inner,
            strategy=self.strategy.value,
            skip_unstructuralizable_cfgs=self.skip_unstructuralizable_cfgs,
            scope=self._scope,
        )
        return program


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
    """A pass to resolve Guppy modifiers (control, dagger, power)."""

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
