from pathlib import Path
from typing import Optional, Literal
import json
from dataclasses import dataclass

from pytket import Circuit
from pytket.passes import (
    CustomPass,
    BasePass,
)

from tket import optimiser
from tket.circuit import Tk2Circuit

from hugr.passes._composable_pass import (
    ComposablePass,
    implement_pass_run,
    PassResult,
)


from hugr.hugr.base import Hugr

# Re-export native bindings.
from ._tket.passes import (
    CircuitChunks,
    greedy_depth_reduce,
    lower_to_pytket,
    badger_optimise,
    chunks,
    tket1_pass,
    normalize_guppy,
    PullForwardError,
    gridsynth,
)

__all__ = [
    "badger_pass",
    # Bindings.
    # TODO: Wrap these in Python classes.
    "CircuitChunks",
    "greedy_depth_reduce",
    "lower_to_pytket",
    "badger_optimise",
    "chunks",
    # TODO: Remove export, use `NormalizeGuppy` instead
    "normalize_guppy",
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
    rebase: bool = False,
    cost_fn: Literal["cx", "rz"] | None = None,
) -> BasePass:
    """Construct a Badger pass.

    The Badger optimiser requires a pre-compiled rewriter produced by the
    `compile-rewriter <https://github.com/quantinuum/tket2/tree/main/badger-optimiser>`_
    utility. If `rewriter` is not specified, a default one will be used.

    The cost function to minimise can be specified by passing `cost_fn` as `'cx'`
    or `'rz'`. If not specified, the default is `'cx'`.

    The arguments `max_threads`, `timeout`, `progress_timeout`, `max_circuit_count`,
    `log_dir` and `rebase` are optional and will be passed on to the Badger
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

    def apply(circuit: Circuit) -> Circuit:
        """Apply Badger optimisation to the circuit."""
        return badger_optimise(
            circuit,
            optimiser=opt,
            max_threads=max_threads,
            timeout=timeout,
            progress_timeout=progress_timeout,
            max_circuit_count=max_circuit_count,
            log_dir=log_dir,
            rebase=rebase,
        )

    return CustomPass(apply, label="tket.badger_pass")


@dataclass
class PytketHugrPass(ComposablePass):
    pytket_pass: BasePass

    """
    A class which provides an interface to apply pytket passes to Hugr programs.

    The user can create a :py:class:`PytketHugrPass` object from any serializable member of `pytket.passes`.
    """

    def __init__(self, pytket_pass: BasePass) -> None:
        """Initialize a PytketHugrPass from a :py:class:`~pytket.passes.BasePass` instance."""
        self.pytket_pass = pytket_pass

    def run(self, hugr: Hugr, *, inplace: bool = True) -> PassResult:
        """Run the pytket pass as a HUGR transform returning a PassResult."""
        return implement_pass_run(
            self,
            hugr=hugr,
            inplace=inplace,
            copy_call=lambda h: self._run_pytket_pass_on_hugr(h, inplace),
        )

    def _run_pytket_pass_on_hugr(self, hugr: Hugr, inplace: bool) -> PassResult:
        pass_json = json.dumps(self.pytket_pass.to_dict())
        compiler_state: Tk2Circuit = Tk2Circuit.from_bytes(hugr.to_bytes())
        opt_program = tket1_pass(compiler_state, pass_json, traverse_subcircuits=True)
        new_hugr = Hugr.from_str(opt_program.to_str())
        # `for_pass` assumes Modified is true by default
        # TODO: if we can extract better info from tket1 as to what happened, use it.
        # Are there better results  we can use too?
        return PassResult.for_pass(self, hugr=new_hugr, inplace=inplace, result=None)


@dataclass
class NormalizeGuppy(ComposablePass):
    simplify_cfgs: bool = True
    remove_tuple_untuple: bool = True
    constant_folding: bool = True
    remove_dead_funcs: bool = True
    inline_dfgs: bool = True
    remove_redundant_order_edges: bool = True

    """Flatten the structure of a Guppy-generated program to enable additional optimisations.

    This should normally be called first before other optimisations.

    Parameters:
    - simplify_cfgs: Whether to simplify CFG control flow.
    - remove_tuple_untuple: Whether to remove tuple/untuple operations.
    - constant_folding: Whether to constant fold the program.
    - remove_dead_funcs: Whether to remove dead functions.
    - inline_dfgs: Whether to inline DFG operations.
    """

    def run(self, hugr: Hugr, *, inplace: bool = True) -> PassResult:
        return implement_pass_run(
            self,
            hugr=hugr,
            inplace=inplace,
            copy_call=lambda h: self._normalize(h, inplace),
        )

    def _normalize(self, hugr: Hugr, inplace: bool) -> PassResult:
        compiler_state: Tk2Circuit = Tk2Circuit.from_bytes(hugr.to_bytes())
        opt_program = normalize_guppy(
            compiler_state,
            simplify_cfgs=self.simplify_cfgs,
            remove_tuple_untuple=self.remove_tuple_untuple,
            constant_folding=self.constant_folding,
            remove_dead_funcs=self.remove_dead_funcs,
            inline_dfgs=self.inline_dfgs,
            remove_redundant_order_edges=self.remove_redundant_order_edges,
        )
        new_hugr = Hugr.from_str(opt_program.to_str())
        return PassResult.for_pass(self, hugr=new_hugr, inplace=inplace, result=None)


@dataclass
class Gridsynth(ComposablePass):
    epsilon: float
    simplify: bool = True

    """Apply the gridsynth algorithm to all Rz gates in the Hugr.

    This includes a NormalizeGuppy pass with all of the constituent passes applied
    before applying gridsynth to standardise the
    format.

    Parameters:
    - epsilon: The allowable error tolerance.
    - simplify: If `True`, each sequence of gridsynth gates is compressed into
      a sequence of H*T and H*Tdg gates, sandwiched by Clifford gates. This sequence
      always has a smaller number of S and H gates, and the same number of T+Tdg gates.
    """

    # TO DO: make the NormalizeGuppy pass optional, in case it is already run
    # before Gridsynth. Need to warn users, at least in docs that if NormalizeGuppy
    # is not run first then Gridsynth is likely to fail. Maybe issue the warning if
    # the option to run NormalizeGuppy is set to False. The option would be specified
    # as a field of the dataclass (would also need to add @dataclass decorator)
    # like for NormalizeGuppy above
    def run(self, hugr: Hugr, *, inplace: bool = True) -> PassResult:
        # inplace option does nothing for now but I retain for consistency of
        # API with other passes
        return implement_pass_run(
            self,
            hugr=hugr,
            inplace=inplace,
            copy_call=lambda h: self._apply_gridsynth_pass(
                hugr, self.epsilon, self.simplify, inplace
            ),
        )

    def _apply_gridsynth_pass(
        self, hugr: Hugr, epsilon: float, simplify: bool, inplace: bool
    ) -> PassResult:
        compiler_state: Tk2Circuit = Tk2Circuit.from_bytes(hugr.to_bytes())
        program = gridsynth(compiler_state, epsilon, simplify)
        new_hugr = Hugr.from_str(program.to_str())
        return PassResult.for_pass(self, hugr=new_hugr, inplace=inplace, result=None)
