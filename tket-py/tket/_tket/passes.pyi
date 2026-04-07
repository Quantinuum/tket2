from pathlib import Path

from .optimiser import BadgerOptimiser
from .state import CompilationState
from hugr.passes.scope import PassScope, GlobalScope

class CircuitChunks:
    def reassemble(self) -> CompilationState:
        """Reassemble the circuit from its chunks."""

    def circuits(self) -> list[CompilationState]:
        """Returns clones of the split circuits."""

    def update_circuit(self, index: int, circ: CompilationState) -> None:
        """Replace a circuit chunk with a new version."""

class PullForwardError(Exception):
    """Error from a `PullForward` operation."""

def normalize_guppy(
    circ: CompilationState,
    *,
    simplify_cfgs: bool = True,
    remove_tuple_untuple: bool = True,
    constant_folding: bool = True,
    remove_dead_funcs: bool = True,
    inline_dfgs: bool = True,
    remove_redundant_order_edges: bool = True,
    squash_borrows: bool = True,
    scope: PassScope = GlobalScope.PRESERVE_PUBLIC,
) -> None:
    """Flatten the structure of a Guppy-generated program to enable additional optimisations.

    This should normally be called first before other optimisations.

    Parameters:
    - simplify_cfgs: Whether to simplify CFG control flow.
    - remove_tuple_untuple: Whether to remove tuple/untuple operations.
    - constant_folding: Whether to constant fold the program.
    - remove_dead_funcs: Whether to remove dead functions.
    - inline_dfgs: Whether to inline DFG operations.
    - remove_redundant_order_edges: Whether to remove redundant order edges.
    """

def greedy_depth_reduce(circ: CompilationState) -> int:
    """Greedy depth reduction of a circuit.

    Mutates the circuit in place and returns the number of moves made.
    """

def badger_optimise(
    circ: CompilationState,
    optimiser: BadgerOptimiser,
    max_threads: int | None = None,
    timeout: int | None = None,
    progress_timeout: int | None = None,
    max_circuit_count: int | None = None,
    log_dir: Path | None = None,
) -> None:
    """Optimise a circuit using the Badger optimiser."""

def chunks(c: CompilationState, max_chunk_size: int) -> CircuitChunks:
    """Split a circuit into chunks of at most `max_chunk_size` gates."""

def tket1_pass(
    circ: CompilationState,
    pass_json: str,
    *,
    scope: PassScope | None = None,
) -> None:
    """Runs a pytket pass on all circuit-like regions under the entrypoint of the
    HUGR.

    Parameters:
    - pass_json: The JSON string of the pytket pass to run. See [pytket
      documentation](https://docs.quantinuum.com/tket/api-docs/passes.html#pytket.passes.BasePass.to_dict)
      for more details.
    - traverse_subcircuits: Whether to recurse into the children of the
      circuit-like regions, and optimise them too.
      nested inside other subregions of the circuit.
    """

def resolve_modifiers(
    circ: CompilationState, scope: PassScope = GlobalScope.PRESERVE_PUBLIC
) -> None:
    """
    Runs a Rust backed pass to resolve quantum modifiers (control, dagger, power).

    :param circ: The input program as a CompilationState.
    :param scope: A scope to control how the pass is applied to HUGR regions.
    """
