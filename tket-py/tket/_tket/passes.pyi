from pathlib import Path
from typing import TypeVar

from .optimiser import BadgerOptimiser
from .circuit import Tk2Circuit
from pytket._tket.circuit import Circuit

CircuitClass = TypeVar("CircuitClass", Circuit, Tk2Circuit)

class CircuitChunks:
    def reassemble(self) -> Circuit | Tk2Circuit:
        """Reassemble the circuit from its chunks."""

    def circuits(self) -> list[Circuit | Tk2Circuit]:
        """Returns clones of the split circuits."""

    def update_circuit(self, index: int, circ: Circuit | Tk2Circuit) -> None:
        """Replace a circuit chunk with a new version."""

class PullForwardError(Exception):
    """Error from a `PullForward` operation."""

def normalize_guppy(
    circ: CircuitClass,
    *,
    simplify_cfgs: bool = True,
    remove_tuple_untuple: bool = True,
    constant_folding: bool = True,
    remove_dead_funcs: bool = True,
    inline_dfgs: bool = True,
    remove_redundant_order_edges: bool = True,
) -> CircuitClass:
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

def greedy_depth_reduce(circ: CircuitClass) -> tuple[CircuitClass, int]:
    """Greedy depth reduction of a circuit.

    Returns the reduced circuit and the depth reduction.
    """

def lower_to_pytket(circ: CircuitClass) -> CircuitClass:
    """Lower the high-level operations in a Hugr so it can be interpreted by pytket."""

def badger_optimise(
    circ: CircuitClass,
    optimiser: BadgerOptimiser,
    max_threads: int | None = None,
    timeout: int | None = None,
    progress_timeout: int | None = None,
    max_circuit_count: int | None = None,
    log_dir: Path | None = None,
    rebase: bool | None = False,
) -> CircuitClass:
    """Optimise a circuit using the Badger optimiser.

    HyperTKET's best attempt at optimising a circuit using circuit rewriting
    and the given Badger optimiser.

    By default, the input circuit will be rebased to Nam, i.e. CX + Rz + H before
    optimising. This can be deactivated by setting `rebase` to `false`, in which
    case the circuit is expected to be in the Nam gate set.

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

def chunks(c: Circuit | Tk2Circuit, max_chunk_size: int) -> CircuitChunks:
    """Split a circuit into chunks of at most `max_chunk_size` gates."""

def tket1_pass(
    circ: CircuitClass,
    pass_json: str,
    *,
    traverse_subcircuits: bool = True,
) -> CircuitClass:
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
