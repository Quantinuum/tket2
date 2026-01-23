from typing import Any, Callable
from pytket._tket.circuit import Circuit as Tk1Circuit

from tket._tket.ops import TketOp
from hugr.envelope import EnvelopeConfig

try:
    from warnings import deprecated  # type: ignore[attr-defined]
except ImportError:
    # Python < 3.13
    def deprecated(_msg):  # type: ignore[no-redef]
        def _deprecated(func):
            return func

        return _deprecated

class Tk2Circuit:
    """Rust representation of a TKET circuit."""

    def __init__(self, circ: Tk1Circuit) -> None:
        """Create a Tk2Circuit from a pytket Circuit."""

    def __hash__(self) -> int:
        """Compute the circuit hash by traversal."""

    def __copy__(self) -> Tk2Circuit:
        """Create a copy of the circuit."""

    def __deepcopy__(self) -> Tk2Circuit:
        """Create a deep copy of the circuit."""

    def hash(self) -> int:
        """Compute the circuit hash by traversal."""

    def circuit_cost(self, cost_fn: Callable[[TketOp], Any]) -> int:
        """Compute the cost of the circuit. Return value must implement __add__."""

    def num_operations(self) -> int:
        """The number of operations in the circuit.

        This includes [`TketOp`]s, pytket ops, and any other custom operations.

        Nested circuits are traversed to count their operations.
        """

    def node_op(self, node: Node) -> bytes:
        """If the node corresponds to a custom op, return it. Otherwise, raise an error."""

    def to_tket1(self) -> Tk1Circuit:
        """Convert to pytket Circuit."""

    def apply_rewrite(self, rw) -> None:
        """Apply a rewrite to the circuit."""

    def node_inputs(self, node: Node) -> list[Wire]:
        """The incoming wires to a node."""

    def node_outputs(self, node: Node) -> list[Wire]:
        """The outgoing wires from a node."""

    def input_node(self) -> Node:
        """The input node of the circuit."""

    def output_node(self) -> Node:
        """The output node of the circuit."""

    def to_hugr_json(self) -> str:
        """Encode the circuit as a HUGR json."""

    @staticmethod
    def from_hugr_json(json: str) -> Tk2Circuit:
        """Decode a HUGR json string to a Tk2Circuit."""

    def to_bytes(self, config: EnvelopeConfig | None = None) -> bytes:
        """Encode the circuit as a HUGR envelope, according to the given config.

        Some envelope formats can be encoded into a string. See :meth:`to_str`.

        Args:
            config: The envelope configuration to use.
        """

    def to_str(self, config: EnvelopeConfig | None = None) -> str:
        """Encode the circuit as a HUGR envelope string.

        Not all envelope formats can be encoded into a string.
        See :meth:`to_bytes` for a more general method.

        Args:
            config: The envelope configuration to use.
                If not given, uses the default textual encoding.
        """

    @staticmethod
    def from_bytes(envelope: bytes, function_name: str | None = None) -> Tk2Circuit:
        """Load a Circuit from a HUGR envelope.

        Some envelope formats can be read from a string. See :meth:`from_str`.

        Args:
            envelope: The byte string representing a Package.
            function_name: The name of the function in the envelope's module to load.
                Defaults to `main`.

        Returns:
            The loaded circuit.
        """

    @staticmethod
    def from_str(envelope: str, function_name: str | None = None) -> Tk2Circuit:
        """Load a Circuit from a HUGR envelope string.

        Not all envelope formats can be represented as strings.
        See :meth:`from_bytes` for a more general method.

        Args:
            envelope: The string representing a Package.
            function_name: The name of the function in the envelope's module to load.
                Defaults to `main`.

        Returns:
            The loaded circuit.
        """

    @deprecated("Use HUGR envelopes instead. See the `to_bytes` and `to_str` methods.")
    def to_package_json(self) -> str:
        """Encode the circuit as a HUGR Package json."""

    @staticmethod
    @deprecated(
        "Use HUGR envelopes instead. See the `from_bytes` and `from_str` methods."
    )
    def from_package_json(json: str, function_name: str | None = None) -> Tk2Circuit:
        """Decode a HUGR Package json to a circuit.

        Traverses the package's modules in order until it finds one containing a
        function named `function_name`, and loads it as a circuit.

        If the json is a hugr json, it will be decoded as a `main` function in an empty module.

        When `function_name` is not given, it defaults to `main`.
        """

    def to_tket1_json(
        self,
    ) -> str:
        """Encode the circuit as a pytket json string."""

    @staticmethod
    def from_tket1_json(json: str) -> Tk2Circuit:
        """Decode a pytket json string to a Tk2Circuit."""

    def render_mermaid(self) -> str:
        """Render the circuit as a Mermaid graph."""

class Node:
    """Handle to node in HUGR."""

    def __init__(self, idx: int) -> None:
        """Create a new node handle."""

class Wire:
    """An outgoing edge from a node in a HUGR, defined by the node and outgoing port."""

    def node(self) -> Node:
        """Source node of wire."""

    def port(self) -> int:
        """Source port of wire."""

class CircuitCost:
    """A cost function for circuits."""

    def __init__(self, cost: Any) -> None:
        """Create a new circuit cost.

        The cost object must implement __add__, __sub__, __eq__, and __lt__."""

def render_circuit_dot(hugr: Tk2Circuit | Tk1Circuit) -> str: ...
def render_circuit_mermaid(hugr: Tk2Circuit | Tk1Circuit) -> str: ...
def validate_circuit(hugr: Tk2Circuit | Tk1Circuit) -> None: ...
def embedded_extensions() -> list[str]:
    """Returns a list of extension ids supported by the Tk2Circuit loader.

    Extensions not in this list must be included in the package when converting
    loading a Tk2Circuit.
    """

class HugrError(Exception): ...
class BuildError(Exception): ...
class ValidationError(Exception): ...
class HUGRSerializationError(Exception): ...
class TK1EncodeError(Exception): ...
