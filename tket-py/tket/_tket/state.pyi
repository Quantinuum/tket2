from typing import Any, Callable

from tket._tket.ops import TketOp
from tket.util import PytketCircuitProto as Tk1Circuit
from hugr.envelope import EnvelopeConfig

try:
    from pytket.circuit import Circuit as Tk1Circuit
except ImportError:
    # Pytket is installed as an optional dependency under the `pytket` extra.
    #
    # If it's not available, we use a duck-typed protocol for type hints instead.
    pass

class CompilationState:
    """Program state definition.

    This is a wrapper around a rust-defined HUGR that is optimised for
    compilation and rewriting.
    """

    def __init__(self) -> None:
        """Create a new empty program."""

    @staticmethod
    def from_tket1(circ: Tk1Circuit) -> CompilationState:
        """Load a program from a legacy pytket Circuit."""

    def apply_rewrite(self, rw) -> None:
        """Apply a rewrite on the circuit."""

    def to_bytes(
        self, config: EnvelopeConfig | None = None, *, omit_tket_exts: bool = True
    ) -> bytes:
        """Encode the circuit as a HUGR envelope.

        Some envelope formats can be encoded into a string. See :meth:`to_str`.

        Args:
            config: The envelope configuration to use.
                If not given, uses the default binary encoding.
            omit_tket_exts: If true, the extensions in :meth:`embedded_extensions`
                will not be not be included in the envelope even when they are used in the
                HUGR. This is useful when sending the HUGR to other components that
                already have the tket extensions available.
        """

    def to_str(
        self, config: EnvelopeConfig | None = None, *, omit_tket_exts: bool = True
    ) -> str:
        """Encode the circuit as a HUGR envelope string.

        Not all envelope formats can be encoded into a string.
        See :meth:`to_bytes` for a more general method.

        Args:
            config: The envelope configuration to use.
                If not given, uses the default textual encoding.
            omit_tket_exts: If true, the extensions in :meth:`embedded_extensions`
                will not be not be included in the envelope even when they are used in the
                HUGR. This is useful when sending the HUGR to other components that
                already have the tket extensions available.
        """

    @staticmethod
    def from_bytes(envelope: bytes) -> CompilationState:
        """Load a CompilationState from a HUGR envelope.

        Some envelope formats can be read from a string. See :meth:`from_str`.

        Args:
            envelope: The byte string representing a Package.

        Returns:
            The loaded program.
        """

    @staticmethod
    def from_str(envelope: str) -> CompilationState:
        """Load a CompilationState from a HUGR envelope string.

        Not all envelope formats can be represented as strings.
        See :meth:`from_bytes` for a more general method.

        Args:
            envelope: The string representing a Package.

        Returns:
            The loaded program.
        """

    def _circuit_cost(self, cost_fn: Callable[[TketOp], Any]) -> Any:
        """Compute the cost of the circuit based on a per-operation cost function."""

    def num_operations(self) -> int:
        """The number of operations in the circuit.

        This includes TketOps, pytket ops, and any other custom operations.

        Nested circuits are traversed to count their operations.
        """

    def hash(self) -> int:
        """Returns a hash of the circuit."""

    def render_mermaid(self) -> str:
        """Render the program as a Mermaid graph."""

    def validate(self) -> None:
        """Validate the program, checking for structural issues."""

    def to_tket1(self) -> Tk1Circuit:
        """Convert the program back to a legacy pytket Circuit."""

    def __copy__(self) -> CompilationState:
        """Copy the program."""

    def __deepcopy__(self, memo: object) -> CompilationState:
        """Deep copy the program."""

class Node:
    """A node in the HUGR graph."""

    def __init__(self, index: int) -> None:
        """Create a new node."""

class Wire:
    """A wire in the HUGR graph."""

    def node(self) -> Node:
        """The source node of the wire."""

    def port(self) -> int:
        """The source port index of the wire."""

class CircuitCost:
    """A cost associated with a circuit."""

def embedded_extensions() -> list[str]:
    """Returns the list of extension ids supported by the CompilationState loader."""

class HugrError(Exception): ...
class BuildError(Exception): ...
class ValidationError(Exception): ...
class HUGRSerializationError(Exception): ...
class TK1EncodeError(Exception): ...
