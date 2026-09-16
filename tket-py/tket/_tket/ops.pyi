from collections.abc import Iterable
from enum import Enum

from tket._tket.types import HugrType

class TketOp(Enum):
    """A rust-backed Tket built-in operation."""

    H = "H"
    CX = "CX"
    CY = "CY"
    CZ = "CZ"
    CRz = "CRz"
    T = "T"
    Tdg = "Tdg"
    S = "S"
    Sdg = "Sdg"
    X = "X"
    Y = "Y"
    Z = "Z"
    Rx = "Rx"
    Ry = "Ry"
    Rz = "Rz"
    Toffoli = "Toffoli"
    Measure = "Measure"
    MeasureFree = "MeasureFree"
    QAlloc = "QAlloc"
    TryQAlloc = "TryQAlloc"
    QFree = "QFree"
    Reset = "Reset"
    V = "V"
    Vdg = "Vdg"

    def __init__(self, op: str) -> None:
        """Create a new TketOp from a string name."""

    @staticmethod
    def values() -> Iterable[TketOp]:
        """Iterate over all operation variants."""

    def to_custom(self) -> CustomOp:
        """Convert to a custom operation."""

    @property
    def name(self) -> str:
        """Get the string name of the operation."""

    @property
    def qualified_name(self) -> str:
        """Return the fully qualified name of the operation, including the extension."""

    def __eq__(self, value: object) -> bool: ...

class Pauli(Enum):
    """Simple enum representation of Pauli matrices."""

    I = "I"
    X = "X"
    Y = "Y"
    Z = "Z"

    def __init__(self, pauli: str) -> None:
        """Create a new Pauli from a string name."""

    @property
    def name(self) -> str:
        """Get the string name of the Pauli."""

    @staticmethod
    def values() -> Iterable[Pauli]:
        """Iterate over all Pauli matrix variants."""

    def __eq__(self, value: object) -> bool: ...

class CustomOp:
    """A HUGR custom operation."""

    def __init__(
        self,
        extension: str,
        op_name: str,
        input_types: list[HugrType],
        output_types: list[HugrType],
    ) -> None:
        """Create a new custom operation from name and input/output types."""

    def to_custom(self) -> CustomOp:
        """Convert to a custom operation. Identity operation."""

    @property
    def name(self) -> str:
        """Fully qualified (including extension) name of the operation."""
