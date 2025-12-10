from __future__ import annotations

from enum import Enum, auto
from typing import Protocol

import tket

from tket._tket.ops import CustomOp
from tket.types import QB_T

__all__ = ["CustomOp", "ToCustomOp", "TketOp", "Pauli"]


class ToCustomOp(Protocol):
    """Operation that can be converted to a HUGR CustomOp."""

    def to_custom(self) -> CustomOp:
        """Convert to a custom operation."""
        ...

    @property
    def name(self) -> str:
        """Name of the operation."""
        ...


class TketOp(Enum):
    """A Tket built-in operation.

    Implements the `ToCustomOp` protocol.
    """

    H = auto()
    CX = auto()
    CY = auto()
    CZ = auto()
    CRz = auto()
    T = auto()
    Tdg = auto()
    S = auto()
    Sdg = auto()
    X = auto()
    Y = auto()
    Z = auto()
    Rz = auto()
    Rx = auto()
    Ry = auto()
    Toffoli = auto()
    Measure = auto()
    MeasureFree = auto()
    QAlloc = auto()
    TryQAlloc = auto()
    QFree = auto()
    Reset = auto()
    V = auto()
    Vdg = auto()

    def to_custom(self) -> CustomOp:
        """Convert to a custom operation."""
        return self._to_rs().to_custom()

    def _to_rs(self) -> "tket._tket.ops.TketOp":  # type: ignore[name-defined]
        """Convert to the Rust-backed TketOp representation."""
        return tket._tket.ops.TketOp(self.name)  # type: ignore[attr-defined]

    @staticmethod
    def _from_rs(op: tket._tket.ops.TketOp) -> "TketOp":  # type: ignore[name-defined]
        """Convert from the Rust-backed TketOp representation."""
        return TketOp[op.name]

    def __eq__(self, other: object) -> bool:
        """Check if two TketOps are equal."""
        if isinstance(other, TketOp):
            return self.name == other.name
        elif isinstance(other, tket._tket.ops.TketOp):  # type: ignore[attr-defined]
            return self == TketOp._from_rs(other)
        elif isinstance(other, str):
            return self.name == other
        return False


class Pauli(Enum):
    """Simple enum representation of Pauli matrices.

    Implements the `ToCustomOp` protocol.
    """

    I = auto()  # noqa: E741
    X = auto()
    Y = auto()
    Z = auto()

    def to_custom(self) -> CustomOp:
        extension_name = "tket.quantum"
        gate_name = self.name
        return CustomOp(extension_name, gate_name, [QB_T], [QB_T])

    def _to_rs(self) -> "tket._tket.ops.Pauli":  # type: ignore[name-defined]
        """Convert to the Rust-backed Pauli representation."""
        return tket._tket.ops.Pauli(self.name)  # type: ignore[attr-defined]

    @staticmethod
    def _from_rs(pauli: tket._tket.ops.Pauli) -> "Pauli":  # type: ignore[name-defined]
        """Convert from the Rust-backed Pauli representation."""
        return Pauli[pauli.name]

    def __eq__(self, other: object) -> bool:
        """Check if two Paulis are equal."""
        if isinstance(other, Pauli):
            return self.name == other.name
        elif isinstance(other, tket._tket.ops.Pauli):  # type: ignore[attr-defined]
            return self == Pauli._from_rs(other)
        elif isinstance(other, str):
            return self.name == other
        return False
