"""Metadata values defined by the TKET compiler.

Examples:
    >>> from hugr import Hugr
    >>> from tket.metadata import InputParameters, MaxQubits, Phase, QubitRegisters
    >>>
    >>> hugr = Hugr()
    >>> node = hugr[hugr.module_root]
    >>>
    >>> node.metadata[MaxQubits] = 3
    >>> node.metadata[InputParameters] = ["theta", "phi"]
    >>> node.metadata[QubitRegisters] = [("q", [0]), ("ancilla", [1])]
    >>> node.metadata[Phase] = "1/2"
    >>> node.metadata[MaxQubits]
    3
    >>> node.metadata.get(QubitRegisters)
    [('q', [0]), ('ancilla', [1])]
"""
# Changes to this file **MUST** be reflected in `tket/src/metadata.rs`

from __future__ import annotations

from typing import TYPE_CHECKING, TypeAlias, TypedDict

from hugr.metadata import Metadata
from ._tket import metadata as _metadata

if TYPE_CHECKING:
    from hugr.utils import JsonType


__all__ = [
    "RewriteTraceValue",
    "MaxQubits",
    "CircuitRewriteTraces",
    "Unitary",
    "InputParameters",
    "OpGroup",
    "BitRegisters",
    "QubitRegisters",
    "Phase",
]


# Identifier for a TKET1 qubit register element.
#
# This can be passed to `pytket.unit_id.Qubit.from_list`
PytketQubit: TypeAlias = tuple[str, list[int]]
# Identifier for a TKET1 bit register element.
#
# This can be passed to `pytket.unit_id.Bit.from_list`
PytketBit: TypeAlias = tuple[str, list[int]]


class RewriteTraceValue(TypedDict):
    """Serialized rewrite trace metadata entry."""

    individual_matches: int


class MaxQubits(Metadata[int]):
    """Metadata key for the number of qubits required to execute a HUGR node."""

    KEY = _metadata.MAX_QUBITS


class CircuitRewriteTraces(Metadata[list[RewriteTraceValue]]):
    """Metadata key for rewrite traces recorded during circuit transformation."""

    KEY = _metadata.CIRCUIT_REWRITE_TRACES


class Unitary(Metadata[int]):
    """Metadata key for unitary/modifier flags stored on a HUGR node."""

    KEY = _metadata.UNITARY


class InputParameters(Metadata[list[str]]):
    """Metadata key for explicit names of input parameter wires."""

    KEY = _metadata.INPUT_PARAMETERS


class OpGroup(Metadata[str]):
    """Metadata key for the pytket ``opgroup`` field on a decoded operation."""

    KEY = _metadata.OP_GROUP


class BitRegisters(Metadata[list[PytketBit]]):
    """Metadata key for explicit names of input bit registers."""

    KEY = _metadata.BIT_REGISTERS

    @classmethod
    def to_json(cls, value: list[PytketBit]) -> JsonType:
        return _store_pytket_register(value)

    @classmethod
    def from_json(cls, value: JsonType) -> list[PytketBit]:
        return _read_pytket_register(cls.KEY, value)


class QubitRegisters(Metadata[list[PytketQubit]]):
    """Metadata key for explicit names of input qubit registers."""

    KEY = _metadata.QUBIT_REGISTERS

    @classmethod
    def to_json(cls, value: list[PytketQubit]) -> JsonType:
        return _store_pytket_register(value)

    @classmethod
    def from_json(cls, value: JsonType) -> list[PytketQubit]:
        return _read_pytket_register(cls.KEY, value)


class Phase(Metadata[str]):
    """Metadata key for the serialized TKET1 global phase expression."""

    KEY = _metadata.PHASE


def _store_pytket_register(value: list[tuple[str, list[int]]]) -> JsonType:
    return [[name, indices] for name, indices in value]  # type: ignore


def _read_pytket_register(key: str, value: JsonType) -> list[tuple[str, list[int]]]:
    if not isinstance(value, list):
        raise TypeError(f"Expected {key} metadata to be a list, but got {type(value)}")

    registers: list[tuple[str, list[int]]] = []
    for entry in value:
        if not isinstance(entry, list) or len(entry) != 2:
            raise TypeError(
                f"Expected each {key} metadata entry to be [name, [indices...]], but got {entry!r}"
            )
        name, indices = entry
        if not isinstance(name, str):
            raise TypeError(
                f"Expected {key} register name to be a string, but got {type(name)}"
            )
        if not isinstance(indices, list) or not all(
            isinstance(index, int) for index in indices
        ):
            raise TypeError(
                f"Expected {key} register indices to be a list of integers, but got {indices!r}"
            )
        registers.append((name, indices))  # type: ignore
    return registers
