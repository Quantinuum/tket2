"""Metadata values defined by the TKET compiler.

Examples:
    >>> from hugr import Hugr
    >>> from tket.metadata import (
    ...     MaxQubitsHint,
    ...     PytketInputParameters,
    ...     PytketPhaseExpr,
    ...     PytketQubitRegisterNames,
    ... )
    >>>
    >>> hugr = Hugr()
    >>> node = hugr[hugr.module_root]
    >>>
    >>> node.metadata[MaxQubitsHint] = 3
    >>> node.metadata[PytketInputParameters] = ["theta", "phi"]
    >>> node.metadata[PytketQubitRegisterNames] = [("q", [0]), ("ancilla", [1])]
    >>> node.metadata[PytketPhaseExpr] = "1/2"
    >>> node.metadata[MaxQubitsHint]
    3
    >>> node.metadata.get(PytketQubitRegisterNames)
    [('q', [0]), ('ancilla', [1])]
"""
# Changes to this file **SHOULD** be reflected in `tket/src/metadata.rs`.

from __future__ import annotations

from typing import TYPE_CHECKING, Literal, TypeAlias, TypedDict

from hugr.metadata import Metadata

from ._tket import metadata as _metadata

if TYPE_CHECKING:
    from hugr.utils import JsonType


__all__ = [
    "RewriteTraceValue",
    "InlineAnnotationValue",
    "InlineAnnotation",
    "CircuitRewriteTraces",
    "UnitaryFlags",
    "PytketInputParameters",
    "PytketOpGroup",
    "PytketBit",
    "PytketQubit",
    "PytketBitRegisterNames",
    "PytketQubitRegisterNames",
    "PytketPhaseExpr",
]


# Identifier for a pytket qubit register element.
#
# This can be passed to `pytket.unit_id.Qubit.from_list`
PytketQubit: TypeAlias = tuple[str, list[int]]
# Identifier for a pytket bit register element.
#
# This can be passed to `pytket.unit_id.Bit.from_list`
PytketBit: TypeAlias = tuple[str, list[int]]


class RewriteTraceValue(TypedDict):
    """Serialized rewrite trace metadata entry."""

    individual_matches: int


class ExpectedQubitsHint(Metadata[int]):
    """Metadata key for the number of qubits required to execute a HUGR node."""

    KEY = _metadata.EXPECTED_QUBITS_HINT
    ALIASES = _metadata.EXPECTED_QUBITS_HINT_ALIASES


InlineAnnotationValue: TypeAlias = Literal["never"] | Literal["best_effort"]


class InlineAnnotation(Metadata[InlineAnnotationValue]):
    """Metadata hinting the compiler that a function declaration should be inlined at its call sites.

    When a function is not annotated, we use a heuristic to determine whether to inline.

    Values:
    - "never": Never inline this function.
    - "best_effort":
        Inline the function if possible.
        This is not guaranteed, the compiler may choose not to inline functions with this annotation.
    """

    KEY = _metadata.INLINE_ANNOTATION

    @classmethod
    def to_json(cls, value: InlineAnnotationValue) -> JsonType:
        return value

    @classmethod
    def from_json(cls, value: JsonType) -> InlineAnnotationValue:
        match value:
            case "never" | "best_effort":
                return value
            case _:
                msg = f"Expected {cls.KEY} metadata to be 'never' or 'best_effort', but got {value!r}"
                raise TypeError(msg)


class CircuitRewriteTraces(Metadata[list[RewriteTraceValue]]):
    """Metadata key for rewrite traces recorded during circuit transformation."""

    KEY = _metadata.CIRCUIT_REWRITE_TRACES


class UnitaryFlags(Metadata[int]):
    """Metadata key for unitary/modifier flags stored on a HUGR node."""

    KEY = _metadata.UNITARY_FLAGS
    ALIASES = _metadata.UNITARY_FLAGS_ALIAS


class PytketInputParameters(Metadata[list[str]]):
    """Metadata key for explicit names of input parameter wires."""

    KEY = _metadata.PYTKET_INPUT_PARAMETERS


class PytketOpGroup(Metadata[str]):
    """Metadata key for the pytket ``opgroup`` field on a decoded operation."""

    KEY = _metadata.PYTKET_OP_GROUP


class PytketBitRegisterNames(Metadata[list[PytketBit]]):
    """Metadata key for explicit names of input bit registers."""

    KEY = _metadata.PYTKET_BIT_REGISTER_NAMES

    @classmethod
    def to_json(cls, value: list[PytketBit]) -> JsonType:
        return _store_pytket_register(value)

    @classmethod
    def from_json(cls, value: JsonType) -> list[PytketBit]:
        return _read_pytket_register(cls.KEY, value)


class PytketQubitRegisterNames(Metadata[list[PytketQubit]]):
    """Metadata key for explicit names of input qubit registers."""

    KEY = _metadata.PYTKET_QUBIT_REGISTER_NAMES

    @classmethod
    def to_json(cls, value: list[PytketQubit]) -> JsonType:
        return _store_pytket_register(value)

    @classmethod
    def from_json(cls, value: JsonType) -> list[PytketQubit]:
        return _read_pytket_register(cls.KEY, value)


class PytketPhaseExpr(Metadata[str]):
    """Metadata key for the serialized pytket global phase expression."""

    KEY = _metadata.PYTKET_PHASE_EXPR


def _store_pytket_register(value: list[tuple[str, list[int]]]) -> JsonType:
    return [[name, list(indices)] for name, indices in value]


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
        register_indices: list[int] = []
        for index in indices:
            if not isinstance(index, int):
                raise TypeError(
                    f"Expected {key} register index to be an integer, but got {type(index)}"
                )
            register_indices.append(index)
        registers.append((name, register_indices))
    return registers
