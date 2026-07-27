import pytest

from dataclasses import dataclass

from tket._state import (
    CompilationState,
)
from tket._ops import TketOp


@dataclass
class CustomCost:
    gate_count: int
    h_count: int

    def __add__(self, other):
        return CustomCost(
            self.gate_count + other.gate_count, self.h_count + other.h_count
        )


def test_cost():
    pytket = pytest.importorskip("pytket")

    circ = CompilationState.from_tket1(
        pytket.Circuit(4).CX(0, 1).H(1).CX(1, 2).CX(0, 3).H(0)
    )

    print(circ.circuit_cost(lambda op: int(op == TketOp.CX)))

    assert circ.circuit_cost(lambda op: int(op == TketOp.CX)) == 3
    assert circ.circuit_cost(lambda op: CustomCost(1, op == TketOp.H)) == CustomCost(
        5, 2
    )


def test_hash():
    pytket = pytest.importorskip("pytket")

    circA = CompilationState.from_tket1(pytket.Circuit(4).CX(0, 1).CX(1, 2).CX(0, 3))
    circB = CompilationState.from_tket1(pytket.Circuit(4).CX(1, 2).CX(0, 1).CX(0, 3))
    circC = CompilationState.from_tket1(pytket.Circuit(4).CX(0, 1).CX(0, 3).CX(1, 2))

    assert hash(circA) != hash(circB)
    assert hash(circA) == hash(circC)


def test_conversion():
    pytket = pytest.importorskip("pytket")
    tk1 = pytket.Circuit(4).CX(0, 2).CX(1, 2).CX(1, 3)

    tk2 = CompilationState.from_tket1(tk1)
    mermaid = tk2.render_mermaid()

    assert type(tk2) is CompilationState
    assert mermaid  # non-empty

    tk1_back = tk2.to_tket1()

    assert tk1_back == tk1
    assert type(tk1_back) is pytket.Circuit


def test_conversion_qsystem():
    pytket = pytest.importorskip("pytket")
    tk1 = pytket.Circuit(2).ZZPhase(0.75, 0, 1).PhasedX(0.25, 0.33, 1)

    tk2 = CompilationState.from_tket1(tk1)
    mermaid = tk2.render_mermaid()

    assert type(tk2) is CompilationState
    assert mermaid  # non-empty

    # Check that we didn't use the opaque tk1 op fallback.
    assert "TKET1.tk1op" not in mermaid
    assert "PhasedX" in mermaid
    assert "ZZPhase" in mermaid

    tk1_back = tk2.to_tket1()

    assert tk1_back == tk1
    assert type(tk1_back) is pytket.Circuit
