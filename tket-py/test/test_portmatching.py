import pytest
from tket._pattern import CircuitPattern, PatternMatcher
from tket._state import CompilationState

pytket = pytest.importorskip("pytket")
from pytket import Circuit  # noqa: E402
from pytket.qasm import circuit_from_qasm_str  # noqa: E402


def _tk(circ: Circuit):
    """Convert a pytket Circuit to a Rust CompilationState."""
    return CompilationState.from_tket1(circ)._inner


def test_simple_matching():
    """a simple circuit matching test"""
    c = _tk(Circuit(2).CX(0, 1).H(1).CX(0, 1))

    p1 = CircuitPattern(_tk(Circuit(2).CX(0, 1).H(1)))
    p2 = CircuitPattern(_tk(Circuit(2).H(0).CX(1, 0)))

    matcher = PatternMatcher(iter([p1, p2]))

    assert len(matcher.find_matches(c)) == 2


def test_non_convex_pattern():
    """two-qubit circuits can't match three-qb ones"""
    p1 = CircuitPattern(_tk(Circuit(3).CX(0, 1).CX(1, 2)))
    matcher = PatternMatcher(iter([p1]))

    c = _tk(Circuit(2).CX(0, 1).CX(1, 0))
    assert len(matcher.find_matches(c)) == 0

    c = _tk(Circuit(3).CX(0, 1).CX(1, 0).CX(1, 2))
    assert len(matcher.find_matches(c)) == 0

    c = _tk(Circuit(3).H(0).CX(0, 1).CX(1, 0).CX(0, 2))
    assert len(matcher.find_matches(c)) == 1


def test_larger_matching():
    """a larger crafted circuit with matches WIP"""
    QASM = """OPENQASM 2.0;
    include "qelib1.inc";

    qreg q[3];

    h q[0];
    h q[1];
    h q[1];
    cx q[1], q[2];
    h q[2];
    cx q[1], q[2];
    cx q[2], q[1];
    cx q[1], q[2];
    cx q[2], q[0];
    """

    c = _tk(circuit_from_qasm_str(QASM))

    p1 = CircuitPattern(_tk(Circuit(2).CX(0, 1).H(1)))
    p2 = CircuitPattern(_tk(Circuit(2).H(0).CX(1, 0)))
    p3 = CircuitPattern(_tk(Circuit(2).CX(0, 1).CX(1, 0)))
    p4 = CircuitPattern(_tk(Circuit(3).CX(0, 1).CX(1, 2)))

    matcher = PatternMatcher(iter([p1, p2, p3, p4]))

    assert len(matcher.find_matches(c)) == 6
