from pytket import Circuit
from pytket.qasm import circuit_from_qasm_str
from tket.pattern import CircuitPattern, PatternMatcher


def test_simple_matching():
    """a simple circuit matching test"""
    c = Circuit(2).CX(0, 1).H(1).CX(0, 1)

    p1 = CircuitPattern(Circuit(2).CX(0, 1).H(1))
    p2 = CircuitPattern(Circuit(2).H(0).CX(1, 0))

    matcher = PatternMatcher(iter([p1, p2]))

    assert len(matcher.find_matches(c)) == 2


def test_non_convex_pattern():
    """two-qubit circuits can't match three-qb ones"""
    p1 = CircuitPattern(Circuit(3).CX(0, 1).CX(1, 2))
    matcher = PatternMatcher(iter([p1]))

    c = Circuit(2).CX(0, 1).CX(1, 0)
    assert len(matcher.find_matches(c)) == 0

    c = Circuit(3).CX(0, 1).CX(1, 0).CX(1, 2)
    assert len(matcher.find_matches(c)) == 0

    c = Circuit(3).H(0).CX(0, 1).CX(1, 0).CX(0, 2)
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

    c = circuit_from_qasm_str(QASM)

    p1 = CircuitPattern(Circuit(2).CX(0, 1).H(1))
    p2 = CircuitPattern(Circuit(2).H(0).CX(1, 0))
    p3 = CircuitPattern(Circuit(2).CX(0, 1).CX(1, 0))
    p4 = CircuitPattern(Circuit(3).CX(0, 1).CX(1, 2))

    matcher = PatternMatcher(iter([p1, p2, p3, p4]))

    assert len(matcher.find_matches(c)) == 6
