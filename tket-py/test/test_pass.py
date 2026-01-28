from pytket import Circuit, OpType
from dataclasses import dataclass
from typing import Callable, Any
from tket.ops import TketOp
from tket.passes import (
    badger_pass,
    greedy_depth_reduce,
    chunks,
    NormalizeGuppy,
    normalize_guppy,
)
from tket.circuit import Tk2Circuit
from tket_exts import tket_registry

from tket.pattern import Rule, RuleMatcher
import hypothesis.strategies as st
from hypothesis.strategies._internal import SearchStrategy
from hypothesis import given, settings

from tket.passes import PytketHugrPass
from pytket.passes import CliffordSimp, SquashRzPhasedX, SequencePass
from hugr.build.base import Hugr

import pytest


@st.composite
def circuits(
    draw: Callable[[SearchStrategy[Any]], Any],
    n_qubits: SearchStrategy[int] = st.integers(min_value=0, max_value=8),
    depth: SearchStrategy[int] = st.integers(min_value=5, max_value=50),
) -> Circuit:
    total_qubits = draw(n_qubits)
    circuit = Circuit(total_qubits)
    if total_qubits == 0:
        return circuit
    for _ in range(draw(depth)):
        gates = [circuit.Rz, circuit.H]
        if total_qubits > 1:
            gates.extend([circuit.CX])
        gate = draw(st.sampled_from(gates))
        control = draw(st.integers(min_value=0, max_value=total_qubits - 1))
        if gate in (circuit.CX,):
            target = draw(
                st.integers(min_value=0, max_value=total_qubits - 1).filter(
                    lambda x: x != control
                )
            )
            gate(control, target)
        if gate == circuit.Rz:
            angle = draw(st.floats(min_value=-2.0, max_value=2.0))
            gate(angle, control)
        if gate == circuit.H:
            gate(control)
    return circuit


@pytest.mark.skip(
    reason="bug to be investigated, see https://github.com/quantinuum/tket2/issues/983"
)
def test_simple_badger_pass_no_opt():
    c = Circuit(3).CCX(0, 1, 2)
    badger = badger_pass(max_threads=1, timeout=0, rebase=True)
    badger.apply(c)
    assert c.n_gates_of_type(OpType.CX) == 6


@dataclass
class DepthOptimisePass:
    def apply(self, circ: Circuit) -> Circuit:
        (circ, n_moves) = greedy_depth_reduce(circ)
        return circ


def test_depth_optimise():
    c = Circuit(4).CX(0, 2).CX(1, 2).CX(1, 3)

    assert c.depth() == 3

    c = DepthOptimisePass().apply(c)

    assert c.depth() == 2


def _depth_impl(circ: Circuit) -> None:
    new, _ = greedy_depth_reduce(circ)

    assert circ.n_gates == new.n_gates
    assert new.depth() <= circ.depth()


@given(circ=circuits())
@settings(print_blob=True, deadline=30)
def test_depth_hyp(circ: Circuit) -> None:
    _depth_impl(circ)


def test_depth_bug() -> None:
    circ = Circuit(3).H(0).CX(1, 0).H(0).CX(0, 2).H(0).CX(1, 2)
    _depth_impl(circ)


def test_chunks():
    c = Circuit(4).CX(0, 2).CX(1, 3).CX(1, 2).CX(0, 3).CX(1, 3)

    assert c.depth() == 3

    circ_chunks = chunks(c, 2)
    circuits = circ_chunks.circuits()
    circ_chunks.update_circuit(0, circuits[0])
    c2 = circ_chunks.reassemble()

    assert c2.depth() == 3
    assert type(c2) is Circuit

    # Split and reassemble, with a tket circuit
    tk2_chunks = chunks(Tk2Circuit(c2), 2)
    tk2 = tk2_chunks.reassemble()

    assert type(tk2) is Tk2Circuit


def test_cx_rule():
    c = Tk2Circuit(Circuit(4).CX(0, 2).CX(1, 2).CX(1, 2))

    rule = Rule(Circuit(2).CX(0, 1).CX(0, 1), Circuit(2))
    matcher = RuleMatcher([rule])

    mtch = matcher.find_match(c)

    c.apply_rewrite(mtch)

    out = c.to_tket1()

    assert out == Circuit(4).CX(0, 2)


def test_multiple_rules():
    circ = Tk2Circuit(Circuit(3).CX(0, 1).H(0).H(1).H(2).Z(0).H(0).H(1).H(2))

    rule1 = Rule(Circuit(1).H(0).Z(0).H(0), Circuit(1).X(0))
    rule2 = Rule(Circuit(1).H(0).H(0), Circuit(1))
    matcher = RuleMatcher([rule1, rule2])

    match_count = 0
    while match := matcher.find_match(circ):
        match_count += 1
        circ.apply_rewrite(match)

    assert match_count == 3

    out = circ.to_tket1()
    assert out == Circuit(3).CX(0, 1).X(0)


def test_clifford_simp_no_swaps():
    c = Tk2Circuit(Circuit(4).CX(0, 2).CX(1, 2).CX(1, 2))
    hugr = Hugr.from_str(c.to_str(), tket_registry())
    cliff_pass = PytketHugrPass(CliffordSimp(allow_swaps=False))
    res = cliff_pass.run(hugr)
    opt_circ = Tk2Circuit.from_bytes(res.hugr.to_bytes())
    assert opt_circ.circuit_cost(lambda op: int(op == TketOp.CX)) == 1


def test_clifford_simp_with_swaps() -> None:
    cx_circ = Tk2Circuit(Circuit(2).CX(0, 1).CX(1, 0))
    hugr = Hugr.from_str(cx_circ.to_str(), tket_registry())
    cliff_pass_perm = PytketHugrPass(CliffordSimp(allow_swaps=True))
    # Simplify 2 CX circuit to a single CX with an implicit swap.
    res = cliff_pass_perm.run(hugr)
    opt_circ = Tk2Circuit.from_bytes(res.hugr.to_bytes())
    assert opt_circ.circuit_cost(lambda op: int(op == TketOp.CX)) == 1


def test_squash_phasedx_rz():
    c = Tk2Circuit(Circuit(1).Rz(0.25, 0).Rz(0.75, 0).Rz(0.25, 0).Rz(-1.25, 0))
    hugr = Hugr.from_str(c.to_str(), tket_registry())
    squash_pass = PytketHugrPass(SquashRzPhasedX())
    opt_hugr = squash_pass(hugr)
    opt_circ = Tk2Circuit.from_bytes(opt_hugr.to_bytes())
    # TODO: We cannot use circuit_cost due to a panic on non-tket ops and there
    # being some parameter loads...
    assert opt_circ.num_operations() == 0


def test_sequence_pass():
    c = Tk2Circuit(
        Circuit(2).CX(0, 1).CX(1, 0).Rz(0.25, 0).Rz(0.75, 0).Rz(0.25, 0).Rz(-1.25, 0)
    )
    hugr = Hugr.from_str(c.to_str(), tket_registry())
    seq_pass = SequencePass([SquashRzPhasedX(), CliffordSimp(allow_swaps=True)])
    clifford_and_squash_pass = PytketHugrPass(seq_pass)
    res_hugr = clifford_and_squash_pass(hugr)
    opt_circ = Tk2Circuit.from_bytes(res_hugr.to_bytes())
    assert opt_circ.num_operations() == 1
    assert opt_circ.circuit_cost(lambda op: int(op == TketOp.CX)) == 1


def test_normalize_guppy():
    """Test the normalize_guppy pass.

    This won't actually do anything useful, we just want to check that the pass
    runs without errors.
    """

    pytket_circ = Circuit(4).CX(0, 2).CX(1, 2).CX(1, 2)
    # TODO: add a more thorough test which checks that the hugr is normalized as expected.
    # test NormalizeGuppy as a ComposablePass
    c1 = Tk2Circuit(pytket_circ)
    hugr = Hugr.from_str(c1.to_str(), tket_registry())

    normalize = NormalizeGuppy()
    clean_hugr = normalize(hugr)
    normal_circ1 = Tk2Circuit.from_bytes(clean_hugr.to_bytes())
    assert normal_circ1.circuit_cost(lambda op: int(op == TketOp.CX)) == 3

    # test normalize_guppy as a function call
    c2 = Tk2Circuit(pytket_circ)
    normal_circ2 = normalize_guppy(c2)
    assert normal_circ2.circuit_cost(lambda op: int(op == TketOp.CX)) == 3
