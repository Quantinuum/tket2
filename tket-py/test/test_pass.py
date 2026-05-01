import tempfile

from pytket import Circuit, OpType
from typing import Callable, Any
import subprocess
from tket._ops import TketOp
from tket.passes import (
    _badger_optimise,
    _greedy_depth_reduce,
    InlineFunctions,
    inline_funcs,
    NormalizeGuppy,
    ModifierResolverPass,
    GlobalScope,
)
from tket._state import CompilationState
from tket_exts import tket_registry

from tket._pattern import Rule, RuleMatcher
import hypothesis.strategies as st
from hypothesis.strategies._internal import SearchStrategy
from hypothesis import given, settings

from tket.passes import PytketHugrPass
from pytket.passes import (
    CliffordSimp,
    SquashRzPhasedX,
    RemoveRedundancies,
    SequencePass,
)
from hugr.build.base import Hugr

import numpy as np
import pytest


from pathlib import Path


normalize = NormalizeGuppy()


def _hugr_from_path(str_path: str) -> Hugr:
    with open(Path(str_path), "rb") as f:
        h = Hugr.from_bytes(f.read())

    return h


def _count_ops(hugr: Hugr, op_string_name: str) -> int:
    count = 0
    for _, data in hugr.nodes():
        if op_string_name in data.op.name():
            count += 1

    return count


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
    state = CompilationState.from_tket1(Circuit(3).CCX(0, 1, 2))
    _badger_optimise(state, max_threads=1, timeout=0)
    c = state.to_tket1()
    assert c.n_gates_of_type(OpType.CX) == 6


def test_depth_optimise():
    c = CompilationState.from_tket1(Circuit(4).CX(0, 2).CX(1, 2).CX(1, 3))

    original = c.to_tket1()
    assert original.depth() == 3

    _greedy_depth_reduce(c)

    result = c.to_tket1()
    assert result.depth() == 2


def _depth_impl(circ: Circuit) -> None:
    tk = CompilationState.from_tket1(circ)
    original_gates = circ.n_gates
    original_depth = circ.depth()

    _greedy_depth_reduce(tk)

    new = tk.to_tket1()
    assert original_gates == new.n_gates
    assert new.depth() <= original_depth


@given(circ=circuits())
@settings(print_blob=True, deadline=30)
def test_depth_hyp(circ: Circuit) -> None:
    _depth_impl(circ)


def test_depth_bug() -> None:
    circ = Circuit(3).H(0).CX(1, 0).H(0).CX(0, 2).H(0).CX(1, 2)
    _depth_impl(circ)


def test_cx_rule():
    c = CompilationState.from_tket1(Circuit(4).CX(0, 2).CX(1, 2).CX(1, 2))

    rule = Rule(
        CompilationState.from_tket1(Circuit(2).CX(0, 1).CX(0, 1))._inner,
        CompilationState.from_tket1(Circuit(2))._inner,
    )
    matcher = RuleMatcher([rule])

    mtch = matcher.find_match(c._inner)

    c._inner.apply_rewrite(mtch)

    out = c.to_tket1()

    assert out == Circuit(4).CX(0, 2)


def test_multiple_rules():
    circ = CompilationState.from_tket1(
        Circuit(3).CX(0, 1).H(0).H(1).H(2).Z(0).H(0).H(1).H(2)
    )

    rule1 = Rule(
        CompilationState.from_tket1(Circuit(1).H(0).Z(0).H(0))._inner,
        CompilationState.from_tket1(Circuit(1).X(0))._inner,
    )
    rule2 = Rule(
        CompilationState.from_tket1(Circuit(1).H(0).H(0))._inner,
        CompilationState.from_tket1(Circuit(1))._inner,
    )
    matcher = RuleMatcher([rule1, rule2])

    match_count = 0
    while match := matcher.find_match(circ._inner):
        match_count += 1
        circ._inner.apply_rewrite(match)

    assert match_count == 3

    out = circ.to_tket1()
    assert out == Circuit(3).CX(0, 1).X(0)


def test_clifford_simp_no_swaps():
    c = CompilationState.from_tket1(Circuit(4).CX(0, 2).CX(1, 2).CX(1, 2))
    hugr = Hugr.from_str(c.to_str(), tket_registry())
    cliff_pass = PytketHugrPass(CliffordSimp(allow_swaps=False))
    res = cliff_pass.run(hugr)
    opt_circ = CompilationState.from_bytes(res.hugr.to_bytes())
    assert opt_circ.circuit_cost(lambda op: int(op == TketOp.CX)) == 1


def test_clifford_simp_with_swaps() -> None:
    cx_circ = CompilationState.from_tket1(Circuit(2).CX(0, 1).CX(1, 0))
    hugr = Hugr.from_str(cx_circ.to_str(), tket_registry())
    cliff_pass_perm = PytketHugrPass(CliffordSimp(allow_swaps=True))
    # Simplify 2 CX circuit to a single CX with an implicit swap.
    res = cliff_pass_perm.run(hugr)
    opt_circ = CompilationState.from_bytes(res.hugr.to_bytes())
    assert opt_circ.circuit_cost(lambda op: int(op == TketOp.CX)) == 1


def test_squash_phasedx_rz():
    c = CompilationState.from_tket1(
        Circuit(1).Rz(0.25, 0).Rz(0.75, 0).Rz(0.25, 0).Rz(-1.25, 0)
    )
    hugr = Hugr.from_str(c.to_str(), tket_registry())
    squash_pass = PytketHugrPass(SquashRzPhasedX())
    opt_hugr = squash_pass(hugr)
    opt_circ = CompilationState.from_bytes(opt_hugr.to_bytes())
    # TODO: We cannot use circuit_cost due to a panic on non-tket ops and there
    # being some parameter loads...
    assert opt_circ.num_operations() == 0


def test_sequence_pass():
    c = CompilationState.from_tket1(
        Circuit(2).CX(0, 1).CX(1, 0).Rz(0.25, 0).Rz(0.75, 0).Rz(0.25, 0).Rz(-1.25, 0)
    )
    hugr = Hugr.from_str(c.to_str(), tket_registry())
    seq_pass = SequencePass([SquashRzPhasedX(), CliffordSimp(allow_swaps=True)])
    clifford_and_squash_pass = PytketHugrPass(seq_pass)
    res_hugr = clifford_and_squash_pass(hugr)
    opt_circ = CompilationState.from_bytes(res_hugr.to_bytes())
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
    c1 = CompilationState.from_tket1(pytket_circ)
    hugr = Hugr.from_str(c1.to_str(), tket_registry())

    normalize = NormalizeGuppy()
    clean_hugr = normalize(hugr)
    normal_circ1 = CompilationState.from_bytes(clean_hugr.to_bytes())
    assert normal_circ1.circuit_cost(lambda op: int(op == TketOp.CX)) == 3


def test_modifier_resolver() -> None:
    normalize = NormalizeGuppy()
    mr_pass = ModifierResolverPass()
    modifier_hugr: Hugr = _hugr_from_path("test_files/guppy_examples/modifiers.hugr")

    normalized = normalize(modifier_hugr)

    assert _count_ops(normalized, "tket.modifier.ControlModifier") == 1
    assert _count_ops(normalized, "tket.modifier.DaggerModifier") == 1

    resolved: Hugr = mr_pass(normalized)

    assert _count_ops(resolved, "tket.modifier.ControlModifier") == 0
    assert _count_ops(resolved, "tket.modifier.DaggerModifier") == 0


def test_modifier_execution() -> None:
    modified_hugrs_dir = Path("test_files/modified_hugrs")
    hugr_results_dir = Path("test_files/run_modifier_examples/hugr_results")
    run_hugrs_dir = Path("test_files/run_modifier_examples")

    expected_results = {
        expected_path.stem: np.load(expected_path).copy()
        for expected_path in sorted(hugr_results_dir.glob("*.npy"))
    }

    for hugr_path in sorted(modified_hugrs_dir.glob("*.hugr")):
        hugr_name = hugr_path.stem.removesuffix("_solved")
        expected_statevector = expected_results[hugr_path.stem]

        with tempfile.TemporaryDirectory() as tmp_dir:
            tmp_path = Path(tmp_dir) / f"{hugr_name}.npy"
            subprocess.run(
                [
                    "uv",
                    "run",
                    "--no-project",
                    "--python",
                    "3.13",
                    "run_hugrs.py",
                    hugr_name,
                    str(tmp_path),
                ],
                cwd=run_hugrs_dir,
                check=True,
            )

            computed_statevector = np.load(tmp_path)
            np.testing.assert_allclose(computed_statevector, expected_statevector)


def test_inline_functions() -> None:
    hugr = _hugr_from_path("test_files/guppy_examples/fn_calls.hugr")

    assert _count_ops(hugr, "Call") == 2

    max_size = InlineFunctions(heuristic=inline_funcs.MaxSize(42))(hugr)

    assert _count_ops(max_size, "Call") == 0

    all = InlineFunctions(heuristic=inline_funcs.All())(hugr)

    assert _count_ops(all, "Call") == 0


def test_issue_1516() -> None:
    """Regression test for issue 1516.

    This was caused by a bug in the decoder that injected new parameter inputs when decoding a modified pytket circuit back into an existing region.

    <https://github.com/quantinuum/tket2/issues/1516>
    """
    hugr = _hugr_from_path("test_files/guppy_examples/issue_1516.hugr")

    # Ensure that the hugr is valid before we start.
    CompilationState.from_python(hugr).validate()

    opt = PytketHugrPass(RemoveRedundancies()).with_scope(
        GlobalScope.PRESERVE_ENTRYPOINT
    )
    opt_hugr = opt(hugr, inplace=False)

    CompilationState.from_python(opt_hugr).validate()
