import importlib.util
import tempfile

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
from tket_exts import modifier, tket_registry

from tket._pattern import Rule, RuleMatcher
import hypothesis.strategies as st
from hypothesis.strategies._internal import SearchStrategy
from hypothesis import given, settings

from tket.passes import (
    PytketHugrPass,
    _QSystemLLVMPass,
    QSystemRebasePass,
    PlatformTarget,
)
from hugr.build.base import Hugr
from hugr.package import Package

import numpy as np
import pytest
from pathlib import Path

# Import the pytket passes, if the `pytket` extra has been installed.
# If not, skip all tests in this file.
pytket = pytest.importorskip("pytket")
from pytket import Circuit, OpType  # noqa: E402
from pytket.passes import (  # noqa: E402
    CliffordSimp,
    SquashRzPhasedX,
    RemoveRedundancies,
    SequencePass,
)


normalize = NormalizeGuppy()


def _hugr_from_path(str_path: str) -> Hugr:
    with open(Path(str_path), "rb") as f:
        h = Package.from_bytes(f.read())

    return h.modules[0]


def _count_ops(hugr: Hugr, op_string_name: str) -> int:
    count = 0
    for _, data in hugr.nodes():
        if op_string_name in data.op.name():
            count += 1

    return count


def _contains_modifiers(module: Hugr) -> bool:
    for _, node_data in module.nodes():
        if (
            modifier.control.qualified_name() in node_data.op.name()
            or modifier.dagger.qualified_name() in node_data.op.name()
        ):
            return True

    return False


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

    match_count = matcher.apply_exhaustive(circ._inner)

    assert match_count == 3

    out = circ.to_tket1()
    assert out == Circuit(3).CX(0, 1).X(0)


def test_apply_exhaustive_reaches_fixed_point() -> None:
    circ = CompilationState.from_tket1(Circuit(3).H(0).H(0).H(1).H(1).H(2).H(2))

    rule = Rule(
        CompilationState.from_tket1(Circuit(1).H(0).H(0))._inner,
        CompilationState.from_tket1(Circuit(1).X(0))._inner,
    )
    matcher = RuleMatcher([rule])

    rewrite_count = matcher.apply_exhaustive(circ._inner)

    assert rewrite_count == 3
    assert circ.to_tket1() == Circuit(3).X(0).X(1).X(2)

    assert matcher.apply_exhaustive(circ._inner) == 0


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
    squash_pass = PytketHugrPass(SquashRzPhasedX(), target=PlatformTarget.Tket)
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


@pytest.mark.parametrize(
    ("target", "expected_rz"),
    [
        (PlatformTarget.Tket, "tket.quantum.Rz"),
        (PlatformTarget.Sol, "tket.qsystem.sol.Rz"),
        (PlatformTarget.Helios, "tket.qsystem.helios.Rz"),
    ],
)
def test_platform_target_decoding(target: PlatformTarget, expected_rz: str):
    """The platform target controls which extension the ambiguous `Rz`
    operation is decoded into."""
    c = CompilationState.from_tket1(Circuit(1).Rz(0.25, 0).Rz(0.25, 0))
    hugr = Hugr.from_str(c.to_str(), tket_registry())

    res = PytketHugrPass(RemoveRedundancies(), target=target).run(hugr)

    assert _count_ops(res.hugr, expected_rz) == 1


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
    normalize = NormalizeGuppy(resolve_modifiers=False)
    normalize_with_modifier_resolution = NormalizeGuppy()
    mr_pass = ModifierResolverPass()
    # We consider a simple hugr for this test
    modifier_hugr: Hugr = _hugr_from_path(
        "test_files/modifier_examples/double_modifier.hugr"
    )

    normalized_and_resolved: Hugr = normalize_with_modifier_resolution(modifier_hugr)
    assert _count_ops(normalized_and_resolved, "tket.modifier.ControlModifier") == 0
    assert _count_ops(normalized_and_resolved, "tket.modifier.DaggerModifier") == 0

    modifier_hugr = _hugr_from_path("test_files/modifier_examples/double_modifier.hugr")
    modifier_hugr = normalize(modifier_hugr)

    assert _count_ops(modifier_hugr, "tket.modifier.ControlModifier") == 1
    assert _count_ops(modifier_hugr, "tket.modifier.DaggerModifier") == 1

    resolved: Hugr = mr_pass(modifier_hugr)

    assert _count_ops(resolved, "tket.modifier.ControlModifier") == 0
    assert _count_ops(resolved, "tket.modifier.DaggerModifier") == 0


# This test uses downstream selene to execute and verify the result of the modifier resolver pass.
#
# That's problematic when updating hugr/tket, as we can only use a selene executor that knows nothing
# about the changes.
#
# TODO: Replace with a local mini-executor test. <https://github.com/Quantinuum/tket2/issues/1648>
@pytest.mark.skip(reason="Uses downstream dependencies, breaks with tket changes.")
def test_modifier_execution() -> None:
    modifier_examples_dir = Path("test_files/modifier_examples")
    hugr_results_dir = Path("test_files/run_modifier_examples/hugr_results")
    run_hugrs_dir = Path("test_files/run_modifier_examples")
    apply_passes_path = run_hugrs_dir / "apply_passes.py"
    spec = importlib.util.spec_from_file_location(
        "run_modifier_examples_apply_passes", apply_passes_path
    )
    assert spec is not None
    assert spec.loader is not None
    apply_passes_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(apply_passes_module)
    apply_passes = apply_passes_module.apply_passes

    expected_results = {
        expected_path.stem: np.load(expected_path).copy()
        for expected_path in sorted(hugr_results_dir.glob("*.npy"))
    }
    for hugr_path in sorted(modifier_examples_dir.glob("*.hugr")):
        hugr_name = hugr_path.stem
        expected_statevector = expected_results[f"{hugr_name}_solved"]

        with tempfile.TemporaryDirectory() as tmp_dir:
            generated_hugrs_dir = Path(tmp_dir) / "modified_hugrs"
            generated_hugrs_dir.mkdir()
            apply_passes([hugr_path], generated_hugrs_dir)

            (run_hugrs_dir / "modified_hugrs").mkdir(exist_ok=True)
            tmp_path = Path(tmp_dir) / f"{hugr_name}.npy"
            subprocess.run(
                [
                    "uv",
                    "run",
                    "--no-project",
                    "--python",
                    "3.13",
                    "run_hugrs.py",
                    str((generated_hugrs_dir / hugr_name).resolve()),
                    str(tmp_path),
                ],
                cwd=run_hugrs_dir,
                check=True,
            )

            computed_statevector = np.load(tmp_path)
            np.testing.assert_allclose(computed_statevector, expected_statevector)


def test_normalize_guppy_on_modifier() -> None:
    """Test the normalize_guppy pass on a hugr with modifiers.

    This won't actually do anything useful, we just want to check that the pass
    runs without errors."""
    normalize = NormalizeGuppy()
    for hugr_path in sorted(Path("test_files/modifier_examples").glob("*.hugr")):
        try:
            normalized = normalize(_hugr_from_path(str(hugr_path)))
            CompilationState.from_python(normalized).validate()
        except Exception as exc:
            raise AssertionError(f"NormalizeGuppy failed for {hugr_path}") from exc
        assert not _contains_modifiers(normalized), (
            f"NormalizeGuppy left modifiers in {hugr_path}"
        )


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


def test_python_qsystem_pass() -> None:
    normalize = NormalizeGuppy()
    hugr = normalize(_hugr_from_path("test_files/guppy_examples/flat_quantum.hugr"))
    qsystem_rebase = QSystemRebasePass()
    qsystem_llvm = _QSystemLLVMPass()
    qsystem_hugr = qsystem_llvm(qsystem_rebase(hugr))
    assert _count_ops(qsystem_hugr, "ZZPhase") == 1
    assert _count_ops(qsystem_hugr, "Custom") == 0
    assert _count_ops(qsystem_hugr, "tket.quantum") == 0


def test_python_qsystem_pass_with_modifiers() -> None:
    """Test that the QSystem passes work on hugrs with modifiers.

    This won't actually do anything useful, we just want to check that the pass
    runs without errors."""
    qsystem_rebase = QSystemRebasePass()
    qsystem_llvm = _QSystemLLVMPass()
    for hugr_path in sorted(Path("test_files/modifier_examples").glob("*.hugr")):
        try:
            qsystem_hugr = qsystem_llvm(qsystem_rebase(_hugr_from_path(str(hugr_path))))
            CompilationState.from_python(qsystem_hugr).validate()
        except Exception as exc:
            raise AssertionError(f"QSystem passes failed for {hugr_path}") from exc
        assert not _contains_modifiers(qsystem_hugr), (
            f"QSystem passes left modifiers in {hugr_path}"
        )
