"""Check modifier resolution by running the emitted bitcode in Selene."""

from pathlib import Path

import numpy as np
import pytest
import selene_sim
from hugr.hugr import Hugr
from hugr.package import Package
from numpy.typing import NDArray
from selene_hugr_qis_compiler import EmulatorState
from selene_quest_plugin import QuestPlugin
from selene_sim.exceptions import SelenePanicError
from selene_sim.instance import SeleneInstance
from tket._state import CompilationState
from tket.passes import ModifierResolverPass
from tket_exts import modifier, tket_registry

TEST_DIR = Path(__file__).resolve().parent
RESULTS_DIR = TEST_DIR / "hugr_results"
EXAMPLES_DIR = TEST_DIR.parent / "modifier_examples"
PANIC_EXAMPLE = "panic_in_control"


def _resolve_modifiers(name: str) -> Hugr:
    envelope = (EXAMPLES_DIR / f"{name}.hugr").read_bytes()
    (hugr,) = Package.from_bytes(envelope, tket_registry()).modules
    resolved = ModifierResolverPass()(hugr)
    modifier_prefix = f"{modifier().name}."
    assert all(
        not data.op.name().startswith(modifier_prefix) for _, data in resolved.nodes()
    )
    CompilationState.from_python(resolved).validate()
    return resolved


def _build(hugr: Hugr, build_dir: Path) -> SeleneInstance:
    bitcode = EmulatorState.from_python(hugr).compile_to_bitcode()
    return selene_sim.build(bitcode, build_dir=build_dir)


def _assert_equal_up_to_global_phase(actual: NDArray, expected: NDArray) -> None:
    phase = np.angle(np.vdot(expected, actual))
    np.testing.assert_allclose(
        actual * np.exp(-1j * phase), expected, atol=1e-8, rtol=0
    )


@pytest.mark.parametrize(
    "name",
    sorted(p.stem for p in EXAMPLES_DIR.glob("*.hugr") if p.stem != PANIC_EXAMPLE),
)
def test_modifier_execution(name: str, tmp_path: Path) -> None:
    runner = _build(_resolve_modifiers(name), tmp_path)
    results = runner.run(QuestPlugin(), n_qubits=9, random_seed=0, timeout=30.0)
    actual = QuestPlugin.extract_states_dict(results)["r"].get_single_state()
    expected = np.load(RESULTS_DIR / f"{name}_solved.npy")
    _assert_equal_up_to_global_phase(actual, expected)


def test_panic_in_control(tmp_path: Path) -> None:
    runner = _build(_resolve_modifiers(PANIC_EXAMPLE), tmp_path)
    with pytest.raises(SelenePanicError, match="first panic") as exc:
        list(runner.run(QuestPlugin(), n_qubits=1, timeout=30.0))
    assert "second panic" not in str(exc.value)
