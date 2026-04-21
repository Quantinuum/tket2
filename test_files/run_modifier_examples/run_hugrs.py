# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang ==0.21.11",
# ]
# ///
"""Run on selene the passed hugrs"""

from pathlib import Path
import sys
import numpy as np
import numpy.typing as npt

from hugr import Hugr
from hugr.hugr.render import RenderConfig
from guppylang.emulator import EmulatorBuilder

sys.path.append(str(Path(__file__).resolve().parents[1]))
from utility import hugr_pdf_directory


def format_statevector(
    state: npt.NDArray[np.complexfloating], threshold: float = 1e-6
) -> str:
    """Pretty-print a statevector, omitting amplitudes below *threshold*.

    Each basis state is shown as a zero-padded binary string, e.g.::

        000 -> 0.7071+0.j, 111 -> 0.7071+0.j
    """
    n_qubits = int(np.round(np.log2(len(state))))
    parts = []
    for idx, amp in enumerate(state):
        if abs(amp) > threshold:
            label = format(idx, f"0{n_qubits}b")
            parts.append(f"\t{label} -> {amp:.4g}")
    return "\n".join(parts) if parts else "(all zero)"


EXPECTED_RESULTS: dict[str, dict[str, complex]] = {
    "ctrl_on_call2_solved": {
        "0110": 0.7071 + 0j,
        "1110": 0.6124 + 0j,
        "1111": 0 - 0.3536j,
    },
    "ctrl_on_call_solved": {
        "00": 0.7071 + 0j,
        "11": 0.7071 + 0j,
    },
    "ctrl_on_x_solved": {
        "00": 0.7071 + 0j,
        "11": 0.7071 + 0j,
    },
    "dagger_on_call_solved": {
        "0": 0.866 + 0j,
        "1": 0 + 0.5j,
    },
    # "double_call_solved": {
    #     "00": 0.7071 + 0j,
    #     "10": 0.6124 + 0j,
    #     "11": 0 - 0.3536j,
    # },
}


def assert_statevector(
    name: str,
    state: npt.NDArray[np.complexfloating],
    atol: float = 1e-3,
) -> None:
    """Assert that *state* matches the expected amplitudes for *name*, if known."""
    expected = EXPECTED_RESULTS.get(name)
    if expected is None:
        return
    n_qubits = int(np.round(np.log2(len(state))))
    for label, exp_amp in expected.items():
        idx = int(label, 2)
        actual = state[idx]
        if not np.isclose(actual, exp_amp, atol=atol):
            raise AssertionError(
                f"{name}: basis state |{label}⟩ — "
                f"expected {exp_amp:.4g}, got {actual:.4g}"
            )
    for idx, actual in enumerate(state):
        label = format(idx, f"0{n_qubits}b")
        if label not in expected and not np.isclose(actual, 0, atol=atol):
            raise AssertionError(
                f"{name}: basis state |{label}⟩ — expected 0, got {actual:.4g}"
            )


modifier_examples_dir = Path(__file__).resolve().parents[1] / "modified_hugrs"

print(modifier_examples_dir)
all_results: list[str] = []
if len(sys.argv) > 1:
    requested_hugr = Path(sys.argv[1] + "_solved.hugr")
    hugr_path = requested_hugr
    if not hugr_path.is_absolute():
        hugr_path = modifier_examples_dir / requested_hugr
    hugr_paths = [hugr_path]
else:
    hugr_paths = sorted(modifier_examples_dir.glob("*.hugr"))

for hugr_path in hugr_paths:
    print(f"Processing {hugr_path}...")
    hugr_bytes = hugr_path.read_bytes()
    hugr = Hugr.from_bytes(hugr_bytes)
    hugr.render_dot(RenderConfig(display_node_id=True)).render(
        f"{hugr_path.stem}", directory=hugr_pdf_directory, cleanup=True
    )

    package = hugr.to_package()

    builder = EmulatorBuilder()
    emulator = builder.build(package, n_qubits=8)
    state = emulator.statevector_sim().run()
    res = state.partial_state_dicts()[0]["r"].as_single_state()
    assert_statevector(hugr_path.stem, res)
    all_results.append(f"{hugr_path.stem}:\n{format_statevector(res)}")

result_path = Path("hugr_results.txt")
result_path.write_text("\n-----\n".join(all_results) + "\n")
print(f"Results saved to {result_path}")
