"""Run resolved modifier examples using the local QIS compiler and Selene."""

import shutil
import sys
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
import numpy.typing as npt
from hugr import Hugr
from selene_runner import run_hugr
from selene_sim.exceptions import SelenePanicError


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
    return "\n".join(parts) if parts else "all amplitudes below threshold"


modifier_examples_dir = Path(__file__).resolve().parent / "modified_hugrs"
result_execution_dir = Path(__file__).resolve().parent / "hugr_results"
all_results: list[str] = []
args = sys.argv[1:]
if len(args) > 2:
    raise SystemExit(f"Usage: {Path(sys.argv[0]).name} [hugr_name] [output_path]")

if args:
    requested_hugr = Path(args[0] + "_solved.hugr")
    hugr_path = requested_hugr
    if not hugr_path.is_absolute():
        hugr_path = modifier_examples_dir / requested_hugr
    hugr_paths = [hugr_path]
else:
    hugr_paths = sorted(modifier_examples_dir.glob("*.hugr"))

result_execution_dir.mkdir(parents=True, exist_ok=True)
for hugr_path in hugr_paths:
    print(f"Running {hugr_path.name}...")
    hugr = Hugr.from_bytes(hugr_path.read_bytes())
    expect_panic = hugr_path.stem == "panic_in_control_solved"
    try:
        with TemporaryDirectory() as build_dir:
            states = run_hugr(hugr, Path(build_dir), n_qubits=1 if expect_panic else 9)
    except SelenePanicError as err:
        if not expect_panic:
            raise
        message = str(err)
        if "first panic" not in message or "second panic" in message:
            raise
        print(f"\t{hugr_path.stem} panicked as expected: {message.splitlines()[0]}")
        continue
    if expect_panic:
        raise SystemExit(f"Expected {hugr_path.stem} to panic, but it completed.")

    res = states["r"].get_single_state()
    output_path = (
        Path(args[1])
        if len(args) >= 2
        else result_execution_dir / f"{hugr_path.stem}.npy"
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.save(output_path, res)
    all_results.append(f"{hugr_path.stem}:\n{format_statevector(res)}")

# Save the result to a text file for easy viewing.
if len(args) < 2:
    result_path = Path(__file__).resolve().parent / "hugr_results.txt"
    result_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.write_text("\n-----\n".join(all_results) + "\n")

if not args or not Path(args[0]).is_absolute():
    shutil.rmtree(modifier_examples_dir)
