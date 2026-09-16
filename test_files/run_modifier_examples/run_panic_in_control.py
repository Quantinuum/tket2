# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.0rc1",
# ]
# ///
"""Run a solved `panic_in_control` hugr and assert it aborts on the first panic.

Like `run_hugrs.py`, this loads a resolved (`*_solved.hugr`) example, builds an
emulator from its package, and runs it. The run is expected to abort on the
first panic ("first panic") and never reach the second ("second panic").

Usage::

    run_panic_in_control.py [hugr_name_or_path]

If no argument is given, `modified_hugrs/panic_in_control_solved.hugr` is used.
"""

import sys
from pathlib import Path

from guppylang.emulator import EmulatorBuilder
from hugr import Hugr

modified_hugrs_dir = Path(__file__).resolve().parent / "modified_hugrs"

args = sys.argv[1:]
if len(args) > 1:
    raise SystemExit(f"Usage: {Path(sys.argv[0]).name} [hugr_name_or_path]")

if args:
    hugr_path = Path(args[0] + "_solved.hugr")
    if not hugr_path.is_absolute():
        hugr_path = modified_hugrs_dir / hugr_path
else:
    hugr_path = modified_hugrs_dir / "panic_in_control_solved.hugr"

hugr = Hugr.from_bytes(hugr_path.read_bytes())
package = hugr.to_package()
emulator = EmulatorBuilder().build(package, n_qubits=1)

try:
    emulator.statevector_sim().run()
except Exception as err:  # noqa: BLE001
    message = str(err)
    assert "first panic" in message, f"expected the first panic, got: {message!r}"
    assert "second panic" not in message.replace("first panic", ""), (
        f"reached the second panic unexpectedly: {message!r}"
    )
    print(f"{hugr_path.stem} panicked as expected: {message}")
else:
    raise AssertionError(f"expected {hugr_path.stem} to panic, but it completed")
