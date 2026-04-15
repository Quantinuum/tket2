# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang >=0.21.6",
# ]
# ///
"""A Guppy Bell-state example with no explicit function inputs or outputs."""

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.builtins import result
from guppylang.std.quantum import cx, h, measure, qubit


@guppy
def bell_test() -> None:
    """Create a |00> + |11> Bell state and measure both qubits."""
    q1 = qubit()
    q2 = qubit()
    h(q1)
    cx(q1, q2)
    result("q1", measure(q1))
    result("q2", measure(q2))


program = bell_test.compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
