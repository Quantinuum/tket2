# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang ==0.21.11",
# ]
# ///
"""A conditional branch inside a loop"""

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.builtins import control
from guppylang.std.quantum import qubit, x, z

from guppylang.experimental import enable_experimental_features

enable_experimental_features()


@guppy
def control_zx(q0: qubit, q1: qubit) -> None:
    with control(q0):
        z(q1)
        x(q1)


program = control_zx.compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
