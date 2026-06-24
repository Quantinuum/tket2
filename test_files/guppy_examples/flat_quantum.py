# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang==1.0.0a5",
#     "guppylang-internals==1.0.0a5",
# ]
# ///
from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.quantum import qubit, h, cx, rz
from guppylang.std.angles import pi


@guppy
def flat_quantum_func(q0: qubit, q1: qubit) -> None:
    h(q0)
    rz(q0, 3 * pi / 4)
    cx(q0, q1)


program = flat_quantum_func.compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
