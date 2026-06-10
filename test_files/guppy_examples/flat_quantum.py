# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang ==0.21.13",
# ]
# [tool.uv.sources]
# guppylang = {git = "https://github.com/quantinuum/guppylang", subdirectory = "guppylang", branch = "ts/future-measure"}
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
