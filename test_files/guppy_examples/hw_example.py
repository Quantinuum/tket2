# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang ==0.21.10",
# ]
# ///

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.angles import angle
from guppylang.std.builtins import qubit
from guppylang.std.qsystem import measure, rz, phased_x


@guppy
def main(b: bool, x: int) -> bool:
    q = qubit()
    phi = angle(0.5)

    rz(q, phi)

    if b or (x > 0):
        rz(q, phi)

    return measure(q)


program = main.compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
