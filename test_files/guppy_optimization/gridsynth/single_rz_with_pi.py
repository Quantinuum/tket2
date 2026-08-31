# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang==1.0.1",
#     "guppylang-internals==1.0.1",
# ]
# ///

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.angles import pi
from guppylang.std.quantum import qubit, rz, discard


@guppy
def main() -> None:
    q = qubit()
    rz(q, pi / 2)
    discard(q)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
