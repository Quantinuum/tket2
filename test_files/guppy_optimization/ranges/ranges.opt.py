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
from guppylang.std.builtins import result
from guppylang.std.quantum import measure, qubit


@guppy
def main() -> None:
    q1, q2, q3, q4 = qubit(), qubit(), qubit(), qubit()
    b1 = measure(q1)
    b2 = measure(q2)
    b3 = measure(q3)
    b4 = measure(q4)

    result("b1", b1.read())
    result("b2", b2.read())
    result("b3", b3.read())
    result("b4", b4.read())


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
