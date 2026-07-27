# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang ==0.21.13",
# ]
# ///

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.quantum import cx, measure, qubit
from guppylang.std.builtins import result


@guppy
def main() -> None:
    q1, q2 = qubit(), qubit()
    cx(q1, q2)
    cx(q1, q2)
    b1 = measure(q1)
    b2 = measure(q2)

    result("b1", b1)
    result("b2", b2)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
