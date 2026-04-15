# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang ==0.21.10",
# ]
# ///
"""Control flow with boolean short-circuiting"""

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.builtins import array, owned, result
from guppylang.std.quantum import cz, h, measure, qubit


@guppy
def main() -> None:
    a = True
    b = True

    q = qubit()
    # Apply loops and unpack the result array
    i = 0
    for i in range(4):
        h(q)

        i += 1
        if i > 4 or not a:
            break

    result("b", measure(q))


program = main.compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
