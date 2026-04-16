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
from guppylang.std.builtins import result
from guppylang.std.quantum import h, measure, qubit


@guppy
def main(a: bool, b: bool) -> None:
    q = qubit()
    if b or not a:
        h(q)

    result("b", measure(q))


program = main.compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
