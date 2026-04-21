# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang ==0.21.13",
# ]
# ///
"""Regression test for https://github.com/Quantinuum/tket2/issues/1516 """

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.builtins import owned
from guppylang.std.quantum import qubit
from guppylang.std.quantum.functional import ch

@guppy
def chfunc(q1: qubit @ owned, q2: qubit @ owned) -> tuple[qubit, qubit]:
    q1, q2 = ch(q1, q2)
    return (q1, q2)


program = chfunc.compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
