# /// script
# requires-python = ">=3.14"
# dependencies = [
#     "guppylang==0.21.13",
# ]
# [tool.uv.sources]
# guppylang = {git = "https://github.com/quantinuum/guppylang", subdirectory = "guppylang", branch = "ts/future-measure"}
# ///

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.builtins import owned
from guppylang.std.quantum import qubit
from guppylang.std.quantum.functional import ch


@guppy
def test(q1: qubit @ owned, q2: qubit @ owned) -> tuple[qubit, qubit]:
    q1, q2 = ch(q1, q2)
    return (q1, q2)


program = test.compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
