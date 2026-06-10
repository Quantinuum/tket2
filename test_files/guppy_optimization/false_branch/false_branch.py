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
from guppylang.std.quantum import h, measure, qubit


@guppy
def main() -> None:
    q = qubit()
    h(q)
    if True:
        h(q)
    if False:
        h(q)
    b = measure(q).read()

    result("b", b)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
