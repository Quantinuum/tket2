# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang >=0.21.6",
# ]
# ///
"""Loop-and-branch Guppy control-flow example."""

from pathlib import Path
from sys import argv

from guppylang import guppy


@guppy
def loop_and_branch(b1: bool, b2: bool) -> int:
    res = 0

    while b1:
        if not b2:
            res += 1
        else:
            res += 2
        b1 = res < 5

    if not b2:
        res *= 2
    else:
        res *= 3

    return res


program = loop_and_branch.compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
