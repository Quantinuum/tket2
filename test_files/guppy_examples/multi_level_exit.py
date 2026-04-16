# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang >=0.21.6",
# ]
# ///
"""A Guppy example with nested loops and a multi-level early return."""

from pathlib import Path
from sys import argv

from guppylang import guppy


@guppy
def multi_level_exit(n: int, m: int) -> int:
    total = 0

    while n > 0:
        inner = m

        while inner > 0:
            if total > 20:
                return total

            if inner == 1:
                break

            total += inner
            inner -= 1

        if total < 5:
            n -= 1
            continue

        total += n
        n -= 1

    return total


program = multi_level_exit.compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
