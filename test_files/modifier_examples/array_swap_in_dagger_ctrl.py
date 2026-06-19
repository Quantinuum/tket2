# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.0a5",
#    "guppylang-internals==1.0.0a5",
# ]
# ///
"""Test that array swap in a dagger and control context works correctly"""

from pathlib import Path
from sys import argv

import guppylang
from guppylang import guppy
from guppylang.std.array import array_swap
from guppylang.std.builtins import array, control, dagger
from guppylang.std.debug import state_result
from guppylang.std.quantum import discard, h, qubit

guppylang.enable_experimental_features()


@guppy
def main() -> None:
    arr = array(1, 1, 2, 1, 1)
    q = qubit()
    with dagger:
        array_swap(arr, 2, 4)
        with control(q):
            array_swap(arr, 0, 4)
    if arr[0] == 2:
        h(q)
    state_result("r", q)
    discard(q)


program = main.compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
