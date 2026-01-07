# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang >=0.21.6",
# ]
# ///

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.quantum import cx, qubit
from guppylang.std.builtins import array


@guppy
def main(arr: array[array[qubit, 5], 3]) -> None:
    cx(arr[0][1], arr[0][2])
    cx(arr[0][1], arr[0][2])


program = main.compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
