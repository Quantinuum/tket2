# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.1",
# ]
# ///
"""Test modifiers with panics"""

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.builtins import control, panic
from guppylang.std.quantum import measure, qubit


@guppy
def bar() -> None:
    q = qubit()
    with control(q):
        panic("first panic")
    panic("second panic")
    measure(q)


program = bar.with_minimal_opt().compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
