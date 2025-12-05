# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang >=0.21.6",
# ]
# ///

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.angles import angle
from guppylang.std.quantum import rx, qubit


@guppy.declare
def func1() -> float: ...


@guppy.declare
def func2(f: float) -> float: ...


@guppy
def unknown_rotations(q: qubit) -> None:
    rotation = func1()
    rx(q, angle(rotation))

    other_rotation = func2(rotation + 1.0)
    rx(q, angle(other_rotation))


program = unknown_rotations.compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
