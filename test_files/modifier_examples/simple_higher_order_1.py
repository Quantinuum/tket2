# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.0a5",
#    "guppylang-internals==1.0.0a5",
# ]
# ///
"""Dagger of a swap on an array"""

from pathlib import Path
from sys import argv
import sys

from guppylang import guppy
from guppylang.std.array import array_swap
from guppylang.std.quantum import discard, qubit, h
from guppylang.std.builtins import array
import guppylang
from guppylang.std.builtins import dagger
from guppylang.std.debug import state_result
from guppylang.std.builtins import (
    Controllable,
    Unitary,
    array,
    control,
    dagger,
)

sys.path.append(str(Path(__file__).resolve().parents[1]))


guppylang.enable_experimental_features()


@guppy(controllable=True)
def apply_control(f: Controllable[[qubit], None], ctrl: qubit, q: qubit) -> None:
    with control(ctrl):
        f(q)


@guppy
def main() -> None:
    ctrl = qubit()
    h(ctrl)
    q = qubit()
    apply_control(h, ctrl, q)

    state_result("r", ctrl, q)
    discard(q)
    discard(ctrl)


program = main.compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
