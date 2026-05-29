# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang @ git+https://github.com/Quantinuum/guppylang.git@na/1637-allow-unitary-flag-with-higher-order-function#subdirectory=guppylang",
# ]
# ///
"""A simple controlled gate using modifiers"""

from pathlib import Path
from sys import argv
import sys
from typing import Callable

from guppylang import guppy
from guppylang.std.builtins import (
    Controllable,
    Unitary,
    array,
    control,
    dagger,
)
from guppylang.std.debug import state_result
from guppylang.std.quantum import discard, discard_array, qubit, angle, rz
from guppylang.std.quantum import h, rx


sys.path.append(str(Path(__file__).resolve().parents[1]))

from guppylang.experimental import enable_experimental_features

enable_experimental_features()


@guppy(unitary=True)
def apply_c(
    g: Unitary[[qubit, angle], None],
    fun: Callable[[float], angle],
    q: qubit,
) -> None:
    a = fun(0.5)
    g(q, a)


@guppy
def fun(f: float) -> angle:
    return angle(f)


@guppy
def main() -> None:
    q = qubit()
    c = qubit()

    h(c)

    with control(c), dagger():
        apply_c(rz, fun, q)
        apply_c(rx, fun, q)

    state_result("r", c, q)
    discard(q)
    discard(c)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
