# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.0a8",
# ]
# [tool.uv.sources]
# guppylang = {git = "https://github.com/quantinuum/guppylang", subdirectory = "guppylang", branch = "main"}
# ///
"""Subscript indexing in dagger and control context"""

from pathlib import Path
from sys import argv
import sys
from typing import Callable

import tket

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
from guppylang.std.builtins import Function


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
def inner(mk_struct: "Function[[int], int]", x: int) -> int:
    return mk_struct(x)


@guppy
def foo(i: int) -> int:
    return i + 1


@guppy
def main() -> None:
    q = qubit()
    c = qubit()

    h(c)
    with dagger:
        inner(foo, 2)

    with control(c), dagger():
        apply_c(rz, fun, q)
        apply_c(rx, fun, q)

    state_result("r", c, q)
    discard(q)
    discard(c)


modifier_resolver = tket.passes.ModifierResolverPass()
program = (
    main.with_minimal_opt().compile()
)  # .with_optimization(modifier_resolver).compile()
Path(argv[0]).with_suffix(suffix=".hugr").write_bytes(program.to_bytes())
program.modules[0].render_dot().view("tmp")
