# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.0rc1",
# ]
# ///
"""Test the use of a higher-order function with complex control flow inside modifiers"""

from pathlib import Path
from sys import argv
from collections.abc import Callable

from guppylang import enable_experimental_features, guppy
from guppylang.std.builtins import Controllable, Unitary, array, control, dagger
from guppylang.std.debug import state_result
from guppylang.std.lang import Function
from guppylang.std.quantum import angle, discard_array, h, qubit, rx, rz

enable_experimental_features()


@guppy
def get_angle(f: float) -> angle:
    return angle(f)


@guppy
def get_get_angle() -> Function[[float], angle]:
    return get_angle


@guppy(unitary=True)
def apply_r(
    f: Unitary[[qubit, angle], None],
    q: array[qubit, 2],
    fun_angle: Callable[[float], angle],
    radiant: float,
) -> None:
    f(q[1], fun_angle(radiant))


@guppy(controllable=True)
def apply_c(
    f: Controllable[[qubit], None],
    g: Unitary[[qubit, angle], None],
    classic_fun: Function[[], Function[[float], angle]],
    q: qubit,
    b: bool,
) -> None:
    n = 3
    if b:
        while n > 0:
            f(q)
            n -= 1
    else:
        get_a = classic_fun()
        angle = get_a(0.25)
        for _ in range(2):
            g(q, get_a(0.25))
            g(q, angle)


@guppy
def main() -> None:
    qs: array[qubit, 2] = array(qubit(), qubit())
    h(qs[0])
    flag = 2 > 10
    with control(qs[0]):
        apply_c(h, rx, get_get_angle, qs[1], True)
        apply_c(h, rx, get_get_angle, qs[1], flag)

    with control(qs[0]), dagger:
        apply_r(rz, qs, get_angle, 0.25)
        apply_r(rz, qs, get_angle, 0.5)

    state_result("r", qs[0], qs[1])
    discard_array(qs)


program = main.with_minimal_opt().compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
