# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.2.1",
# ]
# ///
"""Exercise power modifiers with a small, predictable final state."""

from pathlib import Path
from sys import argv

from guppylang import enable_experimental_features, guppy
from guppylang.std.angles import pi
from guppylang.std.builtins import array, control, dagger, frozenarray, panic, power
from guppylang.std.debug import state_result
from guppylang.std.num import nat
from guppylang.std.quantum import discard_array, h, qubit, ry, s, x

enable_experimental_features()

FLIPS = [0, 1]


@guppy.struct
class Settings:
    exponent: nat
    q_state: bool


@guppy
def fun(q: qubit, settings: Settings, flips: frozenarray[int, 2]) -> None:
    if flips[1] == 1:
        x(q)
        settings.q_state = not settings.q_state


@guppy(unitary=True)
def phases[N: nat](q: array[qubit, N], exponent: nat, index: array[int, 1]) -> None:
    with power(exponent):
        s(q[index[0]])


@guppy
def main() -> None:
    qs = array(qubit() for _ in range(3))
    settings = Settings(nat(3), False)
    flips = FLIPS

    with power(0):
        fun(qs[1], settings, flips)
    # |000>

    # X**3 = X: qs[1] goes |0> -> |1>,
    with power(3):
        fun(qs[1], settings, flips)
    # |0101>
    if not settings.q_state:
        panic("q_state not changed")

    # X**12 = I: qs[1] stays |1>, caller's index stays at 0.
    with power(4), power(settings.exponent):
        fun(qs[1], settings, flips)
    # |010>
    if not settings.q_state:
        panic("q_state changed")

    # (H**9)† = H: prepare a superposition on qs[0].
    with power(3):  # noqa: SIM117
        with dagger:
            with power(settings.exponent):
                h(qs[0])
    # 0.707107|010> + 0.707107|110>

    index = array(0)
    # (S**3)† = S: multiply the qs[0] = |1> component by i.
    with dagger:
        phases(qs, settings.exponent, index)
    # (|010> + i|110>) / sqrt(2).

    # Controlled Ry(pi/6)**3 = controlled Ry(pi/2): qs[2] goes |0> -> |+>.
    with power(settings.exponent):  # noqa: SIM117
        with control(qs[1]):
            ry(qs[2], pi / 6)
            index[0] += 1
    # State: (|010> + |011> + i|110> + i|111>) / 2.

    index[0] -= 1
    if index[0] != 2:
        panic("index not 2")

    # Controlled S†: multiply the qs[2] = |1> components by -i.
    with control(qs[1]), dagger:
        phases(qs, nat(1), index)
    # State: (|010> - i|011> + i|110> + |111>) / 2.
    # Expected amplitudes: 010 -> 0.5, 011 -> -0.5j, 110 -> 0.5j, 111 -> 0.5.
    state_result("r", qs[0], qs[1], qs[2])
    discard_array(qs)


program = main.with_minimal_opt().compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
