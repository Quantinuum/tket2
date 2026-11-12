# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.1.1",
# ]
# ///
"""Exercise power modifiers with a small, predictable final state."""

from pathlib import Path
from sys import argv

from guppylang import enable_experimental_features, guppy
from guppylang.std.angles import pi
from guppylang.std.builtins import array, control, dagger, frozenarray, power
from guppylang.std.debug import state_result
from guppylang.std.num import nat
from guppylang.std.quantum import discard_array, h, qubit, ry, s, x

enable_experimental_features()

FLIPS = [0, 1]


@guppy.struct
class Settings:
    exponent: nat
    index: nat


@guppy
def fun(q: qubit, settings: Settings, flips: frozenarray[int, 2]) -> None:
    settings.index = nat(1 - int(settings.index))
    if flips[settings.index] == 1:
        x(q)


@guppy(unitary=True)
def phases(q: qubit, exponent: nat) -> None:
    # S**exponent: |0> -> |0>, |1> -> i**exponent |1>.
    with power(exponent):
        s(q)


@guppy
def main() -> None:
    qs = array(qubit() for _ in range(3))
    settings = Settings(nat(3), nat(0))
    flips = FLIPS

    # Zero repetitions: qs[1] stays |0>, index stays at 0.
    with power(0):
        fun(qs[1], settings, flips)

    # Indices 1, 0: X, I; qs[1] goes |0> -> |1>, index ends at 0.
    with power(2):
        fun(qs[1], settings, flips)

    # Indices (1, 0)**6: (X, I)**6 = I; qs[1] stays |1>, index ends at 0.
    with power(4), power(settings.exponent):
        fun(qs[1], settings, flips)

    # State (qs[0], qs[1], qs[2]): |010>.

    # (H**9)† = H: prepare a superposition on qs[0].
    with power(3):  # noqa: SIM117
        with dagger:
            with power(settings.exponent):
                h(qs[0])

    # (S**3)† = S: multiply the qs[0] = |1> component by i.
    with dagger:
        phases(qs[0], settings.exponent)

    # State: (|010> + i|110>) / sqrt(2).

    # Controlled Ry(pi/6)**3 = controlled Ry(pi/2): qs[2] goes |0> -> |+>.
    with power(settings.exponent):  # noqa: SIM117
        with control(qs[1]):
            ry(qs[2], pi / 6)

    # State: (|010> + |011> + i|110> + i|111>) / 2.

    # Controlled S†: multiply the qs[2] = |1> components by -i.
    with control(qs[1]), dagger:
        phases(qs[2], nat(1))

    # State: (|010> - i|011> + i|110> + |111>) / 2.

    # Expected amplitudes: 010 -> 0.5, 011 -> -0.5j, 110 -> 0.5j, 111 -> 0.5.
    state_result("r", qs[0], qs[1], qs[2])
    discard_array(qs)


program = main.with_minimal_opt().compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
