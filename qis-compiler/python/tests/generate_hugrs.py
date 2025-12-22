# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "guppylang ==0.21.7",
#     "tket",
# ]
# ///

from pathlib import Path

from guppylang import guppy
from guppylang.std.builtins import array, exit, panic, result
from guppylang.std.qsystem.random import RNG
from guppylang.std.qsystem.utils import get_current_shot
from guppylang.std.quantum import (
    cx,
    discard,
    discard_array,
    h,
    measure,
    measure_array,
    qubit,
    t,
    tdg,
    x,
    z,
)

resources_dir = Path(__file__).parent / "resources"


def check() -> bytes:
    @guppy
    def foo() -> None:
        q = qubit()
        discard(q)

    return foo.compile().to_bytes()


def unsupported_pytket_ops() -> bytes:
    from pytket._tket.circuit import Circuit

    # A pytket circuit with an unsupported op.
    circ = Circuit(2).CSXdg(0, 1)

    guppy_circ = guppy.load_pytket("guppy_circ", circ, use_arrays=False)

    @guppy
    def foo() -> None:
        a, b = qubit(), qubit()
        guppy_circ(a, b)
        discard(a)
        discard(b)

    return foo.compile().to_bytes()


def no_results() -> bytes:
    @guppy
    def bar() -> None:
        q0: qubit = qubit()
        h(q0)
        measure(q0)

    return bar.compile().to_bytes()


def flip_some() -> bytes:
    @guppy
    def main() -> None:
        q0: qubit = qubit()
        q1: qubit = qubit()
        q2: qubit = qubit()
        q3: qubit = qubit()
        x(q0)
        x(q2)
        x(q3)
        result("c0", measure(q0))
        result("c1", measure(q1))
        result("c2", measure(q2))
        result("c3", measure(q3))

    return main.compile().to_bytes()


def discard_qb_array() -> bytes:
    @guppy
    def main() -> None:
        qs = array(qubit() for _ in range(10))
        discard_array(qs)

    return main.compile().to_bytes()


def measure_qb_array() -> bytes:
    @guppy
    def main() -> None:
        qs = array(qubit() for _ in range(10))
        x(qs[0])
        x(qs[2])
        x(qs[3])
        x(qs[9])
        measure_array(qs)

    return main.compile().to_bytes()


def print_array() -> bytes:
    # TODO: Currently fails due to <https://github.com/Quantinuum/guppylang/issues/1408>.
    @guppy
    def main() -> None:
        qs = array(qubit() for _ in range(10))
        x(qs[0])
        x(qs[2])
        x(qs[3])
        x(qs[9])
        cs = measure_array(qs)
        result("cs", cs)
        result("is", array(i for i in range(100)))
        result("fs", array(i * 0.0625 for i in range(100)))

    return main.compile().to_bytes()


def postselect_exit() -> bytes:
    @guppy
    def main() -> None:
        q = qubit()
        h(q)
        outcome = measure(q)
        if outcome:
            exit("Postselection failed", 42)
        result("c", outcome)

    return main.compile().to_bytes()


def postselect_panic() -> bytes:
    @guppy
    def main() -> None:
        q = qubit()
        h(q)
        outcome = measure(q)
        if outcome:
            panic("Postselection failed")
        result("c", outcome)

    return main.compile().to_bytes()


def rus() -> bytes:
    @guppy
    def rus(q: qubit) -> None:
        while True:
            # Prepare ancillary qubits
            a, b = qubit(), qubit()
            h(a)
            h(b)

            tdg(a)
            cx(b, a)
            t(a)
            if not measure(a):
                # First part failed; try again
                discard(b)
                continue

            t(q)
            z(q)
            cx(q, b)
            t(b)
            if measure(b):
                # Success, we are done
                break

            # Otherwise, apply correction
            x(q)

    @guppy
    def main() -> None:
        q = qubit()
        rus(q)
        result("result", measure(q))

    return main.compile().to_bytes()


def print_current_shot() -> bytes:
    @guppy
    def main() -> None:
        result("shot", get_current_shot())

    return main.compile().to_bytes()


def rng() -> bytes:
    @guppy
    def main() -> None:
        rng = RNG(42)
        rint = rng.random_int()
        rint1 = rng.random_int()
        rfloat = rng.random_float()
        rint_bnd = rng.random_int_bounded(100)
        rng.discard()
        result("rint", rint)
        result("rint1", rint1)
        result("rfloat", rfloat)
        result("rint_bnd", rint_bnd)
        rng = RNG(84)
        rint = rng.random_int()
        rfloat = rng.random_float()
        rint_bnd = rng.random_int_bounded(200)
        rng.discard()
        result("rint2", rint)
        result("rfloat2", rfloat)
        result("rint_bnd2", rint_bnd)

    return main.compile().to_bytes()


def entry_args() -> bytes:
    @guppy
    def foo(a: int) -> None:
        result("a", a)

    return foo.compile_function().to_bytes()


if __name__ == "__main__":
    for func in [
        check,
        unsupported_pytket_ops,
        no_results,
        flip_some,
        discard_qb_array,
        measure_qb_array,
        # TODO: Currently fails due to <https://github.com/Quantinuum/guppylang/issues/1408>.
        # print_array,
        postselect_exit,
        postselect_panic,
        rus,
        print_current_shot,
        rng,
        entry_args,
    ]:
        envelope = func()
        (resources_dir / f"{func.__name__}.hugr").write_bytes(envelope)
