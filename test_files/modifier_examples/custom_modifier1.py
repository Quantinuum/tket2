# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.1",
#    "matplotlib",
# ]
# [tool.uv.sources]
# guppylang = {git = "https://github.com/quantinuum/guppylang", subdirectory = "guppylang", rev = "952d728357b09741e5355a213f574a14967ffc12"}
# ///
"""Test the use of a higher-order function with complex control flow inside modifiers"""

from pathlib import Path
from sys import argv

from guppylang import enable_experimental_features, guppy
from guppylang.std.builtins import array, control, dagger, nat, panic, qubit
from guppylang.std.debug import state_result
from guppylang.std.quantum import discard, discard_array, h, s, x, z

enable_experimental_features()


@guppy.unitary
class classic_custom:
    @guppy
    def __call__() -> int:
        return 1

    @guppy
    def controlled[c: nat](_controls: array[qubit, c]) -> int:
        return 99


@guppy.unitary
class foo:
    @guppy(unitary=True)
    def __call__(q: qubit) -> None:
        s(q)

    @guppy
    def daggered(q: qubit) -> None:
        x(q)

    @guppy
    def controlled[c: nat](q: qubit, _controls: array[qubit, c]) -> None:
        with control(_controls):
            h(q)

    @guppy
    def ctrl_daggered[c: nat](q: qubit, _controls: array[qubit, c]) -> None:
        with dagger:
            foo(_controls[1])
        with control(_controls):
            x(q)


@guppy(unitary=True)
def helper(c0: qubit, c1: qubit, q: qubit) -> None:

    with dagger, control(c0, c1):
        foo(q)


@guppy
def main() -> None:
    q = qubit()
    cs = array(qubit(), qubit())

    # we first test classical custom functions
    with control(q):
        y = classic_custom()
        if y != 99:
            panic("Unexpected value")

    with dagger:
        foo(cs[0])
    with control(cs[0]):
        foo(q)
        with control(q):
            foo(cs[1])
            z(cs[1])

    helper(cs[0], cs[1], q)

    state_result("r", cs[0], cs[1], q)
    discard(q)
    discard_array(cs)


program = main.with_minimal_opt().compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
