# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.0rc1",
# ]
# ///
"""Test the a simple use of a higher-order function with inside modifiers

The hugr generated from this script is also used to benchmark the performance of modifier passes resolver
"""

from pathlib import Path
from sys import argv

from guppylang import array, enable_experimental_features, guppy
from guppylang.std.builtins import Unitary, control, dagger
from guppylang.std.debug import state_result
from guppylang.std.quantum import cx, discard_array, h, qubit, s

enable_experimental_features()


@guppy(unitary=True)
def apply_2qubit_gate(f: Unitary[[qubit, qubit], None], q: array[qubit, 3]) -> None:
    f(q[1], q[2])


@guppy(daggerable=True, controllable=True)
def apply_dagger(f: Unitary[[qubit], None], q: array[qubit, 3]) -> None:
    f(q[1])
    apply_2qubit_gate(cx, q)


@guppy
def main() -> None:
    q = array(qubit(), qubit(), qubit())
    h(q[0])

    with dagger, control(q[0]):
        apply_dagger(s, q)
        apply_dagger(h, q)

    state_result("r", q)
    discard_array(q)


program = main.with_minimal_opt().compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
