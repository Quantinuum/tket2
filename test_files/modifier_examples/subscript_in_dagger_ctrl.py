# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.0rc1",
# ]
# ///
"""Subscript indexing in dagger and control context"""

from pathlib import Path
from sys import argv

from guppylang import array, enable_experimental_features, guppy
from guppylang.std.builtins import control, dagger
from guppylang.std.debug import state_result
from guppylang.std.quantum import angle, discard_array, h, qubit, rx, s, x

enable_experimental_features()


@guppy(unitary=True)
def f(controller: qubit, target: qubit) -> None:
    a = angle(1 / 3)
    with control(controller):
        rx(target, a)


@guppy
def main() -> None:
    controller = array(qubit(), qubit())
    array_qubits = array(qubit(), qubit())
    h(controller[0])
    x(controller[1])
    with dagger:
        with control(controller):
            f(array_qubits[0], array_qubits[1])
            s(array_qubits[0])
            h(array_qubits[0])

    state_result("r", controller[0], controller[1], array_qubits[0], array_qubits[1])
    discard_array(array_qubits)
    discard_array(controller)


program = main.with_minimal_opt().compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
