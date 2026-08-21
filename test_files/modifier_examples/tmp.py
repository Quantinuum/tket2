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
from guppylang.std.quantum import angle, discard_array, h, qubit, z, x, discard

enable_experimental_features()


@guppy
def main() -> None:
    controller = array(qubit(), qubit(), qubit())
    target = qubit()
    h(controller[0])
    x(controller[1])
    x(controller[2])
    x(target)
    with control(controller):
        z(target)

    state_result("r", controller[0], controller[1], controller[2], target)
    discard(target)
    discard_array(controller)


program = main.with_minimal_opt().compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
