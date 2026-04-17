# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang ==0.21.11",
# ]
# ///
"""A simple controlled gate using modifiers"""

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.builtins import control
from guppylang.std.quantum import qubit
from guppylang.std.qsystem import measure
from guppylang.std.quantum import h, x

from guppylang.experimental import enable_experimental_features

enable_experimental_features()


@guppy
def main() -> None:
    q1 = qubit()
    q2 = qubit()
    h(q1)
    with control(q1):
        x(q2)

    measure(q1)
    measure(q2)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
program.modules[0].render_dot().render(argv[0].removesuffix(".py") + "_before")
