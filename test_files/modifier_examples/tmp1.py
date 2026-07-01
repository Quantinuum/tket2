# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.0a8",
# ]
# [tool.uv.sources]
# guppylang = {git = "https://github.com/quantinuum/guppylang", subdirectory = "guppylang", branch = "main"}
# ///

from guppylang.decorator import guppy
from guppylang.std.builtins import control, dagger
from guppylang.std.quantum import angle, discard, qubit, rx
from guppylang.std.debug import state_result
from tket.passes import NormalizeGuppy

from pathlib import Path
from sys import argv


@guppy
def main() -> None:
    q = qubit()
    c = qubit()
    # y = 1
    with dagger:
        with control(c):
            rx(q, angle(1 / 2))

    state_result("r", q)
    discard(q)
    discard(c)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
program.modules[0].render_dot().view("tmp1")
