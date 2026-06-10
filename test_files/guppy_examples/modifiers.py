# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang ==0.21.14",
# ]
# [tool.uv.sources]
# guppylang = {git = "https://github.com/quantinuum/guppylang", subdirectory = "guppylang", branch = "ts/future-measure"}
# ///
"""A simple controlled gate using modifiers"""

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.builtins import control, dagger
from guppylang.std.quantum import qubit, s

from guppylang.experimental import enable_experimental_features

enable_experimental_features()


@guppy
def control_sdg(q0: qubit, q1: qubit) -> None:
    with control(q0):
        with dagger:
            s(q1)


program = control_sdg.compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
