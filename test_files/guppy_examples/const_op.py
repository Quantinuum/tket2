# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang==1.0.0a5",
#     "guppylang-internals==1.0.0a5",
# ]
# ///
"""An function defining a constant angle"""

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.angles import angle


@guppy
def const_op() -> angle:
    x = angle(3.141)
    return x


program = const_op.compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
