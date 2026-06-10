# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang ==0.21.13",
# ]
# [tool.uv.sources]
# guppylang = {git = "https://github.com/quantinuum/guppylang", subdirectory = "guppylang", branch = "ts/future-measure"}
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
