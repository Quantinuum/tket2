# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang ==0.21.13",
# ]
# [tool.uv.sources]
# guppylang = {git = "https://github.com/quantinuum/guppylang", subdirectory = "guppylang", branch = "ts/future-measure"}
# ///

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.builtins import result


@guppy
def main() -> None:
    result("b", 0)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
