# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang==1.0.0a5",
#     "guppylang-internals==1.0.0a5",
# ]
# ///
"""Extern operation declaration"""

from pathlib import Path
from sys import argv

from guppylang.decorator import guppy

ext = guppy._extern("ext", ty="float")


@guppy
def main() -> float:
    return ext + ext


program = main.compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
