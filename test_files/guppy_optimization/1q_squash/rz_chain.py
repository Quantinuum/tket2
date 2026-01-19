# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang >=0.21.6",
# ]
# ///

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.quantum import rz, qubit
from guppylang.std.angles import angle


@guppy
def rz_chain(q: qubit) -> None:
    rz(q, angle(1 / 2))
    rz(q, angle(1 / 2))
    rz(q, angle(1 / 2))


program = rz_chain.compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
