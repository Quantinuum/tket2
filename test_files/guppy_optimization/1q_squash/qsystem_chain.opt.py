# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang >=0.21.6",
# ]
# ///

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.quantum import qubit
from guppylang.std.qsystem import rz, phased_x
from guppylang.std.angles import angle


@guppy
def qsystem_chain(q: qubit) -> None:
    phased_x(q, angle(0.368713), angle(1.66415))
    rz(q, angle(0.870616))


program = qsystem_chain.compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
