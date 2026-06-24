# /// script
# requires-python = ">=3.14"
# dependencies = [
#     "guppylang==1.0.0a5",
#     "guppylang-internals==1.0.0a5",
# ]
# ///

from pathlib import Path
from sys import argv

import guppylang
from guppylang import guppy
from guppylang.std.builtins import array, result
from guppylang.std.qsystem import *  # noqa: F403
from guppylang.std.quantum import measure, measure_array, qubit, collect_measurements
from tket.passes import NormalizeGuppy


guppylang.enable_experimental_features()


@guppy
def main_() -> None:
    q1 = qubit()
    qreg1 = array(qubit() for _ in range(2))
    result("q1", measure(q1).read())
    result("qreg1", collect_measurements(measure_array(qreg1)))


program = NormalizeGuppy()(main_.compile_function().modules[0])
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
