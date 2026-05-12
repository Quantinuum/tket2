# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang ==0.21.14",
# ]
# ///

from pathlib import Path
from sys import argv
import sys

from guppylang import guppy
from guppylang.std.array import array_swap
from guppylang.std.quantum import discard, qubit, h, x
from guppylang.std.qsystem import zz_phase
from guppylang.std.builtins import array, comptime, frozenarray, mem_swap
from guppylang.std.angles import pi
from guppylang.defs import GuppyFunctionDefinition
from hugr.hugr.render import RenderConfig
import guppylang
from guppylang.std.builtins import control, dagger
from guppylang.std.debug import state_result

sys.path.append(str(Path(__file__).resolve().parents[1]))


guppylang.enable_experimental_features()


@guppy
def main() -> None:
    arr = array(1, 1, 2, 1, 1)
    with dagger:
        array_swap(arr, 2, 4)
        array_swap(arr, 0, 4)
    q = qubit()
    if arr[0] == 2:
        h(q)
    state_result("r", q)
    discard(q)


program = main.compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
