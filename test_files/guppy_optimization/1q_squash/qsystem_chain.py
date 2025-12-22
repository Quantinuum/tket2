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
    phased_x(q, angle(0.91), angle(0.5))
    phased_x(q, angle(0.53), angle(0))
    phased_x(q, angle(3.29), angle(0.5))
    phased_x(q, angle(0.81), angle(0))
    rz(q, angle(0.62))


program = qsystem_chain.compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())

# pytket code to generate this example.
# For the optimised version replace the call to AutoRebase with AutoSquash.

# from pytket import Circuit, OpType
# from pytket.passes import AutoSquash, AutoRebase
#
# circ = Circuit(1)
#
# circ.Ry(0.91, 0)
# circ.Rx(0.53, 0)
# circ.Ry(-0.71, 0)
# circ.Rx(0.81, 0)
# circ.Rz(0.62, 0)
#
# When you switch to AutoSquash, remove ZZPhase from the set
# AutoRebase({OpType.Rz, OpType.PhasedX, OpType.ZZPhase}).apply(circ)
#
# print(circ.get_commands())
