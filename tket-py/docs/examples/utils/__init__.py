"""Some utility functions for the example notebooks."""

from hugr import Hugr
from hugr.envelope import EnvelopeConfig
from tket.passes import lower_to_pytket
from tket.circuit import Tk2Circuit
from typing import Any


def setup_jupyter_rendering():
    """Set up hugr rendering for Jupyter notebooks."""

    def _repr_hugr(
        h: Hugr, include=None, exclude=None, **kwargs
    ) -> dict[str, bytes | str]:
        return h.render_dot()._repr_mimebundle_(include, exclude, **kwargs)

    def _repr_tk2circ(
        circ: Tk2Circuit, include=None, exclude=None, **kwargs
    ) -> dict[str, bytes | str]:
        h = Hugr.from_bytes(circ.to_bytes(EnvelopeConfig.BINARY))
        return _repr_hugr(h, include, exclude, **kwargs)

    setattr(Hugr, "_repr_mimebundle_", _repr_hugr)
    setattr(Tk2Circuit, "_repr_mimebundle_", _repr_tk2circ)


# TODO: Should this be part of the guppy API? Or tket?
# Takes a RawFunctionDef and converts it to a Tk2Circuit
# BUG: this defaults num_operations to 0 which breaks Rules
def guppy_to_circuit(func_def: Any) -> Tk2Circuit:
    """Convert a Guppy function definition to a `Tk2Circuit`."""

    pkg = func_def.compile_function()

    f_name = pkg.modules[0].entrypoint_op().f_name

    byt = pkg.to_bytes()
    circ = Tk2Circuit.from_bytes(byt, f_name)

    return lower_to_pytket(circ)
