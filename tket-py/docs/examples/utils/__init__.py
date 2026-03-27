"""Some utility functions for the example notebooks."""

from hugr import Hugr
from hugr.envelope import EnvelopeConfig
from tket import TkProgram


def setup_jupyter_rendering():
    """Set up hugr rendering for Jupyter notebooks."""

    def _repr_hugr(
        h: Hugr, include=None, exclude=None, **kwargs
    ) -> dict[str, bytes | str]:
        return h.render_dot()._repr_mimebundle_(include, exclude, **kwargs)

    def _repr_tk2circ(
        circ: TkProgram, include=None, exclude=None, **kwargs
    ) -> dict[str, bytes | str]:
        h = Hugr.from_bytes(circ.to_bytes(EnvelopeConfig.BINARY))
        return _repr_hugr(h, include, exclude, **kwargs)

    setattr(Hugr, "_repr_mimebundle_", _repr_hugr)
    setattr(TkProgram, "_repr_mimebundle_", _repr_tk2circ)
