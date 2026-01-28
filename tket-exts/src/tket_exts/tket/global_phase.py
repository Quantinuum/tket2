"""Global Phase extension operations."""

import functools
from typing import List

from hugr.ext import Extension, OpDef, TypeDef
from hugr.ops import ExtOp
from ._util import TketExtension, load_extension


class GlobalPhaseExtension(TketExtension):
    """Operation to add global phase to unitary operations."""

    @functools.cache
    def __call__(self) -> Extension:
        """Returns the quantum extension"""
        return load_extension("tket.global_phase")

    def TYPES(self) -> List[TypeDef]:
        """Return the types defined by this extension"""
        return []

    def OPS(self) -> List[OpDef]:
        """Return the operations defined by this extension"""
        return [
            self.global_phase.op_def(),
        ]

    @functools.cached_property
    def global_phase(self) -> ExtOp:
        """Applies a global phase to the circuit."""
        return self().get_op("GlobalPhase").instantiate()
