"""QSystem utils extension operations."""

import functools

from hugr.ext import Extension, OpDef, TypeDef
from hugr.ops import ExtOp

from .._util import TketExtension, load_extension


class QSystemUtilsExtension(TketExtension):
    """QSystem's utility operations."""

    @functools.cache
    def __call__(self) -> Extension:
        """Returns the qsystem utils extension"""
        return load_extension("tket.qsystem.utils")

    def TYPES(self) -> list[TypeDef]:
        """Return the types defined by this extension"""
        return []

    def OPS(self) -> list[OpDef]:
        """Return the operations defined by this extension"""
        return [
            self.get_current_shot.op_def(),
        ]

    @functools.cached_property
    def get_current_shot(self) -> ExtOp:
        """Get current shot number."""
        return self().get_op("GetCurrentShot").instantiate()
