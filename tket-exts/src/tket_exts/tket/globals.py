import functools
from typing import List

from hugr.ops import ExtOp

from ._util import TketExtension, load_extension
from hugr.ext import Extension, OpDef, TypeDef


class GlobalsExtension(TketExtension):
    """Global state operations."""

    @functools.cache
    def __call__(self) -> Extension:
        """Returns the globals extension"""
        return load_extension("tket.globals")

    def TYPES(self) -> List[TypeDef]:
        """Return the types defined by this extension"""
        return []

    def OPS(self) -> List[OpDef]:
        """Return the operations defined by this extension"""
        return [
            self.swap.op_def(),
        ]

    @functools.cached_property
    def swap(self) -> ExtOp:
        """Swap the contents of the named global variable with the argument."""
        return self().get_op("swap").instantiate()
