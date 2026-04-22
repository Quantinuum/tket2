import functools
from typing import List

from hugr.ops import ExtOp
from hugr.tys import StringArg, TypeTypeArg, Type

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
            self.swap_def,
        ]

    @functools.cached_property
    def swap_def(self) -> OpDef:
        """Swap the contents of the named global variable with the argument."""
        return self().get_op("swap")

    def swap(self, name: str, ty: Type) -> ExtOp:
        """Swap the contents of the named global variable with the argument."""
        return self().get_op("swap").instantiate([StringArg(name), TypeTypeArg(ty)])
