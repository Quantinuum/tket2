import functools
from typing import List
from ._util import TketExtension, load_extension
from hugr.ext import Extension, OpDef, TypeDef

class GlobalsExtension(TketExtension):
    """GPU interop operations."""

    @functools.cache
    def __call__(self) -> Extension:
        """Returns the gpu extension"""
        return load_extension("tket.globals")


    def OPS(self) -> List[OpDef]:
        """Return the operations defined by this extension"""
        return [
            self.swap_def,
        ]

    @functools.cached_property
    def swap_def(self) -> OpDef:
        """TODO
        """
        return self().get_op("swap")
