"""Shared mutable cells with explicitly managed handles."""

import functools

from hugr.ext import Extension, OpDef, TypeDef
from hugr.tys import ExtType, Type, TypeTypeArg

from ._util import TketExtension, load_extension


class CellExtension(TketExtension):
    """Operations on linear handles to shared mutable storage."""

    @functools.cache
    def __call__(self) -> Extension:
        """Return the Cell extension."""
        return load_extension("tket.cell")

    def TYPES(self) -> list[TypeDef]:
        return [self.cell_t_def]

    def OPS(self) -> list[OpDef]:
        return list(self().operations.values())

    @functools.cached_property
    def cell_t_def(self) -> TypeDef:
        """Return the generic cell handle type definition."""
        return self().get_type("cell")

    def cell_t(self, payload: Type) -> ExtType:
        """Return the handle type for a cell storing the given payload."""
        return self.cell_t_def.instantiate([TypeTypeArg(payload)])
