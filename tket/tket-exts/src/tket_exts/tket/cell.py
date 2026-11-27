"""Shared mutable cells with explicitly managed handles."""

from functools import cache

from hugr.ext import Extension, OpDef, TypeDef
from hugr.tys import ExtType, Type, TypeTypeArg

from ._util import TketExtension, load_extension


class CellExtension(TketExtension):
    """Operations on linear handles to shared mutable storage."""

    @cache
    def __call__(self) -> Extension:
        """Return the Cell extension."""
        return load_extension("tket.cell")

    def TYPES(self) -> list[TypeDef]:
        return [self().get_type("cell")]

    def OPS(self) -> list[OpDef]:
        return list(self().operations.values())

    def cell_t(self, payload: Type) -> ExtType:
        """Return the handle type for a cell storing the given payload."""
        return self().get_type("cell").instantiate([TypeTypeArg(payload)])
