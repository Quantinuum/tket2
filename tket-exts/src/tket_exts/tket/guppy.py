"""Guppy extension operations."""

import functools

from hugr.ext import Extension, OpDef, TypeDef
from hugr.ops import ExtOp
from hugr.tys import Type, TypeTypeArg

from ._util import TketExtension, load_extension


class GuppyExtension(TketExtension):
    """Guppy-specific operations."""

    @functools.cache
    def __call__(self) -> Extension:
        """Returns the guppy extension"""
        return load_extension("tket.guppy")

    def TYPES(self) -> list[TypeDef]:
        """Return the types defined by this extension"""
        return []

    def OPS(self) -> list[OpDef]:
        """Return the operations defined by this extension"""
        return [
            self.drop_def,
        ]

    @functools.cached_property
    def drop_def(self) -> OpDef:
        """Drop the input wire. Applicable to guppy affine types only.

        This is the generic operation definition. For the instantiated operation, see
        `drop`.
        """
        return self().get_op("drop")

    def drop(self, ty: Type) -> ExtOp:
        """Drop the input wire. Applicable to guppy affine types only.

        Args:
            ty: The guppy affine type of the value to drop.
        """
        return self.drop_def.instantiate([TypeTypeArg(ty)])
