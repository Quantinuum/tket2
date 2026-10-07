"""Futures extension operations."""

import functools

from hugr.ext import Extension, OpDef, TypeDef
from hugr.ops import ExtOp
from hugr.tys import ExtType, Type, TypeTypeArg

from ._util import TketExtension, load_extension


class FuturesExtension(TketExtension):
    """Future type and handling operations."""

    @functools.cache
    def __call__(self) -> Extension:
        """Returns the futures extension"""
        return load_extension("tket.futures")

    def TYPES(self) -> list[TypeDef]:
        """Return the types defined by this extension"""
        return [self.future_t_def]

    def OPS(self) -> list[OpDef]:
        """Return the operations defined by this extension"""
        return [
            self.dup_def,
            self.free_def,
            self.read_def,
        ]

    @functools.cached_property
    def future_t_def(self) -> TypeDef:
        """A value that is computed asynchronously.

        This is the generic type definition. For the instantiated type, see `future_t`
        """
        return self().get_type("Future")

    def future_t(self, ty: Type) -> ExtType:
        """A value that is computed asynchronously.

        Args:
            The element type wrapped by the Future.
        """
        return self().get_type("Future").instantiate([TypeTypeArg(ty)])

    @functools.cached_property
    def dup_def(self) -> OpDef:
        """Duplicate a Future. The original is consumed and two Futures are returned.

        This is the generic operation definition. For the instantiated operation, see
        `dup`.
        """
        return self().get_op("Dup")

    def dup(self, ty: Type) -> ExtOp:
        """Duplicate a Future. The original is consumed and two Futures are returned.

        Args:
            ty: The element type of the Future being duplicated.
        """
        return self.dup_def.instantiate([TypeTypeArg(ty)])

    @functools.cached_property
    def free_def(self) -> OpDef:
        """Consume a Future without reading it.

        This is the generic operation definition. For the instantiated operation, see
        `free`.
        """
        return self().get_op("Free")

    def free(self, ty: Type) -> ExtOp:
        """Consume a Future without reading it.

        Args:
            ty: The element type of the Future being consumed.
        """
        return self.free_def.instantiate([TypeTypeArg(ty)])

    @functools.cached_property
    def read_def(self) -> OpDef:
        """Read a value from a Future, consuming it.

        This is the generic operation definition. For the instantiated operation, see
        `read`.
        """
        return self().get_op("Read")

    def read(self, ty: Type) -> ExtOp:
        """Read a value from a Future, consuming it.

        Args:
            ty: The element type of the Future being read.
        """
        return self.read_def.instantiate([TypeTypeArg(ty)])
