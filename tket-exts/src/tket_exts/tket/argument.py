"""Argument reader extension operations."""

import functools
from typing import List

from hugr.ext import Extension, OpDef, TypeDef
from hugr.ops import ExtOp
from hugr.tys import FunctionType, StringArg, Type, TypeTypeArg

from ._util import TketExtension, load_extension


class ArgumentExtension(TketExtension):
    """Operations for reading runtime entrypoint arguments."""

    @functools.cache
    def __call__(self) -> Extension:
        """Returns the argument extension"""
        return load_extension("tket.argument")

    def TYPES(self) -> List[TypeDef]:
        """Return the types defined by this extension"""
        return []

    def OPS(self) -> List[OpDef]:
        """Return the operations defined by this extension"""
        return [
            self.read_arg_def,
        ]

    @functools.cached_property
    def read_arg_def(self) -> OpDef:
        """Read a runtime argument of the given type identified by a string tag.

        This is the generic operation definition. For the instantiated operation, see
        `read_arg`.
        """
        return self().get_op("read_arg")

    def read_arg(self, tag: str, ty: Type) -> ExtOp:
        """Read a runtime argument of type ``ty`` identified by ``tag``.

        Args:
            tag: String tag identifying the argument (matches the provider's key).
            ty: The HUGR type of the argument to read.
        """
        return self.read_arg_def.instantiate(
            [StringArg(tag), TypeTypeArg(ty)],
            FunctionType(input=[], output=[ty]),
        )
