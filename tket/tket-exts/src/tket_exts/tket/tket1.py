"""Compatibility extension for opaque TKET1 operations."""

import functools

from hugr.ext import Extension, OpDef, TypeDef
from hugr.ops import ExtOp
from hugr.tys import FunctionType, StringArg

from ._util import TketExtension, load_extension


class Tket1Extension(TketExtension):
    """Opaque operations retained for TKET1 compatibility."""

    @functools.cache
    def __call__(self) -> Extension:
        """Return the TKET1 compatibility extension."""
        return load_extension("TKET1")

    def TYPES(self) -> list[TypeDef]:
        """Return the types defined by this extension."""
        return []

    def OPS(self) -> list[OpDef]:
        """Return the operations defined by this extension."""
        return [self.tk1op_def]

    @functools.cached_property
    def tk1op_def(self) -> OpDef:
        """Return the generic definition of an opaque TKET1 operation."""
        return self().get_op("tk1op")

    def tk1op(self, payload: str, signature: FunctionType) -> ExtOp:
        """Instantiate an opaque TKET1 operation.

        Args:
            payload: JSON-encoded TKET1 operation and wire metadata.
            signature: Concrete HUGR signature represented by the payload.
        """
        return self.tk1op_def.instantiate(
            [StringArg(payload)], concrete_signature=signature
        )
