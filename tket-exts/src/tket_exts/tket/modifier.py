"""Modifier extension operations."""

import functools
from typing import List

from hugr.ext import Extension, OpDef, TypeDef
from ._util import TketExtension, load_extension


class ModifierExtension(TketExtension):
    """Control, dagger and power modifiers for unitary operations."""

    @functools.cache
    def __call__(self) -> Extension:
        """Returns the quantum extension"""
        return load_extension("tket.modifier")

    def TYPES(self) -> List[TypeDef]:
        """Return the types defined by this extension"""
        return []

    def OPS(self) -> List[OpDef]:
        """Return the operations defined by this extension"""
        return [
            self.control,
            self.dagger,
            self.power,
        ]

    @functools.cached_property
    def control(self) -> OpDef:
        """Control modifier operation."""
        return self().get_op("ControlModifier")

    @functools.cached_property
    def dagger(self) -> OpDef:
        """Dagger modifier operation."""
        return self().get_op("DaggerModifier")

    @functools.cached_property
    def power(self) -> OpDef:
        """Power modifier operation."""
        return self().get_op("PowerModifier")
