"""Measurement extension operations."""

import functools
from typing import List

from hugr.ext import Extension, OpDef, TypeDef
from hugr.ops import ExtOp
from hugr.tys import ExtType

from ._util import TketExtension, load_extension


class MeasurementExtension(TketExtension):
    """Measurement result operations."""

    @functools.cache
    def __call__(self) -> Extension:
        """Returns the measurement extension"""
        return load_extension("tket.measurement")

    def TYPES(self) -> List[TypeDef]:
        """Return the types defined by this extension"""
        return [self.measurement_t.type_def]

    def OPS(self) -> List[OpDef]:
        """Return the operations defined by this extension"""
        return [
            self.dup.op_def(),
            self.free.op_def(),
            self.read.op_def(),
        ]

    @functools.cached_property
    def measurement_t(self) -> ExtType:
        """A type representing the result of a measurement operation."""
        return self().get_type("Measurement").instantiate([])

    @functools.cached_property
    def dup(self) -> ExtOp:
        """Duplicate a measurement, consuming the original value."""
        return self().get_op("Dup").instantiate()

    @functools.cached_property
    def free(self) -> ExtOp:
        """Consume a measurement without reading it."""
        return self().get_op("Free").instantiate()

    @functools.cached_property
    def read(self) -> ExtOp:
        """Read a measurement, consuming it and returning a Hugr bool."""
        return self().get_op("Read").instantiate()
