"""Measurement extension operations."""

import functools
from typing import List

from hugr.ext import Extension, OpDef, TypeDef
from hugr.ops import ExtOp
from hugr.tys import ExtType

from ._util import TketExtension, load_extension


class MeasurementExtension(TketExtension):
    """Operations on measurement types."""

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
            self.read.op_def(),
        ]

    @functools.cached_property
    def measurement_t(self) -> ExtType:
        """A copyable type representing the result of a measurement operation."""
        return self().get_type("Measurement").instantiate([])

    @functools.cached_property
    def read(self) -> ExtOp:
        """Consumes a measurement, converting it into a bool."""
        return self().get_op("Read").instantiate()
