"""Result extension operations."""

import functools

from hugr.ext import Extension, OpDef, TypeDef
from hugr.ops import ExtOp
from hugr.tys import BoundedNatArg, StringArg

from ._util import TketExtension, load_extension


class ResultExtension(TketExtension):
    """Result reporting operations."""

    @functools.cache
    def __call__(self) -> Extension:
        """Returns the result extension"""
        return load_extension("tket.result")

    def TYPES(self) -> list[TypeDef]:
        """Return the types defined by this extension"""
        return []

    def OPS(self) -> list[OpDef]:
        """Return the operations defined by this extension"""
        return [
            self.result_array_bool_def,
            self.result_array_f64_def,
            self.result_array_int_def,
            self.result_array_uint_def,
            self.result_bool_def,
            self.result_f64_def,
            self.result_int_def,
            self.result_uint_def,
        ]

    @functools.cached_property
    def result_array_bool_def(self) -> OpDef:
        """Report an array of boolean results.

        This is the generic operation definition. For the instantiated operation, see
        `result_array_bool`.
        """
        return self().get_op("result_array_bool")

    def result_array_bool(self, label: str, size: int) -> ExtOp:
        """Report an array of boolean results.

        Args:
            label: Label for this result array.
            size: Length of the array.
        """
        return self.result_array_bool_def.instantiate(
            [StringArg(label), BoundedNatArg(size)]
        )

    @functools.cached_property
    def result_array_f64_def(self) -> OpDef:
        """Report an array of floating-point results.

        This is the generic operation definition. For the instantiated operation, see
        `result_array_f64`.
        """
        return self().get_op("result_array_f64")

    def result_array_f64(self, label: str, size: int) -> ExtOp:
        """Report an array of floating-point results.

        Args:
            label: Label for this result array.
            size: Length of the array.
        """
        return self.result_array_f64_def.instantiate(
            [StringArg(label), BoundedNatArg(size)]
        )

    @functools.cached_property
    def result_array_int_def(self) -> OpDef:
        """Report an array of signed integer results.

        This is the generic operation definition. For the instantiated operation, see
        `result_array_int`.
        """
        return self().get_op("result_array_int")

    def result_array_int(self, label: str, size: int, width: int) -> ExtOp:
        """Report an array of signed integer results.

        Args:
            label: Label for this result array.
            size: Length of the array.
            width: Bit width of the integers.
        """
        return self.result_array_int_def.instantiate(
            [StringArg(label), BoundedNatArg(size), BoundedNatArg(width)]
        )

    @functools.cached_property
    def result_array_uint_def(self) -> OpDef:
        """Report an array of unsigned integer results.

        This is the generic operation definition. For the instantiated operation, see
        `result_array_uint`.
        """
        return self().get_op("result_array_uint")

    def result_array_uint(self, label: str, size: int, width: int) -> ExtOp:
        """Report an array of unsigned integer results.

        Args:
            label: Label for this result array.
            size: Length of the array.
            width: Bit width of the integers.
        """
        return self.result_array_uint_def.instantiate(
            [StringArg(label), BoundedNatArg(size), BoundedNatArg(width)]
        )

    @functools.cached_property
    def result_bool_def(self) -> OpDef:
        """Report a boolean result.

        This is the generic operation definition. For the instantiated operation, see
        `result_bool`.
        """
        return self().get_op("result_bool")

    def result_bool(self, label: str) -> ExtOp:
        """Report a boolean result.

        Args:
            label: Label for this result.
        """
        return self.result_bool_def.instantiate([StringArg(label)])

    @functools.cached_property
    def result_f64_def(self) -> OpDef:
        """Report a floating-point result.

        This is the generic operation definition. For the instantiated operation, see
        `result_f64`.
        """
        return self().get_op("result_f64")

    def result_f64(self, label: str) -> ExtOp:
        """Report a floating-point result.

        Args:
            label: Label for this result.
        """
        return self.result_f64_def.instantiate([StringArg(label)])

    @functools.cached_property
    def result_int_def(self) -> OpDef:
        """Report a signed integer result.

        This is the generic operation definition. For the instantiated operation, see
        `result_int`.
        """
        return self().get_op("result_int")

    def result_int(self, label: str, width: int) -> ExtOp:
        """Report a signed integer result.

        Args:
            label: Label for this result.
            width: Bit width of the integer.
        """
        return self.result_int_def.instantiate([StringArg(label), BoundedNatArg(width)])

    @functools.cached_property
    def result_uint_def(self) -> OpDef:
        """Report an unsigned integer result.

        This is the generic operation definition. For the instantiated operation, see
        `result_uint`.
        """
        return self().get_op("result_uint")

    def result_uint(self, label: str, width: int) -> ExtOp:
        """Report an unsigned integer result.

        Args:
            label: Label for this result.
            width: Bit width of the integer.
        """
        return self.result_uint_def.instantiate(
            [StringArg(label), BoundedNatArg(width)]
        )
