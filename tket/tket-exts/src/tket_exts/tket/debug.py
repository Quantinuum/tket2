"""Debug extension operations."""

import functools

from hugr.ext import Extension, OpDef, TypeDef
from hugr.ops import ExtOp
from hugr.tys import BoundedNatArg, StringArg

from ._util import TketExtension, load_extension


class DebugExtension(TketExtension):
    """Extension for debugging operations."""

    @functools.cache
    def __call__(self) -> Extension:
        """Return the debug extension"""
        return load_extension("tket.debug")

    def TYPES(self) -> list[TypeDef]:
        """Return the types defined by this extension"""
        return []

    def OPS(self) -> list[OpDef]:
        """Return the operations defined by this extension"""
        return [
            self.state_result_def,
        ]

    @functools.cached_property
    def state_result_def(self) -> OpDef:
        """Report the state of given qubits in the given order.

        This is the generic operation definition. For the instantiated operation, see
        `stateResult`.
        """
        return self().get_op("StateResult")

    @functools.cache
    def state_result(self, name: str, size: int) -> ExtOp:
        """Report the state of given qubits in the given order.

        Args:
            name: The name of the state result to report.
            size: The size of the qubit array.
        """
        return self.state_result_def.instantiate([StringArg(name), BoundedNatArg(size)])
