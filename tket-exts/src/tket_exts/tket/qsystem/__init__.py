"""QSystem extension operations."""

import functools
import warnings
from typing import List

from .helios import QSystemHeliosExtension
from .random import QSystemRandomExtension
from .sol import QSystemSolExtension
from .utils import QSystemUtilsExtension

from hugr.ext import Extension, OpDef, TypeDef
from hugr.ops import ExtOp
from hugr.tys import BoundedNatArg
from .._util import TketExtension, load_extension

__all__ = [
    "QSystemHeliosExtension",
    "QSystemRandomExtension",
    "QSystemSolExtension",
    "QSystemUtilsExtension",
]


class QSystemExtension(TketExtension):
    """Deprecated: use :class:`QSystemHeliosExtension` or :class:`QSystemSolExtension` instead.

    The combined ``tket.qsystem`` extension has been split into platform-specific
    extensions. Use ``tket_exts.qsystem_helios`` or ``tket_exts.qsystem_sol`` instead.
    """

    @functools.cache
    def __call__(self) -> Extension:
        """Returns the qsystem extension"""
        warnings.warn(
            "QSystemExtension (tket.qsystem) is deprecated. "
            "Use QSystemHeliosExtension (tket.qsystem.helios) or "
            "QSystemSolExtension (tket.qsystem.sol) instead.",
            DeprecationWarning,
            stacklevel=2,
        )
        return load_extension("tket.qsystem")

    def TYPES(self) -> List[TypeDef]:
        """Return the types defined by this extension"""
        return []

    def OPS(self) -> List[OpDef]:
        """Return the operations defined by this extension"""
        return [
            self.lazy_measure.op_def(),
            self.lazy_measure_leaked.op_def(),
            self.lazy_measure_reset.op_def(),
            self.phasedX.op_def(),
            self.qFree.op_def(),
            self.reset.op_def(),
            self.runtime_barrier_def,
            self.Rz.op_def(),
            self.try_QAlloc.op_def(),
            self.ZZPhase.op_def(),
            self.future_to_measure.op_def(),
        ]

    @functools.cached_property
    def lazy_measure(self) -> ExtOp:
        """Lazily measure a qubit and lose it (returns a Future)."""
        return self().get_op("LazyMeasure").instantiate()

    @functools.cached_property
    def lazy_measure_leaked(self) -> ExtOp:
        """Measure a qubit or detect leakage.

        The returned Future is an integer between 0 and 3, where the first two values
        are valid measurement results, and 2 is returned if the qubit was leaked.
        """
        return self().get_op("LazyMeasureLeaked").instantiate()

    @functools.cached_property
    def lazy_measure_reset(self) -> ExtOp:
        """Lazily measure a qubit and reset it to Z |0> (returns a Future)."""
        return self().get_op("LazyMeasureReset").instantiate()

    @functools.cached_property
    def phasedX(self) -> ExtOp:
        """PhasedX gate with two float parameters."""
        return self().get_op("PhasedX").instantiate()

    @functools.cached_property
    def qFree(self) -> ExtOp:
        """Free a qubit (lose track of it)."""
        return self().get_op("QFree").instantiate()

    @functools.cached_property
    def reset(self) -> ExtOp:
        """Reset a qubit to the Z |0> eigenstate."""
        return self().get_op("Reset").instantiate()

    @functools.cached_property
    def future_to_measure(self) -> ExtOp:
        """Convert a future(bool) to a measurement (for compatibility with the
        quantum extension).
        """
        return self().get_op("FutureToMeasurement").instantiate()

    @functools.cached_property
    def runtime_barrier_def(self) -> OpDef:
        """Runtime barrier between operations on argument qubits.

        This is the generic operation definition. For the instantiated operation, see
        `runtimeBarrier`.
        """
        return self().get_op("RuntimeBarrier")

    def runtime_barrier(self, size: int) -> ExtOp:
        """Runtime barrier between operations on argument qubits.

        Args:
            size: Length of the qubit array.
        """
        return self.runtime_barrier_def.instantiate([BoundedNatArg(size)])

    @functools.cached_property
    def Rz(self) -> ExtOp:
        """Rotate a qubit around the Z axis (not physical)."""
        return self().get_op("Rz").instantiate()

    @functools.cached_property
    def try_QAlloc(self) -> ExtOp:
        """Try allocate a qubit in Z |0> (returns Option-like result)."""
        return self().get_op("TryQAlloc").instantiate()

    @functools.cached_property
    def ZZPhase(self) -> ExtOp:
        """Two-qubit ZZ gate with a float angle."""
        return self().get_op("ZZPhase").instantiate()
