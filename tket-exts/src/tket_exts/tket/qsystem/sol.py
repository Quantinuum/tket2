"""QSystem Sol platform extension operations."""

import functools
from typing import List

from hugr.ext import Extension, OpDef, TypeDef
from hugr.ops import ExtOp
from hugr.tys import BoundedNatArg
from .._util import TketExtension, load_extension

__all__ = ["QSystemSolExtension"]


class QSystemSolExtension(TketExtension):
    """Operations for the Sol platform (tket.qsystem.sol)."""

    @functools.cache
    def __call__(self) -> Extension:
        return load_extension("tket.qsystem.sol")

    def TYPES(self) -> List[TypeDef]:
        return []

    def OPS(self) -> List[OpDef]:
        return [
            self.lazy_measure.op_def(),
            self.lazy_measure_leaked.op_def(),
            self.lazy_measure_reset.op_def(),
            self.phasedX.op_def(),
            self.phasedXX.op_def(),
            self.qFree.op_def(),
            self.reset.op_def(),
            self.runtime_barrier_def,
            self.Rz.op_def(),
            self.try_QAlloc.op_def(),
            self.future_to_measurement.op_def(),
        ]

    @functools.cached_property
    def lazy_measure(self) -> ExtOp:
        """Lazily measure a qubit and lose it (returns a Future)."""
        return self().get_op("LazyMeasure").instantiate()

    @functools.cached_property
    def lazy_measure_leaked(self) -> ExtOp:
        """Measure a qubit or detect leakage."""
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
    def phasedXX(self) -> ExtOp:
        """Two-qubit PhasedXX gate (alias 'rpp'), specific to the Sol platform."""
        return self().get_op("PhasedXX").instantiate()

    @functools.cached_property
    def qFree(self) -> ExtOp:
        """Free a qubit (lose track of it)."""
        return self().get_op("QFree").instantiate()

    @functools.cached_property
    def reset(self) -> ExtOp:
        """Reset a qubit to the Z |0> eigenstate."""
        return self().get_op("Reset").instantiate()

    @functools.cached_property
    def runtime_barrier_def(self) -> OpDef:
        """Runtime barrier operation definition."""
        return self().get_op("RuntimeBarrier")

    def runtime_barrier(self, size: int) -> ExtOp:
        """Runtime barrier between operations on argument qubits."""
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
    def future_to_measurement(self) -> ExtOp:
        """Convert a Future[bool] to a Measurement."""
        return self().get_op("FutureToMeasurement").instantiate()
