"""Quantum extension operations."""

import functools

from hugr.ext import Extension, OpDef, TypeDef
from hugr.ops import ExtOp
from hugr.tys import StringArg

from ._util import TketExtension, load_extension


class QuantumExtension(TketExtension):
    """TKET's standard quantum operations."""

    @functools.cache
    def __call__(self) -> Extension:
        """Returns the quantum extension"""
        return load_extension("tket.quantum")

    def TYPES(self) -> list[TypeDef]:
        """Return the types defined by this extension"""
        return []

    def OPS(self) -> list[OpDef]:
        """Return the operations defined by this extension"""
        return [
            self.CRz.op_def(),
            self.CX.op_def(),
            self.CY.op_def(),
            self.CZ.op_def(),
            self.H.op_def(),
            self.measure.op_def(),
            self.measure_free.op_def(),
            self.qAlloc.op_def(),
            self.qFree.op_def(),
            self.reset.op_def(),
            self.Rx.op_def(),
            self.Ry.op_def(),
            self.Rz.op_def(),
            self.S.op_def(),
            self.Sdg.op_def(),
            self.T.op_def(),
            self.Tdg.op_def(),
            self.toffoli.op_def(),
            self.try_QAlloc.op_def(),
            self.V.op_def(),
            self.Vdg.op_def(),
            self.X.op_def(),
            self.Y.op_def(),
            self.Z.op_def(),
            self.symbolic_angle_def,
        ]

    @functools.cached_property
    def CRz(self) -> ExtOp:
        """Controlled Rz rotation with angle parameter."""
        return self().get_op("CRz").instantiate()

    @functools.cached_property
    def CX(self) -> ExtOp:
        """Controlled-X (CNOT)."""
        return self().get_op("CX").instantiate()

    @functools.cached_property
    def CY(self) -> ExtOp:
        """Controlled-Y."""
        return self().get_op("CY").instantiate()

    @functools.cached_property
    def CZ(self) -> ExtOp:
        """Controlled-Z."""
        return self().get_op("CZ").instantiate()

    @functools.cached_property
    def H(self) -> ExtOp:
        """Hadamard."""
        return self().get_op("H").instantiate()

    @functools.cached_property
    def measure(self) -> ExtOp:
        """Measure a qubit producing a classical result (and keep the qubit)."""
        return self().get_op("Measure").instantiate()

    @functools.cached_property
    def measure_free(self) -> ExtOp:
        """Measure a qubit producing a measurement (qubit consumed)."""
        return self().get_op("MeasureFree").instantiate()

    @functools.cached_property
    def qAlloc(self) -> ExtOp:
        """Allocate a new qubit in |0>."""
        return self().get_op("QAlloc").instantiate()

    @functools.cached_property
    def qFree(self) -> ExtOp:
        """Free a qubit (lose track)."""
        return self().get_op("QFree").instantiate()

    @functools.cached_property
    def reset(self) -> ExtOp:
        """Reset a qubit to |0>."""
        return self().get_op("Reset").instantiate()

    @functools.cached_property
    def Rx(self) -> ExtOp:
        """Rotate around X by a rotation parameter."""
        return self().get_op("Rx").instantiate()

    @functools.cached_property
    def Ry(self) -> ExtOp:
        """Rotate around Y by a rotation parameter."""
        return self().get_op("Ry").instantiate()

    @functools.cached_property
    def Rz(self) -> ExtOp:
        """Rotate around Z by a rotation parameter."""
        return self().get_op("Rz").instantiate()

    @functools.cached_property
    def S(self) -> ExtOp:
        """S phase gate (pi/2 around Z)."""
        return self().get_op("S").instantiate()

    @functools.cached_property
    def Sdg(self) -> ExtOp:
        """S dagger gate (-pi/2 around Z)."""
        return self().get_op("Sdg").instantiate()

    @functools.cached_property
    def T(self) -> ExtOp:
        """T phase gate (pi/4 around Z)."""
        return self().get_op("T").instantiate()

    @functools.cached_property
    def Tdg(self) -> ExtOp:
        """T dagger gate (-pi/4 around Z)."""
        return self().get_op("Tdg").instantiate()

    @functools.cached_property
    def toffoli(self) -> ExtOp:
        """Toffoli (CCX)."""
        return self().get_op("Toffoli").instantiate()

    @functools.cached_property
    def try_QAlloc(self) -> ExtOp:
        """Try allocate a qubit, returning None on failure."""
        return self().get_op("TryQAlloc").instantiate()

    @functools.cached_property
    def V(self) -> ExtOp:
        """V gate (sqrt(X))."""
        return self().get_op("V").instantiate()

    @functools.cached_property
    def Vdg(self) -> ExtOp:
        """V dagger gate (sqrt(X))^-1."""
        return self().get_op("Vdg").instantiate()

    @functools.cached_property
    def X(self) -> ExtOp:
        """Pauli-X."""
        return self().get_op("X").instantiate()

    @functools.cached_property
    def Y(self) -> ExtOp:
        """Pauli-Y."""
        return self().get_op("Y").instantiate()

    @functools.cached_property
    def Z(self) -> ExtOp:
        """Pauli-Z."""
        return self().get_op("Z").instantiate()

    @functools.cached_property
    def symbolic_angle_def(self) -> OpDef:
        """Store a sympy expression evaluable to a rotation angle.

        This is the generic operation definition. For the instantiated operation, see
        `symbolic_angle`.
        """
        return self().get_op("symbolic_angle")

    def symbolic_angle(self, expr: str) -> ExtOp:
        """Store a sympy expression evaluable to a rotation angle.

        Args:
            expr: A sympy expression as a string.
        """
        return self.symbolic_angle_def.instantiate([StringArg(expr)])
