"""Pauli-string exponentials with angles in half-turns."""

import functools
from collections.abc import Sequence

from hugr.ext import Extension, OpDef, TypeDef
from hugr.ops import ExtOp
from hugr.tys import BoundedNatArg, FunctionType, ListArg, Qubit

from ._util import TketExtension, load_extension
from .rotation import RotationExtension


class PauliExpExtension(TketExtension):
    """The ``tket.pauli_exp`` extension."""

    @functools.cache
    def __call__(self) -> Extension:
        return load_extension("tket.pauli_exp")

    def TYPES(self) -> list[TypeDef]:
        return []

    def OPS(self) -> list[OpDef]:
        return [self.pauli_exp_def]

    @functools.cached_property
    def pauli_exp_def(self) -> OpDef:
        """The generic Pauli exponential operation definition."""
        return self().get_op("PauliExp")

    def pauli_exp(self, n: int, paulis: Sequence[int]) -> ExtOp:
        """Apply ``exp(-i*pi*angle*(P0 tensor ... tensor P(n-1))/2)``.

        Static arguments are the qubit count and Paulis (I=0, X=1, Y=2, Z=3).
        Inputs are ``n`` individual qubits followed by a ``rotation`` angle in
        half-turns. Outputs are the same qubits in the same order. Identity
        factors retain their wires; the empty string (n=0) gives a global phase.

        Raises:
            ValueError: Invalid qubit count, Pauli value, or sequence length.
        """
        if not isinstance(n, int) or isinstance(n, bool) or not 0 <= n < 2**64:
            raise ValueError("n must be an unsigned 64-bit integer")
        paulis = tuple(paulis)
        if len(paulis) != n:
            raise ValueError("The Pauli sequence must contain exactly n entries")
        if any(
            not isinstance(p, int) or isinstance(p, bool) or not 0 <= p <= 3
            for p in paulis
        ):
            raise ValueError("Paulis must be integers in {0, 1, 2, 3} (I, X, Y, Z)")
        # Rust's custom signature computation is not serialized in the JSON.
        return self.pauli_exp_def.instantiate(
            [BoundedNatArg(n), ListArg([BoundedNatArg(p) for p in paulis])],
            concrete_signature=FunctionType(
                [Qubit] * n + [RotationExtension().rotation], [Qubit] * n
            ),
        )
