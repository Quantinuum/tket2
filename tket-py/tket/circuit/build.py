from __future__ import annotations
from typing import Iterable

from hugr import Hugr, tys, ops
from hugr.package import Package
from hugr.ext import Extension
from hugr.ops import ComWire, Command
from hugr.std.float import FLOAT_T
from hugr.build.function import Module
from hugr.build.tracked_dfg import TrackedDfg
from hugr.envelope import EnvelopeConfig
from tket.circuit import Tk2Circuit

from dataclasses import dataclass


class CircBuild(TrackedDfg):
    """Helper class to build a circuit from commands by tracking qubits,
    allowing commands to be specified by qubit index."""

    @classmethod
    def with_nqb(cls, n_qb: int) -> CircBuild:
        return cls(*[tys.Qubit] * n_qb, track_inputs=True)

    def finish_hugr(self) -> Hugr:
        """Finish building the hugr by setting all the qubits as the output.

        Returns:
            The finished Hugr.
        """
        return self.hugr

    def finish_package(
        self,
        *,
        other_extensions: Iterable[Extension] | None = None,
        function_name="main",
    ) -> Package:
        """Finish building the package by setting all the qubits as the output
        and wrap it in a hugr package with the required extensions.

        Args:
            other_extensions: Other extensions to include in the package.
            function_name: The name of the function containing the circuit in
            the package's module. Defaults to "main".

        Returns:
            The finished package.
        """
        # TODO: Replace with `finish_hugr` once extensions are included in the hugr itself.
        # See https://github.com/quantinuum/hugr/pull/1621
        import tket.extensions as ext

        extensions = [
            ext.rotation(),
            ext.futures(),
            ext.qsystem(),
            ext.quantum(),
            ext.result(),
            *(other_extensions or []),
        ]

        # Convert the DFG into a Function definition
        dfg_op = self.hugr[self.hugr.entrypoint].op
        assert type(dfg_op) is ops.DFG, "CircBuild must have a Dfg root"
        self.hugr[self.hugr.entrypoint].op = ops.FuncDefn(
            function_name, inputs=dfg_op.inputs, _outputs=dfg_op.outputs
        )

        # Insert it into a module, as required by the package.
        module = Module()
        module.hugr.insert_hugr(self.hugr)

        return Package(modules=[module.hugr], extensions=extensions)

    def finish(self, other_extensions: list[Extension] | None = None) -> Tk2Circuit:
        """Finish building the circuit by setting all the qubits as the output
        and validate."""

        return Tk2Circuit.from_str(
            self.finish_package(
                other_extensions=other_extensions, function_name="main"
            ).to_str(EnvelopeConfig.TEXT),
            "main",
        )


def from_coms(*args: Command) -> Tk2Circuit:
    """Build a circuit from a sequence of commands, assuming
    only qubits are referred to by index."""
    commands: list[Command] = []
    n_qb = 0
    # traverses commands twice which isn't great
    for arg in args:
        max_qb = max(i for i in arg.incoming if isinstance(i, int)) + 1
        n_qb = max(n_qb, max_qb)
        commands.append(arg)

    build = CircBuild.with_nqb(n_qb)
    build.extend(*commands)
    build.set_tracked_outputs()
    return build.finish()


def load_custom(serialized: bytes) -> ops.Custom:
    import hugr._serialization.ops as sops
    import json

    # TODO: We should return an "ExtOp" instead
    ext = json.loads(serialized)
    return sops.ExtensionOp(**ext).deserialize()


def id_circ(n_qb: int) -> Tk2Circuit:
    b = CircBuild.with_nqb(n_qb)
    b.set_tracked_outputs()
    return b.finish()


@dataclass(frozen=True)
class QuantumOps(ops.Custom):
    extension: tys.ExtensionId = "tket.quantum"


_OneQbSig = tys.FunctionType.endo([tys.Qubit])


@dataclass(frozen=True)
class OneQbGate(QuantumOps):
    op_name: str  # type: ignore[misc] # no-default fields follows one with a default
    num_out: int = 1
    signature: tys.FunctionType = _OneQbSig

    def __call__(self, q: ComWire) -> Command:
        return super().__call__(q)


H = OneQbGate("H")
PauliX = OneQbGate("X")
PauliY = OneQbGate("Y")
PauliZ = OneQbGate("Z")

_TwoQbSig = tys.FunctionType.endo([tys.Qubit] * 2)


@dataclass(frozen=True)
class TwoQbGate(QuantumOps):
    op_name: str  # type: ignore[misc] # no-default fields follows one with a default
    num_out: int = 2
    signature: tys.FunctionType = _TwoQbSig

    def __call__(self, q0: ComWire, q1: ComWire) -> Command:
        return super().__call__(q0, q1)


CX = TwoQbGate("CX")

_MeasSig = tys.FunctionType([tys.Qubit], [tys.Qubit, tys.Bool])


@dataclass(frozen=True)
class MeasureDef(QuantumOps):
    op_name: str = "Measure"
    num_out: int = 2
    signature: tys.FunctionType = _MeasSig

    def __call__(self, q: ComWire) -> Command:
        return super().__call__(q)


Measure = MeasureDef()

# TODO use angle type once extension is serialised.
_RzSig = tys.FunctionType([tys.Qubit, FLOAT_T], [tys.Qubit])


@dataclass(frozen=True)
class RzDef(QuantumOps):
    op_name: str = "Rz"
    num_out: int = 1
    signature: tys.FunctionType = _RzSig

    def __call__(self, q: ComWire, fl_wire: ComWire) -> Command:
        return super().__call__(q, fl_wire)


Rz = RzDef()


_QallocSig = tys.FunctionType([], [tys.Qubit])


@dataclass(frozen=True)
class QAllocDef(QuantumOps):
    op_name: str = "QAlloc"
    num_out: int = 1
    signature: tys.FunctionType = _QallocSig

    def __call__(self) -> Command:
        return super().__call__()


QAlloc = QAllocDef()


_QfreeSig = tys.FunctionType([tys.Qubit], [])


@dataclass(frozen=True)
class QFreeDef(QuantumOps):
    op_name: str = "QFree"
    num_out: int = 0
    signature: tys.FunctionType = _QfreeSig

    def __call__(self, qb: ComWire) -> Command:
        return super().__call__(qb)


QFree = QFreeDef()
