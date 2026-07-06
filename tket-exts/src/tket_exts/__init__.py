"""HUGR extension definitions for tket circuits."""

from tket_exts.tket.debug import DebugExtension
from tket_exts.tket.global_phase import GlobalPhaseExtension
from tket_exts.tket.globals import GlobalsExtension
from tket_exts.tket.gpu import GpuExtension
from tket_exts.tket.guppy import GuppyExtension
from tket_exts.tket.modifier import ModifierExtension
from tket_exts.tket.rotation import RotationExtension
from tket_exts.tket.futures import FuturesExtension
from tket_exts.tket.qsystem import (
    QSystemExtension,
    QSystemHeliosExtension,
    QSystemRandomExtension,
    QSystemSolExtension,
    QSystemUtilsExtension,
)
from tket_exts.tket.quantum import QuantumExtension
from tket_exts.tket.result import ResultExtension
from tket_exts.tket.wasm import WasmExtension
from tket_exts.tket.measurement import MeasurementExtension
from tket_exts.tket.argument import ArgumentExtension

from hugr.ext import ExtensionRegistry
from tket_exts import tket

# This is updated by our release-please workflow, triggered by this
# annotation: x-release-please-version
__version__ = "0.14.0"

__all__ = [
    "debug",
    "gpu",
    "guppy",
    "rotation",
    "futures",
    "qsystem",
    "qsystem_helios",
    "qsystem_sol",
    "qsystem_random",
    "qsystem_utils",
    "quantum",
    "result",
    "wasm",
    "modifier",
    "global_phase",
    "globals",
    "measurement",
    "argument",
]

debug: DebugExtension = tket.debug.DebugExtension()
gpu: GpuExtension = tket.gpu.GpuExtension()
guppy: GuppyExtension = tket.guppy.GuppyExtension()
rotation: RotationExtension = tket.rotation.RotationExtension()
futures: FuturesExtension = tket.futures.FuturesExtension()
qsystem_helios: QSystemHeliosExtension = tket.qsystem.QSystemHeliosExtension()
qsystem_sol: QSystemSolExtension = tket.qsystem.QSystemSolExtension()
qsystem: QSystemExtension = tket.qsystem.QSystemExtension()
qsystem_random: QSystemRandomExtension = tket.qsystem.QSystemRandomExtension()
qsystem_utils: QSystemUtilsExtension = tket.qsystem.QSystemUtilsExtension()
quantum: QuantumExtension = tket.quantum.QuantumExtension()
result: ResultExtension = tket.result.ResultExtension()
wasm: WasmExtension = tket.wasm.WasmExtension()
modifier: ModifierExtension = tket.modifier.ModifierExtension()
global_phase: GlobalPhaseExtension = tket.global_phase.GlobalPhaseExtension()
globals: GlobalsExtension = tket.globals.GlobalsExtension()
measurement: MeasurementExtension = tket.measurement.MeasurementExtension()
argument: ArgumentExtension = tket.argument.ArgumentExtension()


def tket_registry() -> ExtensionRegistry:
    """Returns an ExtensionRegistry containing all the tket extensions.

    This can be used when loading a Hugr containing tket operations and types

    Returns:
        An ExtensionRegistry containing all the tket extensions.
    """
    tket_exts = [
        tket.debug.DebugExtension(),
        tket.gpu.GpuExtension(),
        tket.guppy.GuppyExtension(),
        tket.rotation.RotationExtension(),
        tket.futures.FuturesExtension(),
        tket.globals.GlobalsExtension(),
        tket.qsystem.QSystemHeliosExtension(),
        tket.qsystem.QSystemSolExtension(),
        tket.qsystem.QSystemExtension(),
        tket.qsystem.QSystemRandomExtension(),
        tket.qsystem.QSystemUtilsExtension(),
        tket.quantum.QuantumExtension(),
        tket.result.ResultExtension(),
        tket.wasm.WasmExtension(),
        tket.modifier.ModifierExtension(),
        tket.global_phase.GlobalPhaseExtension(),
        tket.measurement.MeasurementExtension(),
        tket.argument.ArgumentExtension(),
    ]

    registry = ExtensionRegistry()
    for ext in tket_exts:
        registry.register(ext())
    return registry
