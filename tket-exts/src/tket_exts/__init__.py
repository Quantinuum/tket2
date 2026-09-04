"""HUGR extension definitions for tket circuits."""

from collections.abc import Callable

from hugr.ext import Extension, ExtensionRegistry

from tket_exts import tket
from tket_exts.tket.argument import ArgumentExtension
from tket_exts.tket.debug import DebugExtension
from tket_exts.tket.futures import FuturesExtension
from tket_exts.tket.global_phase import GlobalPhaseExtension
from tket_exts.tket.globals import GlobalsExtension
from tket_exts.tket.gpu import GpuExtension
from tket_exts.tket.guppy import GuppyExtension
from tket_exts.tket.measurement import MeasurementExtension
from tket_exts.tket.modifier import ModifierExtension
from tket_exts.tket.qsystem import (
    QSystemExtension,
    QSystemHeliosExtension,
    QSystemRandomExtension,
    QSystemSolExtension,
    QSystemUtilsExtension,
)
from tket_exts.tket.quantum import QuantumExtension
from tket_exts.tket.result import ResultExtension
from tket_exts.tket.rotation import RotationExtension
from tket_exts.tket.tket1 import Tket1Extension
from tket_exts.tket.wasm import WasmExtension

# This is updated by our release-please workflow, triggered by this
# annotation: x-release-please-version
__version__ = "0.14.2"

__all__ = [
    "argument",
    "debug",
    "futures",
    "global_phase",
    "globals",
    "gpu",
    "guppy",
    "measurement",
    "modifier",
    "qsystem",
    "qsystem_helios",
    "qsystem_random",
    "qsystem_sol",
    "qsystem_utils",
    "quantum",
    "result",
    "rotation",
    "tket1",
    "wasm",
]

debug: DebugExtension = tket.debug.DebugExtension()
gpu: GpuExtension = tket.gpu.GpuExtension()
guppy: GuppyExtension = tket.guppy.GuppyExtension()
rotation: RotationExtension = tket.rotation.RotationExtension()
futures: FuturesExtension = tket.futures.FuturesExtension()
qsystem_helios: QSystemHeliosExtension = tket.qsystem.QSystemHeliosExtension()
qsystem_sol: QSystemSolExtension = tket.qsystem.QSystemSolExtension()
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
tket1: Tket1Extension = tket.tket1.Tket1Extension()

# TODO (deprecated): Remove the deprecated tket.qsystem extension in the next breaking release.
qsystem: QSystemExtension = tket.qsystem.QSystemExtension()


def tket_registry() -> ExtensionRegistry:
    """Returns an ExtensionRegistry containing all the tket extensions.

    This can be used when loading a Hugr containing tket operations and types

    Returns:
        An ExtensionRegistry containing all the tket extensions.
    """
    tket_exts: list[Callable[[], Extension]] = [
        tket.debug.DebugExtension(),
        tket.gpu.GpuExtension(),
        tket.guppy.GuppyExtension(),
        tket.rotation.RotationExtension(),
        tket.futures.FuturesExtension(),
        tket.globals.GlobalsExtension(),
        tket.qsystem.QSystemHeliosExtension(),
        tket.qsystem.QSystemSolExtension(),
        qsystem._extension,
        tket.qsystem.QSystemRandomExtension(),
        tket.qsystem.QSystemUtilsExtension(),
        tket.quantum.QuantumExtension(),
        tket.result.ResultExtension(),
        tket.wasm.WasmExtension(),
        tket.modifier.ModifierExtension(),
        tket.global_phase.GlobalPhaseExtension(),
        tket.measurement.MeasurementExtension(),
        tket.argument.ArgumentExtension(),
        tket.tket1.Tket1Extension(),
    ]

    registry = ExtensionRegistry()
    for ext in tket_exts:
        registry.register(ext())
    return registry
