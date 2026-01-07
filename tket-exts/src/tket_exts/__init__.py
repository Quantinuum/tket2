"""HUGR extension definitions for tket circuits."""

from tket_exts.tket.bool import BoolExtension
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
    QSystemRandomExtension,
    QSystemUtilsExtension,
)
from tket_exts.tket.quantum import QuantumExtension
from tket_exts.tket.result import ResultExtension
from tket_exts.tket.wasm import WasmExtension

from typing_extensions import deprecated
from hugr.ext import Extension
from tket_exts import tket

# This is updated by our release-please workflow, triggered by this
# annotation: x-release-please-version
__version__ = "0.12.1"

__all__ = [
    "bool",
    "debug",
    "gpu",
    "guppy",
    "rotation",
    "futures",
    "qsystem",
    "qsystem_random",
    "qsystem_utils",
    "quantum",
    "result",
    "wasm",
    "modifier",
    "global_phase",
    "globals"
]

bool: BoolExtension = tket.bool.BoolExtension()
debug: DebugExtension = tket.debug.DebugExtension()
gpu: GpuExtension = tket.gpu.GpuExtension()
guppy: GuppyExtension = tket.guppy.GuppyExtension()
rotation: RotationExtension = tket.rotation.RotationExtension()
futures: FuturesExtension = tket.futures.FuturesExtension()
qsystem: QSystemExtension = tket.qsystem.QSystemExtension()
qsystem_random: QSystemRandomExtension = tket.qsystem.QSystemRandomExtension()
qsystem_utils: QSystemUtilsExtension = tket.qsystem.QSystemUtilsExtension()
quantum: QuantumExtension = tket.quantum.QuantumExtension()
result: ResultExtension = tket.result.ResultExtension()
wasm: WasmExtension = tket.wasm.WasmExtension()
modifier: ModifierExtension = tket.modifier.ModifierExtension()
global_phase: GlobalPhaseExtension = tket.global_phase.GlobalPhaseExtension()
globals: GlobalsExtension = tket.globals.GlobalsExtension()


@deprecated("Use tket_exts.bool() instead")
def opaque_bool() -> Extension:
    return bool()
