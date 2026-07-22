"""HUGR extension definitions for tket circuits.

Extensions may be imported directly as follows:

>>> import tket.extensions as ext
>>>
>>> argument_extension = ext.argument()
>>> debug_extension = ext.debug()
>>> futures_extension = ext.futures()
>>> global_phase_extension = ext.global_phase()
>>> gpu_extension = ext.gpu()
>>> guppy_extension = ext.guppy()
>>> measurement_extension = ext.measurement()
>>> modifier_extension = ext.modifier()
>>> qsystem_extension = ext.qsystem()
>>> qsystem_helios_extension = ext.qsystem_helios()
>>> qsystem_sol_extension = ext.qsystem_sol()
>>> qsystem_random_extension = ext.qsystem_random()
>>> qsystem_utils_extension = ext.qsystem_utils()
>>> quantum_extension = ext.quantum()
>>> result_extension = ext.result()
>>> rotation_extension = ext.rotation()
>>> wasm_extension = ext.wasm()
"""

from tket_exts import (
    argument,
    debug,
    futures,
    global_phase,
    gpu,
    guppy,
    measurement,
    modifier,
    qsystem,
    qsystem_helios,
    qsystem_sol,
    qsystem_random,
    qsystem_utils,
    quantum,
    result,
    rotation,
    wasm,
)
from tket_exts.tket.argument import ArgumentExtension
from tket_exts.tket.debug import DebugExtension
from tket_exts.tket.futures import FuturesExtension
from tket_exts.tket.global_phase import GlobalPhaseExtension
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
from tket_exts.tket.wasm import WasmExtension

__all__ = [
    "argument",
    "debug",
    "futures",
    "global_phase",
    "gpu",
    "guppy",
    "measurement",
    "modifier",
    "qsystem",
    "qsystem_helios",
    "qsystem_sol",
    "qsystem_random",
    "qsystem_utils",
    "quantum",
    "result",
    "rotation",
    "wasm",
    "ArgumentExtension",
    "DebugExtension",
    "FuturesExtension",
    "GlobalPhaseExtension",
    "GpuExtension",
    "GuppyExtension",
    "MeasurementExtension",
    "ModifierExtension",
    "QSystemExtension",
    "QSystemHeliosExtension",
    "QSystemRandomExtension",
    "QSystemSolExtension",
    "QSystemUtilsExtension",
    "QuantumExtension",
    "ResultExtension",
    "RotationExtension",
    "WasmExtension",
]
