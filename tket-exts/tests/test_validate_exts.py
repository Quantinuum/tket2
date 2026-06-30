from typing import Callable, List, Tuple
from hugr.ops import ExtOp
from hugr.tys import ExtType, Bool
import tket_exts

import pytest
from tket_exts.tket._util import TketExtension


def ext_debug() -> Tuple[TketExtension, List[ExtType], List[ExtOp]]:
    ext = tket_exts.debug
    return (
        ext,
        [],
        [ext.state_result("test", 1)],
    )


def ext_gpu() -> Tuple[TketExtension, List[ExtType], List[ExtOp]]:
    ext = tket_exts.gpu
    return (
        ext,
        [ext.context, ext.func([], []), ext.module, ext.result([])],
        [
            ext.call([], []),
            ext.dispose_context,
            ext.get_context,
            ext.lookup_by_id(42, [], []),
            ext.lookup_by_name("test", [], []),
            ext.read_result([]),
        ],
    )


def ext_guppy() -> Tuple[TketExtension, List[ExtType], List[ExtOp]]:
    ext = tket_exts.guppy
    rot_t = tket_exts.rotation.rotation  # Arbitrary non-linear type for testing.
    return (
        ext,
        [],
        [ext.drop(rot_t)],
    )


def ext_futures() -> Tuple[TketExtension, List[ExtType], List[ExtOp]]:
    ext = tket_exts.futures
    rot_t = tket_exts.rotation.rotation  # Arbitrary non-linear type for testing.
    return (
        ext,
        [ext.future_t(rot_t)],
        [ext.dup(rot_t), ext.free(rot_t), ext.read(rot_t)],
    )


def ext_measurement() -> Tuple[TketExtension, List[ExtType], List[ExtOp]]:
    ext = tket_exts.measurement
    return (
        ext,
        [ext.measurement_t],
        [ext.read],
    )


def ext_globals() -> Tuple[TketExtension, List[ExtType], List[ExtOp]]:
    ext = tket_exts.globals
    return (
        ext,
        [],
        [
            ext.with_op("test-name", Bool.type_arg(), [], [Bool, Bool], []),
            ext.map("test-name", Bool.type_arg(), [Bool, Bool], [Bool, Bool], []),
        ],
    )


def ext_qsystem_helios() -> Tuple[TketExtension, List[ExtType], List[ExtOp]]:
    ext = tket_exts.qsystem_helios
    return (
        ext,
        [],
        [
            ext.lazy_measure,
            ext.lazy_measure_leaked,
            ext.lazy_measure_reset,
            ext.phasedX,
            ext.qFree,
            ext.reset,
            ext.runtime_barrier(1),
            ext.Rz,
            ext.try_QAlloc,
            ext.ZZPhase,
            ext.future_to_measurement,
        ],
    )


def ext_qsystem_sol() -> Tuple[TketExtension, List[ExtType], List[ExtOp]]:
    ext = tket_exts.qsystem_sol
    return (
        ext,
        [],
        [
            ext.lazy_measure,
            ext.lazy_measure_leaked,
            ext.lazy_measure_reset,
            ext.phasedX,
            ext.phasedXX,
            ext.qFree,
            ext.reset,
            ext.runtime_barrier(1),
            ext.Rz,
            ext.try_QAlloc,
            ext.future_to_measurement,
        ],
    )


def ext_qsystem_random() -> Tuple[TketExtension, List[ExtType], List[ExtOp]]:
    ext = tket_exts.qsystem_random
    return (
        ext,
        [ext.context],
        [
            ext.delete_RNGContext,
            ext.new_RNGContext,
            ext.random_float,
            ext.random_int,
            ext.random_int_bounded,
            ext.random_advance,
        ],
    )


def ext_qsystem_utils() -> Tuple[TketExtension, List[ExtType], List[ExtOp]]:
    ext = tket_exts.qsystem_utils
    return (
        ext,
        [],
        [ext.get_current_shot],
    )


def ext_quantum() -> Tuple[TketExtension, List[ExtType], List[ExtOp]]:
    ext = tket_exts.quantum
    return (
        ext,
        [],
        [
            ext.CRz,
            ext.CX,
            ext.CY,
            ext.CZ,
            ext.H,
            ext.measure,
            ext.measure_free,
            ext.qAlloc,
            ext.qFree,
            ext.reset,
            ext.Rx,
            ext.Ry,
            ext.Rz,
            ext.S,
            ext.Sdg,
            ext.T,
            ext.Tdg,
            ext.toffoli,
            ext.try_QAlloc,
            ext.V,
            ext.Vdg,
            ext.X,
            ext.Y,
            ext.Z,
            ext.symbolic_angle("test"),
        ],
    )


def ext_result() -> Tuple[TketExtension, List[ExtType], List[ExtOp]]:
    ext = tket_exts.result
    return (
        ext,
        [],
        [
            ext.result_array_bool("test", 1),
            ext.result_array_f64("test", 1),
            ext.result_array_int("test", 1, 1),
            ext.result_array_uint("test", 1, 1),
            ext.result_bool("test"),
            ext.result_f64("test"),
            ext.result_int("test", 1),
            ext.result_uint("test", 1),
        ],
    )


def ext_rotation() -> Tuple[TketExtension, List[ExtType], List[ExtOp]]:
    ext = tket_exts.rotation
    return (
        ext,
        [ext.rotation],
        [ext.from_halfturns, ext.from_halfturns_unchecked, ext.radd, ext.to_halfturns],
    )


def ext_wasm() -> Tuple[TketExtension, List[ExtType], List[ExtOp]]:
    ext = tket_exts.wasm
    return (
        ext,
        [ext.context, ext.func([], []), ext.module, ext.result([])],
        [
            ext.call([], []),
            ext.dispose_context,
            ext.get_context,
            ext.lookup_by_id(42, [], []),
            ext.lookup_by_name("test", [], []),
            ext.read_result([]),
        ],
    )


def ext_argument() -> Tuple[TketExtension, List[ExtType], List[ExtOp]]:
    ext = tket_exts.argument
    return (
        ext,
        [],
        [ext.read_arg("test", Bool)],
    )


@pytest.mark.parametrize(
    "ext_vals",
    [
        ext_debug,
        ext_gpu,
        ext_guppy,
        ext_futures,
        ext_globals,
        ext_measurement,
        ext_qsystem_helios,
        ext_qsystem_sol,
        ext_qsystem_random,
        ext_qsystem_utils,
        ext_quantum,
        ext_result,
        ext_rotation,
        ext_wasm,
        ext_argument,
    ],
)
def test_exported_extension(
    ext_vals: Callable[[], Tuple[TketExtension, List[ExtType], List[ExtOp]]],
):
    (ext, instantiated_types, instantiated_ops) = ext_vals()

    e = ext()
    assert e.version == ext.version

    types = ext.TYPES()
    assert len(types) == len(e.types)
    assert len(instantiated_types) == len(types), (
        "Please add missing type tests for " + e.name
    )
    for ty in instantiated_types:
        assert ty.type_def.name in e.types
        assert len(ty.type_def.params) == len(ty.args)

    ops = ext.OPS()
    assert len(ops) == len(e.operations)
    assert len(instantiated_ops) == len(ops), (
        "Please add missing op tests for " + e.name
    )
    for op in instantiated_ops:
        assert op.op_def().name in e.operations

    return
