import pytest
from hugr import ops, tys
from hugr.build import Dfg
from hugr.package import Package
from tket_exts import cell, tket_registry


@pytest.mark.parametrize("payload", [tys.Bool, tys.Qubit])
def test_cell_roundtrip(payload: tys.Type) -> None:
    handle = cell.cell_t(payload)
    assert handle.type_bound() == tys.TypeBound.Linear
    graph = Dfg(handle)
    arg = payload.type_arg()
    dup = (
        cell()
        .get_op("Dup")
        .instantiate([arg], tys.FunctionType([handle], [handle, handle]))
    )
    graph.set_outputs(*graph.add_op(dup, *graph.inputs()))
    package = Package.from_bytes(graph.hugr.to_package().to_bytes(), tket_registry())
    root = package.modules[0].entrypoint_op()
    assert isinstance(root, ops.DFG)
    assert root.signature.output == [handle, handle]


def test_cell_definitions() -> None:
    assert {op.name for op in cell.OPS()} == {
        "New",
        "Read",
        "Write",
        "Swap",
        "Dup",
        "Eq",
        "Free",
        "Map",
    }
    assert cell.TYPES() == [cell().get_type("cell")]
