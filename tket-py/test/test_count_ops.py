from pathlib import Path
from tket.passes import NormalizeGuppy
from tket._state import CompilationState
import pytest


def load_example(example_name: str) -> CompilationState:
    """Load a guppy example and normalize it."""
    # Load the hugr file from test_files/guppy_examples
    hugr_path = (
        Path(__file__).parent.parent.parent
        / "test_files"
        / "guppy_examples"
        / f"{example_name}.hugr"
    )

    with open(hugr_path, "rb") as f:
        hugr_bytes = f.read()
    circ = CompilationState.from_bytes(hugr_bytes)

    # Normalize the guppy circuit before returning
    NormalizeGuppy()._run_tk(circ)
    return circ


testdata = [
    ("empty_func", 0),
    ("const_op", 0),
    ("one_rz", 2),
    ("loop_conditional", 5),
    ("conditional_loop", 5),
    ("fn_calls", 2),
    ("repeat_until_success", 21),
    ("extern_def", 1),
]


@pytest.mark.parametrize("example_name,expected", testdata)
def test_count_ops(example_name, expected):
    circ = load_example(example_name)
    assert circ.num_operations() == expected
