from pathlib import Path
from tket.passes import normalize_guppy
from tket.circuit import Tk2Circuit
import pytest


def load_example(example_name: str) -> Tk2Circuit:
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
    circ = Tk2Circuit.from_bytes(hugr_bytes)

    # Normalize the guppy circuit before returning
    return normalize_guppy(circ)


testdata = [
    ("empty_func", 0),
    ("const_op", 0),
    ("one_rz", 2),
    ("loop_conditional", 8),
    ("conditional_loop", 8),
    ("fn_calls", 2),
    ("repeat_until_success", 21),
    # TODO: Requires this fix from hugr-rs
    # <https://github.com/Quantinuum/hugr/pull/2986>
    # ("extern_def", 2),
]


@pytest.mark.parametrize("example_name,expected", testdata)
def test_count_ops(example_name, expected):
    circ = load_example(example_name)
    assert circ.num_operations() == expected
