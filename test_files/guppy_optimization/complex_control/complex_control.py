# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang >=0.21.6",
# ]
# ///
"""A complex Guppy control-flow example with helper calls and array updates."""

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.builtins import array


@guppy
def make_array(base: int) -> array[int, 6]:
    """Make a fixed-size array of integers starting from the input base."""
    return array(base + i for i in range(6))


@guppy
def increment_all(xs: array[int, 6], delta: int) -> array[int, 6]:
    """Increment all entries of the input array by a fixed delta."""
    return array(xs[i] + delta + i for i in range(6))


@guppy
def complex_control(check_first: bool, return_sum: bool) -> int:
    xs = make_array(42)
    total = 0
    do_loop = True

    while do_loop:
        xs = increment_all(xs, 1)

        if check_first:
            total = total + xs[0]
            xs = increment_all(xs, 2)
        else:
            total = total + xs[1]
            xs = increment_all(xs, 3)

        do_loop = total < 40

    if return_sum:
        return total + xs[0] + xs[1] + xs[2] + xs[3] + xs[4] + xs[5]

    return total


program = complex_control.compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
