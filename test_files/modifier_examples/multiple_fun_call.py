# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.2",
# ]
# ///
"""
Test multiple function calls with modifiers.
Issue: https://github.com/Quantinuum/guppylang/issues/2461
"""

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.builtins import control, dagger
from guppylang.std.debug import state_result
from guppylang.std.quantum import discard, h, measure, qubit


@guppy(unitary=True)
def f(c: qubit, q: qubit) -> None:
    with control(c):
        h(q)


@guppy
def main() -> None:
    c = qubit()
    q = qubit()
    f(c, q)
    with dagger:
        f(c, q)
    for _ in range(3):
        f(c, q)
        with dagger:
            f(c, q)
    state_result("r", q)
    discard(c)
    discard(q)


program = main.with_minimal_opt().compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
