# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang ==0.21.11",
# ]
# ///
"""Run on selene the passed hugrs"""

from pathlib import Path

from hugr import Hugr
from hugr.package import Package
from guppylang.emulator import EmulatorBuilder

# TODO: better folder
modifier_examples_dir = Path(__file__).resolve().parents[1] / "mermaid_output"
render_dir = Path(__file__).resolve().parents[1] / "0_hugr_pdf"

print(modifier_examples_dir)
for hugr_path in modifier_examples_dir.glob("*.hugr"):
    print(f"Processing {hugr_path}...")
    hugr_bytes = hugr_path.read_bytes()
    hugr = Hugr.from_bytes(hugr_bytes)
    # TODO: not need this
    hugr.render_dot().render(
        f"{hugr_path.stem}_read", directory=render_dir, cleanup=True
    )

    package = Package([hugr], extensions=[])

    builder = EmulatorBuilder()
    emulator = builder.build(package, n_qubits=5)
    res = emulator.with_shots(10).run().collated_counts()
    print(f"Results for {hugr_path.stem}:\n\t{res}")
