# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang ==0.21.11",
# ]
# ///
"""Run on selene the passed hugrs"""

from pathlib import Path
import sys

from hugr import Hugr
from hugr.package import Package
from guppylang.emulator import EmulatorBuilder

sys.path.append(str(Path(__file__).resolve().parents[1]))
from utility import hugr_pdf_directory

modifier_examples_dir = Path(__file__).resolve().parents[1] / "modified_hugrs"

print(modifier_examples_dir)
all_results: list[str] = []
for hugr_path in modifier_examples_dir.glob("*.hugr"):
    print(f"Processing {hugr_path}...")
    hugr_bytes = hugr_path.read_bytes()
    hugr = Hugr.from_bytes(hugr_bytes)
    hugr.render_dot().render(
        f"{hugr_path.stem}", directory=hugr_pdf_directory, cleanup=True
    )

    package = Package([hugr], extensions=[])

    builder = EmulatorBuilder()
    emulator = builder.build(package, n_qubits=5)
    res = emulator.with_shots(10).run().collated_counts()
    all_results.append(f"{hugr_path.stem}: {res}")

result_path = Path("hugr_results.txt")
result_path.write_text("\n".join(all_results) + "\n")
print(f"Results saved to {result_path}")
