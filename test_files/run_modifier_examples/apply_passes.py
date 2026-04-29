from tket.passes import (
    NormalizeGuppy,
    ModifierResolverPass,
)


from hugr.build.base import Hugr


from pathlib import Path
import sys


normalize = NormalizeGuppy()


def _hugr_from_path(str_path: str) -> Hugr:
    with open(Path(str_path), "rb") as f:
        h = Hugr.from_bytes(f.read())

    return h


mr_pass = ModifierResolverPass()
modifier_examples_dir = Path(__file__).resolve().parents[1] / "modifier_examples"
modified_hugrs_dir = Path(__file__).resolve().parents[1] / "modified_hugrs"
modified_hugrs_dir.mkdir(parents=True, exist_ok=True)


input_paths = (
    [modifier_examples_dir / (sys.argv[1] + ".hugr")]
    if len(sys.argv) > 1
    else modifier_examples_dir.glob("*.hugr")
)

for input_path in input_paths:
    print(f"Processing {input_path.name}")
    modifier_hugr = _hugr_from_path(str(input_path))
    normalized = normalize(modifier_hugr)
    resolved: Hugr = mr_pass(normalized)

    output_path = modified_hugrs_dir / f"{input_path.stem}_solved.hugr"
    output_path.write_bytes(resolved.to_bytes())
