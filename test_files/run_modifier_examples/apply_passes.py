import sys
from collections.abc import Iterator
from pathlib import Path

from hugr.build.base import Hugr
from hugr.ops import Custom, ExtOp
from tket._state import CompilationState
from tket.passes import ModifierResolverPass, Normalize
from tket_exts import modifier

mr_pass = ModifierResolverPass()
normalize = Normalize()


def _hugr_from_path(str_path: str) -> Hugr:
    with open(Path(str_path), "rb") as f:
        h = Hugr.from_bytes(f.read())

    return h


def apply_passes(input_paths: list[Path] | Iterator[Path], output_dir: Path) -> None:
    modifier_extension_name = modifier().name
    for input_path in input_paths:
        print(f"Processing {input_path.name}")
        hugr = _hugr_from_path(str(input_path))
        resolved: Hugr = mr_pass(hugr)

        from hugr.hugr.render import RenderConfig

        resolved.render_dot(RenderConfig(max_node_label_length=None)).view(
            f"{input_path.stem}_resolved"
        )

        assert not any(
            (
                isinstance(data.op, Custom)
                and data.op.extension == modifier_extension_name
            )
            or (
                isinstance(data.op, ExtOp)
                and data.op.op_def().get_extension().name == modifier_extension_name
            )
            for _, data in resolved.nodes()
        ), f"Unresolved modifier nodes remain in {input_path.name}"
        CompilationState.from_python(resolved).validate()
        output_path = output_dir / f"{input_path.stem}_solved.hugr"
        output_path.write_bytes(resolved.to_bytes())


if __name__ == "__main__":
    modifier_examples_dir = Path(__file__).resolve().parents[1] / "modifier_examples"
    modified_hugrs_dir = Path(__file__).resolve().parent / "modified_hugrs"
    modified_hugrs_dir.mkdir(parents=True, exist_ok=True)
    input_paths = (
        [modifier_examples_dir / (sys.argv[1] + ".hugr")]
        if len(sys.argv) > 1
        else modifier_examples_dir.glob("*.hugr")
    )
    apply_passes(input_paths, modified_hugrs_dir)
