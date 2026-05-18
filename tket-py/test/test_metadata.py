from hugr import Hugr, ops

from tket.metadata import (
    InlineAnnotation,
)


def test_inline_annotation_round_trip() -> None:
    hugr = Hugr[ops.Module]()
    node = hugr[hugr.module_root]

    node.metadata[InlineAnnotation] = "best_effort"

    assert node.metadata[InlineAnnotation] == "best_effort"
