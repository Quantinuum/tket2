from hugr import Hugr, ops

from tket.metadata import (
    InlineAnnotation,
    InlineAnnotationValue,
)


def test_inline_annotation_round_trip() -> None:
    hugr = Hugr[ops.Module]()
    node = hugr[hugr.module_root]

    node.metadata[InlineAnnotation] = InlineAnnotationValue.BEST_EFFORT

    assert node.metadata[InlineAnnotation.KEY] == "best_effort"
    assert node.metadata[InlineAnnotation] is InlineAnnotationValue.BEST_EFFORT
