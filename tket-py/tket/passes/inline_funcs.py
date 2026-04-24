"""Function inlining pass."""

from __future__ import annotations
from dataclasses import dataclass
from typing_extensions import Protocol


class InlineFuncsHeuristic(Protocol):
    """Marker protocol for inline-function heuristics."""


@dataclass(frozen=True)
class MaxSize(InlineFuncsHeuristic):
    """Inline non-recursive functions with at most `size` descendants."""

    size: int


@dataclass(frozen=True)
class All(InlineFuncsHeuristic):
    """Inline all non-recursive functions."""
