"""A Protocol for a composable pass."""

from hugr.passes.composable import (
    ComposablePass,
    ComposedPass,
    PassName,
    PassResult,
    implement_pass_run,
)

__all__ = [
    "ComposablePass",
    "ComposedPass",
    "PassName",
    "PassResult",
    "implement_pass_run",
]
