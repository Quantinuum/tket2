"""Scope configuration for a pass.

A `PassScope` defines the parts of a HUGR that a pass should be applied to, and
which parts is it allowed to modify. Each variant defines three properties: `root`,
`preserve_interface` and `recursive`.

From these, `regions` and `in_scope` can be derived.

A pass will always optimize the entrypoint region, unless the entrypoint is the
module root.
"""

from hugr.passes.scope import (
    ABCEnumMeta,
    GlobalScope,
    InScope,
    LocalScope,
    PassScope,
    PassScopeBase,
)

__all__ = [
    "ABCEnumMeta",
    "GlobalScope",
    "InScope",
    "LocalScope",
    "PassScope",
    "PassScopeBase",
]
