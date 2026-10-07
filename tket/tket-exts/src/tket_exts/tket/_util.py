"""Utility functions for tket extensions."""

import pkgutil
from typing import Protocol

from hugr.ext import Extension, OpDef, TypeDef
from semver import Version


def load_extension(name: str) -> Extension:
    import tket_exts

    replacement = name.replace(".", "/")
    json_str = pkgutil.get_data(tket_exts.__name__, f"data/{replacement}.json")
    assert json_str is not None, f"Could not load json for extension {name}"
    return Extension.from_json(json_str.decode())


class TketExtension(Protocol):
    """A protocol for tket extensions."""

    def TYPES(self) -> list[TypeDef]: ...
    def OPS(self) -> list[OpDef]: ...
    def __call__(self) -> Extension: ...

    @property
    def version(self) -> Version:
        """The version of the extension"""
        return self().version
