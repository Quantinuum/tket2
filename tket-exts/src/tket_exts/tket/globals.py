import functools
from typing import List

from hugr.ops import ExtOp
from hugr.tys import StringArg, TypeTypeArg, Type, TypeArg, ListArg

from ._util import TketExtension, load_extension
from hugr.ext import Extension, OpDef, TypeDef


class GlobalsExtension(TketExtension):
    """Global state operations."""

    @functools.cache
    def __call__(self) -> Extension:
        """Returns the globals extension"""
        return load_extension("tket.globals")

    def TYPES(self) -> List[TypeDef]:
        """Return the types defined by this extension"""
        return []

    def OPS(self) -> List[OpDef]:
        """Return the operations defined by this extension"""
        return [
            self.swap_def,
            self.with_def,
            self.map_def,
        ]

    @functools.cached_property
    def swap_def(self) -> OpDef:
        """Swap the contents of the named global variable with the argument."""
        return self().get_op("swap")

    @functools.cached_property
    def with_def(self) -> OpDef:
        """Set the global variable and run a function"""
        return self().get_op("with")

    @functools.cached_property
    def map_def(self) -> OpDef:
        """Map a function over the contents of the named global variable."""
        return self().get_op("map")

    def swap(self, name: str, ty: Type) -> ExtOp:
        """Swap the contents of the named global variable with the argument."""
        return self().get_op("swap").instantiate([StringArg(name), TypeTypeArg(ty)])

    def with_op(
        self, name: str, ty_arg: TypeArg, inputs: List[Type], outputs: List[Type]
    ) -> ExtOp:
        return (
            self()
            .get_op("with")
            .instantiate(
                [
                    StringArg(name),
                    ty_arg,
                    ListArg([TypeTypeArg(t) for t in inputs]),
                    ListArg([TypeTypeArg(t) for t in outputs]),
                ]
            )
        )

    def map(self, name: str, ty_arg: TypeArg, inputs, outputs) -> ExtOp:
        return (
            self()
            .get_op("map")
            .instantiate(
                [
                    StringArg(name),
                    ty_arg,
                    ListArg([TypeTypeArg(t) for t in inputs]),
                    ListArg([TypeTypeArg(t) for t in outputs]),
                ]
            )
        )
