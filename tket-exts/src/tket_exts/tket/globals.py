import functools

from hugr.ext import Extension, OpDef, TypeDef
from hugr.ops import ExtOp
from hugr.tys import ListArg, StringArg, Type, TypeArg, TypeTypeArg

from ._util import TketExtension, load_extension


class GlobalsExtension(TketExtension):
    """Global state operations."""

    @functools.cache
    def __call__(self) -> Extension:
        """Returns the globals extension"""
        return load_extension("tket.globals")

    def TYPES(self) -> list[TypeDef]:
        """Return the types defined by this extension"""
        return []

    def OPS(self) -> list[OpDef]:
        """Return the operations defined by this extension"""
        return [
            self.with_def,
            self.map_def,
        ]

    @functools.cached_property
    def with_def(self) -> OpDef:
        """Set the global variable and run a function"""
        return self().get_op("with")

    @functools.cached_property
    def map_def(self) -> OpDef:
        """Map a function over the contents of the named global variable."""
        return self().get_op("map")

    def with_op(
        self,
        name: str,
        ty_arg: TypeArg,
        inputs: list[Type],
        outputs: list[Type],
        impl_outputs: list[Type],
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
                    ListArg([TypeTypeArg(t) for t in impl_outputs]),
                ]
            )
        )

    def map(
        self,
        name: str,
        ty_arg: TypeArg,
        inputs: list[Type],
        outputs: list[Type],
        impl_outputs: list[Type],
    ) -> ExtOp:
        return (
            self()
            .get_op("map")
            .instantiate(
                [
                    StringArg(name),
                    ty_arg,
                    ListArg([TypeTypeArg(t) for t in inputs]),
                    ListArg([TypeTypeArg(t) for t in outputs]),
                    ListArg([TypeTypeArg(t) for t in impl_outputs]),
                ]
            )
        )
