"""GPU extension operations."""

import functools

from hugr.ext import Extension, OpDef, TypeDef
from hugr.ops import ExtOp
from hugr.tys import BoundedNatArg, ExtType, ListArg, StringArg, Type, TypeTypeArg

from ._util import TketExtension, load_extension


class GpuExtension(TketExtension):
    """GPU interop operations."""

    @functools.cache
    def __call__(self) -> Extension:
        """Returns the gpu extension"""
        return load_extension("tket.gpu")

    def TYPES(self) -> list[TypeDef]:
        """Return the types defined by this extension"""
        return [
            self.context.type_def,
            self.func_def,
            self.module.type_def,
            self.result_def,
        ]

    def OPS(self) -> list[OpDef]:
        """Return the operations defined by this extension"""
        return [
            self.call_def,
            self.dispose_context.op_def(),
            self.get_context.op_def(),
            self.lookup_by_id_def,
            self.lookup_by_name_def,
            self.read_result_def,
        ]

    @functools.cached_property
    def context(self) -> ExtType:
        """GPU context."""
        return self().get_type("context").instantiate([])

    @functools.cached_property
    def func_def(self) -> TypeDef:
        """GPU function type signature (generic definition).

        This is the generic type definition. For the instantiated type, see `func`.
        """
        return self().get_type("func")

    def func(self, inputs: list[Type], outputs: list[Type]) -> ExtType:
        """GPU function type signature (instantiated).

        Args:
            inputs: List of input types.
            outputs: List of output types.
        """
        return (
            self()
            .get_type("func")
            .instantiate(
                [
                    ListArg([TypeTypeArg(t) for t in inputs]),
                    ListArg([TypeTypeArg(t) for t in outputs]),
                ]
            )
        )

    @functools.cached_property
    def result_def(self) -> TypeDef:
        """GPU module."""
        return self().get_type("result")

    def result(self, outputs: list[Type]) -> ExtType:
        return self.result_def.instantiate([ListArg([TypeTypeArg(t) for t in outputs])])

    @functools.cached_property
    def module(self) -> ExtType:
        """GPU module."""
        return self().get_type("module").instantiate([])

    @functools.cached_property
    def call_def(self) -> OpDef:
        """Call a function in a context, returning a `Result`.

        This is the generic operation definition. For the instantiated operation, see
        `call`.
        """
        return self().get_op("call")

    def call(self, inputs: list[Type], outputs: list[Type]) -> ExtOp:
        """Call a function in a context, returning a `Result`.

        Args:
            inputs: Function input types.
            outputs: Function output types.
        """
        return self.call_def.instantiate(
            [
                ListArg([TypeTypeArg(t) for t in inputs]),
                ListArg([TypeTypeArg(t) for t in outputs]),
            ]
        )

    @functools.cached_property
    def dispose_context(self) -> ExtOp:
        """Dispose of a GPU context."""
        return self().get_op("dispose_context").instantiate()

    @functools.cached_property
    def get_context(self) -> ExtOp:
        """Retrieve a context by handle."""
        return self().get_op("get_context").instantiate()

    @functools.cached_property
    def lookup_by_id_def(self) -> OpDef:
        """Lookup a function in a module by id.

        This is the generic operation definition. For the instantiated operation, see
        `lookup_by_id`.
        """
        return self().get_op("lookup_by_id")

    def lookup_by_id(self, id: int, inputs: list[Type], outputs: list[Type]) -> ExtOp:
        """Lookup a function in a module by name and signature.

        Args:
            id: Function id to look up.
            inputs: Function input types.
            outputs: Function output types.
        """
        return self.lookup_by_id_def.instantiate(
            [
                BoundedNatArg(id),
                ListArg([TypeTypeArg(t) for t in inputs]),
                ListArg([TypeTypeArg(t) for t in outputs]),
            ]
        )

    @functools.cached_property
    def lookup_by_name_def(self) -> OpDef:
        """Lookup a function in a module by name.

        This is the generic operation definition. For the instantiated operation, see
        `lookup_by_name`.
        """
        return self().get_op("lookup_by_name")

    def lookup_by_name(
        self, name: str, inputs: list[Type], outputs: list[Type]
    ) -> ExtOp:
        """Lookup a function in a module by name and signature.

        Args:
            name: Function name to look up.
            inputs: Function input types.
            outputs: Function output types.
        """
        return self.lookup_by_name_def.instantiate(
            [
                StringArg(name),
                ListArg([TypeTypeArg(t) for t in inputs]),
                ListArg([TypeTypeArg(t) for t in outputs]),
            ]
        )

    @functools.cached_property
    def read_result_def(self) -> OpDef:
        """Read the result of a function call.

        This is the generic operation definition. For the instantiated operation, see
        `read_result`.
        """
        return self().get_op("read_result")

    def read_result(self, outputs: list[Type]) -> ExtOp:
        """Read the result of a function call.

        Args:
            outputs: The output types of the call
        """
        return self.read_result_def.instantiate(
            [ListArg([TypeTypeArg(t) for t in outputs])]
        )
