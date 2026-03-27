# Re-export native bindings
from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from hugr.envelope import EnvelopeConfig
from hugr.ext import ExtensionRegistry
from .._tket import program as _program
from .build import CircBuild, Command

from hugr.hugr.base import Hugr
from hugr.package import Package

# Re-export types from the Rust module
Node = _program.Node
Wire = _program.Wire
CircuitCost = _program.CircuitCost
embedded_extensions = _program.embedded_extensions
HugrError = _program.HugrError
BuildError = _program.BuildError
ValidationError = _program.ValidationError
HUGRSerializationError = _program.HUGRSerializationError
TK1EncodeError = _program.TK1EncodeError

if TYPE_CHECKING:
    from .rewrite import CircuitRewrite


__all__ = [
    "CircBuild",
    "Command",
    # Bindings.
    # TODO: Wrap these in Python classes.
    "TkProgram",
    "Node",
    "Wire",
    "CircuitCost",
    "embedded_extensions",
    "HugrError",
    "BuildError",
    "ValidationError",
    "HUGRSerializationError",
    "TK1EncodeError",
]


@dataclass
class TkProgram:
    """A quantum circuit represented as a HUGR.

    This representation is optimized for compilation and rewriting. For building
    and direct manipulation of programs, the `hugr.Hugr` python class should be
    used instead.
    """

    _inner: _program.TkProgram = field(default_factory=_program.TkProgram)
    # Optional registry of python-defined extensions, used to load the hugr back
    # into Python.
    #
    # This is only an optimization to avoid having to encode extensions in the
    # serialization roundtrip.
    _py_extensions: ExtensionRegistry | None = None

    @staticmethod
    def from_tket1(circ) -> TkProgram:
        """Create a TkProgram from a legacy pytket Circuit."""
        return TkProgram(_inner=_program.TkProgram.from_tket1(circ))

    @staticmethod
    def from_python(hugr: Hugr | Package) -> TkProgram:
        """Convert a python-backed Hugr to a TkProgram."""
        py_extensions = None
        # Get extensions used by this hugr that are not already in the Rust registry.
        if isinstance(hugr, Hugr):
            embedded = set(_program.embedded_extensions())
            res = hugr.used_extensions()
            py_extensions = res.used_extensions
            extensions = [
                ext
                for id, ext in res.used_extensions.extensions.items()
                if id not in embedded
            ]
            # Wrap the hugr in a package with the non-standard extensions.
            package = Package(modules=[hugr], extensions=extensions)
        elif isinstance(hugr, Package):
            package = hugr
        else:
            raise ValueError(f"Expected a Hugr or Package, got {type(hugr)}")

        return TkProgram(
            _inner=_program.TkProgram.from_bytes(package.to_bytes()),
            _py_extensions=py_extensions,
        )

    def to_python(self) -> Package:
        """Convert this TkProgram back to a python Hugr package."""
        # Convert the inner hugr to bytes and load it in Python.
        hugr_bytes = self._inner.to_bytes()
        package = Package.from_bytes(hugr_bytes, self._py_extensions)
        if self._py_extensions is not None:
            # TODO: Requires hugr-py release. Instead we resolve each hugr separately.
            # package.resolve_extensions(self._py_extensions)
            for module in package.modules:
                module.resolve_extensions(self._py_extensions)
        return package

    @staticmethod
    def from_bytes(envelope: bytes) -> TkProgram:
        """Deserialize a byte string to a TkProgram.

        Some envelope formats can be read from a string. See :meth:`from_str`.

        Args:
            envelope: The byte string representing a Package.

        Returns:
            The loaded program.
        """
        # TODO: Allow passing an extension registry to use when loading the
        # envelope from the hugr side. This will require encoding the extensions
        # (as json), passing them, and loading them in
        # `_program.TkProgram.from_bytes` before parsing the envelope.
        #
        # Remember to filter out the embedded extensions from _program.embedded_extensions(),
        # since we use those already when loading things in Rust.

        return TkProgram(
            _inner=_program.TkProgram.from_bytes(envelope),
            _py_extensions=None,
        )

    @staticmethod
    def from_str(envelope: str) -> TkProgram:
        """Deserialize a string to a TkProgram.

        Not all envelope formats can be read from a string.
        See :meth:`from_bytes` for a more general method.

        Args:
            envelope: The string representing a Package.

        Returns:
            The loaded program.
        """
        return TkProgram(
            _inner=_program.TkProgram.from_str(envelope),
            _py_extensions=None,
        )

    def to_bytes(self, config: EnvelopeConfig | None = None) -> bytes:
        """Serialize the program to a HUGR envelope byte string.

        Some envelope formats can be encoded into a string. See :meth:`to_str`.
        """
        return self._inner.to_bytes(config)

    def to_str(self, config: EnvelopeConfig | None = None) -> str:
        """Serialize the program to a HUGR envelope string.

        Not all envelope formats can be encoded into a string.
        See :meth:`to_bytes` for a more general method.
        """
        return self._inner.to_str(config)

    def apply_rewrite(self, rewrite: CircuitRewrite) -> None:
        """Apply a rewrite command to this program."""
        self._inner.apply_rewrite(rewrite)

    def __hash__(self) -> int:
        """Hash the program."""
        return self._inner.hash()

    def __copy__(self) -> TkProgram:
        """Copy the program."""
        import copy

        return TkProgram(copy.copy(self._inner), self._py_extensions)

    def render_mermaid(self) -> str:
        """Render the program as a mermaid string."""
        return self._inner.render_mermaid()

    def validate(self) -> None:
        """Validate the program."""
        self._inner.validate()

    def circuit_cost(self, cost_fn):
        """Compute the cost of the circuit based on a per-operation cost function."""
        return self._inner._circuit_cost(cost_fn)

    def num_operations(self) -> int:
        """Returns the number of operations in the circuit."""
        return self._inner.num_operations()

    def to_tket1(self):
        """Convert the program back to a legacy pytket Circuit."""
        return self._inner.to_tket1()
