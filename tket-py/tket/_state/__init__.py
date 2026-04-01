# Re-export native bindings
from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from hugr.envelope import EnvelopeConfig
from hugr.ext import ExtensionRegistry
from .._tket import state as _state
from .build import CircBuild, Command

from hugr.hugr.base import Hugr
from hugr.package import Package

# Re-export types from the Rust module
Node = _state.Node
Wire = _state.Wire
CircuitCost = _state.CircuitCost
embedded_extensions = _state.embedded_extensions
HugrError = _state.HugrError
BuildError = _state.BuildError
ValidationError = _state.ValidationError
HUGRSerializationError = _state.HUGRSerializationError
TK1EncodeError = _state.TK1EncodeError

if TYPE_CHECKING:
    from tket._rewrite import CircuitRewrite


__all__ = [
    "CircBuild",
    "Command",
    # Bindings.
    # TODO: Wrap these in Python classes.
    "CompilationState",
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
class CompilationState:
    """A quantum circuit represented as a HUGR.

    This representation is optimized for compilation and rewriting. For building
    and direct manipulation of programs, the `hugr.Hugr` python class should be
    used instead.
    """

    _inner: _state.CompilationState = field(default_factory=_state.CompilationState)
    # Optional registry of python-defined extensions, used to load the hugr back
    # into Python.
    #
    # This is only an optimization to avoid having to encode extensions in the
    # serialization roundtrip.
    _py_extensions: ExtensionRegistry | None = None

    @staticmethod
    def from_tket1(circ) -> CompilationState:
        """Create a CompilationState from a legacy pytket Circuit."""
        return CompilationState(_inner=_state.CompilationState.from_tket1(circ))

    @staticmethod
    def from_python(hugr: Hugr | Package) -> CompilationState:
        """Convert a python-backed Hugr to a CompilationState."""
        py_extensions = None
        # Get extensions used by this hugr that are not already in the Rust registry.
        if isinstance(hugr, Hugr):
            embedded = set(_state.embedded_extensions())
            res = hugr.used_extensions()
            py_extensions = res.used_extensions
            extensions = [
                ext
                for ext in res.used_extensions.extensions
                if ext.name not in embedded
            ]
            # Wrap the hugr in a package with the non-standard extensions.
            package = Package(modules=[hugr], extensions=extensions)
        elif isinstance(hugr, Package):
            package = hugr
        else:
            raise ValueError(f"Expected a Hugr or Package, got {type(hugr)}")

        return CompilationState(
            _inner=_state.CompilationState.from_bytes(package.to_bytes()),
            _py_extensions=py_extensions,
        )

    def to_python(self) -> Package:
        """Convert this CompilationState back to a python Hugr package."""
        # Convert the inner hugr to bytes and load it in Python.
        hugr_bytes = self._inner.to_bytes()
        package = Package.from_bytes(hugr_bytes, self._py_extensions)
        if self._py_extensions is not None:
            # Resolve the extensions in the loaded package using the python registry, if needed.
            # TODO: Use the `package.resolve_extensions` for clarity once it's been released in `hugr-py 0.16.0`.
            package.used_extensions(self._py_extensions)
        return package

    @staticmethod
    def from_bytes(envelope: bytes) -> CompilationState:
        """Deserialize a byte string to a CompilationState.

        Some envelope formats can be read from a string. See :meth:`from_str`.

        Args:
            envelope: The byte string representing a Package.

        Returns:
            The loaded program.
        """
        # TODO: Allow passing an extension registry to use when loading the
        # envelope from the hugr side. This will require encoding the extensions
        # (as json), passing them, and loading them in
        # `_program.CompilationState.from_bytes` before parsing the envelope.
        #
        # Remember to filter out the embedded extensions from _program.embedded_extensions(),
        # since we use those already when loading things in Rust.

        return CompilationState(
            _inner=_state.CompilationState.from_bytes(envelope),
            _py_extensions=None,
        )

    @staticmethod
    def from_str(envelope: str) -> CompilationState:
        """Deserialize a string to a CompilationState.

        Not all envelope formats can be read from a string.
        See :meth:`from_bytes` for a more general method.

        Args:
            envelope: The string representing a Package.

        Returns:
            The loaded program.
        """
        return CompilationState(
            _inner=_state.CompilationState.from_str(envelope),
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

    def __copy__(self) -> CompilationState:
        """Copy the program."""
        import copy

        return CompilationState(copy.copy(self._inner), self._py_extensions)

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
