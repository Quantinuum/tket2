# Re-export native bindings

from hugr.ext import ExtensionRegistry
from .._tket.circuit import (
    Tk2Circuit,
    Node,
    Wire,
    CircuitCost,
    validate_circuit,
    render_circuit_dot,
    render_circuit_mermaid,
    HugrError,
    BuildError,
    ValidationError,
    HUGRSerializationError,
    TK1EncodeError,
    embedded_extensions,
)
from .build import CircBuild, Command

from hugr.hugr.base import Hugr
from hugr.package import Package

__all__ = [
    "CircBuild",
    "Command",
    # Bindings.
    # TODO: Wrap these in Python classes.
    "Tk2Circuit",
    "Node",
    "Wire",
    "CircuitCost",
    "validate_circuit",
    "render_circuit_dot",
    "render_circuit_mermaid",
    "HugrError",
    "BuildError",
    "ValidationError",
    "HUGRSerializationError",
    "TK1EncodeError",
]


def _hugr_to_tk2circuit(hugr: Hugr) -> tuple[Tk2Circuit, ExtensionRegistry]:
    """Convert a Hugr to a Tk2Circuit, including non-standard extensions.

    This wraps the hugr in a Package with its used extensions (excluding those
    already embedded in the Rust loader) so that non-standard extensions are
    properly encoded and can be decoded by Rust.

    Returns:
        A tuple containing the Tk2Circuit and the ExtensionRegistry required to
        extract the Hugr back.
    """
    # Get extensions used by this hugr that are not already in the Rust registry.
    embedded = set(embedded_extensions())
    res = hugr.used_extensions()
    extensions = [
        ext for id, ext in res.used_extensions.extensions.items() if id not in embedded
    ]
    # Wrap the hugr in a package with the non-standard extensions.
    package = Package(modules=[hugr], extensions=extensions)
    return Tk2Circuit.from_bytes(package.to_bytes()), res.used_extensions
