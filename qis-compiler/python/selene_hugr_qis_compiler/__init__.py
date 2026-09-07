"""Python interface for compiling HUGRs to Quantinuum QIS."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal, TypeAlias

from hugr.hugr import Hugr
from hugr.package import Package
from typing_extensions import deprecated

from . import selene_hugr_qis_compiler as _native

Platform: TypeAlias = Literal["helios", "sol"]

HugrReadError = _native.HugrReadError

__all__ = [
    "EmulatorState",
    "HugrReadError",
    "Platform",
    "check_hugr",
    "compile_to_bitcode",
    "compile_to_llvm_ir",
]


@dataclass(frozen=True, init=False)
class EmulatorState:
    """A validated HUGR prepared once for a fixed emulator platform."""

    _inner: _native.EmulatorState = field(repr=False)

    def __init__(self) -> None:
        """Prevent construction without selecting an explicit input format."""
        raise TypeError("Use EmulatorState.from_bytes or EmulatorState.from_python")

    @classmethod
    def from_bytes(
        cls, envelope: bytes, *, platform: Platform = "helios"
    ) -> EmulatorState:
        """Load, validate, and lower a serialized HUGR package."""
        state = object.__new__(cls)
        object.__setattr__(
            state, "_inner", _native.EmulatorState(envelope, platform=platform)
        )
        return state

    @classmethod
    def from_python(
        cls, hugr: Hugr | Package, *, platform: Platform = "helios"
    ) -> EmulatorState:
        """Validate and lower a Python HUGR."""
        if isinstance(hugr, Hugr):
            used_extensions = hugr.used_extensions().used_extensions
            extensions = [
                extension
                for extension in used_extensions.extensions
                if not _native.has_compatible_extension(
                    str(extension.name), str(extension.version)
                )
            ]
            package = Package(modules=[hugr], extensions=extensions)
        elif isinstance(hugr, Package):
            package = hugr
        else:
            raise TypeError(f"Expected a Hugr or Package, got {type(hugr)}")

        return cls.from_bytes(package.to_bytes(), platform=platform)

    def compile_to_llvm_ir(
        self,
        *,
        opt_level: int = 2,
        target_triple: str = "native",
        emit_debug: bool = False,
    ) -> str:
        """Emit the prepared HUGR as LLVM IR."""
        return self._inner.compile_to_llvm_ir(
            opt_level=opt_level,
            target_triple=target_triple,
            emit_debug=emit_debug,
        )

    def compile_to_bitcode(
        self,
        *,
        opt_level: int = 2,
        target_triple: str = "native",
        emit_debug: bool = False,
    ) -> bytes:
        """Emit the prepared HUGR as LLVM bitcode."""
        return self._inner.compile_to_bitcode(
            opt_level=opt_level,
            target_triple=target_triple,
            emit_debug=emit_debug,
        )


@deprecated("Use `EmulatorState.from_bytes` instead.")
def check_hugr(pkg_bytes: bytes) -> None:
    """Load and validate a HUGR without applying emulator lowering.

    Deprecated: use :meth:`EmulatorState.from_bytes`, which also verifies that
    the HUGR can be lowered for the selected emulator platform.
    """
    _native.check_hugr(pkg_bytes)


@deprecated("Use `EmulatorState.from_bytes(...).compile_to_llvm_ir(...)` instead.")
def compile_to_llvm_ir(
    pkg_bytes: bytes,
    *,
    opt_level: int = 2,
    target_triple: str = "native",
    platform: Platform = "helios",
    emit_debug: bool = False,
) -> str:
    """Compile serialized HUGR to LLVM IR.

    Deprecated: use :meth:`EmulatorState.from_bytes` followed by
    :meth:`EmulatorState.compile_to_llvm_ir`.
    """
    return EmulatorState.from_bytes(pkg_bytes, platform=platform).compile_to_llvm_ir(
        opt_level=opt_level,
        target_triple=target_triple,
        emit_debug=emit_debug,
    )


@deprecated("Use `EmulatorState.from_bytes(...).compile_to_bitcode(...)` instead.")
def compile_to_bitcode(
    pkg_bytes: bytes,
    *,
    opt_level: int = 2,
    target_triple: str = "native",
    platform: Platform = "helios",
    emit_debug: bool = False,
) -> bytes:
    """Compile serialized HUGR to LLVM bitcode.

    Deprecated: use :meth:`EmulatorState.from_bytes` followed by
    :meth:`EmulatorState.compile_to_bitcode`.
    """
    return EmulatorState.from_bytes(pkg_bytes, platform=platform).compile_to_bitcode(
        opt_level=opt_level,
        target_triple=target_triple,
        emit_debug=emit_debug,
    )


# This is updated by our release-please workflow, triggered by this
# annotation: x-release-please-version
__version__ = "0.5.0"
