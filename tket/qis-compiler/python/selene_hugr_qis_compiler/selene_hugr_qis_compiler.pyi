from typing import Literal

class EmulatorState:
    """A validated HUGR prepared once for a fixed emulator platform."""

    def __init__(
        self,
        pkg_bytes: bytes,
        *,
        platform: Literal["helios", "sol"] = "helios",
    ) -> None: ...
    def compile_to_bitcode(
        self,
        *,
        opt_level: int = 2,
        target_triple: str = "native",
        emit_debug: bool = False,
    ) -> bytes: ...
    def compile_to_llvm_ir(
        self,
        *,
        opt_level: int = 2,
        target_triple: str = "native",
        emit_debug: bool = False,
    ) -> str: ...

def check_hugr(pkg_bytes: bytes) -> None:
    """Load serialized HUGR and validate it.

    Raises:
        HugrReadError if the HUGR is invalid.
    """
    ...

def embedded_extensions() -> list[tuple[str, str]]:
    """Return extension names and versions available to the native loader."""
    ...

def has_compatible_extension(name: str, version: str) -> bool:
    """Return whether the native loader can provide a compatible extension."""
    ...

class HugrReadError(Exception):
    """Raised when reading HUGR fails"""
