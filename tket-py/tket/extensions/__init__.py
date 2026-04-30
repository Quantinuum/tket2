from tket_exts import (
    debug,
    guppy,
    rotation,
    futures,
    qsystem,
    qsystem_random,
    qsystem_utils,
    quantum,
    result,
    wasm,
)

# TODO: Remove once tket no longer supports tket-exts 0.10.*
try:
    from tket_exts import bool  # type: ignore[attr-defined]
    from tket_exts import gpu  # type: ignore[attr-defined] # noqa: F401

    new_exts = ["gpu"]
except ImportError:
    new_exts = []


# TODO: Remove the deprecated `bool` export in a breaking change.
__all__ = [
    "debug",
    "bool",
    "guppy",
    "rotation",
    "futures",
    "qsystem",
    "qsystem_random",
    "qsystem_utils",
    "quantum",
    "result",
    "wasm",
    *new_exts,
]
