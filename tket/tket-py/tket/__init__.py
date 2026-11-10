"""TKET is an open source quantum compiler developed by Quantinuum.

Central to TKET's design is its hardware agnosticism which allows researchers
and quantum software developers to take advantage of its powerful compilation
for many different quantum architectures.

This is the Python front-end for TKET, providing a high-level interface for
working with quantum circuits. See also the Rust library with the same name on
[crates.io](https://crates.io/crates/tket).
"""

# On macOS devices importing tket._tket before pytket causes subsequent invocations of
# pytket to segfault (see https://github.com/Quantinuum/tket/issues/2267). Until that is
# resolved, preload pytket as a workaround.
import pytket  # noqa: I001 F401

from . import extensions, metadata, passes, platform, util

__all__ = [
    "extensions",
    "metadata",
    "passes",
    "platform",
    "util",
]


# Updated by the release-please workflow.
__version__ = "0.15.14"  # x-release-please-version
