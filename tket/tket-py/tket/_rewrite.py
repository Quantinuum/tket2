# Re-export native bindings
# TODO: Wrap these in Python classes.
from ._tket.rewrite import CircuitRewrite, ECCRewriter, Subcircuit

__all__ = [
    "CircuitRewrite",
    "ECCRewriter",
    "Rewriter",
    "Subcircuit",
    "default_ecc_rewriter",
]

Rewriter = ECCRewriter | list["Rewriter"]


def default_ecc_rewriter() -> ECCRewriter:
    """Load the default ecc rewriter."""
    try:
        import tket_eccs
    except ImportError:
        raise ValueError(
            "The default rewriter is not available. Please specify a path to a rewriter or install tket-eccs."
        )

    rewriter = tket_eccs.nam_6_3()
    return ECCRewriter.load_precompiled(rewriter)
