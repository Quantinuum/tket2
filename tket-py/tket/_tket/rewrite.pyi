from pathlib import Path
from tket._tket.state import Node, CompilationState

class ECCRewriter:
    @staticmethod
    def load_precompiled(filename: str | Path) -> ECCRewriter:
        """Load a precompiled rewriter from a file."""

    @staticmethod
    def compile_eccs(filename: str | Path) -> ECCRewriter:
        """Compile an ECC rewriter from a JSON file."""

    def get_rewrites(self, circ: CompilationState) -> list[CircuitRewrite]:
        """Get rewrites for a circuit."""

class CircuitRewrite:
    """A rewrite rule for circuits."""

    def __init__(
        self,
        source_position: Subcircuit,
        source_circ: CompilationState,
        replacement: CompilationState,
    ) -> None:
        """Create a new circuit rewrite rule."""

    def node_count_delta(self) -> int:
        """The change in node count from the rewrite.

        A positive value indicates an increase in node count.
        """

    def replacement(self) -> CompilationState:
        """The replacement circuit."""

class Subcircuit:
    """A subcircuit of a circuit."""

    def __init__(self, nodes: list[Node], circ: CompilationState) -> None:
        """Create a new subcircuit."""
