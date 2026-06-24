from typing import Iterator
from hugr.passes.scope import PassScope
from .state import Node, CompilationState
from .rewrite import CircuitRewrite

class Rule:
    """A rewrite rule defined by a left hand side and right hand side of an equation."""

    def __init__(
        self,
        l: CompilationState,  # noqa: E741
        r: CompilationState,
    ) -> None:
        """Create a new rewrite rule."""

    def lhs(self) -> CompilationState:
        """Get the left hand side of the rule.

        This is the pattern that is matched in the circuit.
        """

    def rhs(self) -> CompilationState:
        """Get the right hand side of the rule.

        This is the pattern that is replaced in the circuit.
        """

class RuleMatcher:
    """A matcher for multiple rewrite rule."""

    def __init__(self, rules: list[Rule]) -> None:
        """Create a new rule matcher."""

    def find_match(self, circ: CompilationState) -> CircuitRewrite | None:
        """Find a match of the rules in the circuit."""

    def find_matches(self, circ: CompilationState) -> list[CircuitRewrite]:
        """Find all matches of the rules in the circuit."""

    def apply_exhaustive(
        self, circ: CompilationState, scope: PassScope | None = None
    ) -> int:
        """Apply the first matching rule repeatedly within the selected scope.

        Mutates the provided circuit and returns the number of rewrites applied.
        Non-circuit scope regions are skipped, and the original HUGR entrypoint
        is restored before returning, including when an error occurs.
        """

class CircuitPattern:
    """A pattern that matches a circuit exactly."""

    def __init__(self, circ: CompilationState) -> None:
        """Create a new circuit pattern."""

class PatternMatcher:
    """A matcher object for fast pattern matching on circuits."""

    def __init__(self, patterns: Iterator[CircuitPattern]) -> None:
        """Create a new pattern matcher."""

    def find_match(self, circ: CompilationState) -> PatternMatch | None:
        """Find a match of the patterns in the circuit."""

    def find_matches(self, circ: CompilationState) -> list[PatternMatch]:
        """Find all matches of the patterns in the circuit."""

class PatternMatch:
    """A convex pattern match in a circuit"""

    def pattern_id(self) -> PatternID:
        """The id of the matched pattern."""

    def root(self) -> Node:
        """The root node for the pattern in the matched circuit."""

class PatternID:
    """An identifier for a pattern in a pattern matcher."""

    def __int__(self) -> int:
        """Get the integer value of the pattern id."""

class InvalidPatternError(Exception):
    """Conversion error between a pattern and a circuit."""

class InvalidReplacementError(Exception):
    """An error occurred while constructing a pattern match replacement."""
