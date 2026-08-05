from collections.abc import Iterator

from hugr.passes.scope import PassScope

from .rewrite import CircuitRewrite
from .state import CompilationState, Node

class Rule:
    """A rewrite rule defined by a left hand side and right hand side of an equation."""

    def __init__(
        self,
        l: CompilationState,
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

    def apply_all_matches_once(
        self, circ: CompilationState, scope: PassScope | None = None
    ) -> int:
        """Find all matching rules once within each circuit-compatible region.

        For each region, this method performs exactly one matching scan against
        a single HUGR snapshot. It constructs all corresponding rewrites from
        that snapshot and then applies them in matcher order.

        If some matches are found after applying a rewrite, the function raises
        an assertion error.


        Non-circuit regions are skipped. The original HUGR entrypoint is
        restored before returning, including when an error occurs.

        Returns the total number of successfully applied rewrites.

        Validity requirement:
            All matches returned by the matching scan must have pairwise
            disjoint invalidation sets. In particular, no HUGR node may belong
            to more than one matched subgraph in the same scan. This ensures
            that applying one rewrite cannot invalidate another rewrite
            constructed from the HUGR state at the beginning of the scan.
            If this condition does not hold, a later rewrite may refer to nodes
            invalidated by an earlier rewrite and must not be applied using
            this method.

            For single-operation rules, such as ``T -> S`` and ``Tdg -> Sdg``,
            this condition is satisfied when each operation node is matched at
            most once.
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
