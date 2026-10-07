from enum import Enum

class HugrType:
    """Value types in HUGR."""

    def __init__(self, extension: str, type_name: str, bound: TypeBound) -> None:
        """Create a new named Custom type."""

    @staticmethod
    def qubit() -> HugrType:
        """Qubit type from HUGR prelude."""

    @staticmethod
    def bool() -> HugrType:
        """Boolean type (HUGR 2-ary unit sum)."""

class TypeBound(Enum):
    """HUGR type bounds."""

    Any = 0  # Any type
    Copyable = 1  # Copyable type
    Eq = 2  # Equality-comparable type
