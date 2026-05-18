"""Utility definitions for tket-py."""

from typing import Protocol


class PytketCircuitProto(Protocol):
    """Protocol for classes that can be serialized as a pytket Circuit.

    This is used to allow type annotations that refer to pytket Circuits without
    requiring pytket to be installed.
    """

    def to_dict(self) -> dict:
        """Convert this circuit to a dictionary representation.

        Returns:
            A JSON serializable dictionary representation of the Circuit.
        """


class PytketPassProto(Protocol):
    """Protocol for classes that can be serialized as a pytket BasePass.

    This is used to allow type annotations that refer to pytket Passes without
    requiring pytket to be installed.
    """

    def to_dict(self) -> dict:
        """Convert this pass to a dictionary representation.

        Returns:
            A JSON serializable dictionary representation of the Pass.
        """
