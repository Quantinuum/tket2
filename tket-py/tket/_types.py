from __future__ import annotations

from tket._tket.types import HugrType, TypeBound

__all__ = ["HugrType", "TypeBound", "QB_T", "BOOL_T"]


QB_T = HugrType.qubit()
BOOL_T = HugrType.bool()
