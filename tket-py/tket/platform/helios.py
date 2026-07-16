"""Platform configuration interface for HUGRs targeting Helios."""

from __future__ import annotations

from typing import Any

from hugr.hugr.base import Hugr
from hugr.package import Package

from .._tket import metadata as _metadata

#: Metadata key under which the Helios platform configuration is stored.
HELIOS_CONFIG_META_KEY = _metadata.HELIOS_CONFIG_KEY

__all__ = [
    "HELIOS_CONFIG_META_KEY",
    "_set_platform_config",
]


def _set_platform_config(
    hugr: Package | Hugr[Any],
    squash_rxys: bool = True,
    enable_dd: bool = False,
    leakage_repump: bool = False,
) -> None:
    """EXPERIMENTAL: Set Helios-specific job configuration options on a compiled HUGR.
    This is currently provided for development purposes only - options may not work as
    expected (or at all) and the interface may change in future releases.

    Note that this configuration is *Helios-only* - it only has effect on cloud
    submissions to the Helios hardware and Helios-specific emulators. In particular, it
    has no effect for local simulator runs.

    Args:
        hugr: A compiled HUGR package or module to configure.
        squash_rxys: Whether to combine single-qubit gates at runtime
            (independent of any compile-time squashing). Defaults to True.
        enable_dd: Enable dynamical decoupling. Defaults to False.
        leakage_repump: Enable leakage repump. Defaults to False.
    """
    config = {
        "squash_rxys": squash_rxys,
        "enable_dd": enable_dd,
        "leakage_repump": leakage_repump,
    }
    modules = hugr.modules if isinstance(hugr, Package) else [hugr]
    for module in modules:
        module[module.module_root].metadata[HELIOS_CONFIG_META_KEY] = config.copy()
