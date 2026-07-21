from hugr import Hugr, ops
from hugr.package import Package

from tket.metadata import HeliosPlatformConfig, HeliosPlatformConfigValue
from tket.platform.helios import _set_platform_config
from tket._tket.metadata import HELIOS_PLATFORM_CONFIG as HELIOS_CONFIG_META_KEY


def _make_hugr() -> Hugr[ops.Module]:
    return Hugr[ops.Module]()


def test_set_platform_config_defaults_hugr() -> None:
    hugr = _make_hugr()

    _set_platform_config(hugr)

    config = hugr[hugr.module_root].metadata[HELIOS_CONFIG_META_KEY]
    assert config == {
        "squash_rxys": True,
        "enable_dd": False,
        "leakage_repump": False,
    }


def test_set_platform_config_custom_hugr() -> None:
    hugr = _make_hugr()

    _set_platform_config(hugr, squash_rxys=False, enable_dd=True, leakage_repump=True)

    assert hugr[hugr.module_root].metadata[HeliosPlatformConfig] == (
        HeliosPlatformConfigValue(
            squash_rxys=False,
            enable_dd=True,
            leakage_repump=True,
        )
    )


def test_set_platform_config_defaults_package() -> None:
    package = Package([_make_hugr(), _make_hugr()])

    _set_platform_config(package)

    for module in package.modules:
        config = module[module.module_root].metadata[HELIOS_CONFIG_META_KEY]
        assert config == {
            "squash_rxys": True,
            "enable_dd": False,
            "leakage_repump": False,
        }


def test_set_platform_config_custom_package() -> None:
    package = Package([_make_hugr(), _make_hugr()])

    _set_platform_config(
        package, squash_rxys=False, enable_dd=True, leakage_repump=True
    )

    for module in package.modules:
        config = module[module.module_root].metadata[HELIOS_CONFIG_META_KEY]
        assert config == {
            "squash_rxys": False,
            "enable_dd": True,
            "leakage_repump": True,
        }


def test_set_platform_config_independent_copies() -> None:
    """Each module gets its own config dict, not a shared reference."""
    package = Package([_make_hugr(), _make_hugr()])

    _set_platform_config(package)

    first, second = package.modules
    first[first.module_root].metadata[HELIOS_CONFIG_META_KEY]["squash_rxys"] = False  # type: ignore

    assert second[second.module_root].metadata[HELIOS_CONFIG_META_KEY] == {
        "squash_rxys": True,
        "enable_dd": False,
        "leakage_repump": False,
    }
