//! This module defines the Hugr extensions used by tket-qsystem.
use std::sync::{Arc, LazyLock};

use hugr::Extension;
use hugr::extension::ExtensionRegistry;

pub mod classical_compute;
pub use classical_compute::gpu;
pub use classical_compute::wasm;
pub mod futures;
pub mod qsystem;
pub mod random;
pub mod result;
pub mod utils;

/// Extension registry including std, tket, and qsystem extensions.
///
/// This is the default registry for consumers that need to load HUGRs using
/// the extensions embedded in the tket-qsystem distribution.
pub static REGISTRY: LazyLock<ExtensionRegistry> = LazyLock::new(|| {
    let mut registry = tket::extension::REGISTRY.to_owned();
    registry.extend(qsystem_extensions());
    registry
});

/// Returns the extension definitions owned by the `tket-qsystem` crate.
///
/// The list intentionally excludes std and base tket extensions so callers can
/// combine it with [`tket::extension::tket_extensions`] or another base
/// registry without duplicating either list.
pub fn qsystem_extensions() -> [Arc<Extension>; 9] {
    [
        gpu::EXTENSION.to_owned(),
        qsystem::EXTENSION.to_owned(),
        qsystem::helios::EXTENSION.to_owned(),
        qsystem::sol::EXTENSION.to_owned(),
        futures::EXTENSION.to_owned(),
        random::EXTENSION.to_owned(),
        result::EXTENSION.to_owned(),
        utils::EXTENSION.to_owned(),
        wasm::EXTENSION.to_owned(),
    ]
}
