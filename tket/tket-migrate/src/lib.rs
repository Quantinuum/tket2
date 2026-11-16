//! Migrate HUGR extension operations and types between versions.
//!
//! Configure an [`ExtensionUpdater`] with operation and type mappings and any
//! additional local extensions, then call [`ExtensionUpdater::migrate`].

/// Default mappings for migrating TKET measurement and boolean operations.
pub mod default_maps;
pub mod error;
mod extension_loading;
/// Apply extension migrations to a HUGR.
pub mod hugr_migration;
/// Define versioned operations, types, and their replacements.
pub mod update_maps;

pub use error::{MigrationError, ReplacementError, VersionedElementError};
pub use hugr_migration::{ExtensionUpdater, migrate_hugr_default};
pub use update_maps::{
    OpMapping, OpReplacementTemplate, TypeMapping, TypeReplacementTemplate, VersionedElement,
};
