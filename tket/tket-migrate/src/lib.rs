//! Migrate HUGR extension operations and types between versions.
//!
//! Configure an [`ExtensionUpdater`] with operation and type mappings and any
//! required new extensions, then call [`ExtensionUpdater::migrate`].

/// Default mappings for migrating TKET measurement and boolean operations.
pub mod default_maps;
/// Apply extension migrations to a HUGR.
pub mod hugr_migration;
/// Define versioned operations, types, and their replacements.
pub mod update_maps;

pub use hugr_migration::ExtensionUpdater;
pub use update_maps::{
    OpMapping, OpReplacementTemplate, TypeMapping, TypeReplacementTemplate, VersionedElement,
};
