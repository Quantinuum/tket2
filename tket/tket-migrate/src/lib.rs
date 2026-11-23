//! Migrate HUGR extension operations, types, and constants between versions.
//!
//! Use [`migrate_hugr_default`] to migrate an existing HUGR, or
//! [`migrate_bytes_default`] to load and migrate a serialized HUGR.
//! Both use bundled extension definitions and the default migration rules to
//! update legacy TKET quantum and boolean operations, the opaque boolean type, and its constants.
//! The migrated HUGR is validated before being returned.

/// Default mappings for migrating TKET measurement and boolean operations.
mod default_maps;
mod error;
mod extension_loading;
/// Apply extension migrations to a HUGR.
mod hugr_migration;
/// Define versioned operations, types, and their replacements.
mod update_maps;

pub use error::{ExtensionLookupError, MigrationError, ReplacementError, VersionedElementError};
pub use hugr_migration::{migrate_bytes_default, migrate_hugr_default};
