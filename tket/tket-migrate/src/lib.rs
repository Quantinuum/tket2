//! Migrate HUGR extension operations, types, and constants between versions.
//!
//! Use [`migrate_hugr_default`] to migrate an existing HUGR, or
//! [`migrate_bytes_default`] to load and migrate a serialized HUGR.
//! Both use bundled extension definitions and the default migration rules to
//! update legacy TKET quantum and boolean operations, the opaque boolean type, and its constants.
//! The migrated HUGR is validated before being returned.
//!
//! ```
//! use hugr::Hugr;
//! use tket_migrate::migrate_bytes_default;
//!
//! fn load(bytes: &[u8]) -> Result<Hugr, Box<dyn std::error::Error>> {
//!     Ok(migrate_bytes_default(bytes)?)
//! }
//! ```

/// Default mappings for TKET quantum operations and legacy boolean operations, types, and constants.
mod default_maps;
mod error;
mod extension_loading;
/// Apply extension migrations to a HUGR.
mod hugr_migration;
/// Define versioned operations, types, and their replacements.
mod update_maps;

pub use hugr_migration::{migrate_bytes_default, migrate_hugr_default};
