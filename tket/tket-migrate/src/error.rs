//! Errors produced when resolving mappings and migrating a HUGR.

use hugr::{
    Node,
    builder::BuildError,
    extension::{SignatureError, Version},
    hugr::ValidationError,
};
use thiserror::Error;
use tket::passes::replace_types::ReplaceTypesError;

use crate::update_maps::VersionedElement;

/// An extension lookup or operation instantiation failed.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum ExtensionLookupError {
    /// The registry lacks the requested extension version.
    #[error("The required extension {name}@{version} is missing from `LOCAL_REGISTRY`")]
    MissingExtension {
        /// Requested extension name.
        name: String,
        /// Requested extension version.
        version: Version,
    },
    /// The extension has no definition for the requested operation.
    #[error("Operation {name} is missing from the extension")]
    MissingOperation {
        /// Requested operation name.
        name: String,
    },
    /// The operation definition cannot be instantiated without type arguments.
    #[error("Could not instantiate operation {name}: {source}")]
    InstantiateOperation {
        /// Requested operation name.
        name: String,
        /// The signature error returned by HUGR.
        #[source]
        source: SignatureError,
    },
}

/// An extension is missing or cannot provide the requested operation or type.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum VersionedElementError {
    /// Looking up the extension or instantiating its operation failed.
    #[error("We cannot initialize {element}: {source}")]
    ExtensionLookup {
        /// The element needing the extension.
        element: VersionedElement,
        /// The underlying lookup or instantiation error.
        #[source]
        source: ExtensionLookupError,
    },
    /// The extension exists, but the type definition is absent.
    #[error("Type {0} is missing from its extension")]
    MissingType(VersionedElement),
    /// The type definition cannot be instantiated.
    #[error("Could not instantiate type {element}: {source}")]
    InstantiateType {
        /// The type being instantiated.
        element: VersionedElement,
        /// The signature error returned by HUGR.
        #[source]
        source: SignatureError,
    },
}

/// An operation replacement template cannot be constructed.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum ReplacementError {
    /// Migrating the old operation's signature failed.
    #[error("Could not migrate replacement signature: {0}")]
    Signature(#[from] ReplaceTypesError),
    /// The replacement operations cannot be connected into a valid HUGR.
    #[error("Could not build replacement HUGR: {0}")]
    Build(#[from] BuildError),
}

/// A configured extension migration cannot be applied to a HUGR.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum MigrationError {
    /// Loading the input HUGR failed.
    #[error("Could not load HUGR: {0}")]
    Read(#[from] hugr::envelope::ReadError),
    /// A source definition cannot be resolved or instantiated.
    #[error(transparent)]
    SourceElement(#[from] VersionedElementError),
    /// A configured replacement cannot be constructed.
    #[error(transparent)]
    Replacement(#[from] ReplacementError),
    /// Applying the replacements to the HUGR failed.
    #[error("Could not apply extension replacements: {0}")]
    ReplaceTypes(#[from] ReplaceTypesError),
    /// Validating the migrated HUGR failed.
    #[error("The migration produced an invalid HUGR: {0}")]
    Validate(#[from] ValidationError<Node>),
}
