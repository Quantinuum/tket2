//! Errors produced when resolving mappings and migrating a HUGR.

use hugr::{
    builder::BuildError,
    extension::{SignatureError, resolution::ExtensionResolutionError},
};
use thiserror::Error;
use tket::passes::replace_types::ReplaceTypesError;

use crate::VersionedElement;

/// A registered extension cannot provide the requested operation or type.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum VersionedElementError {
    /// The extension exists, but the operation definition is absent.
    #[error("Operation {0} is missing from its extension")]
    MissingOperation(VersionedElement),
    /// The extension exists, but the type definition is absent.
    #[error("Type {0} is missing from its extension")]
    MissingType(VersionedElement),
    /// The operation definition cannot be instantiated.
    #[error("Could not instantiate operation {element}: {source}")]
    InstantiateOperation {
        /// The operation being instantiated.
        element: VersionedElement,
        /// The signature error returned by HUGR.
        #[source]
        source: SignatureError,
    },
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

/// A replacement type or operation template cannot be constructed.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum ReplacementError {
    /// The extension version required by a replacement operation is absent.
    #[error("Replacement operation {0} requires a missing extension")]
    MissingOperationExtension(VersionedElement),
    /// The extension version required by a replacement type is absent.
    #[error("Replacement type {0} requires a missing extension")]
    MissingTypeExtension(VersionedElement),
    /// A replacement definition cannot be resolved or instantiated.
    #[error(transparent)]
    Element(#[from] VersionedElementError),
    /// Migrating the signature of an empty replacement failed.
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
    /// A source definition cannot be resolved or instantiated.
    #[error(transparent)]
    SourceElement(#[from] VersionedElementError),
    /// A configured replacement cannot be constructed.
    #[error(transparent)]
    Replacement(#[from] ReplacementError),
    /// Applying the replacements to the HUGR failed.
    #[error("Could not apply extension replacements: {0}")]
    ReplaceTypes(#[from] ReplaceTypesError),
    /// Resolving dependencies of the new extensions failed.
    #[error("Could not register new extensions: {0}")]
    RegisterExtensions(#[source] ExtensionResolutionError),
}
