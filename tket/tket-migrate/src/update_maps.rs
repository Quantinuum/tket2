use hugr::types::CustomType;
use hugr::{
    builder::{DFGBuilder, Dataflow, DataflowHugr},
    extension::Version,
    ops::{DataflowOpTrait, ExtensionOp, Value, constant::OpaqueValue},
    types::{Transformable, Type},
};
use std::{collections::HashMap, fmt};
use tket::passes::{
    ReplaceTypes,
    replace_types::{NodeTemplate, ReplaceTypesError},
};

use crate::error::{ReplacementError, VersionedElementError};
use crate::extension_loading::{get_extension, get_op_by_name};

/// Identifies an operation or type by name, extension, and version.
#[derive(Clone, Debug, Eq, Hash, PartialEq)]
pub struct VersionedElement {
    id: String,
    extension_id: String,
    version: Version,
}

impl VersionedElement {
    /// Identifies an operation or type within a specific extension version.
    fn new(id: String, extension_id: String, version: Version) -> Self {
        Self {
            id,
            extension_id,
            version,
        }
    }

    /// Instantiates the extension operation from the local registry.
    ///
    /// Returns an error when the extension or definition is missing or cannot be instantiated.
    pub(crate) fn get_instantiated_op(&self) -> Result<ExtensionOp, VersionedElementError> {
        get_extension(&self.extension_id, &self.version)
            .and_then(|extension| get_op_by_name(&self.id, extension))
            .map_err(|source| VersionedElementError::ExtensionLookup {
                element: self.clone(),
                source,
            })
    }

    /// Returns an error when the extension or type definition is missing or cannot be instantiated.
    pub(crate) fn get_type(&self) -> Result<CustomType, VersionedElementError> {
        let extension = get_extension(&self.extension_id, &self.version).map_err(|source| {
            VersionedElementError::ExtensionLookup {
                element: self.clone(),
                source,
            }
        })?;
        let definition = extension
            .get_type(self.id.as_str())
            .ok_or_else(|| VersionedElementError::MissingType(self.clone()))?;
        let ty = definition.instantiate([]).map_err(|source| {
            VersionedElementError::InstantiateType {
                element: self.clone(),
                source,
            }
        })?;
        Ok(ty)
    }
}

impl fmt::Display for VersionedElement {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} in {}@{}", self.id, self.extension_id, self.version)
    }
}

/// A recipe for creating the replacement
#[derive(Clone, Debug)]
pub(crate) enum OpReplacementTemplate {
    /// An empty replacement template. States that the target should be removed.
    Empty,
    /// The recipe for creating the replacement.
    TemplateInstance(NodeTemplate),
}

impl OpReplacementTemplate {
    /// Builds a replacement template.
    pub(crate) fn get_op_replace(
        &self,
        old_op: &ExtensionOp,
        replacer: &ReplaceTypes,
    ) -> Result<NodeTemplate, ReplacementError> {
        match self {
            OpReplacementTemplate::TemplateInstance(template) => Ok(template.clone()),
            OpReplacementTemplate::Empty => {
                // A passthrough must use the migrated types on both sides.
                let mut signature = old_op.signature().into_owned();
                signature.transform(replacer)?;
                let builder = DFGBuilder::new(signature)?;
                let wires = builder.input_wires();
                Ok(NodeTemplate::linked_hugr(
                    builder.finish_hugr_with_outputs(wires)?,
                ))
            }
        }
    }
}

/// Maps operation names to their replacements.
pub(crate) type OpMapping = VersionedMapping<OpReplacementTemplate>;

/// Maps type names to their replacement types.
pub(crate) type TypeMapping = VersionedMapping<Type>;

/// Maps source type names to converters for constants of those types.
pub(crate) type ConstMapping = VersionedMapping<ConstConverter>;

/// Converts an opaque value and can use the active pass to migrate its contents.
type ConstConverter = fn(&OpaqueValue, &ReplaceTypes) -> Result<Value, ReplaceTypesError>;

/// Replacements for named elements of one source extension version.
///
/// # Fields
///
/// - `extension_id` identifies the extension that defines the source operations or types.
/// - `version` specifies the source version of that extension.
/// - `map` associates each [`VersionedElement`] with its operation replacement,
///   replacement type, or constant converter.
#[derive(Debug)]
pub(crate) struct VersionedMapping<R> {
    extension_id: String,
    version: Version,
    map: HashMap<VersionedElement, R>,
}

impl<R> VersionedMapping<R> {
    /// Creates an empty mapping for the given extension version.
    pub(crate) fn new(extension_id: String, version: Version) -> Self {
        Self {
            extension_id,
            version,
            map: HashMap::new(),
        }
    }

    /// Adds or overwrites the replacement for a named element.
    pub(crate) fn insert(&mut self, name: impl Into<String>, replacement: R) {
        let source =
            VersionedElement::new(name.into(), self.extension_id.clone(), self.version.clone());
        self.map.insert(source, replacement);
    }

    /// Adds multiple replacements at once.
    pub(crate) fn insert_all(&mut self, entries: Vec<(impl Into<String>, R)>) {
        for (name, replacement) in entries {
            self.insert(name, replacement);
        }
    }

    /// Iterates over versioned source elements and their replacements.
    pub(crate) fn iter(&self) -> impl Iterator<Item = (&VersionedElement, &R)> {
        self.map.iter()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::error::ExtensionLookupError;
    use hugr::{
        HugrView,
        extension::{Version, prelude::bool_t},
        ops::OpTrait,
        types::Signature,
    };
    use std::error::Error;
    use tket::passes::{ReplaceTypes, replace_types::NodeTemplate};

    fn old_bool(id: &str) -> VersionedElement {
        VersionedElement::new(id.into(), "tket.bool".into(), Version::new(0, 2, 0))
    }

    #[test]
    fn instantiate_operation_and_type_from_registered_extension() -> Result<(), Box<dyn Error>> {
        let operation = old_bool("not").get_instantiated_op()?;
        assert_eq!(operation.unqualified_id(), "not");
        assert_eq!(operation.extension_version(), Version::new(0, 2, 0));
        let ty = old_bool("bool").get_type()?;
        assert_eq!(ty.name(), "bool");
        Ok(())
    }

    #[test]
    fn missing_extension_or_version_is_an_error() {
        for (name, version) in [
            ("test.missing", Version::new(1, 0, 0)),
            ("tket.bool", Version::new(99, 0, 0)),
        ] {
            let op = VersionedElement::new("not".into(), name.into(), version.clone());
            let ty = VersionedElement::new("bool".into(), name.into(), version.clone());
            for (requested, error) in [
                (&op, op.get_instantiated_op().unwrap_err()),
                (&ty, ty.get_type().unwrap_err()),
            ] {
                let VersionedElementError::ExtensionLookup {
                    element,
                    source:
                        ExtensionLookupError::MissingExtension {
                            name: missing_name,
                            version: missing_version,
                        },
                } = error
                else {
                    panic!("Expected MissingExtension for {requested}, got {error:?}");
                };
                assert_eq!(&element, requested);
                assert_eq!(missing_name, name);
                assert_eq!(missing_version, version);
            }
        }
    }

    #[test]
    fn missing_definition_in_existing_extension_is_an_error() {
        let requested = old_bool("nonexistent");
        assert!(matches!(
            requested.get_instantiated_op(),
            Err(VersionedElementError::ExtensionLookup {
                element,
                source: ExtensionLookupError::MissingOperation { name },
            }) if element == requested && name == requested.id
        ));
        assert!(matches!(
            requested.get_type(),
            Err(VersionedElementError::MissingType(element)) if element == requested
        ));
    }

    #[test]
    fn operation_with_required_type_arguments_reports_instantiation_error() {
        let requested = VersionedElement::new(
            "symbolic_angle".into(),
            "tket.quantum".into(),
            Version::new(0, 2, 1),
        );
        assert!(matches!(
            requested.get_instantiated_op(),
            Err(VersionedElementError::ExtensionLookup {
                element,
                source: ExtensionLookupError::InstantiateOperation { name, .. },
            }) if element == requested && name == requested.id
        ));
    }

    #[test]
    fn empty_replacements_use_migrated_boolean_signatures() -> Result<(), Box<dyn Error>> {
        let mut replacer = ReplaceTypes::default();
        replacer.set_replace_type(old_bool("bool").get_type()?, bool_t());
        for name in ["make_opaque", "read"] {
            let operation = old_bool(name).get_instantiated_op()?;
            let template = OpReplacementTemplate::Empty.get_op_replace(&operation, &replacer)?;
            let NodeTemplate::LinkedHugr(replacement, _) = template else {
                panic!("Expected a graph replacement for {name}");
            };
            replacement.validate()?;
            assert_eq!(
                replacement
                    .entrypoint_optype()
                    .dataflow_signature()
                    .unwrap()
                    .as_ref(),
                &Signature::new_endo([bool_t()]),
                "{name} must become a boolean passthrough"
            );
        }
        Ok(())
    }
}
