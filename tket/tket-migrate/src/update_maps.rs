use hugr::HugrView;
use hugr::types::CustomType;
use hugr::{
    builder::{DFGBuilder, Dataflow, DataflowHugr},
    extension::Version,
    ops::{DataflowOpTrait, ExtensionOp},
    types::{Transformable, Type},
};
use std::{collections::HashMap, fmt};
use tket::passes::{ReplaceTypes, replace_types::NodeTemplate};

use crate::error::{ReplacementError, VersionedElementError};

/// Represents an Extension Op by its name, the extension it belongs to, and its version.
#[derive(Clone, Debug, Eq, Hash, PartialEq)]
pub(crate) struct VersionedElement {
    id: String,
    // todo: use Extension here
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

    /// Instantiates the extension operation from the given Hugr view.
    ///
    /// Returns an error when the extension or definition is missing or cannot be instantiated.
    pub(crate) fn get_instantiated_op<T: HugrView>(
        &self,
        hugr: &T,
    ) -> Result<ExtensionOp, VersionedElementError> {
        // LOCAL REGITRY
        let Some(extension) = hugr
            .extensions()
            .get_exact(&self.extension_id, &self.version)
        else {
            return Err(VersionedElementError::MissingExtension(self.clone()));
        };
        let definition = extension
            .get_op(self.id.as_str())
            .ok_or_else(|| VersionedElementError::MissingOperation(self.clone()))?;
        let operation = ExtensionOp::new(definition.clone(), []).map_err(|source| {
            VersionedElementError::InstantiateOperation {
                element: self.clone(),
                source,
            }
        })?;
        Ok(operation)
    }

    /// Returns an error when the extension or type definition is missing or cannot be instantiated.
    pub(crate) fn get_type<T: HugrView>(
        &self,
        hugr: &T,
    ) -> Result<CustomType, VersionedElementError> {
        let Some(extension) = hugr
            .extensions()
            .get_exact(&self.extension_id, &self.version)
        else {
            return Err(VersionedElementError::MissingExtension(self.clone()));
        };
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

#[derive(Debug)]
/// Represents a mapping from an old operation to its replacement(s).
/// It is defined by the version of the operation to replace and the mapping provided.
///
/// # Fields
///
/// - `extension_id`: identifier of the extension that the mapped operations belong to.
/// - `version`: version of the extension that the mapped (old) operations come from.
/// - `map`: lookup from a versioned operation (name and version) to the
///   [`OpReplacementTemplate`] used to replace it.
pub(crate) struct OpMapping {
    extension_id: String,
    version: Version,
    map: HashMap<VersionedElement, OpReplacementTemplate>,
}

impl OpMapping {
    /// Maps operation names from the given extension version to their replacements.
    pub(crate) fn new(extension_id: String, version: Version) -> Self {
        Self {
            extension_id,
            version,
            map: HashMap::new(),
        }
    }

    /// Adds or overwrites the replacement for an operation in this extension version.
    pub(crate) fn insert(&mut self, name: impl Into<String>, replacement: OpReplacementTemplate) {
        let source =
            VersionedElement::new(name.into(), self.extension_id.clone(), self.version.clone());
        self.map.insert(source, replacement);
    }

    /// Adds multiple replacements at once.
    pub(crate) fn insert_all(&mut self, entries: Vec<(impl Into<String>, OpReplacementTemplate)>) {
        for (k, v) in entries {
            self.insert(k, v);
        }
    }

    /// Iterates over source operations and their replacements.
    pub(crate) fn iter(&self) -> impl Iterator<Item = (&VersionedElement, &OpReplacementTemplate)> {
        self.map.iter()
    }
}

#[derive(Debug)]
/// Maps type names from one source extension version to their replacement types.
///
/// The mapping is described by the following fields:
/// - `extension_id`: identifier of the source extension.
/// - `version`: version of the source extension.
/// - `map`: replacement types keyed by the versioned source type.
pub(crate) struct TypeMapping {
    /// Identifier of the source extension.
    extension_id: String,
    /// Version of the source extension.
    version: Version,
    /// Replacement types keyed by the versioned source type.
    map: HashMap<VersionedElement, Type>,
}

impl TypeMapping {
    /// Creates an empty type mapping for the given extension version.
    pub(crate) fn new(extension_id: String, version: Version) -> Self {
        Self {
            extension_id,
            version,
            map: HashMap::new(),
        }
    }

    /// Adds or overwrites the replacement for a type in this extension version.
    pub(crate) fn insert(&mut self, name: impl Into<String>, replacement: Type) {
        let source =
            VersionedElement::new(name.into(), self.extension_id.clone(), self.version.clone());
        self.map.insert(source, replacement);
    }

    /// Iterates over source types and their replacements.
    pub(crate) fn iter(&self) -> impl Iterator<Item = (&VersionedElement, &Type)> {
        self.map.iter()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hugr_migration::test_helpers::*;
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

    fn missing(id: &str) -> VersionedElement {
        VersionedElement::new(id.into(), "test.missing".into(), Version::new(1, 0, 0))
    }

    #[test]
    fn instantiate_operation_and_type_from_registered_extension() -> Result<(), Box<dyn Error>> {
        let hugr = bool_graph()?;
        let operation = old_bool("not").get_instantiated_op(&hugr)?;
        assert_eq!(operation.unqualified_id(), "not");
        assert_eq!(operation.extension_version(), Version::new(0, 2, 0));
        let ty = old_bool("bool").get_type(&hugr)?;
        assert_eq!(ty.name(), "bool");
        Ok(())
    }

    #[test]
    fn missing_extension_or_version_is_an_error() -> Result<(), Box<dyn Error>> {
        let hugr = bool_graph()?;
        assert!(matches!(
            missing("not").get_instantiated_op(&hugr),
            Err(VersionedElementError::MissingExtension(_))
        ));
        assert!(matches!(
            missing("bool").get_type(&hugr),
            Err(VersionedElementError::MissingExtension(_))
        ));

        // The extension exists, but the requested version does not.
        let mut operation = old_bool("not");
        operation.version = Version::new(99, 0, 0);
        assert!(matches!(
            operation.get_instantiated_op(&hugr),
            Err(VersionedElementError::MissingExtension(_))
        ));
        let mut ty = old_bool("bool");
        ty.version = Version::new(99, 0, 0);
        assert!(matches!(
            ty.get_type(&hugr),
            Err(VersionedElementError::MissingExtension(_))
        ));
        Ok(())
    }

    #[test]
    fn missing_definition_in_existing_extension_is_an_error() -> Result<(), Box<dyn Error>> {
        let hugr = bool_graph()?;
        let requested = old_bool("nonexistent");
        assert!(matches!(
            requested.get_instantiated_op(&hugr),
            Err(VersionedElementError::MissingOperation(element)) if element == requested
        ));
        assert!(matches!(
            requested.get_type(&hugr),
            Err(VersionedElementError::MissingType(element)) if element == requested
        ));
        Ok(())
    }

    #[test]
    fn empty_replacements_use_migrated_boolean_signatures() -> Result<(), Box<dyn Error>> {
        let hugr = bool_graph()?;
        let mut replacer = ReplaceTypes::default();
        replacer.set_replace_type(old_bool("bool").get_type(&hugr)?, bool_t());
        for name in ["make_opaque", "read"] {
            let operation = old_bool(name).get_instantiated_op(&hugr)?;
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
