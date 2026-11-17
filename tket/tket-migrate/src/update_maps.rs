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
pub struct VersionedElement {
    id: String,
    extension_id: String,
    version: Version,
}

impl VersionedElement {
    /// Identifies an operation or type within a specific extension version.
    pub fn new(id: String, extension_id: String, version: Version) -> Self {
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
pub enum OpReplacementTemplate {
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

#[derive(Debug, Default)]
/// Represents a mapping from an old operation to its replacement(s).
pub struct OpMapping {
    map: HashMap<VersionedElement, OpReplacementTemplate>,
}

impl OpMapping {
    /// Creates operation mappings from a map of versioned sources to replacements.
    pub fn new(map: HashMap<VersionedElement, OpReplacementTemplate>) -> Self {
        Self { map }
    }

    /// Adds a replacement, overwriting any existing mapping for the same source.
    pub fn insert(&mut self, old_op: VersionedElement, replacement: OpReplacementTemplate) {
        self.map.insert(old_op, replacement);
    }

    /// Iterates over source operations and their replacements.
    pub(crate) fn iter(&self) -> impl Iterator<Item = (&VersionedElement, &OpReplacementTemplate)> {
        self.map.iter()
    }
}

impl From<Vec<(VersionedElement, OpReplacementTemplate)>> for OpMapping {
    fn from(entries: Vec<(VersionedElement, OpReplacementTemplate)>) -> Self {
        Self {
            map: entries.into_iter().collect(),
        }
    }
}

#[derive(Debug)]
/// Mapping used to update signature of input/output ports of dataflow and controlflow operations.
///
/// Maps a Type, declared as a `VersionedElement`, to a new Type.
pub struct TypeMapping {
    map: HashMap<VersionedElement, Type>,
}

impl TypeMapping {
    /// Creates type mappings from a map of versioned sources to replacements.
    pub fn new(map: HashMap<VersionedElement, Type>) -> Self {
        Self { map }
    }

    /// Adds a replacement, overwriting any existing mapping for the same source.
    pub fn insert(&mut self, old_type: VersionedElement, new_type: Type) {
        self.map.insert(old_type, new_type);
    }

    /// Iterates over source types and their replacements.
    pub(crate) fn iter(&self) -> impl Iterator<Item = (&VersionedElement, &Type)> {
        self.map.iter()
    }
}

impl From<Vec<(VersionedElement, Type)>> for TypeMapping {
    fn from(entries: Vec<(VersionedElement, Type)>) -> Self {
        Self {
            map: entries.into_iter().collect(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hugr_migration::test_helpers::*;
    use hugr::{
        Extension, HugrView,
        extension::{TypeDefBound, Version, prelude::bool_t},
        hugr::hugrmut::HugrMut,
        ops::OpTrait,
        types::{PolyFuncType, Signature, TypeBound},
    };
    use std::error::Error;
    use tket::passes::{ReplaceTypes, replace_types::NodeTemplate};

    #[test]
    fn versioned_element_display_format() {
        let element = VersionedElement::new(
            "MeasureFree".into(),
            "tket.quantum".into(),
            Version::new(1, 2, 3),
        );
        assert_eq!(format!("{element}"), "MeasureFree in tket.quantum@1.2.3");
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
    fn instantiation_errors_preserve_element_and_source() -> Result<(), Box<dyn Error>> {
        let extension = Extension::try_new_arc(
            "test.parameterized".try_into()?,
            Version::new(1, 0, 0),
            |extension, extension_ref| {
                extension.add_type(
                    "param_type".into(),
                    vec![TypeBound::Copyable.into()],
                    String::new(),
                    TypeDefBound::copyable(),
                    extension_ref,
                )?;
                extension.add_op(
                    "param_op".into(),
                    String::new(),
                    PolyFuncType::new(vec![TypeBound::Copyable.into()], Signature::new_endo([])),
                    extension_ref,
                )?;
                Ok::<_, Box<dyn Error>>(())
            },
        )?;
        let mut hugr = identity_graph()?;
        hugr.use_extensions([extension]);
        let operation = VersionedElement::new(
            "param_op".into(),
            "test.parameterized".into(),
            Version::new(1, 0, 0),
        );
        let ty = VersionedElement::new(
            "param_type".into(),
            "test.parameterized".into(),
            Version::new(1, 0, 0),
        );
        let error = operation.get_instantiated_op(&hugr).unwrap_err();
        assert!(
            error
                .source()
                .unwrap()
                .is::<hugr::extension::SignatureError>()
        );
        assert!(matches!(
            error,
            VersionedElementError::InstantiateOperation {
                element,
                source: hugr::extension::SignatureError::TypeArgMismatch(_),
            } if element == operation
        ));
        let error = ty.get_type(&hugr).unwrap_err();
        assert!(
            error
                .source()
                .unwrap()
                .is::<hugr::extension::SignatureError>()
        );
        assert!(matches!(
            error,
            VersionedElementError::InstantiateType {
                element,
                source: hugr::extension::SignatureError::TypeArgMismatch(_),
            } if element == ty
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

    #[test]
    fn empty_replacement_cannot_connect_unmigrated_types() -> Result<(), Box<dyn Error>> {
        let hugr = bool_graph()?;
        let operation = old_bool("make_opaque").get_instantiated_op(&hugr)?;
        let error = OpReplacementTemplate::Empty
            .get_op_replace(&operation, &ReplaceTypes::default())
            .unwrap_err();
        assert!(error.source().unwrap().is::<hugr::builder::BuildError>());
        assert!(matches!(error, ReplacementError::Build(_)));
        Ok(())
    }
}
