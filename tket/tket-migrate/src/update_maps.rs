use hugr::HugrView;
use hugr::types::CustomType;
use hugr::{
    builder::{DFGBuilder, Dataflow, DataflowHugr},
    extension::Version,
    ops::{DataflowOpTrait, ExtensionOp},
    types::{Signature, Transformable, Type},
};
use std::{collections::HashMap, fmt};
use tket::passes::{ReplaceTypes, replace_types::NodeTemplate};

use crate::error::{ReplacementError, VersionedElementError};

/// Represents an Extension Op by its name, the extension it belongs to, and its version.
#[derive(Clone, Debug, Eq, Hash, PartialEq)]
pub struct VersionedElement {
    pub(crate) id: String,
    pub(crate) extension_id: String,
    pub(crate) version: Version,
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
    /// Returns `None` when the source extension version is absent.
    ///
    /// Returns an error when the definition is missing or cannot be instantiated.
    pub fn get_instantiated_op<T: HugrView>(
        &self,
        hugr: &T,
    ) -> Result<Option<ExtensionOp>, VersionedElementError> {
        let Some(extension) = hugr
            .extensions()
            .get_exact(&self.extension_id, &self.version)
        else {
            return Ok(None);
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
        Ok(Some(operation))
    }

    /// Returns `None` when the source extension version is absent.
    /// Returns an error when the type definition is missing or cannot be instantiated.
    pub fn get_type<T: HugrView>(
        &self,
        hugr: &T,
    ) -> Result<Option<CustomType>, VersionedElementError> {
        let Some(extension) = hugr
            .extensions()
            .get_exact(&self.extension_id, &self.version)
        else {
            return Ok(None);
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
        Ok(Some(ty))
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
    /// A list of versioned elements declared as name, extension, and version.
    ///
    /// The vector contains at least one element. If more elements are present, they are connected in sequence.
    VersionedElements(Vec<VersionedElement>),
    /// The recipe for creating the replacement.
    TemplateInstance(NodeTemplate),
}

impl OpReplacementTemplate {
    /// Builds a replacement template
    pub fn get_op_replace<T: HugrView>(
        &self,
        old_op: &ExtensionOp,
        hugr: &T,
        replacer: &ReplaceTypes,
    ) -> Result<NodeTemplate, ReplacementError> {
        match self {
            OpReplacementTemplate::TemplateInstance(template) => Ok(template.clone()),
            OpReplacementTemplate::Empty => Self::get_node_template(old_op, &[], hugr, replacer),
            OpReplacementTemplate::VersionedElements(v) => {
                Self::get_node_template(old_op, v, hugr, replacer)
            }
        }
    }

    fn get_node_template<T: HugrView>(
        old_op: &ExtensionOp,
        versioned_elements: &[VersionedElement],
        hugr: &T,
        replacer: &ReplaceTypes,
    ) -> Result<NodeTemplate, ReplacementError> {
        let operations = versioned_elements
            .iter()
            .map(|element| {
                element
                    .get_instantiated_op(hugr)?
                    .ok_or_else(|| ReplacementError::MissingOperationExtension(element.clone()))
            })
            .collect::<Result<Vec<_>, _>>()?;

        let signature = match (operations.first(), operations.last()) {
            (Some(first), Some(last)) => Signature::new(
                first.signature().input().clone(),
                last.signature().output().clone(),
            ),
            _ => {
                // A passthrough must use the migrated types on both sides.
                let mut signature = old_op.signature().into_owned();
                signature.transform(replacer)?;
                signature
            }
        };
        let mut builder = DFGBuilder::new(signature)?;
        let mut wires = builder.input_wires().collect::<Vec<_>>();
        for operation in operations {
            wires = builder
                .add_dataflow_op(operation, wires)?
                .outputs()
                .collect();
        }
        Ok(NodeTemplate::linked_hugr(
            builder.finish_hugr_with_outputs(wires)?,
        ))
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

    /// Looks up the replacement for an instantiated operation and its version.
    pub fn get_replacement(&self, operation: &ExtensionOp) -> Option<&OpReplacementTemplate> {
        let versioned_element = VersionedElement::new(
            operation.unqualified_id().to_string(),
            operation.extension_id().to_string(),
            operation.extension_version().clone(),
        );

        self.map.get(&versioned_element)
    }

    /// Iterates over source operations and their replacements.
    pub fn iter(&self) -> impl Iterator<Item = (&VersionedElement, &OpReplacementTemplate)> {
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

/// A recipe for creating the replacement
///
/// Represents the possible ways to replace a type: either by specifying a versioned element of a custom type or by providing an instance of the new type.
#[derive(Debug)]
pub enum TypeReplacementTemplate {
    /// A versioned element representing the a type.
    VersionedElement(VersionedElement),
    /// An instance of the new type.
    Type(Type),
}

impl TypeReplacementTemplate {
    /// Retrieves the type represented by this replacement template.
    /// Returns an error when a versioned replacement cannot be resolved or instantiated.
    pub fn get_type<T: HugrView>(&self, hugr: &T) -> Result<Type, ReplacementError> {
        match self {
            TypeReplacementTemplate::VersionedElement(element) => {
                let replacement = element
                    .get_type(hugr)?
                    .ok_or_else(|| ReplacementError::MissingTypeExtension(element.clone()))?;
                Ok(replacement.into())
            }
            TypeReplacementTemplate::Type(t) => Ok(t.clone()),
        }
    }
}

#[derive(Debug)]
/// Mapping used to update signature of input/output ports of dataflow and controlflow operations.
///
/// Maps a Type, declared as a `VersionedElement`, to a new Type.
pub struct TypeMapping {
    map: HashMap<VersionedElement, TypeReplacementTemplate>,
}

impl TypeMapping {
    /// Creates type mappings from a map of versioned sources to replacements.
    pub fn new(map: HashMap<VersionedElement, TypeReplacementTemplate>) -> Self {
        Self { map }
    }

    /// Adds a replacement, overwriting any existing mapping for the same source.
    pub fn insert(&mut self, old_type: VersionedElement, new_type: TypeReplacementTemplate) {
        self.map.insert(old_type, new_type);
    }

    /// Iterates over source types and their replacements.
    pub fn iter(&self) -> impl Iterator<Item = (&VersionedElement, &TypeReplacementTemplate)> {
        self.map.iter()
    }

    // pub fn get_replacement(&self, old_type: &VersionedElement) -> Option<&TypeReplacementTemplate> {
    //     self.get_new_type(old_type)
    // }
}

impl From<Vec<(VersionedElement, TypeReplacementTemplate)>> for TypeMapping {
    fn from(entries: Vec<(VersionedElement, TypeReplacementTemplate)>) -> Self {
        Self {
            map: entries.into_iter().collect(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hugr_migration::{ExtensionUpdater, test_helpers::*};
    use hugr::{
        Extension, HugrView,
        extension::{TypeDefBound, Version, prelude::bool_t, simple_op::MakeRegisteredOp},
        hugr::hugrmut::HugrMut,
        ops::OpTrait,
        std_extensions::logic::LogicOp,
        types::{PolyFuncType, Signature, TypeBound},
    };
    use std::{collections::HashMap, error::Error};
    use tket::passes::{ReplaceTypes, replace_types::NodeTemplate};

    #[derive(Clone, Copy, Debug)]
    enum MapConstruction {
        Vector,
        HashMap,
        Insert,
    }

    fn negation_maps(
        construction: MapConstruction,
        replacement: NodeTemplate,
    ) -> (OpMapping, TypeMapping) {
        let op = (
            old_bool("not"),
            OpReplacementTemplate::TemplateInstance(replacement),
        );
        let ty = (old_bool("bool"), TypeReplacementTemplate::Type(bool_t()));
        match construction {
            MapConstruction::Vector => (vec![op].into(), vec![ty].into()),
            MapConstruction::HashMap => (
                OpMapping::new(HashMap::from([op])),
                TypeMapping::new(HashMap::from([ty])),
            ),
            MapConstruction::Insert => {
                let mut ops = OpMapping::default();
                let mut types = TypeMapping::new(HashMap::new());
                ops.insert(op.0, op.1);
                types.insert(ty.0, ty.1);
                (ops, types)
            }
        }
    }

    #[test]
    fn map_constructors_support_all_node_template_forms() -> Result<(), Box<dyn Error>> {
        for construction in [
            MapConstruction::Vector,
            MapConstruction::HashMap,
            MapConstruction::Insert,
        ] {
            let templates = [
                NodeTemplate::SingleOp(LogicOp::Not.to_extension_op()?.into()),
                NodeTemplate::CompoundOp(Box::new(builtin_negation_graph()?)),
                NodeTemplate::linked_hugr(builtin_negation_graph()?),
            ];
            for template in templates {
                let description = format!("{construction:?}, {template:?}");
                let (ops_map, types_map) = negation_maps(construction, template);
                let mut updater =
                    ExtensionUpdater::new(old_boolean_graph(true)?, ops_map, types_map, vec![]);
                updater.migrate()?;
                let migrated = updater.get_hugr();
                migrated.validate()?;
                assert_eq!(
                    migrated
                        .entrypoint_optype()
                        .dataflow_signature()
                        .unwrap()
                        .as_ref(),
                    &Signature::new_endo([bool_t()]),
                    "{description}"
                );
                let operations = migrated
                    .nodes()
                    .filter_map(|node| migrated.get_optype(node).as_extension_op())
                    .collect::<Vec<_>>();
                assert_eq!(operations.len(), 1, "{description}");
                assert_eq!(
                    operations[0].qualified_id().to_string(),
                    "logic.Not",
                    "{description}"
                );
            }
        }
        Ok(())
    }

    #[test]
    fn insert_and_vector_construction_replace_duplicate_entries() -> Result<(), Box<dyn Error>> {
        for construction in [MapConstruction::Vector, MapConstruction::Insert] {
            let bad_op = OpReplacementTemplate::VersionedElements(vec![missing("target_op")]);
            let good_op = OpReplacementTemplate::TemplateInstance(NodeTemplate::SingleOp(
                LogicOp::Not.to_extension_op()?.into(),
            ));
            let bad_type = TypeReplacementTemplate::VersionedElement(missing("target_type"));
            let good_type = TypeReplacementTemplate::Type(bool_t());
            let (ops, types) = match construction {
                MapConstruction::Vector => (
                    vec![(old_bool("not"), bad_op), (old_bool("not"), good_op)].into(),
                    vec![(old_bool("bool"), bad_type), (old_bool("bool"), good_type)].into(),
                ),
                MapConstruction::Insert => {
                    let mut ops = OpMapping::new(HashMap::from([(old_bool("not"), bad_op)]));
                    let mut types = TypeMapping::new(HashMap::from([(old_bool("bool"), bad_type)]));
                    ops.insert(old_bool("not"), good_op);
                    types.insert(old_bool("bool"), good_type);
                    (ops, types)
                }
                MapConstruction::HashMap => unreachable!(),
            };
            assert_eq!(ops.iter().count(), 1);
            assert_eq!(types.iter().count(), 1);
            let mut updater = ExtensionUpdater::new(old_boolean_graph(true)?, ops, types, vec![]);
            updater.migrate()?;
            updater.get_hugr().validate()?;
            assert!(updater.get_hugr().nodes().any(|node| {
                updater
                    .get_hugr()
                    .get_optype(node)
                    .as_extension_op()
                    .is_some_and(|op| op.qualified_id() == "logic.Not")
            }));
        }
        Ok(())
    }

    #[test]
    fn instantiate_operation_and_type_from_registered_extension() -> Result<(), Box<dyn Error>> {
        let hugr = bool_graph()?;
        let operation = old_bool("not").get_instantiated_op(&hugr)?.unwrap();
        assert_eq!(operation.unqualified_id(), "not");
        assert_eq!(operation.extension_version(), Version::new(0, 2, 0));
        let ty = old_bool("bool").get_type(&hugr)?.unwrap();
        assert_eq!(ty.name(), "bool");
        Ok(())
    }

    #[test]
    fn missing_extension_or_version_returns_none() -> Result<(), Box<dyn Error>> {
        let hugr = bool_graph()?;
        assert!(missing("not").get_instantiated_op(&hugr)?.is_none());
        assert!(missing("bool").get_type(&hugr)?.is_none());

        // The extension exists, but the requested version does not.
        let mut operation = old_bool("not");
        operation.version = Version::new(99, 0, 0);
        assert!(operation.get_instantiated_op(&hugr)?.is_none());
        let mut ty = old_bool("bool");
        ty.version = Version::new(99, 0, 0);
        assert!(ty.get_type(&hugr)?.is_none());
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
        assert!(matches!(
            TypeReplacementTemplate::VersionedElement(requested.clone()).get_type(&hugr),
            Err(ReplacementError::Element(VersionedElementError::MissingType(element)))
                if element == requested
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
        replacer.set_replace_type(old_bool("bool").get_type(&hugr)?.unwrap(), bool_t());
        for name in ["make_opaque", "read"] {
            let operation = old_bool(name).get_instantiated_op(&hugr)?.unwrap();
            let template =
                OpReplacementTemplate::Empty.get_op_replace(&operation, &hugr, &replacer)?;
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
        let operation = old_bool("make_opaque").get_instantiated_op(&hugr)?.unwrap();
        let error = OpReplacementTemplate::Empty
            .get_op_replace(&operation, &hugr, &ReplaceTypes::default())
            .unwrap_err();
        assert!(error.source().unwrap().is::<hugr::builder::BuildError>());
        assert!(matches!(error, ReplacementError::Build(_)));
        Ok(())
    }
}
