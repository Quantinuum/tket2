use hugr::HugrView;
use hugr::{Hugr, extension::Version, ops::ExtensionOp, types::Type};
use std::collections::HashMap;
use tket::passes::replace_types::NodeTemplate;

/// Represents an Extension Op by its name, the extension it belongs to, and its version.
#[derive(Debug, Eq, Hash, PartialEq)]
pub struct VersionedElement {
    pub(crate) id: String,
    pub(crate) extension_id: String,
    pub(crate) version: Version,
}

impl VersionedElement {
    pub fn new(id: String, extension_id: String, version: Version) -> Self {
        Self {
            id,
            extension_id,
            version,
        }
    }

    /// Instantiates the extension operation from the given Hugr view.
    pub fn get_instantiated<T: HugrView>(&self, hugr: &T) -> ExtensionOp {
        // NICOLA: TODO: we should have a proper error here
        hugr.extensions()
            .get_exact(&self.extension_id, &self.version)
            .expect("Extension version is missing from the registry")
            .instantiate_extension_op(&self.id, [])
            .expect("Failed to instantiate extension operation")
    }
}

/// A recipe for creating the replacement
#[derive(Debug)]
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

#[derive(Debug, Default)]
/// Represents a mapping from an old operation to its replacement(s).
pub struct OpUpdateMap {
    map: HashMap<VersionedElement, OpReplacementTemplate>,
}

impl OpUpdateMap {
    pub fn new(map: HashMap<VersionedElement, OpReplacementTemplate>) -> Self {
        Self { map }
    }

    pub fn insert(&mut self, old_op: VersionedElement, replacement: OpReplacementTemplate) {
        self.map.insert(old_op, replacement);
    }

    pub fn get_replacement(&self, operation: &ExtensionOp) -> Option<&OpReplacementTemplate> {
        let versioned_element = VersionedElement::new(
            operation.unqualified_id().to_string(),
            operation.extension_id().to_string(),
            operation.extension_version().clone(),
        );

        let Some(replacement) = self.map.get(&versioned_element) else {
            return None;
        };

        Some(replacement)
    }
}

impl From<Vec<(VersionedElement, OpReplacementTemplate)>> for OpUpdateMap {
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

#[derive(Debug)]
/// Mapping used to update signature of input/output ports of dataflow and controlflow operations.
///
/// Maps a Type, declared as a `VersionedElement`, to a new Type.
pub struct TypeMapping {
    map: HashMap<VersionedElement, TypeReplacementTemplate>,
}

impl TypeMapping {
    pub fn new(map: HashMap<VersionedElement, TypeReplacementTemplate>) -> Self {
        Self { map }
    }

    pub fn insert(&mut self, old_type: VersionedElement, new_type: TypeReplacementTemplate) {
        self.map.insert(old_type, new_type);
    }

    fn get_new_type(&self, old_type: &VersionedElement) -> Option<&TypeReplacementTemplate> {
        self.map.get(old_type)
    }
}

impl From<Vec<(VersionedElement, TypeReplacementTemplate)>> for TypeMapping {
    fn from(entries: Vec<(VersionedElement, TypeReplacementTemplate)>) -> Self {
        Self {
            map: entries.into_iter().collect(),
        }
    }
}
