use hugr::HugrView;
use hugr::{
    builder::{DFGBuilder, Dataflow, DataflowHugr},
    extension::Version,
    ops::{DataflowOpTrait, ExtensionOp},
    types::{Signature, Type},
};
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
    pub fn get_instantiated_op<T: HugrView>(&self, hugr: &T) -> ExtensionOp {
        // NICOLA: TODO: we should have a proper error here
        hugr.extensions()
            .get_exact(&self.extension_id, &self.version)
            .expect("Extension version is missing from the registry")
            .instantiate_extension_op(&self.id, [])
            .expect("Failed to instantiate extension operation")
    }

    pub fn get_type<T: HugrView>(&self, hugr: &T) -> CustomType {
        // NICOLA: TODO: we should have a proper error here
        hugr.extensions()
            .get_exact(&self.extension_id, &self.version)
            .expect("Extension version is missing from the registry")
            .get_type(self.id.as_str())
            .expect("Type is missing from the extension")
            .instantiate([])
            .expect("Failed to instantiate type")
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

impl OpReplacementTemplate {
    pub fn get_op_replace<T: HugrView>(&self, old_op: &ExtensionOp, hugr: &T) -> NodeTemplate {
        match self {
            OpReplacementTemplate::TemplateInstance(template) => template.clone(),
            OpReplacementTemplate::Empty => Self::get_node_template(old_op, &[], hugr),
            OpReplacementTemplate::VersionedElements(v) => Self::get_node_template(old_op, v, hugr),
        }
    }

    fn get_node_template<T: HugrView>(
        old_op: &ExtensionOp,
        versioned_elements: &[VersionedElement],
        hugr: &T,
    ) -> NodeTemplate {
        let operations = versioned_elements
            .iter()
            .map(|element| element.get_instantiated(hugr))
            .collect::<Vec<_>>();

        let signature = match (operations.first(), operations.last()) {
            (Some(first), Some(last)) => Signature::new(
                first.signature().input().clone(),
                last.signature().output().clone(),
            ),
            _ => old_op.signature().into_owned(),
        };
        let mut builder = DFGBuilder::new(signature).expect("Failed to build replacement");
        let mut wires = builder.input_wires().collect::<Vec<_>>();
        for operation in operations {
            // NICOLA: TODO: we should have a proper error here
            wires = builder
                .add_dataflow_op(operation, wires)
                .expect("Replacement operations have incompatible signatures")
                .outputs()
                .collect();
        }
        // NICOLA: TODO: we should have a proper error here
        NodeTemplate::linked_hugr(builder.finish_hugr_with_outputs(wires).unwrap())
    }
}

#[derive(Debug, Default)]
/// Represents a mapping from an old operation to its replacement(s).
pub struct OpMapping {
    map: HashMap<VersionedElement, OpReplacementTemplate>,
}

impl OpMapping {
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
