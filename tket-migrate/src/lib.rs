//! Utilities for inspecting extension versions on operations and edges in serialized HUGRs.

use std::io::{self, BufRead, Write};

use hugr::builder::{DFGBuilder, Dataflow, DataflowHugr};
use hugr::envelope::ReadError;
use hugr::extension::resolution::WeakExtensionRegistry;
use hugr::extension::{ExtensionRegistry, Version};
use hugr::hugr::hugrmut::HugrMut;
use hugr::hugr::views::SiblingSubgraph;
use hugr::ops::{DataflowOpTrait, ExtensionOp, OpType};
use hugr::std_extensions::STD_REG;
use hugr::types::{CustomType, EdgeKind, PolyFuncType, Signature, Term, Type};
use hugr::{Extension, Hugr, HugrView, Node, PortIndex, SimpleReplacement};
use thiserror::Error;

#[derive(Clone, Debug)]
#[allow(unused, missing_docs)]
/// Represents an element (operation or type) with a specific version in an extension.
pub struct VersionedElement {
    name: String,
    extension_name: String,
    version: Version,
}

#[allow(unused, missing_docs)]
impl VersionedElement {
    pub fn new(name: String, extension_name: String, version: Version) -> Self {
        Self {
            name,
            extension_name,
            version,
        }
    }
}

#[derive(Debug)]
/// Represents a mapping from an old operation to its replacement(s).
pub struct OpUpdateMap {
    old_op: VersionedElement,
    replacement: Vec<VersionedElement>,
}

#[allow(unused, missing_docs)]
impl OpUpdateMap {
    pub fn new(old_op: VersionedElement, replacement: Vec<VersionedElement>) -> Self {
        Self {
            old_op,
            replacement,
        }
    }
}

#[derive(Clone, Debug)]
#[allow(unused)]
/// Mapping used to update signature of input/output ports of dataflow and controlflow operations.
pub struct TypeMapping {
    old_type: VersionedElement,
    new_type: VersionedElement,
}

impl TypeMapping {
    /// Create a mapping from an old versioned type to its replacement.
    pub fn new(old_type: VersionedElement, new_type: VersionedElement) -> Self {
        Self { old_type, new_type }
    }

    fn get_new_type(&self) -> &VersionedElement {
        &self.new_type
    }
}

#[derive(Debug)]
#[allow(unused, missing_docs)]
pub struct UndatingMap {
    op_update_maps: Vec<OpUpdateMap>,
    type_mappings: Vec<TypeMapping>,
}

#[allow(unused, missing_docs)]
impl UndatingMap {
    pub fn new(op_update_maps: Vec<OpUpdateMap>, type_mappings: Vec<TypeMapping>) -> Self {
        Self {
            op_update_maps,
            type_mappings,
        }
    }

    fn filter_op(
        &self,
        op_qualified_id: &str,
        extension_version: &Version,
    ) -> Option<&OpUpdateMap> {
        self.op_update_maps.iter().find(|op_update_map| {
            op_update_map.old_op.version == *extension_version
                && op_qualified_id
                    .strip_prefix(&op_update_map.old_op.extension_name)
                    .and_then(|name| name.strip_prefix('.'))
                    == Some(op_update_map.old_op.name.as_str())
        })
    }

    fn filter_type(
        &self,
        type_qualified_id: &str,
        extension_version: &Version,
    ) -> Option<&TypeMapping> {
        self.type_mappings.iter().find(|type_mapping| {
            type_mapping.old_type.version == *extension_version
                && type_qualified_id
                    .strip_prefix(&type_mapping.old_type.extension_name)
                    .and_then(|name| name.strip_prefix('.'))
                    == Some(type_mapping.old_type.name.as_str())
        })
    }
}

/// ?
#[derive(Debug)]
#[allow(unused)]
pub struct ExtensionUpdater {
    hugr: Hugr,
    updating_map: UndatingMap,
}

#[allow(unused)]
impl ExtensionUpdater {
    // ...
    #[allow(missing_docs)]
    pub fn new(hugr: Hugr, updating_map: UndatingMap) -> Self {
        Self { hugr, updating_map }
    }

    pub(crate) fn get_hugr(&self) -> &Hugr {
        &self.hugr
    }

    fn get_op(&self, op: &VersionedElement) -> ExtensionOp {
        self.hugr
            .extensions()
            .get_exact(&op.extension_name, &op.version)
            .expect(&format!(
                "{} version {} is missing from the registry",
                op.extension_name, op.version
            ))
            .instantiate_extension_op(&op.name, [])
            .expect(&format!(
                "failed to instantiate {} {} version {}",
                op.extension_name, op.name, op.version
            ))
    }

    #[allow(dead_code, missing_docs)]
    pub fn migrate_hugr(&mut self, new_extensions: Vec<Extension>) {
        self.add_new_extension(new_extensions);
        for node in self.hugr.nodes().collect::<Vec<_>>() {
            self.update_node(node);
        }
    }

    fn add_new_extension(&mut self, new_extensions: Vec<Extension>) {
        let mut extensions = STD_REG.to_owned();
        extensions.extend(self.hugr.extensions().clone());
        let new_ext_registry = ExtensionRegistry::new_with_extension_resolution(
            new_extensions,
            &WeakExtensionRegistry::from(&extensions),
        )
        .unwrap();
        extensions.extend(new_ext_registry);
        self.hugr.use_extensions(extensions);
        std::fs::write(
            "updated_extension_registry.json",
            serde_json::to_string_pretty(&self.hugr.extensions()).unwrap(),
        )
        .unwrap();
        println!("saved Hugr extensions");
    }

    fn update_node(&mut self, old_node: Node) {
        match self.hugr.get_optype(old_node) {
            OpType::Module(_) => {}
            OpType::FuncDefn(_) => {}
            OpType::FuncDecl(_) => {}
            OpType::AliasDecl(_) => {}
            OpType::AliasDefn(_) => {}
            OpType::Const(_) => {}
            OpType::Input(_) => {}
            OpType::Output(_) => {}
            OpType::Call(_) => {}
            OpType::CallIndirect(_) => {}
            OpType::LoadConstant(_) => {}
            OpType::LoadFunction(_) => {}
            OpType::DFG(_) => {}
            OpType::ExtensionOp(_) => self.update_extension_op(old_node),
            OpType::OpaqueOp(_) => {}
            OpType::Tag(_) => {}
            OpType::DataflowBlock(_) => {}
            OpType::ExitBlock(_) => {}
            OpType::TailLoop(_) => {}
            OpType::CFG(_) => {}
            OpType::Conditional(_) => {}
            OpType::Case(_) => {}
            _ => {}
        }
    }

    fn update_extension_op(&mut self, old_node: Node) {
        let OpType::ExtensionOp(operation) = self.hugr.get_optype(old_node) else {
            // assert never
            return;
        };
        let Some(op_update_map) = self
            .updating_map
            .filter_op(&operation.qualified_id(), &operation.extension_version())
        else {
            return;
        };
        let replacement = op_update_map.replacement.clone();
        let replacement_ops = replacement
            .iter()
            .map(|op| self.get_op(op))
            .collect::<Vec<_>>();
        println!(
            "Upgraded:\n{}@{}\nwith:\n{}",
            operation.qualified_id(),
            operation.extension_version(),
            replacement
                .iter()
                .map(|op| format!("{}.{}@{}", op.extension_name, op.name, op.version))
                .collect::<Vec<_>>()
                .join(", "),
        );
        self.replace_node_preserving_connections(old_node, replacement_ops);
        println!("========")
    }

    fn replace_node_preserving_connections(
        &mut self,
        node: Node,
        replacement_ops: Vec<ExtensionOp>,
    ) {
        let subgraph = SiblingSubgraph::from_node(node, &self.hugr);
        let old_signature = subgraph.signature(&self.hugr);
        let signature = match (replacement_ops.first(), replacement_ops.last()) {
            (Some(first), Some(last)) => Signature::new(
                first.signature().input().clone(),
                last.signature().output().clone(),
            ),
            _ => Signature::new_endo(old_signature.input().clone()),
        };
        let mut builder = DFGBuilder::new(signature).expect("failed to create replacement builder");
        let mut wires = builder.input_wires().collect::<Vec<_>>();
        for op in replacement_ops {
            wires = builder
                .add_dataflow_op(op, wires)
                .expect("replacement operations have incompatible signatures")
                .outputs()
                .collect();
        }
        // With no replacement operations, the inputs pass straight through.
        let replacement = builder
            .finish_hugr_with_outputs(wires)
            .expect("replacement operations do not form a valid dataflow graph");
        // Migration may change boundary types while preserving port positions.
        // Neighbouring operations can still have old types until they are updated,
        // so the host must be validated after the complete migration.
        let replacement = SimpleReplacement::new_unchecked(subgraph, replacement);
        self.hugr
            .apply_patch(replacement)
            .expect("failed to replace operation");
    }
}
