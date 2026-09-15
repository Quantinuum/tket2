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
#[allow(unused, missing_docs)]
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

// -----------------------------
// Old testing stuff
// -----------------------------

/// An error encountered while loading or printing a serialized HUGR.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum PrintExtensionVersionsError {
    /// The serialized HUGR could not be loaded.
    #[error(transparent)]
    Load(#[from] ReadError),
    /// The output could not be written.
    #[error(transparent)]
    Write(#[from] io::Error),
}

/// Load a serialized HUGR and print extension versions saved on operations and edges.
#[allow(dead_code, missing_docs)]
pub fn print_extension_versions(
    serialized_hugr: impl BufRead,
    extensions: Option<&ExtensionRegistry>,
    mut output: impl Write,
) -> Result<(), PrintExtensionVersionsError> {
    let hugr = Hugr::load(serialized_hugr, extensions)?;

    for node in hugr.nodes() {
        match hugr.get_optype(node) {
            OpType::ExtensionOp(operation) => {
                writeln!(
                    output,
                    "{}: {}",
                    operation.qualified_id(),
                    operation.extension_version()
                )?;
            }
            OpType::OpaqueOp(operation) => {
                if let Some(version) = operation.extension_version() {
                    writeln!(output, "{}: {version}", operation.qualified_id())?;
                }
            }
            _ => {}
        }
    }

    for source in hugr.nodes() {
        for source_port in hugr.node_outputs(source) {
            let Some(edge_kind) = hugr.get_optype(source).port_kind(source_port) else {
                continue;
            };

            for (target, target_port) in hugr.linked_inputs(source, source_port) {
                visit_edge_custom_types(&edge_kind, &mut |custom_type| {
                    if let Some(version) = custom_type.extension_version() {
                        writeln!(
                            output,
                            "edge {source}:{} -> {target}:{}: {}.{}: {version}",
                            source_port.index(),
                            target_port.index(),
                            custom_type.extension(),
                            custom_type.name(),
                        )?;
                    }
                    Ok(())
                })?;
            }
        }
    }

    Ok(())
}

#[allow(dead_code, missing_docs)]
fn visit_edge_custom_types(
    edge_kind: &EdgeKind,
    visit: &mut impl FnMut(&CustomType) -> io::Result<()>,
) -> io::Result<()> {
    match edge_kind {
        EdgeKind::Value(typ) | EdgeKind::Const(typ) => visit_term(typ, visit),
        EdgeKind::Function(function) => visit_poly_func_type(function, visit),
        _ => Ok(()),
    }
}

#[allow(dead_code, missing_docs)]
fn visit_poly_func_type(
    function: &PolyFuncType,
    visit: &mut impl FnMut(&CustomType) -> io::Result<()>,
) -> io::Result<()> {
    for param in function.params() {
        visit_term(param, visit)?;
    }
    for typ in function
        .body()
        .input()
        .iter()
        .chain(function.body().output().iter())
    {
        visit_term(typ, visit)?;
    }
    Ok(())
}

#[allow(dead_code, missing_docs)]
fn visit_term(
    term: &Term,
    visit: &mut impl FnMut(&CustomType) -> io::Result<()>,
) -> io::Result<()> {
    match term {
        Term::ExtensionType(custom_type) => {
            visit(custom_type)?;
            for arg in custom_type.args() {
                visit_term(arg, visit)?;
            }
        }
        Term::FunctionType(function) => {
            visit_term(function.input(), visit)?;
            visit_term(function.output(), visit)?;
        }
        Term::SumType(sum) => {
            for variant in sum.variants() {
                visit_term(variant, visit)?;
            }
        }
        Term::List(terms)
        | Term::ListConcat(terms)
        | Term::Tuple(terms)
        | Term::TupleConcat(terms) => {
            for term in terms {
                visit_term(term, visit)?;
            }
        }
        Term::ListKind(term) | Term::TupleKind(term) => visit_term(term, visit)?,
        Term::ConstKind(typ) => visit_term(typ, visit)?,
        _ => {}
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::io::Cursor;

    use hugr::builder::{DFGBuilder, Dataflow, DataflowHugr};
    use hugr::envelope::EnvelopeConfig;
    use hugr::extension::prelude::bool_t;
    use hugr::ops::DataflowOpTrait;
    use hugr::ops::handle::NodeHandle;
    use hugr::std_extensions::arithmetic::int_types::{VERSION as INT_VERSION, int_type};
    use hugr::std_extensions::collections::array::{VERSION as ARRAY_VERSION, array_type};
    use hugr::std_extensions::logic::{LogicOp, VERSION};
    use hugr::types::Signature;
    use hugr::{HugrView, PortIndex};

    use super::{
        ExtensionRegistry, ExtensionUpdater, STD_REG, UndatingMap, WeakExtensionRegistry,
        print_extension_versions,
    };

    #[test]
    fn replacement_migrates_bool_types() {
        let registry = ExtensionRegistry::new_with_extension_resolution(
            [serde_json::from_str(include_str!("../data/bool-0.2.0.json")).unwrap()],
            &WeakExtensionRegistry::from(&*STD_REG),
        )
        .unwrap();
        let old_extension = registry.get("tket.bool").unwrap();
        let old_op = |name: &str| old_extension.instantiate_extension_op(name, []).unwrap();
        let new_not = STD_REG
            .get("logic")
            .unwrap()
            .instantiate_extension_op("Not", [])
            .unwrap();

        for replacement_count in [1, 3] {
            for reverse in [false, true] {
                let mut builder =
                    DFGBuilder::new(Signature::new([bool_t()], [bool_t(), bool_t()])).unwrap();
                let [input] = builder.input_wires_arr();
                let make = builder
                    .add_dataflow_op(old_op("make_opaque"), [input])
                    .unwrap();
                let not = builder
                    .add_dataflow_op(old_op("not"), make.outputs())
                    .unwrap();
                let read = builder
                    .add_dataflow_op(old_op("read"), not.outputs())
                    .unwrap();
                let hugr = builder
                    .finish_hugr_with_outputs([read.out_wire(0); 2])
                    .unwrap();
                let mut updater = ExtensionUpdater::new(hugr, UndatingMap::new(vec![], vec![]));
                let mut replacements = vec![
                    (make.node(), vec![]),
                    (not.node(), vec![new_not.clone(); replacement_count]),
                    (read.node(), vec![]),
                ];
                if reverse {
                    replacements.reverse();
                }
                for (node, ops) in replacements {
                    updater.replace_node_preserving_connections(node, ops);
                }

                let hugr = updater.get_hugr();
                hugr.validate().unwrap();
                let operations = hugr
                    .nodes()
                    .filter_map(|n| hugr.get_optype(n).as_extension_op());
                assert_eq!(operations.clone().count(), replacement_count);
                assert!(
                    operations
                        .into_iter()
                        .all(|op| op.qualified_id() == "logic.Not")
                );
                let [_, output] = hugr.get_io(hugr.entrypoint()).unwrap();
                assert_eq!(
                    hugr.single_linked_output(output, 0),
                    hugr.single_linked_output(output, 1)
                );
            }
        }
    }

    #[test]
    fn replacement_rejects_arity_changes_before_mutating() {
        for old_name in ["And", "Not"] {
            let extension = STD_REG.get("logic").unwrap();
            let old_op = extension.instantiate_extension_op(old_name, []).unwrap();
            let mut builder = DFGBuilder::new(old_op.signature().into_owned()).unwrap();
            let operation = builder
                .add_dataflow_op(old_op, builder.input_wires())
                .unwrap();
            let hugr = builder
                .finish_hugr_with_outputs(operation.outputs())
                .unwrap();
            let original = hugr.clone();
            let mut updater = ExtensionUpdater::new(hugr, UndatingMap::new(vec![], vec![]));
            let replacement = if old_name == "And" {
                vec![] // Two inputs cannot pass through to one output.
            } else {
                vec![extension.instantiate_extension_op("And", []).unwrap()]
            };
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                updater.replace_node_preserving_connections(operation.node(), replacement);
            }));
            assert!(result.is_err());
            assert_eq!(updater.get_hugr(), &original);
        }
    }

    #[test]
    fn replacement_preserves_connections() {
        for replacement_count in [0, 1, 3] {
            // Cover both fan-out and a discarded (unconnected) output.
            for output_count in [0, 2] {
                let mut builder =
                    DFGBuilder::new(Signature::new([bool_t()], vec![bool_t(); output_count]))
                        .unwrap();
                let [input] = builder.input_wires_arr();
                let operation = builder.add_dataflow_op(LogicOp::Not, [input]).unwrap();
                let node = operation.node();
                let output = operation.out_wire(0);
                let hugr = builder
                    .finish_hugr_with_outputs(vec![output; output_count])
                    .unwrap();
                let [input_node, output_node] = hugr.get_io(hugr.entrypoint()).unwrap();
                let op = hugr.get_optype(node).as_extension_op().unwrap().clone();
                let mut updater = ExtensionUpdater::new(hugr, UndatingMap::new(vec![], vec![]));

                updater.replace_node_preserving_connections(node, vec![op; replacement_count]);

                let hugr = updater.get_hugr();
                hugr.validate().unwrap();
                assert_eq!(
                    hugr.nodes()
                        .filter(|&n| hugr.get_optype(n).is_extension_op())
                        .count(),
                    replacement_count
                );
                let mut source = input_node;
                for _ in 0..replacement_count {
                    let (next, port) = hugr.single_linked_input(source, 0).unwrap();
                    assert_eq!(port.index(), 0);
                    assert!(hugr.get_optype(next).is_extension_op());
                    source = next;
                }
                for port in 0..output_count {
                    assert_eq!(
                        hugr.single_linked_output(output_node, port),
                        Some((source, 0.into()))
                    );
                }
            }
        }
    }

    #[test]
    fn prints_versions_from_serialized_operations() {
        let mut builder = DFGBuilder::new(Signature::new_endo([bool_t()])).unwrap();
        let [input] = builder.input_wires_arr();
        let operation = builder.add_dataflow_op(LogicOp::Not, [input]).unwrap();
        let mut hugr = builder
            .finish_hugr_with_outputs(operation.outputs())
            .unwrap();
        let serialized = hugr.store_str(EnvelopeConfig::text()).unwrap();
        let mut output = Vec::new();

        print_extension_versions(Cursor::new(serialized), None, &mut output).unwrap();
        // println!("Extension:\n{:#?}", hugr.extensions());
        std::fs::write(
            "extension_registry.json",
            serde_json::to_string_pretty(&hugr.extensions()).unwrap(),
        )
        .unwrap();
        println!(
            "Extension registry:\n{:#?}",
            hugr.resolve_extension_defs(hugr.clone().extensions())
        );
        println!("Output:\n{}", String::from_utf8_lossy(&output));
        assert_eq!(
            String::from_utf8(output).unwrap(),
            format!("logic.Not: {VERSION}\n")
        );
    }

    #[test]
    fn ignores_operations_without_extension_versions() {
        let hugr = hugr::Hugr::new();
        let serialized = hugr.store_str(EnvelopeConfig::text()).unwrap();
        let mut output = Vec::new();

        print_extension_versions(Cursor::new(serialized), None, &mut output).unwrap();

        assert!(output.is_empty());
    }

    #[test]
    fn prints_versions_from_nested_edge_types() {
        let edge_type = array_type(2, int_type(5));
        let builder = DFGBuilder::new(Signature::new_endo([edge_type])).unwrap();
        let [input] = builder.input_wires_arr();
        let hugr = builder.finish_hugr_with_outputs([input]).unwrap();
        let serialized = hugr.store_str(EnvelopeConfig::text()).unwrap();
        let mut output = Vec::new();

        print_extension_versions(Cursor::new(serialized), None, &mut output).unwrap();

        std::fs::write(
            "extension_registry.json",
            serde_json::to_string_pretty(&hugr.extensions()).unwrap(),
        )
        .unwrap();

        let output = String::from_utf8(output).unwrap();
        println!("Output:\n{output}");
        assert_eq!(output.lines().count(), 6, "{output}");
        assert_eq!(
            output
                .matches(&format!("collections.array.array: {ARRAY_VERSION}"))
                .count(),
            3
        );
        assert_eq!(
            output
                .matches(&format!("arithmetic.int.types.int: {INT_VERSION}"))
                .count(),
            3
        );
        assert!(output.lines().all(|line| line.starts_with("edge ")));
    }
}
