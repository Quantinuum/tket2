//! Utilities for inspecting extension versions on operations and edges in serialized HUGRs.

use std::io::{self, BufRead, Write};

use hugr::envelope::ReadError;
use hugr::extension::resolution::WeakExtensionRegistry;
use hugr::extension::{ExtensionRegistry, Version};
use hugr::hugr::hugrmut::HugrMut;
use hugr::ops::{ExtensionOp, OpType};
use hugr::types::{CustomType, EdgeKind, PolyFuncType, Term};
use hugr::{Extension, Hugr, HugrView, Node, PortIndex};
use thiserror::Error;

#[derive(Clone, Debug)]
#[allow(unused, missing_docs)]
pub struct VersionedOp {
    name: String,
    extension_name: String,
    version: Version,
}

#[allow(unused, missing_docs)]
impl VersionedOp {
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
    old_op: VersionedOp,
    replacement: Vec<VersionedOp>,
}

#[allow(unused, missing_docs)]
impl OpUpdateMap {
    pub fn new(old_op: VersionedOp, replacement: Vec<VersionedOp>) -> Self {
        Self {
            old_op,
            replacement,
        }
    }
}

#[derive(Debug)]
#[allow(unused, missing_docs)]
pub struct UndatingMap {
    op_update_maps: Vec<OpUpdateMap>,
}

#[allow(unused, missing_docs)]
impl UndatingMap {
    pub fn new(op_update_maps: Vec<OpUpdateMap>) -> Self {
        Self { op_update_maps }
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

    fn get_op(&self, op: &VersionedOp) -> ExtensionOp {
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
    pub fn update_op(&mut self, new_extensions: Vec<Extension>) {
        self.add_new_extension(new_extensions);
        for node in self.hugr.nodes().collect::<Vec<_>>() {
            self.update_node(node);
        }
    }

    fn add_new_extension(&mut self, new_extensions: Vec<Extension>) {
        let hugr_extensions = self.hugr.extensions();
        let new_ext_registry = ExtensionRegistry::new_with_extension_resolution(
            new_extensions,
            &WeakExtensionRegistry::from(hugr_extensions),
        )
        .unwrap();
        self.hugr.use_extensions(new_ext_registry);
        std::fs::write(
            "updated_extension_registry.json",
            serde_json::to_string_pretty(&self.hugr.extensions()).unwrap(),
        )
        .unwrap();
        println!("saved Hugr extensions");
    }

    fn update_node(&mut self, node: Node) {
        let OpType::ExtensionOp(operation) = self.hugr.get_optype(node) else {
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
        self.replace_node_preserving_connections(node, replacement_ops);
        println!("========")
    }

    fn replace_node_preserving_connections(
        &mut self,
        node: Node,
        replacement_ops: Vec<ExtensionOp>,
    ) {
        let parent = self.hugr.get_parent(node).expect("operation has no parent");
        let incoming = self
            .hugr
            .node_inputs(node)
            .flat_map(|port| {
                self.hugr
                    .linked_outputs(node, port)
                    .map(move |(source, source_port)| (source, source_port, port))
            })
            .collect::<Vec<_>>();
        let outgoing = self
            .hugr
            .node_outputs(node)
            .flat_map(|port| {
                self.hugr
                    .linked_inputs(node, port)
                    .map(move |(target, target_port)| (port, target, target_port))
            })
            .collect::<Vec<_>>();

        self.hugr.remove_node(node);
        let replacement_nodes = replacement_ops
            .into_iter()
            .map(|op| self.hugr.add_node_with_parent(parent, op))
            .collect::<Vec<_>>();

        let Some((&first, rest)) = replacement_nodes.split_first() else {
            for (source, source_port, input_port) in incoming {
                for (_, target, target_port) in outgoing
                    .iter()
                    .filter(|(output_port, _, _)| output_port.index() == input_port.index())
                {
                    self.hugr
                        .connect(source, source_port, *target, *target_port);
                }
            }
            return;
        };

        for (source, source_port, target_port) in incoming {
            self.hugr.connect(source, source_port, first, target_port);
        }
        for nodes in replacement_nodes.windows(2) {
            for (output, input) in self
                .hugr
                .node_outputs(nodes[0])
                .filter(|port| {
                    matches!(
                        self.hugr.get_optype(nodes[0]).port_kind(*port),
                        Some(EdgeKind::Value(_))
                    )
                })
                .zip(self.hugr.node_inputs(nodes[1]).filter(|port| {
                    matches!(
                        self.hugr.get_optype(nodes[1]).port_kind(*port),
                        Some(EdgeKind::Value(_))
                    )
                }))
                .collect::<Vec<_>>()
            {
                self.hugr.connect(nodes[0], output, nodes[1], input);
            }
        }
        let last = rest.last().copied().unwrap_or(first);
        for (source_port, target, target_port) in outgoing {
            self.hugr.connect(last, source_port, target, target_port);
        }
    }

    // -----------------------------
    // Old testing stuff
    // -----------------------------
}
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

    use hugr::HugrView;
    use hugr::builder::{DFGBuilder, Dataflow, DataflowHugr};
    use hugr::envelope::EnvelopeConfig;
    use hugr::extension::prelude::bool_t;
    use hugr::std_extensions::arithmetic::int_types::{VERSION as INT_VERSION, int_type};
    use hugr::std_extensions::collections::array::{VERSION as ARRAY_VERSION, array_type};
    use hugr::std_extensions::logic::{LogicOp, VERSION};
    use hugr::types::Signature;

    use super::print_extension_versions;

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
