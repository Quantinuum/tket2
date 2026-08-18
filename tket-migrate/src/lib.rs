//! Utilities for inspecting extension versions on operations and edges in serialized HUGRs.

use std::io::{self, BufRead, Write};

use hugr::envelope::ReadError;
use hugr::extension::ExtensionRegistry;
use hugr::ops::OpType;
use hugr::types::{CustomType, EdgeKind, PolyFuncType, Term};
use hugr::{Hugr, HugrView, PortIndex};
use thiserror::Error;

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
///
/// Operations are written in `qualified.operation: version` format. For every
/// connected edge, each extension type contained in its type is written in
/// `edge source:port -> target:port: qualified.type: version` format.
/// `extensions` has the same semantics as the corresponding argument to
/// [`Hugr::load`]: when it is `None`, the standard extension registry is used.
///
/// # Errors
///
/// Returns an error if the HUGR cannot be loaded or the output cannot be
/// written.
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
