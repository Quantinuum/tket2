//! Upgrade a serialized HUGR to a compatible extension version.

use std::error::Error;
use std::io;
use std::sync::Arc;

use hugr::builder::{DFGBuilder, Dataflow, DataflowHugr};
use hugr::envelope::EnvelopeConfig;
use hugr::extension::prelude::{PRELUDE, bool_t, usize_t};
use hugr::extension::{Extension, ExtensionId, ExtensionRegistry, Version};
use hugr::ops::OpType;
use hugr::types::Signature;
use hugr::{Hugr, HugrView};

const OP_NAME: &str = "select";

fn make_extension(version: Version) -> Arc<Extension> {
    Extension::new_arc(
        ExtensionId::new_unchecked("example.operations"),
        version,
        |extension, extension_ref| {
            // A versioned operation with a non-trivial signature: given a
            // boolean selector and two integers, produce a single integer.
            extension
                .add_op(
                    OP_NAME.into(),
                    "A versioned select operation".into(),
                    Signature::new([bool_t(), usize_t(), usize_t()], [usize_t()]),
                    extension_ref,
                )
                .unwrap();
        },
    )
}

fn operation_version(hugr: &Hugr) -> Option<Version> {
    hugr.nodes().find_map(|node| match hugr.get_optype(node) {
        OpType::ExtensionOp(operation) if operation.unqualified_id() == OP_NAME => {
            Some(operation.extension_version())
        }
        OpType::OpaqueOp(operation) if operation.unqualified_id() == OP_NAME => {
            operation.extension_version().cloned()
        }
        _ => None,
    })
}
/// Upgrade a serialized HUGR to a compatible extension version.
fn main() -> Result<(), Box<dyn Error>> {
    // Build and serialize a HUGR using version 1.0.0 of the extension.
    let old_extension = make_extension(Version::new(1, 0, 0));
    let old_operation = old_extension.instantiate_extension_op(OP_NAME, [])?;

    let mut builder = DFGBuilder::new(Signature::new(
        [bool_t(), usize_t(), usize_t()],
        [usize_t()],
    ))?;
    let [selector, on_true, on_false] = builder.input_wires_arr();
    let operation = builder.add_dataflow_op(old_operation, [selector, on_true, on_false])?;
    let hugr = builder.finish_hugr_with_outputs(operation.outputs())?;
    let serialized = hugr.store_str(EnvelopeConfig::text())?;

    // Reload the serialized HUGR using its original extension definition.
    // The prelude is required because the operation signature uses `usize`.
    let old_registry = ExtensionRegistry::new([old_extension, PRELUDE.clone()]);
    let mut hugr = Hugr::load_str(&serialized, Some(&old_registry))?;
    let before = operation_version(&hugr)
        .ok_or_else(|| io::Error::other("versioned operation was not found"))?;
    println!("before: example.operations.{OP_NAME}@{before}");

    // Register a compatible version and use resolve_extension_defs to relink
    // every compatible operation and custom type in the HUGR.
    let new_extension = make_extension(Version::new(2, 1, 0));
    let mut target_registry = old_registry;
    target_registry.register(new_extension);
    hugr.resolve_extension_defs(&target_registry)?;
    hugr.validate()?;

    let after = operation_version(&hugr)
        .ok_or_else(|| io::Error::other("upgraded operation was not found"))?;
    println!("after:  example.operations.{OP_NAME}@{after}");

    // The upgraded HUGR can now be serialized again.
    let _upgraded = hugr.store_str(EnvelopeConfig::text())?;

    Ok(())
}
