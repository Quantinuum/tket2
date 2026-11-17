use std::io;

use crate::{
    error::MigrationError,
    extension_loading::LOCAL_REGISTRY,
    update_maps::{OpMapping, TypeMapping},
};
use hugr::Hugr;
use hugr::{HugrView, hugr::hugrmut::HugrMut};
use tket::passes::{ComposablePass, ReplaceTypes};

/// Applies lists of operation and type mappings in a single replacement pass.
///
/// All type mappings are registered before operation replacements are built.
/// Later mappings override earlier mappings for the same source.
fn migrate(
    mut hugr: Hugr,
    op_mappings: &[&OpMapping],
    type_mappings: &[&TypeMapping],
) -> Result<Hugr, MigrationError> {
    let mut replacer = ReplaceTypes::default();

    for (old_type, new_type) in type_mappings.iter().flat_map(|mapping| mapping.iter()) {
        let old_type = old_type.get_type(&hugr)?;
        replacer.set_replace_type(old_type, new_type.clone());
    }

    for (old_element, replacement) in op_mappings.iter().flat_map(|mapping| mapping.iter()) {
        let old_op = old_element.get_instantiated_op(&hugr)?;
        let template = replacement.get_op_replace(&old_op, &replacer)?;
        replacer.set_replace_op(&old_op, template);
    }

    replacer.run(&mut hugr)?;

    // remove unused extensions
    let registry = hugr.extensions().clone();
    hugr.resolve_extension_defs(&registry).unwrap();

    hugr.validate()?;

    Ok(hugr)
}

/// Migrates legacy TKET measurement and boolean operations using local extensions.
///
/// The HUGR may contain unresolved operations and types and no extension registry.
pub fn migrate_hugr_default(mut hugr: Hugr) -> Result<Hugr, MigrationError> {
    // Ensure that the hugr has all the extensions
    hugr.use_extensions(&*LOCAL_REGISTRY);
    migrate(
        hugr,
        &[
            &*crate::default_maps::MEAS_OP_MAPPING,
            &*crate::default_maps::BOOL_OP_MAPPING,
        ],
        &[&*crate::default_maps::BOOL_TYPE_MAPPING],
    )
}

/// Migrates legacy TKET measurement and boolean operations using local extensions.
///
/// Reads a HUGR from bytes and migrates it.
pub fn migrate_bytes_default(bytes: impl io::BufRead) -> Result<Hugr, MigrationError> {
    let hugr = Hugr::load(bytes, Some(&LOCAL_REGISTRY))?;
    migrate_hugr_default(hugr)
}

#[cfg(test)]
pub(crate) mod test_helpers {
    use hugr::{
        Extension, Hugr,
        builder::{
            DFGBuilder, Dataflow, DataflowHugr, DataflowSubContainer, FunctionBuilder, SubContainer,
        },
        extension::{ExtensionRegistry, prelude::bool_t, resolution::WeakExtensionRegistry},
        std_extensions::{STD_REG, logic::LogicOp},
        types::{Signature, Type},
    };
    use std::{error::Error, fs::File, io::BufReader, path::PathBuf};

    const QUANTUM_EXTENSION: &str = "tket.quantum";
    const BOOL_EXTENSION: &str = "tket.bool";

    pub(crate) fn load_registry(paths: &[PathBuf]) -> Result<ExtensionRegistry, Box<dyn Error>> {
        let mut registry = STD_REG.to_owned();
        let extensions = paths
            .iter()
            .map(|path| -> Result<Extension, Box<dyn Error>> {
                Ok(serde_json::from_reader(BufReader::new(File::open(path)?))?)
            })
            .collect::<Result<Vec<_>, _>>()?;
        let custom_extensions = ExtensionRegistry::new_with_extension_resolution(
            extensions,
            &WeakExtensionRegistry::from(&registry),
        )?;
        registry.extend(custom_extensions.clone());
        Ok(registry)
    }

    pub(crate) fn build_old_hugr(registry: &ExtensionRegistry) -> Result<Hugr, Box<dyn Error>> {
        let quantum = registry
            .get(QUANTUM_EXTENSION)
            .ok_or("tket.quantum is missing from the registry")?;
        let bool_extension = registry
            .get(BOOL_EXTENSION)
            .ok_or("tket.bool is missing from the registry")?;
        let qalloc = quantum.instantiate_extension_op("QAlloc", [])?;
        let h = quantum.instantiate_extension_op("H", [])?;
        let measure_free = quantum.instantiate_extension_op("MeasureFree", [])?;
        let read = bool_extension.instantiate_extension_op("read", [])?;

        let mut builder = DFGBuilder::new(Signature::new(vec![], vec![]))?;
        let qubit = builder.add_dataflow_op(qalloc, [])?.out_wire(0);
        let qubit = builder.add_dataflow_op(h, [qubit])?.out_wire(0);
        let measurement = builder.add_dataflow_op(measure_free, [qubit])?.out_wire(0);
        let boolean = builder.add_dataflow_op(read, [measurement])?.out_wire(0);
        builder.add_dataflow_op(LogicOp::Not, [boolean])?;
        Ok(builder.finish_hugr_with_outputs([])?)
    }

    pub(crate) fn build_bool_hugr(registry: &ExtensionRegistry) -> Result<Hugr, Box<dyn Error>> {
        let bool_extension = registry
            .get(BOOL_EXTENSION)
            .ok_or("tket.bool is missing from the registry")?;
        let make_opaque = bool_extension.instantiate_extension_op("make_opaque", [])?;
        let not = bool_extension.instantiate_extension_op("not", [])?;
        let and = bool_extension.instantiate_extension_op("and", [])?;
        let eq = bool_extension.instantiate_extension_op("eq", [])?;
        let or = bool_extension.instantiate_extension_op("or", [])?;
        let xor = bool_extension.instantiate_extension_op("xor", [])?;
        let read = bool_extension.instantiate_extension_op("read", [])?;

        let mut builder = DFGBuilder::new(Signature::new(vec![bool_t(); 5], [bool_t()]))?;
        let inputs = builder.input_wires();
        let mut values = Vec::with_capacity(inputs.len());
        for input in inputs {
            values.push(
                builder
                    .add_dataflow_op(make_opaque.clone(), [input])?
                    .out_wire(0),
            );
        }

        let value = builder.add_dataflow_op(not, [values[0]])?.out_wire(0);
        let value = builder
            .add_dataflow_op(and, [value, values[1]])?
            .out_wire(0);
        let value = builder.add_dataflow_op(eq, [value, values[2]])?.out_wire(0);
        let value = builder.add_dataflow_op(or, [value, values[3]])?.out_wire(0);
        let value = builder
            .add_dataflow_op(xor, [value, values[4]])?
            .out_wire(0);
        let output = builder.add_dataflow_op(read, [value])?.out_wire(0);

        Ok(builder.finish_hugr_with_outputs([output])?)
    }

    pub(crate) fn build_bool_cfg_hugr(
        registry: &ExtensionRegistry,
    ) -> Result<Hugr, Box<dyn Error>> {
        let bool_extension = registry
            .get(BOOL_EXTENSION)
            .ok_or("tket.bool is missing from the registry")?;
        let boolean: Type = bool_extension
            .get_type("bool")
            .ok_or("tket.bool.bool is missing from the extension")?
            .instantiate([])?
            .into();
        let not = bool_extension.instantiate_extension_op("not", [])?;
        let signature = Signature::new([boolean.clone()], [boolean.clone()]);
        let mut function = FunctionBuilder::new("bool_cfg", signature.clone())?;
        let [mut value] = function.input_wires_arr();

        for _ in 0..2 {
            let mut cfg =
                function.cfg_builder([(boolean.clone(), value)], [boolean.clone()].into())?;
            let mut block = cfg.entry_builder([vec![].into()], [boolean.clone()].into())?;
            let [input] = block.input_wires_arr();
            let mut dfg = block.dfg_builder(signature.clone(), [input])?;
            let [input] = dfg.input_wires_arr();
            let result = dfg.add_dataflow_op(not.clone(), [input])?.out_wire(0);
            let dfg = dfg.finish_with_outputs([result])?;
            let tag = block.make_sum(0, [vec![].into()], [])?;
            let block = block.finish_with_outputs(tag, dfg.outputs())?;
            cfg.branch(&block, 0, &cfg.exit_block())?;
            value = cfg.finish_sub_container()?.out_wire(0);
        }

        Ok(function.finish_hugr_with_outputs([value])?)
    }

    pub(crate) fn bool_graph() -> Result<Hugr, Box<dyn Error>> {
        let fixture =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("extensions/tket/bool-0.2.0.json");
        let registry = load_registry(&[fixture])?;
        build_bool_hugr(&registry)
    }

    pub(crate) fn identity_graph() -> Result<Hugr, Box<dyn Error>> {
        let builder = DFGBuilder::new(Signature::new_endo([bool_t()]))?;
        let inputs = builder.input_wires();
        Ok(builder.finish_hugr_with_outputs(inputs)?)
    }
}

#[cfg(test)]
mod tests {
    use super::{migrate, migrate_bytes_default, migrate_hugr_default, test_helpers::*};
    use crate::update_maps::{OpMapping, OpReplacementTemplate, TypeMapping, VersionedElement};
    use hugr::{
        Hugr, HugrView,
        builder::{DFGBuilder, Dataflow, DataflowHugr},
        envelope::EnvelopeConfig,
        extension::Version,
        hugr::hugrmut::HugrMut,
        ops::{DataflowOpTrait, OpTrait},
        package::Package,
    };
    use std::{collections::HashMap, error::Error, path::PathBuf};
    use tket::passes::replace_types::NodeTemplate;

    fn migrate_from_hugr_and_bytes(hugr: Hugr) -> Result<[Hugr; 2], Box<dyn Error>> {
        // Package::from_hugr omits extension definitions, so loading the bytes
        // must resolve the legacy operations and types from the local registry.
        let package = Package::from_hugr(hugr.clone());
        assert!(package.extensions.is_empty());
        let mut bytes = Vec::new();
        package.store(&mut bytes, EnvelopeConfig::text())?;
        assert!(Hugr::load(bytes.as_slice(), None).is_err());
        Ok([
            migrate_hugr_default(hugr)?,
            migrate_bytes_default(bytes.as_slice())?,
        ])
    }

    #[test]
    fn empty_maps_preserve_an_existing_graph() -> Result<(), Box<dyn Error>> {
        // Keep real old-extension operations: migration must not change them.
        let hugr = bool_graph()?;
        let before = hugr.mermaid_string();
        let new_hugr = migrate(
            hugr,
            &[&OpMapping::new(HashMap::new())],
            &[&TypeMapping::new(HashMap::new())],
        )?;
        new_hugr.validate()?;
        assert_eq!(new_hugr.mermaid_string(), before);
        Ok(())
    }

    #[test]
    fn valid_mapping_for_unused_operation_preserves_graph() -> Result<(), Box<dyn Error>> {
        let mut hugr = identity_graph()?;
        hugr.use_extensions([hugr::std_extensions::logic::EXTENSION.clone()]);
        let before = hugr.mermaid_string();
        let new_hugr = migrate(
            hugr,
            &[&vec![(
                VersionedElement::new("Not".into(), "logic".into(), Version::new(0, 1, 0)),
                OpReplacementTemplate::TemplateInstance(NodeTemplate::linked_hugr(
                    identity_graph()?
                )),
            )]
            .into()],
            &[&TypeMapping::new(HashMap::new())],
        )?;
        new_hugr.validate()?;
        assert_eq!(new_hugr.mermaid_string(), before);
        Ok(())
    }

    #[test]
    /// Test that signatures are updated correctly
    fn migrate_boolean_dataflow_and_control_flow() -> Result<(), Box<dyn Error>> {
        let fixture =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("extensions/tket/bool-0.2.0.json");
        let registry = load_registry(&[fixture])?;
        for hugr in [build_bool_hugr(&registry)?, build_bool_cfg_hugr(&registry)?] {
            hugr.validate()?;
            for migrated in migrate_from_hugr_and_bytes(hugr)? {
                migrated.validate()?;
                assert!(migrated.nodes().all(|node| {
                    migrated
                        .get_optype(node)
                        .as_extension_op()
                        .is_none_or(|op| op.extension_id().to_string() != "tket.bool")
                }));
            }
        }
        Ok(())
    }

    #[test]
    fn migrate_quantum_operations_to_corresponding_new_operations() -> Result<(), Box<dyn Error>> {
        let crate_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let registry = load_registry(&[
            crate_dir.join("extensions/tket/rotation-0.2.0.json"),
            crate_dir.join("extensions/tket/bool-0.2.0.json"),
            crate_dir.join("extensions/tket/quantum-0.2.1.json"),
        ])?;
        let quantum = registry
            .get_exact("tket.quantum", &Version::new(0, 2, 1))
            .unwrap();

        for (name, _) in quantum.operations() {
            // MeasureFree has a compound replacement tested separately;
            // symbolic_angle is not migrated by the default mappings.
            if matches!(name.as_str(), "MeasureFree" | "symbolic_angle") {
                continue;
            }
            let old_op = quantum.instantiate_extension_op(name, [])?;
            let signature = old_op.signature().into_owned();
            let mut builder = DFGBuilder::new(signature.clone())?;
            let inputs = builder.input_wires();
            let outputs = builder.add_dataflow_op(old_op, inputs)?.outputs();
            let hugr = builder.finish_hugr_with_outputs(outputs)?;
            hugr.validate()?;

            for migrated in migrate_from_hugr_and_bytes(hugr)? {
                migrated.validate()?;
                let operations = migrated
                    .nodes()
                    .filter_map(|node| migrated.get_optype(node).as_extension_op())
                    .collect::<Vec<_>>();
                let expected =
                    tket::extension::TKET_EXTENSION.instantiate_extension_op(name, [])?;
                assert_eq!(
                    operations,
                    vec![&expected],
                    "Incorrect replacement for {name}"
                );
                assert_eq!(
                    operations[0].extension_version(),
                    Version::new(0, 3, 0),
                    "Extension version was not updated for {name}",
                );
                assert_eq!(
                    migrated
                        .entrypoint_optype()
                        .dataflow_signature()
                        .unwrap()
                        .as_ref(),
                    &signature,
                    "Migration changed the signature of {name}",
                );
            }
        }
        Ok(())
    }

    #[test]
    fn migrate_quantum_measurement_to_new_extension_version() -> Result<(), Box<dyn Error>> {
        let crate_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let registry = load_registry(&[
            crate_dir.join("extensions/tket/rotation-0.2.0.json"),
            crate_dir.join("extensions/tket/bool-0.2.0.json"),
            crate_dir.join("extensions/tket/quantum-0.2.1.json"),
        ])?;
        let hugr = build_old_hugr(&registry)?;
        hugr.validate()?;
        for migrated in migrate_from_hugr_and_bytes(hugr)? {
            migrated.validate()?;
            let operations = migrated
                .nodes()
                .filter_map(|node| migrated.get_optype(node).as_extension_op())
                .collect::<Vec<_>>();
            let measurement = operations
                .iter()
                .find(|op| op.unqualified_id() == "MeasureFree")
                .expect("Measurement must remain in the migrated graph");
            assert_eq!(measurement.extension_version(), Version::new(0, 3, 0));
            assert!(operations.iter().any(|op| {
                op.extension_id().to_string() == "tket.measurement" && op.unqualified_id() == "Read"
            }));
            assert!(
                operations
                    .iter()
                    .all(|op| op.extension_id().to_string() != "tket.bool")
            );
        }
        Ok(())
    }
}
