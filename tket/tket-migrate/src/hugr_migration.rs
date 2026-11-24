use std::io;

use crate::{
    default_maps::DEFAULT_MIGRATION,
    error::MigrationError,
    extension_loading::LOCAL_REGISTRY,
    update_maps::{ConstMapping, OpMapping, OpReplacementTemplate, TypeMapping, VersionedMapping},
};
use hugr::{Hugr, HugrView};
use tket::passes::{ComposablePass, ReplaceTypes};

/// Ordered groups of operation, type, and constant mappings.
pub(crate) struct Migration {
    op_mappings: Vec<OpMapping>,
    type_mappings: Vec<TypeMapping>,
    const_mappings: Vec<ConstMapping>,
}

impl Migration {
    /// Creates an empty migration.
    pub(crate) fn new() -> Self {
        Self {
            op_mappings: Vec::new(),
            type_mappings: Vec::new(),
            const_mappings: Vec::new(),
        }
    }

    /// Adds operation replacements for one source extension version.
    pub(crate) fn add_op_mapping(&mut self, mapping: OpMapping) {
        self.op_mappings.push(mapping);
    }

    /// Adds type replacements for one source extension version.
    pub(crate) fn add_type_mapping(&mut self, mapping: TypeMapping) {
        self.type_mappings.push(mapping);
    }

    /// Adds constant converters for one source extension version.
    pub(crate) fn add_const_mapping(&mut self, mapping: ConstMapping) {
        self.const_mappings.push(mapping);
    }

    /// Applies the rules in one pass and returns the validated HUGR.
    ///
    /// All type replacements are registered before operation templates are built.
    fn apply(&self, mut hugr: Hugr) -> Result<Hugr, MigrationError> {
        let mut replacer = ReplaceTypes::new_with_op_version_matching(true);

        for (old_type, new_type) in self.type_mappings.iter().flat_map(VersionedMapping::iter) {
            let old_type = old_type.get_type()?;
            replacer.set_replace_type(old_type, new_type.clone());
        }

        for (old_element, replacement_template) in
            self.op_mappings.iter().flat_map(VersionedMapping::iter)
        {
            match replacement_template {
                OpReplacementTemplate::Simple(replacement) => {
                    let old_op = old_element.get_instantiated_op()?;
                    replacer.set_replace_op(&old_op, replacement.get_replace(&old_op, &replacer)?);
                }
                OpReplacementTemplate::Parametrized(par_replacement) => {
                    let replacement = par_replacement.clone();
                    replacer.set_replace_parametrized_op(old_element.get_op()?, move |args, rt| {
                        replacement(args, rt)
                    });
                }
            }
        }

        for (source, converter) in self.const_mappings.iter().flat_map(VersionedMapping::iter) {
            replacer.replace_consts(source.get_type()?, *converter);
        }

        replacer.run(&mut hugr)?;
        hugr.resolve_extension_defs(&LOCAL_REGISTRY).unwrap();
        hugr.validate()?;

        Ok(hugr)
    }
}

/// Upgrades `tket.quantum@0.2.1` operations to `0.3.0` and migrates
/// `tket.bool@0.2.0` operations, types, and constants to their HUGR equivalents.
///
/// Returns the validated HUGR after resolving extension definitions against the bundled registry.
pub fn migrate_hugr_default(hugr: Hugr) -> Result<Hugr, MigrationError> {
    DEFAULT_MIGRATION.apply(hugr)
}

/// Loads a serialized HUGR using the bundled registry and applies [`migrate_hugr_default`].
pub fn migrate_bytes_default(bytes: impl io::BufRead) -> Result<Hugr, MigrationError> {
    let hugr = Hugr::load(bytes, Some(&LOCAL_REGISTRY))?;
    migrate_hugr_default(hugr)
}

#[cfg(test)]
pub(crate) mod test_helpers {
    use crate::default_maps::ConstBool;
    use hugr::{
        Extension, Hugr,
        builder::{
            DFGBuilder, Dataflow, DataflowHugr, DataflowSubContainer, FunctionBuilder, SubContainer,
        },
        extension::{
            ExtensionRegistry, Version, prelude::bool_t, resolution::WeakExtensionRegistry,
        },
        std_extensions::{STD_REG, logic::LogicOp},
        types::{Signature, Type},
    };
    use std::{error::Error, fs::File, io::BufReader, path::PathBuf};

    const QUANTUM_EXTENSION: &str = "tket.quantum";
    const BOOL_EXTENSION: &str = "tket.bool";

    pub(super) fn load_registry(paths: &[PathBuf]) -> Result<ExtensionRegistry, Box<dyn Error>> {
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

    /// Builds a `tket.quantum@0.2.1` graph that allocates a qubit, applies H, and
    /// measures it with `MeasureFree`, then reads and negates the `tket.bool@0.2.0` result.
    pub(super) fn build_old_hugr(registry: &ExtensionRegistry) -> Result<Hugr, Box<dyn Error>> {
        let quantum = registry
            .get(QUANTUM_EXTENSION)
            .ok_or("tket.quantum is missing from the registry")?;
        let bool_extension = registry
            .get(BOOL_EXTENSION)
            .ok_or("tket.bool is missing from the registry")?;
        assert_eq!(
            quantum.version(),
            &Version::new(0, 2, 1),
            "{QUANTUM_EXTENSION}"
        );
        assert_eq!(
            bool_extension.version(),
            &Version::new(0, 2, 0),
            "{BOOL_EXTENSION}"
        );
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

    /// Builds a dataflow graph that converts five HUGR booleans to `tket.bool@0.2.0`,
    /// combines them with legacy logic operations and a loaded `ConstBool(true)`,
    /// and reads the result back as a HUGR boolean.
    pub(super) fn build_legacy_bool_hugr(
        registry: &ExtensionRegistry,
    ) -> Result<Hugr, Box<dyn Error>> {
        let bool_extension = registry
            .get(BOOL_EXTENSION)
            .ok_or("tket.bool is missing from the registry")?;
        assert_eq!(
            bool_extension.version(),
            &Version::new(0, 2, 0),
            "{BOOL_EXTENSION}"
        );
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
            .add_dataflow_op(and.clone(), [value, values[1]])?
            .out_wire(0);
        let value = builder.add_dataflow_op(eq, [value, values[2]])?.out_wire(0);
        let value = builder.add_dataflow_op(or, [value, values[3]])?.out_wire(0);
        let value = builder
            .add_dataflow_op(xor, [value, values[4]])?
            .out_wire(0);
        let truth = builder.add_load_value(ConstBool(true));
        let value = builder.add_dataflow_op(and, [value, truth])?.out_wire(0);
        let output = builder.add_dataflow_op(read, [value])?.out_wire(0);

        Ok(builder.finish_hugr_with_outputs([output])?)
    }

    pub(super) fn build_bool_cfg_hugr(
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

    /// Builds the legacy boolean dataflow graph using the bundled `tket.bool@0.2.0` definition.
    pub(crate) fn bool_graph() -> Result<Hugr, Box<dyn Error>> {
        let fixture =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("extensions/tket/bool-0.2.0.json");
        let registry = load_registry(&[fixture])?;
        build_legacy_bool_hugr(&registry)
    }

    pub(super) fn identity_graph() -> Result<Hugr, Box<dyn Error>> {
        let builder = DFGBuilder::new(Signature::new_endo([bool_t()]))?;
        let inputs = builder.input_wires();
        Ok(builder.finish_hugr_with_outputs(inputs)?)
    }
}

#[cfg(test)]
mod tests {
    use super::{Migration, migrate_bytes_default, migrate_hugr_default, test_helpers::*};
    use crate::update_maps::{OpMapping, OpReplacementTemplate, SimpleTemplate};
    use hugr::{
        Hugr, HugrView,
        builder::{DFGBuilder, Dataflow, DataflowHugr},
        envelope::EnvelopeConfig,
        extension::Version,
        hugr::hugrmut::HugrMut,
        ops::{DataflowOpTrait, OpTrait},
        package::Package,
    };
    use std::{error::Error, path::PathBuf};
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
        let new_hugr = Migration::new().apply(hugr)?;
        new_hugr.validate()?;
        assert_eq!(new_hugr.mermaid_string(), before);
        Ok(())
    }

    #[test]
    /// Test composition of default and added rules not -> Not -> Empty
    fn extended_default_migration_can_be_reused() -> Result<(), Box<dyn Error>> {
        let mut migration = Migration::default();
        // The default rules turn the legacy `not` into `logic.Not`.
        let mut ops = OpMapping::new("logic".into(), Version::new(0, 1, 0));
        ops.insert("Not", OpReplacementTemplate::Simple(SimpleTemplate::Empty));
        migration.add_op_mapping(ops);

        let graph = bool_graph()?;
        for hugr in [graph.clone(), graph] {
            let migrated = migration.apply(hugr)?;
            let names: Vec<_> = migrated
                .nodes()
                .filter_map(|n| migrated.get_optype(n).as_extension_op())
                .map(|op| op.unqualified_id().to_string())
                .collect();
            assert!(names.iter().any(|name| name == "And"));
            assert!(names.iter().all(|name| name != "Not"));
        }
        Ok(())
    }

    #[test]
    fn valid_mapping_for_unused_operation_preserves_graph() -> Result<(), Box<dyn Error>> {
        let mut hugr = identity_graph()?;
        hugr.use_extensions([hugr::std_extensions::logic::EXTENSION.clone()]);
        let before = hugr.mermaid_string();
        let mut ops = OpMapping::new("logic".into(), Version::new(0, 1, 0));
        ops.insert(
            "Not",
            OpReplacementTemplate::Simple(SimpleTemplate::TemplateInstance(
                NodeTemplate::linked_hugr(identity_graph()?),
            )),
        );
        let mut migration = Migration::new();
        migration.add_op_mapping(ops);
        let new_hugr = migration.apply(hugr)?;
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
        for hugr in [
            build_legacy_bool_hugr(&registry)?,
            build_bool_cfg_hugr(&registry)?,
        ] {
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

    /// Checks that migration preserves the wiring and angle expression of an `H`–`Rz` circuit
    /// for both in-memory and serialized HUGRs.
    #[test]
    fn migrate_quantum_circuit_with_symbolic_rotation() -> Result<(), Box<dyn Error>> {
        use hugr::{extension::prelude::qb_t, types::Signature};

        let quantum =
            crate::extension_loading::get_extension("tket.quantum", &Version::new(0, 2, 1))?;
        let expression = "theta + pi/4".to_owned();
        let h = quantum.instantiate_extension_op("H", [])?;
        let angle =
            quantum.instantiate_extension_op("symbolic_angle", [expression.clone().into()])?;
        let rz = quantum.instantiate_extension_op("Rz", [])?;

        let mut builder = DFGBuilder::new(Signature::new_endo([qb_t()]))?;
        let [qubit] = builder.input_wires_arr();
        let qubit = builder.add_dataflow_op(h, [qubit])?.out_wire(0);
        let angle = builder.add_dataflow_op(angle, [])?.out_wire(0);
        let rotated = builder.add_dataflow_op(rz, [qubit, angle])?.out_wire(0);
        let hugr = builder.finish_hugr_with_outputs([rotated])?;
        hugr.validate()?;

        for migrated in migrate_from_hugr_and_bytes(hugr)? {
            let [input, output] = migrated.get_io(migrated.entrypoint()).unwrap();
            let (rz, _) = migrated.single_linked_output(output, 0).unwrap();
            let (h, _) = migrated.single_linked_output(rz, 0).unwrap();
            let (angle, _) = migrated.single_linked_output(rz, 1).unwrap();
            assert_eq!(migrated.single_linked_output(h, 0), Some((input, 0.into())));

            for (node, name) in [(h, "H"), (angle, "symbolic_angle"), (rz, "Rz")] {
                let op = migrated.get_optype(node).as_extension_op().unwrap();
                assert_eq!(op.extension_id(), quantum.name());
                assert_eq!(op.extension_version(), Version::new(0, 3, 0));
                assert_eq!(op.unqualified_id(), name);
            }
            let angle = migrated.get_optype(angle).as_extension_op().unwrap();
            assert_eq!(angle.args(), &[expression.clone().into()]);
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
            // MeasureFree has a compound replacement tested separately.
            if name.as_str() == "MeasureFree" {
                continue;
            }
            let args = match name.as_str() {
                "symbolic_angle" => vec!["theta + pi/4".to_owned().into()],
                _ => vec![],
            };
            let old_op = quantum.instantiate_extension_op(name, args.clone())?;
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
                    tket::extension::TKET_EXTENSION.instantiate_extension_op(name, args.clone())?;
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
