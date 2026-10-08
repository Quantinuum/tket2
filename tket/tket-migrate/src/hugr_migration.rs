use crate::{
    error::MigrationError,
    update_maps::{OpMapping, TypeMapping},
};
use hugr::HugrView;
use hugr::hugr::hugrmut::HugrMut;
use hugr::{
    Extension, Hugr,
    extension::{ExtensionRegistry, resolution::WeakExtensionRegistry},
    std_extensions::STD_REG,
};
use tket::passes::{ComposablePass, ReplaceTypes};

/// Applies operation and type migrations to a HUGR.
pub struct ExtensionUpdater {
    hugr: Hugr,
    op_mapping: OpMapping,
    type_mapping: TypeMapping,
    new_extensions: Vec<Extension>,
}

impl ExtensionUpdater {
    /// Creates an updater with the mappings and new extensions to register.
    pub fn new(
        hugr: Hugr,
        op_mapping: OpMapping,
        type_mapping: TypeMapping,
        new_extensions: Vec<Extension>,
    ) -> Self {
        Self {
            hugr,
            op_mapping,
            type_mapping,
            new_extensions,
        }
    }

    /// Returns the HUGR, including any changes made by migration.
    pub fn get_hugr(&self) -> &Hugr {
        &self.hugr
    }

    /// Registers new extensions and applies the configured replacements.
    ///
    /// Mappings whose source extension version is absent are skipped.
    /// Errors identify failed source lookups, replacement construction, pass
    /// execution, and extension registration.
    /// Panics if we fail to resolve extension definitions after the migration.
    pub fn migrate(&mut self) -> Result<(), MigrationError> {
        self.add_new_extension()?;
        let mut replacer = ReplaceTypes::default();

        for (old_type, replacement) in self.type_mapping.iter() {
            let Some(old_type) = old_type.get_type(&self.hugr)? else {
                continue;
            };
            replacer.set_replace_type(old_type, replacement.get_type(&self.hugr)?);
        }

        for (old_element, replacement) in self.op_mapping.iter() {
            let Some(old_op) = old_element.get_instantiated_op(&self.hugr)? else {
                continue;
            };
            let template = replacement.get_op_replace(&old_op, &self.hugr, &replacer)?;
            replacer.set_replace_op(&old_op, template);
        }

        replacer.run(&mut self.hugr)?;

        // remove unused extensions
        let registry = self.hugr.extensions().clone();
        self.hugr.resolve_extension_defs(&registry).unwrap();
        Ok(())
    }

    fn add_new_extension(&mut self) -> Result<(), MigrationError> {
        let mut extensions = STD_REG.to_owned();
        extensions.extend(self.hugr.extensions().clone());
        let new_ext_registry = ExtensionRegistry::new_with_extension_resolution(
            std::mem::take(&mut self.new_extensions),
            &WeakExtensionRegistry::from(&extensions),
        )
        .map_err(MigrationError::RegisterExtensions)?;
        extensions.extend(new_ext_registry);
        self.hugr.use_extensions(extensions);
        Ok(())
    }
}

#[cfg(test)]
pub(crate) mod test_helpers {
    use super::ExtensionUpdater;
    use crate::{
        default_maps::{get_measurement_migratation_op_map, get_measurement_migratation_type_map},
        update_maps::VersionedElement,
    };
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
    use std::{
        error::Error,
        fs::File,
        io::BufReader,
        path::{Path, PathBuf},
    };

    const QUANTUM_EXTENSION: &str = "tket.quantum";
    const BOOL_EXTENSION: &str = "tket.bool";

    pub(crate) fn load_extension(path: &Path) -> Result<Extension, Box<dyn Error>> {
        Ok(serde_json::from_reader(BufReader::new(File::open(path)?))?)
    }

    pub(crate) fn load_extensions(paths: &[PathBuf]) -> Result<Vec<Extension>, Box<dyn Error>> {
        paths
            .iter()
            .map(|path| load_extension(path))
            .collect::<Result<Vec<_>, _>>()
    }

    pub(crate) fn load_registry(paths: &[PathBuf]) -> Result<ExtensionRegistry, Box<dyn Error>> {
        let mut registry = STD_REG.to_owned();
        let extensions = load_extensions(paths)?;
        let custom_extensions = ExtensionRegistry::new_with_extension_resolution(
            extensions,
            &WeakExtensionRegistry::from(&registry),
        )?;
        registry.extend(custom_extensions.clone());
        Ok(registry)
    }

    pub(crate) fn load_new_extensions() -> Result<Vec<Extension>, Box<dyn Error>> {
        let crate_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));

        let extension_dir = crate_dir.join("../tket-exts/src/tket_exts/data/tket");

        let new_extension_paths = vec![
            extension_dir.join("rotation.json"),
            extension_dir.join("measurement.json"),
            extension_dir.join("quantum.json"),
        ];
        load_extensions(&new_extension_paths)
    }

    pub(crate) fn migrate_hugr(hugr: Hugr) -> Result<ExtensionUpdater, Box<dyn Error>> {
        let op_mapping = get_measurement_migratation_op_map();
        let mut updater = ExtensionUpdater::new(
            hugr,
            op_mapping,
            get_measurement_migratation_type_map(),
            load_new_extensions()?,
        );
        updater.migrate()?;
        Ok(updater)
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

    pub(crate) fn old_bool(id: &str) -> VersionedElement {
        VersionedElement::new(id.into(), "tket.bool".into(), Version::new(0, 2, 0))
    }

    pub(crate) fn missing(id: &str) -> VersionedElement {
        VersionedElement::new(id.into(), "test.missing".into(), Version::new(1, 0, 0))
    }

    pub(crate) fn bool_graph() -> Result<Hugr, Box<dyn Error>> {
        let fixture = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../test_files/old_extensions/bool-0.2.0.json");
        let registry = load_registry(&[fixture])?;
        build_bool_hugr(&registry)
    }

    pub(crate) fn identity_graph() -> Result<Hugr, Box<dyn Error>> {
        let builder = DFGBuilder::new(Signature::new_endo([bool_t()]))?;
        let inputs = builder.input_wires();
        Ok(builder.finish_hugr_with_outputs(inputs)?)
    }

    pub(crate) fn boolean_registry() -> Result<hugr::extension::ExtensionRegistry, Box<dyn Error>> {
        let fixture = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../test_files/old_extensions/bool-0.2.0.json");
        load_registry(&[fixture])
    }

    pub(crate) fn old_boolean_graph(with_negation: bool) -> Result<Hugr, Box<dyn Error>> {
        let registry = boolean_registry()?;
        let extension = registry
            .get_exact("tket.bool", &Version::new(0, 2, 0))
            .unwrap();
        let boolean: Type = extension.get_type("bool").unwrap().instantiate([])?.into();
        let mut builder = DFGBuilder::new(Signature::new_endo([boolean]))?;
        let [input] = builder.input_wires_arr();
        let output = if with_negation {
            builder
                .add_dataflow_op(extension.instantiate_extension_op("not", [])?, [input])?
                .out_wire(0)
        } else {
            input
        };
        Ok(builder.finish_hugr_with_outputs([output])?)
    }

    pub(crate) fn builtin_negation_graph() -> Result<Hugr, Box<dyn Error>> {
        let mut builder = DFGBuilder::new(Signature::new_endo([bool_t()]))?;
        let [input] = builder.input_wires_arr();
        let output = builder.add_dataflow_op(LogicOp::Not, [input])?.out_wire(0);
        Ok(builder.finish_hugr_with_outputs([output])?)
    }
}

#[cfg(test)]
mod tests {
    use super::{ExtensionUpdater, test_helpers::*};
    use crate::error::{MigrationError, ReplacementError, VersionedElementError};
    use crate::update_maps::{
        OpMapping, OpReplacementTemplate, TypeMapping, TypeReplacementTemplate, VersionedElement,
    };
    use hugr::{
        Extension, HugrView,
        extension::{TypeDefBound, Version, prelude::bool_t},
        hugr::hugrmut::HugrMut,
        ops::OpTrait,
        types::{Signature, Type},
    };
    use std::{collections::HashMap, error::Error, path::PathBuf, sync::Arc};
    use tket::passes::replace_types::NodeTemplate;

    fn new_boolean_extension() -> Result<Arc<Extension>, Box<dyn Error>> {
        Extension::try_new_arc(
            "tket.bool".try_into()?,
            Version::new(0, 3, 0),
            |extension, extension_ref| {
                extension.add_type(
                    "bool".into(),
                    vec![],
                    "Migrated boolean".into(),
                    TypeDefBound::copyable(),
                    extension_ref,
                )?;
                Ok::<_, Box<dyn Error>>(())
            },
        )
    }

    #[test]
    fn empty_maps_preserve_an_existing_graph() -> Result<(), Box<dyn Error>> {
        // Keep real old-extension operations: the updater must not change them.
        let hugr = bool_graph()?;
        let before = hugr.mermaid_string();
        let mut updater = ExtensionUpdater::new(
            hugr,
            OpMapping::new(HashMap::new()),
            TypeMapping::new(HashMap::new()),
            vec![],
        );
        updater.migrate()?;
        updater.get_hugr().validate()?;
        assert_eq!(updater.get_hugr().mermaid_string(), before);
        Ok(())
    }

    #[test]
    fn replace_operation_with_empty_type_map_and_no_new_extensions() -> Result<(), Box<dyn Error>> {
        let mut updater = ExtensionUpdater::new(
            builtin_negation_graph()?,
            vec![(
                VersionedElement::new("Not".into(), "logic".into(), Version::new(0, 1, 0)),
                OpReplacementTemplate::TemplateInstance(NodeTemplate::linked_hugr(
                    identity_graph()?
                )),
            )]
            .into(),
            TypeMapping::new(HashMap::new()),
            vec![],
        );
        updater.migrate()?;
        let migrated = updater.get_hugr();
        migrated.validate()?;
        assert!(
            migrated
                .nodes()
                .all(|node| migrated.get_optype(node).as_extension_op().is_none())
        );
        assert_eq!(
            migrated
                .entrypoint_optype()
                .dataflow_signature()
                .unwrap()
                .as_ref(),
            &Signature::new_endo([bool_t()])
        );
        Ok(())
    }

    #[test]
    fn versioned_type_replacement_uses_new_or_existing_extensions() -> Result<(), Box<dyn Error>> {
        for register_via_updater in [true, false] {
            let target =
                VersionedElement::new("bool".into(), "tket.bool".into(), Version::new(0, 3, 0));
            let mut hugr = old_boolean_graph(false)?;
            let extension = new_boolean_extension()?;
            let expected: Type = extension.get_type("bool").unwrap().instantiate([])?.into();
            let new_extensions = if register_via_updater {
                // Exercise the same deserialization/resolution path as loaded extensions.
                vec![serde_json::from_str(&serde_json::to_string(
                    extension.as_ref(),
                )?)?]
            } else {
                hugr.use_extensions([extension]);
                vec![]
            };
            let mut updater = ExtensionUpdater::new(
                hugr,
                OpMapping::default(),
                vec![(
                    old_bool("bool"),
                    TypeReplacementTemplate::VersionedElement(target.clone()),
                )]
                .into(),
                new_extensions,
            );
            updater.migrate()?;
            let migrated = updater.get_hugr();
            migrated.validate()?;
            assert_eq!(
                migrated
                    .entrypoint_optype()
                    .dataflow_signature()
                    .unwrap()
                    .as_ref(),
                &Signature::new_endo([expected]),
                "register_via_updater={register_via_updater}"
            );
            assert!(target.get_type(migrated)?.is_some());
        }
        Ok(())
    }

    #[test]
    fn valid_mapping_for_unused_operation_preserves_graph() -> Result<(), Box<dyn Error>> {
        let hugr = identity_graph()?;
        let before = hugr.mermaid_string();
        let mut updater = ExtensionUpdater::new(
            hugr,
            vec![(
                VersionedElement::new("Not".into(), "logic".into(), Version::new(0, 1, 0)),
                OpReplacementTemplate::TemplateInstance(NodeTemplate::linked_hugr(
                    identity_graph()?
                )),
            )]
            .into(),
            TypeMapping::new(HashMap::new()),
            vec![],
        );
        updater.migrate()?;
        updater.get_hugr().validate()?;
        assert_eq!(updater.get_hugr().mermaid_string(), before);
        Ok(())
    }

    #[test]
    fn absent_sources_are_skipped_without_resolving_replacements() -> Result<(), Box<dyn Error>> {
        let hugr = identity_graph()?;
        let before = hugr.mermaid_string();
        let mut updater = ExtensionUpdater::new(
            hugr,
            vec![(
                missing("source_op"),
                OpReplacementTemplate::VersionedElements(vec![missing("target_op")]),
            )]
            .into(),
            vec![(
                missing("source_type"),
                TypeReplacementTemplate::VersionedElement(missing("target_type")),
            )]
            .into(),
            vec![],
        );
        // Neither source extension exists, so both mappings should be ignored.
        // The replacements are also missing: looking them up would fail the migration.
        updater.migrate()?;
        updater.get_hugr().validate()?;
        assert_eq!(updater.get_hugr().mermaid_string(), before);
        Ok(())
    }

    #[test]
    fn missing_required_replacement_operation_is_an_error() -> Result<(), Box<dyn Error>> {
        let mut updater = ExtensionUpdater::new(
            bool_graph()?,
            vec![(
                old_bool("not"),
                OpReplacementTemplate::VersionedElements(vec![missing("target_op")]),
            )]
            .into(),
            vec![].into(),
            vec![],
        );
        let error = updater.migrate().unwrap_err();
        assert!(matches!(
            error,
            MigrationError::Replacement(ReplacementError::MissingOperationExtension(element))
                if element == missing("target_op")
        ));
        Ok(())
    }

    #[test]
    fn missing_required_replacement_type_is_an_error() -> Result<(), Box<dyn Error>> {
        let mut updater = ExtensionUpdater::new(
            bool_graph()?,
            vec![].into(),
            vec![(
                old_bool("bool"),
                TypeReplacementTemplate::VersionedElement(missing("target_type")),
            )]
            .into(),
            vec![],
        );
        let error = updater.migrate().unwrap_err();
        assert!(matches!(
            error,
            MigrationError::Replacement(ReplacementError::MissingTypeExtension(element))
                if element == missing("target_type")
        ));
        Ok(())
    }

    #[test]
    fn missing_source_definition_is_identified() -> Result<(), Box<dyn Error>> {
        let mut updater = ExtensionUpdater::new(
            bool_graph()?,
            vec![(old_bool("nonexistent"), OpReplacementTemplate::Empty)].into(),
            vec![].into(),
            vec![],
        );
        assert!(matches!(
            updater.migrate(),
            Err(MigrationError::SourceElement(VersionedElementError::MissingOperation(element)))
                if element == old_bool("nonexistent")
        ));
        Ok(())
    }

    #[test]
    fn unsupported_replacement_container_preserves_pass_error() -> Result<(), Box<dyn Error>> {
        let mut updater = ExtensionUpdater::new(
            old_boolean_graph(true)?,
            vec![(
                old_bool("not"),
                // A function is not a valid CompoundOp replacement container.
                OpReplacementTemplate::TemplateInstance(NodeTemplate::CompoundOp(Box::new(
                    build_bool_cfg_hugr(&boolean_registry()?)?,
                ))),
            )]
            .into(),
            vec![(old_bool("bool"), TypeReplacementTemplate::Type(bool_t()))].into(),
            vec![],
        );
        let error = updater.migrate().unwrap_err();
        assert!(
            error
                .source()
                .unwrap()
                .is::<tket::passes::replace_types::ReplaceTypesError>()
        );
        assert!(matches!(
            error,
            MigrationError::ReplaceTypes(
                tket::passes::replace_types::ReplaceTypesError::AddTemplateError(..)
            )
        ));
        Ok(())
    }

    #[test]
    fn migrate_boolean_dataflow_and_control_flow() -> Result<(), Box<dyn Error>> {
        let fixture = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../test_files/old_extensions/bool-0.2.0.json");
        let registry = load_registry(&[fixture])?;
        for hugr in [build_bool_hugr(&registry)?, build_bool_cfg_hugr(&registry)?] {
            hugr.validate()?;
            let updater = migrate_hugr(hugr)?;
            let migrated = updater.get_hugr();
            migrated.validate()?;
            assert!(migrated.nodes().all(|node| {
                migrated
                    .get_optype(node)
                    .as_extension_op()
                    .is_none_or(|op| op.extension_id().to_string() != "tket.bool")
            }));
        }
        Ok(())
    }

    #[test]
    fn migrate_quantum_measurement_to_new_extension_version() -> Result<(), Box<dyn Error>> {
        let crate_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let registry = load_registry(&[
            crate_dir.join("../tket-exts/src/tket_exts/data/tket/rotation.json"),
            crate_dir.join("../../test_files/old_extensions/bool-0.2.0.json"),
            crate_dir.join("../../test_files/old_extensions/quantum-0.2.1.json"),
        ])?;
        let hugr = build_old_hugr(&registry)?;
        hugr.validate()?;
        let updater = migrate_hugr(hugr)?;
        let migrated = updater.get_hugr();
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
        Ok(())
    }
}
