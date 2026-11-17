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
    migrate(
        hugr,
        &[
            &*crate::default_maps::MEAS_OP_MAPPING,
            &*crate::default_maps::BOOL_OP_MAPPING,
        ],
        &[&*crate::default_maps::BOOL_TYPE_MAPPING],
    )
}

#[cfg(test)]
pub(crate) mod test_helpers {
    use crate::update_maps::VersionedElement;
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

    pub(crate) fn migrate_hugr(hugr: Hugr) -> Result<Hugr, Box<dyn Error>> {
        Ok(super::migrate_hugr_default(hugr)?)
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

    pub(crate) fn boolean_registry() -> Result<hugr::extension::ExtensionRegistry, Box<dyn Error>> {
        let fixture =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("extensions/tket/bool-0.2.0.json");
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
    use super::{migrate, test_helpers::*};
    use crate::error::{MigrationError, VersionedElementError};
    use crate::update_maps::{OpMapping, OpReplacementTemplate, TypeMapping, VersionedElement};
    use hugr::{
        Extension, HugrView,
        extension::{TypeDefBound, Version, prelude::bool_t, simple_op::MakeRegisteredOp},
        hugr::hugrmut::HugrMut,
        ops::OpTrait,
        std_extensions::logic::LogicOp,
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

    #[derive(Clone, Copy, Debug)]
    enum MapConstruction {
        Vector,
        HashMap,
        Insert,
    }

    fn negation_maps(
        construction: MapConstruction,
        replacement: NodeTemplate,
    ) -> (OpMapping, TypeMapping) {
        let op = (
            old_bool("not"),
            OpReplacementTemplate::TemplateInstance(replacement),
        );
        let ty = (old_bool("bool"), bool_t());
        match construction {
            MapConstruction::Vector => (vec![op].into(), vec![ty].into()),
            MapConstruction::HashMap => (
                OpMapping::new(HashMap::from([op])),
                TypeMapping::new(HashMap::from([ty])),
            ),
            MapConstruction::Insert => {
                let mut ops = OpMapping::default();
                let mut types = TypeMapping::new(HashMap::new());
                ops.insert(op.0, op.1);
                types.insert(ty.0, ty.1);
                (ops, types)
            }
        }
    }

    #[test]
    fn map_constructors_support_all_node_template_forms() -> Result<(), Box<dyn Error>> {
        for construction in [
            MapConstruction::Vector,
            MapConstruction::HashMap,
            MapConstruction::Insert,
        ] {
            let templates = [
                NodeTemplate::SingleOp(LogicOp::Not.to_extension_op()?.into()),
                NodeTemplate::CompoundOp(Box::new(builtin_negation_graph()?)),
                NodeTemplate::linked_hugr(builtin_negation_graph()?),
            ];
            for template in templates {
                let description = format!("{construction:?}, {template:?}");
                let (ops_map, types_map) = negation_maps(construction, template);
                let mut hugr = old_boolean_graph(true)?;
                hugr.use_extensions([hugr::std_extensions::logic::EXTENSION.clone()]);
                let migrated = migrate(hugr, &[&ops_map], &[&types_map])?;
                migrated.validate()?;
                assert_eq!(
                    migrated
                        .entrypoint_optype()
                        .dataflow_signature()
                        .unwrap()
                        .as_ref(),
                    &Signature::new_endo([bool_t()]),
                    "{description}"
                );
                let operations = migrated
                    .nodes()
                    .filter_map(|node| migrated.get_optype(node).as_extension_op())
                    .collect::<Vec<_>>();
                assert_eq!(operations.len(), 1, "{description}");
                assert_eq!(
                    operations[0].qualified_id().to_string(),
                    "logic.Not",
                    "{description}"
                );
            }
        }
        Ok(())
    }

    #[test]
    fn insert_and_vector_construction_replace_duplicate_entries() -> Result<(), Box<dyn Error>> {
        for construction in [MapConstruction::Vector, MapConstruction::Insert] {
            let bad_op = OpReplacementTemplate::Empty;
            let good_op = OpReplacementTemplate::TemplateInstance(NodeTemplate::SingleOp(
                LogicOp::Not.to_extension_op()?.into(),
            ));
            let bad_type = Type::UNIT;
            let good_type = bool_t();
            let (ops, types) = match construction {
                MapConstruction::Vector => (
                    vec![(old_bool("not"), bad_op), (old_bool("not"), good_op)].into(),
                    vec![(old_bool("bool"), bad_type), (old_bool("bool"), good_type)].into(),
                ),
                MapConstruction::Insert => {
                    let mut ops = OpMapping::new(HashMap::from([(old_bool("not"), bad_op)]));
                    let mut types = TypeMapping::new(HashMap::from([(old_bool("bool"), bad_type)]));
                    ops.insert(old_bool("not"), good_op);
                    types.insert(old_bool("bool"), good_type);
                    (ops, types)
                }
                MapConstruction::HashMap => unreachable!(),
            };
            assert_eq!(ops.iter().count(), 1);
            assert_eq!(types.iter().count(), 1);
            let mut hugr = old_boolean_graph(true)?;
            hugr.use_extensions([hugr::std_extensions::logic::EXTENSION.clone()]);
            let migrated = migrate(hugr, &[&ops], &[&types])?;
            migrated.validate()?;
            assert!(migrated.nodes().any(|node| {
                migrated
                    .get_optype(node)
                    .as_extension_op()
                    .is_some_and(|op| op.qualified_id() == "logic.Not")
            }));
        }
        Ok(())
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
    fn replace_operation_with_empty_type_map_and_no_new_extensions() -> Result<(), Box<dyn Error>> {
        let hugr = builtin_negation_graph()?;
        let migrated = migrate(
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
    fn concrete_type_replacement_uses_registered_extension() -> Result<(), Box<dyn Error>> {
        let target =
            VersionedElement::new("bool".into(), "tket.bool".into(), Version::new(0, 3, 0));
        let mut hugr = old_boolean_graph(false)?;
        let extension = new_boolean_extension()?;
        let expected: Type = extension.get_type("bool").unwrap().instantiate([])?.into();
        hugr.use_extensions([extension]);
        let migrated = migrate(
            hugr,
            &[&OpMapping::default()],
            &[&vec![(old_bool("bool"), expected.clone())].into()],
        )?;
        migrated.validate()?;
        assert_eq!(
            migrated
                .entrypoint_optype()
                .dataflow_signature()
                .unwrap()
                .as_ref(),
            &Signature::new_endo([expected]),
        );
        target.get_type(&migrated)?;
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
    fn absent_source_extensions_are_errors() -> Result<(), Box<dyn Error>> {
        let op_mapping = vec![(missing("source_op"), OpReplacementTemplate::Empty)].into();
        let type_mapping = vec![(missing("source_type"), bool_t())].into();
        assert!(matches!(
            migrate(identity_graph()?, &[&op_mapping], &[]),
            Err(MigrationError::SourceElement(VersionedElementError::MissingExtension(element)))
                if element == missing("source_op")
        ));
        assert!(matches!(
            migrate(identity_graph()?, &[], &[&type_mapping]),
            Err(MigrationError::SourceElement(VersionedElementError::MissingExtension(element)))
                if element == missing("source_type")
        ));
        Ok(())
    }

    #[test]
    fn missing_source_definition_is_identified() -> Result<(), Box<dyn Error>> {
        let hugr = bool_graph()?;
        assert!(matches!(
            migrate(
                hugr,
                &[&vec![(old_bool("nonexistent"), OpReplacementTemplate::Empty)].into()],
                &[&vec![].into()],
            ),
            Err(MigrationError::SourceElement(VersionedElementError::MissingOperation(element)))
                if element == old_bool("nonexistent")
        ));
        Ok(())
    }

    #[test]
    fn unsupported_replacement_container_preserves_pass_error() -> Result<(), Box<dyn Error>> {
        let hugr = old_boolean_graph(true)?;
        let error = migrate(
            hugr,
            &[&vec![(
                old_bool("not"),
                // A function is not a valid CompoundOp replacement container.
                OpReplacementTemplate::TemplateInstance(NodeTemplate::CompoundOp(Box::new(
                    build_bool_cfg_hugr(&boolean_registry()?)?,
                ))),
            )]
            .into()],
            &[&vec![(old_bool("bool"), bool_t())].into()],
        )
        .unwrap_err();
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
        let fixture =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("extensions/tket/bool-0.2.0.json");
        let registry = load_registry(&[fixture])?;
        for hugr in [build_bool_hugr(&registry)?, build_bool_cfg_hugr(&registry)?] {
            hugr.validate()?;
            let migrated = migrate_hugr(hugr)?;
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
            crate_dir.join("extensions/tket/rotation-0.2.0.json"),
            crate_dir.join("extensions/tket/bool-0.2.0.json"),
            crate_dir.join("extensions/tket/quantum-0.2.1.json"),
        ])?;
        let hugr = build_old_hugr(&registry)?;
        hugr.validate()?;
        let migrated = migrate_hugr(hugr)?;
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
