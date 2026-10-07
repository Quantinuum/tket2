use std::{collections::HashMap, error::Error, path::PathBuf, sync::Arc};

use hugr::{
    Extension, Hugr, HugrView,
    builder::{DFGBuilder, Dataflow, DataflowHugr},
    extension::{TypeDefBound, Version, prelude::bool_t, simple_op::MakeRegisteredOp},
    hugr::hugrmut::HugrMut,
    ops::OpTrait,
    std_extensions::logic::LogicOp,
    types::{Signature, Type},
};
use tket::passes::{ReplaceTypes, replace_types::NodeTemplate};

use crate::{
    build_bool_cfg_hugr, build_bool_hugr, build_old_hugr,
    hugr_migration::ExtensionUpdater,
    load_registry, migrate_hugr,
    update_maps::{
        OpMapping, OpReplacementTemplate, TypeMapping, TypeReplacementTemplate, VersionedElement,
    },
};

fn old_bool(id: &str) -> VersionedElement {
    VersionedElement::new(id.into(), "tket.bool".into(), Version::new(0, 2, 0))
}

fn missing(id: &str) -> VersionedElement {
    VersionedElement::new(id.into(), "test.missing".into(), Version::new(1, 0, 0))
}

fn bool_graph() -> Result<Hugr, Box<dyn Error>> {
    let fixture = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("data/bool-0.2.0.json");
    let registry = load_registry(&[fixture])?;
    build_bool_hugr(&registry)
}

fn identity_graph() -> Result<Hugr, Box<dyn Error>> {
    let builder = DFGBuilder::new(Signature::new_endo([bool_t()]))?;
    let inputs = builder.input_wires();
    Ok(builder.finish_hugr_with_outputs(inputs)?)
}

fn boolean_registry() -> Result<hugr::extension::ExtensionRegistry, Box<dyn Error>> {
    let fixture = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("data/bool-0.2.0.json");
    load_registry(&[fixture])
}

fn old_boolean_graph(with_negation: bool) -> Result<Hugr, Box<dyn Error>> {
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

fn builtin_negation_graph() -> Result<Hugr, Box<dyn Error>> {
    let mut builder = DFGBuilder::new(Signature::new_endo([bool_t()]))?;
    let [input] = builder.input_wires_arr();
    let output = builder.add_dataflow_op(LogicOp::Not, [input])?.out_wire(0);
    Ok(builder.finish_hugr_with_outputs([output])?)
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
    let ty = (old_bool("bool"), TypeReplacementTemplate::Type(bool_t()));
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
            let mut updater =
                ExtensionUpdater::new(old_boolean_graph(true)?, ops_map, types_map, vec![]);
            updater.migrate()?;
            let migrated = updater.get_hugr();
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
        let bad_op = OpReplacementTemplate::VersionedElements(vec![missing("target_op")]);
        let good_op = OpReplacementTemplate::TemplateInstance(NodeTemplate::SingleOp(
            LogicOp::Not.to_extension_op()?.into(),
        ));
        let bad_type = TypeReplacementTemplate::VersionedElement(missing("target_type"));
        let good_type = TypeReplacementTemplate::Type(bool_t());
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
        let mut updater = ExtensionUpdater::new(old_boolean_graph(true)?, ops, types, vec![]);
        updater.migrate()?;
        updater.get_hugr().validate()?;
        assert!(updater.get_hugr().nodes().any(|node| {
            updater
                .get_hugr()
                .get_optype(node)
                .as_extension_op()
                .is_some_and(|op| op.qualified_id().to_string() == "logic.Not")
        }));
    }
    Ok(())
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
            OpReplacementTemplate::TemplateInstance(NodeTemplate::linked_hugr(identity_graph()?)),
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

fn new_boolean_extension() -> Result<Arc<Extension>, Box<dyn Error>> {
    Ok(Extension::try_new_arc(
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
    )?)
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
            OpReplacementTemplate::TemplateInstance(NodeTemplate::linked_hugr(identity_graph()?)),
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
fn instantiate_operation_and_type_from_registered_extension() -> Result<(), Box<dyn Error>> {
    let hugr = bool_graph()?;
    let operation = old_bool("not").get_instantiated_op(&hugr)?.unwrap();
    assert_eq!(operation.unqualified_id(), "not");
    assert_eq!(operation.extension_version(), Version::new(0, 2, 0));
    let ty = old_bool("bool").get_type(&hugr)?.unwrap();
    assert_eq!(ty.name(), "bool");
    Ok(())
}

#[test]
fn missing_extension_or_version_returns_none() -> Result<(), Box<dyn Error>> {
    let hugr = bool_graph()?;
    assert!(missing("not").get_instantiated_op(&hugr)?.is_none());
    assert!(missing("bool").get_type(&hugr)?.is_none());

    // The extension exists, but the requested version does not.
    let mut operation = old_bool("not");
    operation.version = Version::new(99, 0, 0);
    assert!(operation.get_instantiated_op(&hugr)?.is_none());
    let mut ty = old_bool("bool");
    ty.version = Version::new(99, 0, 0);
    assert!(ty.get_type(&hugr)?.is_none());
    Ok(())
}

#[test]
fn missing_definition_in_existing_extension_is_an_error() -> Result<(), Box<dyn Error>> {
    let hugr = bool_graph()?;
    assert!(old_bool("nonexistent").get_instantiated_op(&hugr).is_err());
    assert!(old_bool("nonexistent").get_type(&hugr).is_err());
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
    let error = updater.migrate().unwrap_err().to_string();
    assert!(error.contains("Replacement operation target_op"), "{error}");
    assert!(error.contains("test.missing@1.0.0"), "{error}");
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
    let error = updater.migrate().unwrap_err().to_string();
    assert!(error.contains("Replacement type target_type"), "{error}");
    assert!(error.contains("test.missing@1.0.0"), "{error}");
    Ok(())
}

#[test]
fn empty_replacements_use_migrated_boolean_signatures() -> Result<(), Box<dyn Error>> {
    let hugr = bool_graph()?;
    let mut replacer = ReplaceTypes::default();
    replacer.set_replace_type(old_bool("bool").get_type(&hugr)?.unwrap(), bool_t());
    for name in ["make_opaque", "read"] {
        let operation = old_bool(name).get_instantiated_op(&hugr)?.unwrap();
        let template = OpReplacementTemplate::Empty.get_op_replace(&operation, &hugr, &replacer)?;
        let NodeTemplate::LinkedHugr(replacement, _) = template else {
            panic!("Expected a graph replacement for {name}");
        };
        replacement.validate()?;
        assert_eq!(
            replacement
                .entrypoint_optype()
                .dataflow_signature()
                .unwrap()
                .as_ref(),
            &Signature::new_endo([bool_t()]),
            "{name} must become a boolean passthrough"
        );
    }
    Ok(())
}

#[test]
fn empty_replacement_cannot_connect_unmigrated_types() -> Result<(), Box<dyn Error>> {
    let hugr = bool_graph()?;
    let operation = old_bool("make_opaque").get_instantiated_op(&hugr)?.unwrap();
    assert!(
        OpReplacementTemplate::Empty
            .get_op_replace(&operation, &hugr, &ReplaceTypes::default())
            .is_err()
    );
    Ok(())
}

#[test]
fn migrate_boolean_dataflow_and_control_flow() -> Result<(), Box<dyn Error>> {
    let fixture = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("data/bool-0.2.0.json");
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
        crate_dir.join("data/bool-0.2.0.json"),
        crate_dir.join("data/quantum-0.2.1.json"),
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
