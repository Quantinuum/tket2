//! Generate semantically equivalent HUGRs with two quantum extension versions.

mod default_maps;
mod hugr_migration;
#[cfg(test)]
mod tests;
mod update_maps;
use default_maps::{get_measurement_migratation_op_map, get_measurement_migratation_type_map};
use hugr::{
    Extension, Hugr, HugrView,
    builder::{
        DFGBuilder, Dataflow, DataflowHugr, DataflowSubContainer, FunctionBuilder, SubContainer,
    },
    extension::{ExtensionRegistry, prelude::bool_t, resolution::WeakExtensionRegistry},
    std_extensions::{STD_REG, logic::LogicOp},
    types::{Signature, Type},
};
use hugr_migration::ExtensionUpdater;
use std::{
    error::Error,
    fs::{self, File},
    io::BufReader,
    path::{Path, PathBuf},
};

const QUANTUM_EXTENSION: &str = "tket.quantum";
const BOOL_EXTENSION: &str = "tket.bool";

fn migrate_hugr(hugr: Hugr) -> Result<ExtensionUpdater, Box<dyn Error>> {
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

#[allow(dead_code)]
fn update_measure_op() -> Result<(), Box<dyn Error>> {
    // here i want to create an older hugr, add the new extension, look for the measure op, change the measure op to new measurement op
    Ok(())
}

fn load_extension(path: &Path) -> Result<Extension, Box<dyn Error>> {
    Ok(serde_json::from_reader(BufReader::new(File::open(path)?))?)
}

fn load_extensions(paths: &[PathBuf]) -> Result<Vec<Extension>, Box<dyn Error>> {
    Ok(paths
        .iter()
        .map(|path| load_extension(path))
        .collect::<Result<Vec<_>, _>>()?)
}

fn load_registry(paths: &[PathBuf]) -> Result<ExtensionRegistry, Box<dyn Error>> {
    let mut registry = STD_REG.to_owned();
    let extensions = load_extensions(paths)?;
    let custom_extensions = ExtensionRegistry::new_with_extension_resolution(
        extensions,
        &WeakExtensionRegistry::from(&registry),
    )?;
    registry.extend(custom_extensions.clone());
    Ok(registry)
}

fn load_new_extensions() -> Result<Vec<Extension>, Box<dyn Error>> {
    let crate_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));

    let extension_dir = crate_dir.join("../tket-exts/src/tket_exts/data/tket");

    let new_extension_paths = vec![
        extension_dir.join("rotation.json"),
        extension_dir.join("measurement.json"),
        extension_dir.join("quantum.json"),
    ];
    load_extensions(&new_extension_paths)
}

fn generate(
    extension_paths: &[PathBuf],
    output_path: &Path,
    build_hugr: impl FnOnce(&ExtensionRegistry) -> Result<Hugr, Box<dyn Error>>,
    save_file: bool,
) -> Result<Hugr, Box<dyn Error>> {
    let registry = load_registry(extension_paths)?;
    let hugr = build_hugr(&registry)?;

    if save_file {
        std::fs::write(output_path.with_extension("mmd"), hugr.mermaid_string())?;
    }
    Ok(hugr)
}

fn build_old_hugr(registry: &ExtensionRegistry) -> Result<Hugr, Box<dyn Error>> {
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

// NICOLA: todo:
// - use https://docs.rs/tket/latest/tket/passes/replace_types/struct.ReplaceTypes.html
// -  update map new node should be the replacement operation already made (same with types)
fn main1() -> Result<(), Box<dyn Error>> {
    let crate_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let fixture_dir = crate_dir.join("data");
    let extension_dir = crate_dir.join("../tket-exts/src/tket_exts/data/tket");
    let rotation = extension_dir.join("rotation.json");

    let old_output = crate_dir.join("quantum-0.2.1.hugr");
    let _old_hugr = generate(
        &[
            rotation.clone(),
            fixture_dir.join("bool-0.2.0.json"),
            fixture_dir.join("quantum-0.2.1.json"),
        ],
        &old_output,
        build_old_hugr,
        true,
    )?;

    let updater = migrate_hugr(_old_hugr)?;

    std::fs::write("updated1.mmd", updater.get_hugr().mermaid_string())?;
    println!("+++++++++++++++++");

    updater.get_hugr().validate()?;
    Ok(())
}

fn build_bool_hugr(registry: &ExtensionRegistry) -> Result<Hugr, Box<dyn Error>> {
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

fn main2() -> Result<(), Box<dyn Error>> {
    let crate_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let bool_extension = crate_dir.join("data/bool-0.2.0.json");
    let output = crate_dir.join("bool-0.2.0.hugr");
    let old_bool_hugr = generate(&[bool_extension], &output, build_bool_hugr, true)?;
    old_bool_hugr.validate()?;

    let updater = migrate_hugr(old_bool_hugr)?;

    std::fs::write("updated2.mmd", updater.get_hugr().mermaid_string())?;
    println!("+++++++++++++++++");

    // NICOLA todo: remove not used extensions

    updater.get_hugr().validate()?;

    Ok(())
}

fn build_bool_cfg_hugr(registry: &ExtensionRegistry) -> Result<Hugr, Box<dyn Error>> {
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
        let mut cfg = function.cfg_builder([(boolean.clone(), value)], [boolean.clone()].into())?;
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

fn main3() -> Result<(), Box<dyn Error>> {
    let crate_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let bool_extension = crate_dir.join("data/bool-0.2.0.json");
    let output = crate_dir.join("bool-0.2.0.hugr");
    let old_bool_hugr = generate(&[bool_extension], &output, build_bool_cfg_hugr, true)?;
    old_bool_hugr.validate()?;

    let updater = migrate_hugr(old_bool_hugr)?;

    fs::write("updated3.mmd", updater.get_hugr().mermaid_string())?;
    println!("+++++++++++++++++");
    updater.get_hugr().validate()?;
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    main1()?;
    main2()?;
    main3()?;
    Ok(())
}
