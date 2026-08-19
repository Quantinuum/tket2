//! Generate semantically equivalent HUGRs with two quantum extension versions.

use std::{
    error::Error,
    fs::File,
    io::{BufReader, BufWriter},
    path::{Path, PathBuf},
};

use hugr::{
    Extension, Hugr, HugrView,
    builder::{DFGBuilder, Dataflow, DataflowHugr},
    envelope::EnvelopeConfig,
    extension::{ExtensionRegistry, resolution::WeakExtensionRegistry},
    std_extensions::{STD_REG, logic::LogicOp},
    types::Signature,
};

const QUANTUM_EXTENSION: &str = "tket.quantum";
const BOOL_EXTENSION: &str = "tket.bool";
const MEASUREMENT_EXTENSION: &str = "tket.measurement";

fn load_extension(path: &Path) -> Result<Extension, Box<dyn Error>> {
    Ok(serde_json::from_reader(BufReader::new(File::open(path)?))?)
}

fn load_registry(paths: &[PathBuf]) -> Result<ExtensionRegistry, Box<dyn Error>> {
    let mut registry = STD_REG.to_owned();
    let extensions = paths
        .iter()
        .map(|path| load_extension(path))
        .collect::<Result<Vec<_>, _>>()?;
    let custom_extensions = ExtensionRegistry::new_with_extension_resolution(
        extensions,
        &WeakExtensionRegistry::from(&registry),
    )?;
    registry.extend(custom_extensions.clone());
    Ok(registry)
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

fn build_new_hugr(registry: &ExtensionRegistry) -> Result<Hugr, Box<dyn Error>> {
    let quantum = registry
        .get(QUANTUM_EXTENSION)
        .ok_or("tket.quantum is missing from the registry")?;
    let measurement_extension = registry
        .get(MEASUREMENT_EXTENSION)
        .ok_or("tket.measurement is missing from the registry")?;
    let qalloc = quantum.instantiate_extension_op("QAlloc", [])?;
    let h = quantum.instantiate_extension_op("H", [])?;
    let measure_free = quantum.instantiate_extension_op("MeasureFree", [])?;
    let read = measurement_extension.instantiate_extension_op("Read", [])?;

    let mut builder = DFGBuilder::new(Signature::new(vec![], vec![]))?;
    let qubit = builder.add_dataflow_op(qalloc, [])?.out_wire(0);
    let qubit = builder.add_dataflow_op(h, [qubit])?.out_wire(0);
    let measurement = builder.add_dataflow_op(measure_free, [qubit])?.out_wire(0);
    let boolean = builder.add_dataflow_op(read, [measurement])?.out_wire(0);
    builder.add_dataflow_op(LogicOp::Not, [boolean])?;
    Ok(builder.finish_hugr_with_outputs([])?)
}

fn generate(
    extension_paths: &[PathBuf],
    output_path: &Path,
    build_hugr: impl FnOnce(&ExtensionRegistry) -> Result<Hugr, Box<dyn Error>>,
) -> Result<(), Box<dyn Error>> {
    let registry = load_registry(extension_paths)?;
    let hugr = build_hugr(&registry)?;

    std::fs::write(output_path.with_extension("mmd"), hugr.mermaid_string())?;
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    let crate_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let extension_dir = crate_dir.join("../tket-exts/src/tket_exts/data/tket");
    let fixture_dir = crate_dir.join("data");
    let rotation = extension_dir.join("rotation.json");

    let old_output = crate_dir.join("quantum-0.2.1.hugr");
    generate(
        &[
            rotation.clone(),
            fixture_dir.join("bool-0.2.0.json"),
            fixture_dir.join("quantum-0.2.1.json"),
        ],
        &old_output,
        build_old_hugr,
    )?;

    let new_output = crate_dir.join("quantum-0.3.0.hugr");
    generate(
        &[
            rotation,
            extension_dir.join("measurement.json"),
            extension_dir.join("quantum.json"),
        ],
        &new_output,
        build_new_hugr,
    )?;

    println!("wrote {}", old_output.display());
    println!("wrote {}", new_output.display());
    Ok(())
}
