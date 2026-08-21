//! Generate semantically equivalent HUGRs with two quantum extension versions.

use std::{
    error::Error,
    fs::File,
    io::BufReader,
    path::{Path, PathBuf},
};

use hugr::{
    Extension, Hugr, HugrView,
    builder::{DFGBuilder, Dataflow, DataflowHugr},
    extension::{ExtensionRegistry, resolution::WeakExtensionRegistry},
    std_extensions::{STD_REG, logic::LogicOp},
    types::Signature,
};

const QUANTUM_EXTENSION: &str = "tket.quantum";
const BOOL_EXTENSION: &str = "tket.bool";
const MEASUREMENT_EXTENSION: &str = "tket.measurement";

pub(crate) fn load_extension(path: &Path) -> Result<Extension, Box<dyn Error>> {
    Ok(serde_json::from_reader(BufReader::new(File::open(path)?))?)
}

pub(crate) fn load_registry(paths: &[PathBuf]) -> Result<ExtensionRegistry, Box<dyn Error>> {
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

pub(crate) fn build_new_hugr(registry: &ExtensionRegistry) -> Result<Hugr, Box<dyn Error>> {
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

pub(crate) fn generate(
    extension_paths: &[PathBuf],
    output_path: &Path,
    build_hugr: impl FnOnce(&ExtensionRegistry) -> Result<Hugr, Box<dyn Error>>,
) -> Result<Hugr, Box<dyn Error>> {
    let registry = load_registry(extension_paths)?;
    let hugr = build_hugr(&registry)?;

    std::fs::write(output_path.with_extension("mmd"), hugr.mermaid_string())?;
    Ok(hugr)
}
