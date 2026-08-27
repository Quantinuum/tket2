//! Generate semantically equivalent HUGRs with two quantum extension versions.

mod lib;
mod testing_func;
use hugr::{
    HugrView,
    builder::{DFGBuilder, Dataflow, DataflowHugr},
    extension::{ExtensionRegistry, Version, prelude::bool_t},
    types::Signature,
};
use std::{error::Error, io::BufReader, path::PathBuf};

use lib::ExtensionUpdater;
use testing_func::{build_new_hugr, build_old_hugr, generate};

use crate::testing_func::load_extensions;

#[allow(dead_code)]
fn update_measure_op() -> Result<(), Box<dyn Error>> {
    // here i want to create an older hugr, add the new extension, look for the measure op, change the measure op to new measurement op
    Ok(())
}

fn main1() -> Result<(), Box<dyn Error>> {
    let crate_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let extension_dir = crate_dir.join("../tket-exts/src/tket_exts/data/tket");
    let fixture_dir = crate_dir.join("data");
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

    let new_extension_paths = vec![
        extension_dir.join("measurement.json"),
        extension_dir.join("quantum.json"),
    ];

    let updating_map = lib::UndatingMap::new(vec![
        lib::OpUpdateMap::new(
            lib::VersionedOp::new(
                "MeasureFree".to_string(),
                "tket.quantum".to_string(),
                Version::new(0, 2, 1),
            ),
            vec![
                lib::VersionedOp::new(
                    "MeasureFree".to_string(),
                    "tket.quantum".to_string(),
                    Version::new(0, 3, 0),
                ),
                lib::VersionedOp::new(
                    "Read".to_string(),
                    "tket.measurement".to_string(),
                    Version::new(0, 1, 0),
                ),
            ],
        ),
        lib::OpUpdateMap::new(
            lib::VersionedOp::new(
                "read".to_string(),
                "tket.bool".to_string(),
                Version::new(0, 2, 0),
            ),
            vec![],
        ),
    ]);

    let extensions = load_extensions(&new_extension_paths).unwrap();
    let mut updater = ExtensionUpdater::new(_old_hugr, updating_map);
    updater.update_op(extensions);

    std::fs::write("updated.mmd", updater.get_hugr().mermaid_string())?;
    println!("+++++++++++++++++");

    // NICOLA todo: remove not used extensions

    updater.get_hugr().validate()?;

    // let new_output = crate_dir.join("quantum-0.3.0.hugr");
    // let _new_hugr = generate(&new_extension_paths, &new_output, build_new_hugr, false)?;

    // println!("wrote {}", old_output.display());
    // // println!("wrote {}", new_output.display());
    Ok(())
}

fn build_bool_hugr(registry: &ExtensionRegistry) -> Result<hugr::Hugr, Box<dyn Error>> {
    let bool_extension = registry
        .get("tket.bool")
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
    let hugr = generate(&[bool_extension], &output, build_bool_hugr, true)?;
    hugr.validate()?;
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    main2()?;
    Ok(())
}
