//! Generate semantically equivalent HUGRs with two quantum extension versions.

mod lib;
mod testing_func;
use hugr::{HugrView, extension::Version};
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
        false,
    )?;

    let new_extension_paths = vec![
        extension_dir.join("measurement.json"),
        extension_dir.join("quantum.json"),
    ];

    let updating_map = lib::UndatingMap::new(vec![
        lib::OpUpdateMap::new(
            "MeasureFree".to_string(),
            "tket.quantum".to_string(),
            Version::new(0, 2, 1),
            "MeasureFree".to_string(),
            "tket.quantum".to_string(),
            Version::new(0, 3, 0),
        ),
        lib::OpUpdateMap::new(
            "read".to_string(),
            "tket.bool".to_string(),
            Version::new(0, 2, 0),
            "Read".to_string(),
            "tket.measurement".to_string(),
            Version::new(0, 1, 0),
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

fn main() -> Result<(), Box<dyn Error>> {
    main1()?;
    Ok(())
}
