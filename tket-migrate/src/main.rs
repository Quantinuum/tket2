//! Generate semantically equivalent HUGRs with two quantum extension versions.

mod testing_func;
use std::{error::Error, path::PathBuf};

use testing_func::{build_new_hugr, build_old_hugr, generate};

#[allow(dead_code)]
fn update_measure_op() -> Result<(), Box<dyn Error>> {
    // here i want to create an older hugr, add the new extension, look for the measure op, change the measure op to new measurement op
    Ok(())
}

#[allow(dead_code)]
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
    )?;

    let new_output = crate_dir.join("quantum-0.3.0.hugr");
    let _new_hugr = generate(
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

fn main() -> Result<(), Box<dyn Error>> {
    main1()
}
