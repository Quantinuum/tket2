//! Generate semantically equivalent HUGRs with two quantum extension versions.

mod lib;
mod testing_func;
use hugr::{Extension, HugrView, extension::Version};
use std::{error::Error, path::PathBuf};

use lib::ExtensionUpdater;
use testing_func::{build_bool_hugr, build_old_hugr, generate};

use crate::{
    lib::{OpUpdateMap, UndatingMap, VersionedOp},
    testing_func::load_extensions,
};

#[allow(dead_code)]
fn update_measure_op() -> Result<(), Box<dyn Error>> {
    // here i want to create an older hugr, add the new extension, look for the measure op, change the measure op to new measurement op
    Ok(())
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

    let updating_map = UndatingMap::new(vec![
        OpUpdateMap::new(
            VersionedOp::new(
                "MeasureFree".to_string(),
                "tket.quantum".to_string(),
                Version::new(0, 2, 1),
            ),
            vec![
                VersionedOp::new(
                    "MeasureFree".to_string(),
                    "tket.quantum".to_string(),
                    Version::new(0, 3, 0),
                ),
                VersionedOp::new(
                    "Read".to_string(),
                    "tket.measurement".to_string(),
                    Version::new(0, 1, 0),
                ),
            ],
        ),
        OpUpdateMap::new(
            VersionedOp::new(
                "read".to_string(),
                "tket.bool".to_string(),
                Version::new(0, 2, 0),
            ),
            vec![],
        ),
    ]);

    let mut updater = ExtensionUpdater::new(_old_hugr, updating_map);
    updater.update_op(load_new_extensions()?);

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

fn main2() -> Result<(), Box<dyn Error>> {
    let crate_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let bool_extension = crate_dir.join("data/bool-0.2.0.json");
    let output = crate_dir.join("bool-0.2.0.hugr");
    let old_bool_hugr = generate(&[bool_extension], &output, build_bool_hugr, true)?;
    old_bool_hugr.validate()?;

    let updating_map = UndatingMap::new(vec![
        OpUpdateMap::new(
            VersionedOp::new(
                "and".to_string(),
                "tket.bool".to_string(),
                Version::new(0, 2, 0),
            ),
            vec![VersionedOp::new(
                "And".to_string(),
                "logic".to_string(),
                Version::new(0, 1, 0),
            )],
        ),
        OpUpdateMap::new(
            VersionedOp::new(
                "eq".to_string(),
                "tket.bool".to_string(),
                Version::new(0, 2, 0),
            ),
            vec![VersionedOp::new(
                "Eq".to_string(),
                "logic".to_string(),
                Version::new(0, 1, 0),
            )],
        ),
        OpUpdateMap::new(
            VersionedOp::new(
                "not".to_string(),
                "tket.bool".to_string(),
                Version::new(0, 2, 0),
            ),
            vec![VersionedOp::new(
                "Not".to_string(),
                "logic".to_string(),
                Version::new(0, 1, 0),
            )],
        ),
        OpUpdateMap::new(
            VersionedOp::new(
                "or".to_string(),
                "tket.bool".to_string(),
                Version::new(0, 2, 0),
            ),
            vec![VersionedOp::new(
                "Or".to_string(),
                "logic".to_string(),
                Version::new(0, 1, 0),
            )],
        ),
        OpUpdateMap::new(
            VersionedOp::new(
                "xor".to_string(),
                "tket.bool".to_string(),
                Version::new(0, 2, 0),
            ),
            vec![VersionedOp::new(
                "Xor".to_string(),
                "logic".to_string(),
                Version::new(0, 1, 0),
            )],
        ),
        OpUpdateMap::new(
            VersionedOp::new(
                "read".to_string(),
                "tket.bool".to_string(),
                Version::new(0, 2, 0),
            ),
            vec![],
        ),
        OpUpdateMap::new(
            VersionedOp::new(
                "make_opaque".to_string(),
                "tket.bool".to_string(),
                Version::new(0, 2, 0),
            ),
            vec![],
        ),
    ]);

    let mut updater = ExtensionUpdater::new(old_bool_hugr, updating_map);
    updater.update_op(load_new_extensions()?);

    std::fs::write("updated.mmd", updater.get_hugr().mermaid_string())?;
    println!("+++++++++++++++++");

    // NICOLA todo: remove not used extensions

    updater.get_hugr().validate()?;

    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    main2()?;
    Ok(())
}
