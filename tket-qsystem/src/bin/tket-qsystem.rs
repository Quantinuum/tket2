//! CLI for tket-qsystem

use anyhow::Result;
use clap::Parser as _;
use hugr::extension::ExtensionRegistry;
use tket_qsystem::cli::CliArgs;

fn main() -> Result<()> {
    match CliArgs::parse() {
        CliArgs::GenExtensions(args) => {
            let reg = ExtensionRegistry::new([
                tket::extension::TKET_EXTENSION.to_owned(),
                tket::extension::rotation::ROTATION_EXTENSION.to_owned(),
                tket::extension::bool::BOOL_EXTENSION.to_owned(),
                tket::extension::debug::DEBUG_EXTENSION.to_owned(),
                tket::extension::guppy::GUPPY_EXTENSION.to_owned(),
                tket::extension::global_phase::GLOBAL_PHASE_EXTENSION.to_owned(),
                tket::extension::modifier::MODIFIER_EXTENSION.to_owned(),
                tket::extension::globals::EXTENSION.to_owned(),
                tket_qsystem::extension::gpu::EXTENSION.to_owned(),
                tket_qsystem::extension::qsystem::EXTENSION.to_owned(),
                tket_qsystem::extension::futures::EXTENSION.to_owned(),
                tket_qsystem::extension::random::EXTENSION.to_owned(),
                tket_qsystem::extension::result::EXTENSION.to_owned(),
                tket_qsystem::extension::utils::EXTENSION.to_owned(),
                tket_qsystem::extension::wasm::EXTENSION.to_owned(),
                tket_qsystem::extension::wasm::EXTENSION.to_owned(),
            ]);

            args.run_dump(&reg)?;
        }
        _ => {
            eprintln!("Unknown command");
            std::process::exit(1);
        }
    };

    Ok(())
}
