//! CLI for tket-qsystem

use anyhow::Result;
use clap::Parser as _;
use hugr_core::extension::ExtensionRegistry;
use tket_qsystem::cli::CliArgs;

fn main() -> Result<()> {
    match CliArgs::parse() {
        CliArgs::GenExtensions(args) => {
            let registry = ExtensionRegistry::new(tket_qsystem::extension::qsystem_extensions());
            args.run_dump(&registry)?;
        }
        _ => {
            eprintln!("Unknown command");
            std::process::exit(1);
        }
    };

    Ok(())
}
