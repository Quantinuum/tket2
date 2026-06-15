//! CLI for tket-qsystem

use anyhow::Result;
use clap::Parser as _;
use tket_qsystem::cli::CliArgs;

fn main() -> Result<()> {
    match CliArgs::parse() {
        CliArgs::GenExtensions(args) => {
            args.run_dump(&tket_qsystem::extension::REGISTRY)?;
        }
        _ => {
            eprintln!("Unknown command");
            std::process::exit(1);
        }
    };

    Ok(())
}
