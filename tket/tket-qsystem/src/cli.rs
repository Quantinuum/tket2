//! CLI tools for tket-qsystem.

use clap::Parser;

/// CLI arguments.
#[derive(Parser, Debug)]
#[clap(version = "1.0", long_about = None)]
#[clap(about = "tket-qsystem CLI tools.")]
#[group(id = "tket-qsystem")]
#[non_exhaustive]
pub enum CliArgs {
    /// Generate serialized extensions.
    GenExtensions(hugr_cli::extensions::ExtArgs),
}
