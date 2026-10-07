use std::fs;
use std::path::Path;
use std::process::exit;
use std::time::Instant;

use clap::Parser;

use tket::rewrite::ECCRewriter;

/// Program to precompile patterns from files into a PatternMatcher stored as binary file.
#[derive(Parser, Debug)]
#[clap(version = "1.0", long_about = None)]
#[clap(
    about = "Precompiles ECC sets into a TKET Rewriter. The resulting binary files can be loaded into TKET for circuit optimisation."
)]
struct CmdLineArgs {
    // TODO: Differentiate between TK1 input and ECC input
    /// Name of input file/folder
    #[arg(
        short,
        long,
        value_name = "FILE",
        help = "Sets the input file to use. It must be a JSON file of ECC sets in the Quartz format."
    )]
    input: String,
    /// Name of output file/folder
    #[arg(
        short,
        long,
        value_name = "FILE",
        default_value = ".",
        help = "Sets the output file or folder. Defaults to \"matcher.rwr\" if no file name is provided. The extension of the file name will always be set or amended to be `.rwr`."
    )]
    output: String,
}

fn main() {
    let opts = CmdLineArgs::parse();

    let input_path = Path::new(&opts.input);
    let output_path = Path::new(&opts.output);

    if !input_path.is_file() || input_path.extension().unwrap() != "json" {
        panic!("Input must be a JSON file");
    };
    let start_time = Instant::now();
    println!("Compiling rewriter...");
    let Ok(rewriter) = ECCRewriter::try_from_eccs_json_file(input_path) else {
        eprintln!(
            "Unable to load ECC file {input_path:?}. Is it a JSON file of Quartz-generated ECCs?"
        );
        exit(1);
    };
    print!("Saving to file...");
    let output_file = if output_path.is_dir() {
        output_path.join("matcher.rwr")
    } else {
        output_path.to_path_buf()
    };
    let write_time = Instant::now();
    let output_file = rewriter.save_binary(output_file).unwrap();
    println!(" done in {:?}", write_time.elapsed());

    println!("Written rewriter to {output_file:?}");

    // Print the file size of output_file in megabytes
    if let Ok(metadata) = fs::metadata(&output_file) {
        let file_size = metadata.len() as f64 / (1024.0 * 1024.0);
        println!("File size: {file_size:.2} MB");
    }
    let elapsed = start_time.elapsed();
    println!(
        "Done in {}.{:03} seconds",
        elapsed.as_secs(),
        elapsed.subsec_millis()
    );
}
