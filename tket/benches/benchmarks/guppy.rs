//! Helpers for loading checked-in Guppy fixtures in benchmarks.
//!
//! These helpers mirror the test-side Guppy fixture loading style so benchmark
//! inputs stay easy to discover and reuse.

use std::fs;
use std::io::BufReader;
use std::path::Path;

use hugr::Hugr;

const GUPPY_EXAMPLES_DIR: &str = "../test_files/guppy_examples";
const GUPPY_OPTIMIZATION_DIR: &str = "../test_files/guppy_optimization";

/// Loads a checked-in Guppy example by name.
///
/// One-file examples live directly under `test_files/guppy_examples` as
/// `foo.hugr`. Older optimization fixtures still use the directory layout
/// `test_files/guppy_optimization/foo/foo.hugr`.
///
/// # Errors
///
/// Returns an error when the fixture file cannot be opened.
pub fn load_guppy_example(name: &str) -> std::io::Result<Hugr> {
    let one_file = Path::new(GUPPY_EXAMPLES_DIR).join(format!("{name}.hugr"));
    let nested = Path::new(GUPPY_OPTIMIZATION_DIR)
        .join(name)
        .join(format!("{name}.hugr"));
    let file = if one_file.exists() { one_file } else { nested };
    let reader = fs::File::open(file)?;
    let reader = BufReader::new(reader);
    Ok(Hugr::load(reader, None).expect("guppy fixture should be a valid HUGR"))
}
