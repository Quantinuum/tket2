//! Helpers for loading checked-in Guppy optimization fixtures in benchmarks.
//!
//! These helpers mirror the test-side Guppy fixture loading style so benchmark
//! inputs stay easy to discover and reuse.

use std::fs;
use std::io::BufReader;
use std::path::Path;

use hugr::Hugr;

const GUPPY_EXAMPLES_DIR: &str = "../test_files/guppy_optimization";

/// Loads a checked-in Guppy optimization example by directory name.
///
/// This expects the standard fixture layout used in `test_files`, where an
/// example named `foo` lives at `foo/foo.hugr`.
///
/// # Errors
///
/// Returns an error when the fixture file cannot be opened.
pub fn load_guppy_example(name: &str) -> std::io::Result<Hugr> {
    let file = Path::new(GUPPY_EXAMPLES_DIR)
        .join(name)
        .join(format!("{name}.hugr"));
    let reader = fs::File::open(file)?;
    let reader = BufReader::new(reader);
    Ok(Hugr::load(reader, None).expect("guppy fixture should be a valid HUGR"))
}
