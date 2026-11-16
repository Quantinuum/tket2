//! Load local extension definitions for migration.

use std::sync::{Arc, LazyLock};

use hugr::{Extension, extension::ExtensionRegistry};

/// Registry containing local extensions for migration.
pub static LOCAL_REGISTRY: LazyLock<ExtensionRegistry> = LazyLock::new(|| local_registry());

fn local_registry() -> ExtensionRegistry {
    let mut legacy_extensions = Vec::<Arc<Extension>>::new();
    let mut directories = vec![std::path::PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/extensions"
    ))];
    while let Some(directory) = directories.pop() {
        for entry in std::fs::read_dir(directory).unwrap() {
            let entry = entry.unwrap();
            let file_type = entry.file_type().unwrap();
            let path = entry.path();
            if file_type.is_dir() {
                directories.push(path);
            } else if file_type.is_file() && path.extension().is_some_and(|ext| ext == "json") {
                let reader = std::io::BufReader::new(std::fs::File::open(path).unwrap());
                legacy_extensions.push(Arc::new(serde_json::from_reader(reader).unwrap()));
            }
        }
    }

    ExtensionRegistry::new(legacy_extensions)
}
