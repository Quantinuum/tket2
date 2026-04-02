//! Build script for the tket1-passes crate
//!
//! Use bindgen to generate Rust bindings for the C library.

use std::env;
use std::path::{Path, PathBuf};
use std::process::Command;

const CONAN_REMOTE: &str = "https://quantinuumsw.jfrog.io/artifactory/api/conan/tket1-libs";

fn main() {
    // // Configure when to rerun the build script
    println!("cargo:rerun-if-env-changed=TKET_C_API_PATH");
    println!("cargo:rerun-if-changed=conanfile.txt");
    println!("cargo:rerun-if-changed=build.rs");

    let custom_tket_path = env::var("TKET_C_API_PATH").ok().map(PathBuf::from);

    let target = SupportedPlatform::from_target_str(&env::var("TARGET").unwrap());

    let header_path = if let Some(path) = custom_tket_path {
        cargo_set_custom_lib_path(&[path.join("lib"), path.join("lib64")]);
        let header_path = path.join("include").join("tket-c-api.h");

        assert!(
            header_path.exists(),
            "TKET_C_API_PATH is set to {} but header was not found at {}",
            path.display(),
            header_path.display()
        );
        header_path
    } else {
        // Get dependencies from conan

        // 0. Check that conan is installed
        assert!(
            Command::new("conan").arg("--version").output().is_ok(),
            "conan is not installed"
        );

        // 1. Add the conan remote to get the tket-c-api source and binaries
        add_conan_remote_if_missing(CONAN_REMOTE);

        // 2. Select one of the pre-built conan profiles if possible (fallback to default)
        let (profile_name, needs_build) = target
            .map(|t| {
                let profile = t.get_prebuilt_conan_profile();
                let needs_build = t.needs_build_from_source();
                (profile, needs_build)
            })
            .unwrap_or_else(|| {
                println!("cargo:warning=Unrecognized platform. Defaulting to default conan profile and building from source.");
                ("default", true)
            });
        let profile_path = Path::new("./conan-profiles").join(profile_name);
        println!("Using conan profile: {}", profile_path.display());

        // 3. Prepare the conan install command
        let mut conan_install = conan2::ConanInstall::new();
        conan_install.build_type("Release");

        if profile_name == "default" {
            // For default profile, let conan auto-detect
            conan_install.detect_profile().build("missing");
        } else {
            // Use the specified profile
            conan_install.profile(profile_path.to_str().unwrap());
            if needs_build {
                conan_install.build("missing");
            }
        }

        // 4. Run the conan install command and point cargo to the correct install paths
        conan_install.verbosity(conan2::ConanVerbosity::Verbose);
        let cargo_instructions = conan_install.run().parse();
        cargo_instructions.emit();

        // 5. Get header path for bindgen
        cargo_instructions
            .include_paths()
            .into_iter()
            .map(|incl_path| incl_path.join("tket-c-api.h"))
            .find(|header| header.exists())
            .expect("required tket-c-api.h header not found")
    };

    // Link in standard C++ library.
    if target.is_some_and(|t| t.is_macos()) {
        println!("cargo:rustc-link-lib=c++");
    } else if target.is_some_and(|t| t == SupportedPlatform::WindowsX86) {
        // On Windows with MSVC, don't link stdc++ - MSVC runtime is used by default
        // The tket-c-api library should already be linked against the correct runtime
    } else {
        println!("cargo:rustc-link-lib=stdc++");
    }

    // Generate bindings using bindgen
    let bindings = bindgen::Builder::default()
        .header(header_path.to_str().unwrap())
        .generate()
        .expect("Unable to generate bindings");

    // Write the bindings to the $OUT_DIR/bindings.rs file.
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap()).join("bindings.rs");
    bindings
        .write_to_file(&out_path)
        .expect("Couldn't write bindings!");
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum SupportedPlatform {
    // Platforms with pre-built binaries in tket-libs conan remote
    MacOsX86,
    MacOsArm,
    LinuxX86,
    // Manually added profiles for cross-compilation (no pre-built binaries)
    WindowsX86,
    LinuxArmv8,
    LinuxAarch64Gnu,
    LinuxAarch64Musl,
    LinuxArmv7Gnu,
    LinuxArmv7Musl,
    LinuxI686Gnu,
    LinuxI686Musl,
    LinuxX86_64Musl,
}

impl SupportedPlatform {
    fn from_target_str(target: &str) -> Option<Self> {
        match target {
            // Exact matches for manually added cross-compilation targets
            "aarch64-unknown-linux-gnu" => Some(SupportedPlatform::LinuxAarch64Gnu),
            "aarch64-unknown-linux-musl" => Some(SupportedPlatform::LinuxAarch64Musl),
            "armv7-unknown-linux-gnueabihf" => Some(SupportedPlatform::LinuxArmv7Gnu),
            "armv7-unknown-linux-musleabihf" => Some(SupportedPlatform::LinuxArmv7Musl),
            "i686-unknown-linux-gnu" => Some(SupportedPlatform::LinuxI686Gnu),
            "i686-unknown-linux-musl" => Some(SupportedPlatform::LinuxI686Musl),
            "x86_64-unknown-linux-musl" => Some(SupportedPlatform::LinuxX86_64Musl),
            // Fallback patterns for platforms with pre-built binaries
            t if t.contains("apple") && t.contains("x86") => Some(SupportedPlatform::MacOsX86),
            t if t.contains("apple") && t.contains("aarch64") => Some(SupportedPlatform::MacOsArm),
            t if t.contains("windows") && t.contains("x86") => Some(SupportedPlatform::WindowsX86),
            t if t.contains("linux") && t.contains("x86_64") && !t.contains("musl") => {
                Some(SupportedPlatform::LinuxX86)
            }
            t if t.contains("linux") && t.contains("armv8") => Some(SupportedPlatform::LinuxArmv8),
            _ => None,
        }
    }

    fn is_macos(&self) -> bool {
        matches!(
            self,
            SupportedPlatform::MacOsX86 | SupportedPlatform::MacOsArm
        )
    }

    fn get_prebuilt_conan_profile(&self) -> &'static str {
        match self {
            // Platforms with pre-built binaries in tket-libs conan remote
            // See ./conan-profiles/README.md for more details
            SupportedPlatform::MacOsX86 => "macos-15-intel",
            SupportedPlatform::MacOsArm => "macos-15",
            SupportedPlatform::WindowsX86 => "windows-2025",
            SupportedPlatform::LinuxX86 => "linux-x86_64-gcc13",
            SupportedPlatform::LinuxArmv8 => "linux-armv8-gcc14",
            // Manually added profiles for cross-compilation (will build from source)
            SupportedPlatform::LinuxAarch64Gnu => "linux-armv8-gcc14",
            SupportedPlatform::LinuxAarch64Musl => "linux-armv8-gcc14",
            SupportedPlatform::LinuxArmv7Gnu => "linux-armv7-gcc14",
            SupportedPlatform::LinuxArmv7Musl => "linux-armv7-gcc14",
            SupportedPlatform::LinuxI686Gnu => "linux-i686-gcc14",
            SupportedPlatform::LinuxI686Musl => "linux-i686-gcc14",
            SupportedPlatform::LinuxX86_64Musl => "linux-x86_64-gcc14",
        }
    }

    /// Some selected platforms publish pre-built binaries.
    ///
    /// For all others, we need to pass `--build=missing` to conan and build
    /// from source.
    fn needs_build_from_source(&self) -> bool {
        !matches!(
            self,
            SupportedPlatform::MacOsX86 | SupportedPlatform::MacOsArm | SupportedPlatform::LinuxX86
        )
    }
}

fn add_conan_remote_if_missing(conan_remote: &str) {
    let check_exists = Command::new("conan")
        .args(["remote", "list", "-fjson"])
        .output()
        .ok();

    let stdout_str =
        check_exists.map(|output| String::from_utf8_lossy(&output.stdout).into_owned());
    if stdout_str.is_none_or(|s| !s.contains(conan_remote)) {
        println!("Adding conan remote: {conan_remote}");
        Command::new("conan")
            .args(["remote", "add", "tket-libs", conan_remote, "--index", "0"])
            .output()
            .unwrap();
    }
}

fn cargo_set_custom_lib_path(search_path: &[PathBuf]) {
    for path in search_path {
        if path.exists() {
            println!("cargo:rustc-link-search={}", path.display());
        }
    }

    let lib_name = "tket-c-api";
    println!("cargo:rustc-link-lib={lib_name}");

    if cfg!(target_os = "windows") {
        // If on windows, also link tket and symengine, as we're
        // providing static libs
        println!("cargo:rustc-link-lib=tket");
        println!("cargo:rustc-link-lib=symengine");
        println!("cargo:rustc-link-lib=tkassert");
        println!("cargo:rustc-link-lib=tklog");
        println!("cargo:rustc-link-lib=tkrng");
        println!("cargo:rustc-link-lib=tktokenswap");
        println!("cargo:rustc-link-lib=tkwsm");
    }
}
