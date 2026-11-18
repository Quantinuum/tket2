This directory contains Conan profiles copied from the
[tket repository](https://github.com/Quantinuum/tket/tree/99123e8ba4ac5156a1b3854f3b9d8a01ac9aa393/conan-profiles),
plus the local `linux-armv7-gcc14` and `linux-i686-gcc14` profiles for additional
cross-compilation targets.

The build script in `../build.rs` selects `macos-15-intel`, `macos-26`,
`linux-x86_64-gcc14`, or `windows-2025-vs2026` for the corresponding native target.
Other supported Linux targets use the matching GCC 14 profile and allow building
missing packages from source. Windows also allows building missing packages.
Unrecognized targets use Conan's detected default profile.

The macOS ARM selection matches upstream's release builds, with
`MACOSX_DEPLOYMENT_TARGET=14.0` set in the `macos-26` profile.
Emscripten builds consume a library supplied through `TKET_C_API_PATH`. See the
[Python wheels workflow](../../../.github/workflows/python-wheels.yml) for the
wheel build environments.
