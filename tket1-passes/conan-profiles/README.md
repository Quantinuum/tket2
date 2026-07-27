This directory lists the conan profiles for which pre-built tket-c-api binaries
are available in the tket-libs conan remote.
The build script in `../build.rs` will try to use one of these profiles if possible.

They have been copied over from the [tket repository](https://github.com/quantinuum/tket/tree/71116c2c509d1b65ee07b066934d64ea34cc3295/conan-profiles) where possible,
otherwise they have been created manually for the additional platforms we build python wheels for. See the workflow [python-wheels.yml](https://github.com/quantinuum/tket2/blob/main/.github/workflows/python-wheels.yml) for more details.
