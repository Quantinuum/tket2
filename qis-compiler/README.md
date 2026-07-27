# The HUGR-QIS compiler for Selene

This package provides compilation from HUGR to LLVM adhering
to the Quantinuum QIS.

## Usage

Guppy can produce a HUGR Envelope with:
```python
from guppylang.decorator import guppy

@guppy
def main() -> None:
    # ...

hugr = main.compile()
hugr_envelope = hugr.package.to_bytes()
```

This can then be compiled to LLVM IR or bitcode
using this package:

```python
from qis_compiler import (
    compile_to_llvm_ir,
    compile_to_bitcode,
)

# Compile to LLVM IR for the host system
# and receive it as a string
ir = compile_to_llvm_ir(hugr_envelope)

# Compile to LLVM bitcode for the host system
# and receive it as a bytes object
bitcode = compile_to_bitcode(hugr_envelope)
```

If you wish to target a specific architecture or platform,
you can pass the triple as an argument to the compilation functions:

```python
# Compile to LLVM IR for Apple Silicon
ir_apple_silicon = compile_to_llvm_ir(
    hugr_envelope,
    target_triple="aarch64-apple-darwin",
)
# Compile to LLVM bitcode for x86_64 MSVC
bitcode_apple_silicon = compile_to_bitcode(
    hugr_envelope,
    target_triple="x86_64-windows-msvc",
)
```

## Development

### Snapshot testing
This package uses snapshot testing for its compiler output.
Sample programs are defined using guppy in the script `python/tests/generate_hugrs.py`.
To add a new test case add a new function to generate the desired HUGR program and
call it from the `if __name__ == "__main__":` block.

The script has self contained dependencies with a pinned guppy version
defined in comments at the top of the file. `uv` will create a virtual environment
and install the dependencies when you run the script with `uv run python
tests/generate_hugrs.py`. This can also be done using `just regenerate` in the
`python/tests` directory.

HUGR regeneration should not be done in conjunction with other changes to the compiler,
to isolate possible changes to the snapshots. It should only be required if guppylang
updates are required for testing the compiler.
