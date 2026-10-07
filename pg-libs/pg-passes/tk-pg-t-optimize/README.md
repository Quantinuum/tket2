# tk-pg-t-optimize

`tk-pg-t-optimize` provides `TOptimizationPass`, which reduces the T count of
Clifford + T Pauli graphs using phase polynomial resynthesis by the TODD
algorithm. Optional ancillas allow Hadamard gadgets to combine more rotations
into each optimization batch, increasing the performance of the pass.

The pass expects a pauli graph which has been processed by the 
`CanonicalFormPass` pass, and performs best after also applying phase folding 
from the `RotationMergingPass`. Input measurements, resets, conditional operations, 
black boxes and arbitrary Rz gates are not currently supported.

By default, the pass uses no ancillas. `with_ancilla_budget` reserves the last
qubits of the input graph as ancillas; these must already be present, idle, and
prepared in zero. The pass does not allocate additional qubits.

Using ancillas hadamard gadgets which contain measurements, resets, and 
conditional Clifford.

To synthesize the result into gates apply`GreedySynthPass`.

Bit-vector operations are scalar by default. The optional `unstable_simd`
feature enables portable SIMD and requires a nightly Rust toolchain.

## License

This project is licensed under Apache License, Version 2.0 ([LICENCE][] or <https://www.apache.org/licenses/LICENSE-2.0>).

  [LICENCE]: https://github.com/quantinuum/tket2/blob/main/LICENCE
