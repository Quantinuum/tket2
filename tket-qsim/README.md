# tket-qsim

This is a small, dense-unitary simulator for testing quantum compiler
passes in this repository. Given a unitary quantum program represented as a
HUGR, it composes the matrices of its gates and returns the program's full
unitary matrix.

The crate is intended for correctness tests, especially those that check
whether a lowering, optimization, rewrite, or cross-compilation pass preserves
the program semantics. It is not intended to be a general-purpose or high-performance
quantum simulator: memory and runtime grow exponentially with the number of
qubits.

## Testing a HUGR transformation

Simulate the program before and after applying the transformation, then compare
the resulting matrices:

```rust
use std::collections::HashMap;
use tket_qsim::simulate_circuit;
use tket_qsystem::extension::qsystem::{
    LowerTketToQSystemPass, QSystemPlatform,
};

// Construct or load the unitary HUGR to test.
let original = make_test_hugr();
let expected = simulate_circuit(&original, &HashMap::new()).unwrap();

let mut lowered = original.clone();
LowerTketToQSystemPass::new(QSystemPlatform::Helios)
    .run(&mut lowered)
    .unwrap();

// Lowerings may represent gate decompositions as calls to generated
// functions. tket-qsim simulates a flat entrypoint dataflow graph, so inline
// calls and nested DFGs before simulating the transformed program.
use tket::passes::{
  InlineDFGsPass, InlineFunctionsPass,
  inline_funcs::InlineFuncsHeuristic,
};
InlineFunctionsPass::default()
    .with_heuristic(InlineFuncsHeuristic::All)
    .run(&mut lowered)
    .unwrap();
InlineDFGsPass::default().run(&mut lowered).unwrap();

// Compare the unitary matrices of the original and transformed programs
let actual = simulate_circuit(&lowered, &HashMap::new()).unwrap();
assert!(actual.approx_eq_up_to_global_phase(&expected, 1e-10));
```

### Equality comparison

Use `approx_eq` when the transformation must preserve the complete matrix,
including global phase. Use `approx_eq_up_to_global_phase` when global phase is
not semantically relevant.

### HUGRs with floating-point parameters

If a circuit receives concrete floating-point parameters through function
inputs, pass them to `simulate_circuit` as a map from input-port index to value:

```rust
let params = HashMap::from([(2, 0.25), (3, -0.7)]);
let unitary = simulate_circuit(&hugr, &params).unwrap();
```

Parameters embedded as supported HUGR constants and simple floating-point
arithmetic are resolved automatically. Tests should otherwise use purely
unitary circuits with a fixed number of input and output qubits.

## Adding a new quantum operation

Adding a new quantum gate has two parts:

1. Implement `Simulatable` for the operation in the appropriate module:
   [`src/tket_ops.rs`](src/tket_ops.rs),
   [`src/helios_ops.rs`](src/helios_ops.rs), or
   [`src/sol_ops.rs`](src/sol_ops.rs). `num_qubits` declares the gate arity and
   `unitary` constructs its `UnitaryMatrix` from the supplied parameters.
2. Register the operation in [`src/simulate.rs`](src/simulate.rs), specifying
   which input ports contain qubits and which contain floating-point
   parameters. This lets the HUGR traversal resolve the gate's wires and
   arguments.

### Matrix conventions

Matrices passed to `UnitaryMatrix::from_row_major` use computational-basis,
row-major order. For example, the X gate is:

```rust
use num_complex::Complex64;
use tket_qsim::UnitaryMatrix;

let zero = Complex64::ZERO;
let one = Complex64::new(1.0, 0.0);
let x = UnitaryMatrix::from_row_major(1, vec![zero, one, one, zero]);
```

For multi-qubit gates, the first local qubit is the most-significant qubit in
the basis order. A two-qubit matrix therefore uses
`|00>, |01>, |10>, |11>`.

### A warning on angle conventions

Document each gate's angle convention carefully. For example, the tket rotation
parameters use half-turns, while the Helios and Sol operations currently use radians.

Finally, add test cases to [`tests/reference_unitaries.rs`](tests/reference_unitaries.rs).
Prefer comparing against roqoqo's `<QuantumOp>::unitary_matrix()` rather than copying
a matrix literal into the test. If roqoqo has no direct equivalent, construct
the reference from independently composed roqoqo gate matrices. Include zero,
positive, negative, and nontrivial parameter values so that sign, scale, and
angle-unit errors are detectable.

## Testing

Run the crate tests with:

```text
cargo test -p tket-qsim
```
