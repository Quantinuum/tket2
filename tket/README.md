# tket: The Hardware Agnostic Quantum Compiler

[![build_status][]](https://github.com/quantinuum/tket2/actions)
![msrv][]
[![codecov][]](https://codecov.io/gh/quantinuum/tket2)

TKET is an open source quantum compiler developed by Quantinuum. Central to
TKET's design is its hardware agnosticism which allows researchers and quantum
software developers to take advantage of its powerful compilation for many
different quantum architectures.

Circuits are represented using the HUGR IR defined in the
[hugr] crate. TKET augments Hugr with
* The [`Circuit`] trait, providing a high-level interface for working with HUGRs representing quantum circuits
* a HUGR extension with quantum operations
* A composable pass system for optimising circuits
* A number of built-in rewrite utilities and passes for common optimisations

This crate is interoperable with legacy [`pytket`] circuits via its serial
encoding.

# Using TKET

Defining a circuit in TKET is currently done by using the low-level [hugr Builder] API, or by loading tket1 circuits from JSON files.

```rust
use tket::{Circuit, Hugr};

// Load a tket1 circuit.
let mut circ: Hugr = tket::json::load_tk1_json_file("test_files/pytket/barenco_tof_5.json").unwrap();

assert_eq!(circ.qubit_count(), 9);
assert_eq!(circ.num_gates(), 170);

// Traverse the circuit and print the gates.
for command in circ.commands() {
    println!("{:?}", command.optype());
}

// Render the circuit as a mermaid diagram.
println!("{}", circ.mermaid_string());

// Optimise the circuit.
tket::passes::apply_greedy_commutation(&mut circ);
```

Please read the [API documentation here][].

## Features

- `portmatching`
  Enables pattern matching using the [`portmatching`][] crate.

- `rewrite-tracing`
  Adds opt-in tracking of the rewrites applied to a circuit.

## Recent Changes

See [CHANGELOG][] for a list of changes. The minimum supported rust
version will only change on major releases.

## Developing TKET

See [DEVELOPMENT.md][] for instructions on setting up the development environment.

## License

This project is licensed under Apache License, Version 2.0 ([LICENSE][] or http://www.apache.org/licenses/LICENSE-2.0).

  [build_status]: https://github.com/quantinuum/tket2/actions/workflows/ci.yml/badge.svg
  [msrv]: https://img.shields.io/crates/msrv/tket
  [codecov]: https://img.shields.io/codecov/c/gh/quantinuum/tket2?logo=codecov
  [hugr]: https://lib.rs/crates/hugr
  [hugr Builder]: https://docs.rs/hugr/latest/hugr/builder/index.html
  [API documentation here]: https://docs.rs/tket/
  [`Circuit`]: https://docs.rs/tket/latest/tket/trait.Circuit.html
  [`pytket`]: https://github.com/quantinuum/tket
  [`portmatching`]: https://lib.rs/crates/portmatching
  [LICENSE]: https://github.com/quantinuum/tket2/blob/main/LICENCE
  [CHANGELOG]: https://github.com/quantinuum/tket2/blob/main/tket/CHANGELOG.md
  [DEVELOPMENT.md]: https://github.com/quantinuum/tket2/blob/main/DEVELOPMENT.md
