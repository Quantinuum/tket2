![](/hugr/assets/hugr_logo.svg)

# hugr-passes

[![build_status][]](https://github.com/quantinuum/tket2/actions)
[![crates][]](https://crates.io/crates/hugr-passes)
[![msrv][]](https://github.com/quantinuum/tket2)
[![codecov][]](https://codecov.io/gh/quantinuum/tket2)

The Hierarchical Unified Graph Representation (HUGR, pronounced _hugger_) is the
common representation of quantum circuits and operations in the Quantinuum
ecosystem.

It provides a high-fidelity representation of operations, that facilitates
compilation and encodes runnable programs.

The HUGR specification is [here](https://quantinuum.github.io/hugr/specification/index.html).

This crate provides compilation passes that act on HUGR programs.

## Usage

Add the dependency to your project:

```bash
cargo add hugr-passes
```

Please read the [API documentation here][].

## Recent Changes

See [CHANGELOG][] for a list of changes. The minimum supported rust
version will only change on major releases.

## Development

See [DEVELOPMENT.md](https://github.com/quantinuum/tket2/blob/main/DEVELOPMENT.md) for instructions on setting up the development environment.

## License

This project is licensed under Apache License, Version 2.0 ([LICENSE][] or http://www.apache.org/licenses/LICENSE-2.0).

  [API documentation here]: https://docs.rs/hugr-passes/
  [build_status]: https://github.com/quantinuum/tket2/actions/workflows/ci.yml/badge.svg?branch=main
  [msrv]: https://img.shields.io/crates/msrv/hugr-passes
  [crates]: https://img.shields.io/crates/v/hugr-passes
  [codecov]: https://img.shields.io/codecov/c/gh/quantinuum/tket2?logo=codecov
  [LICENSE]: https://github.com/quantinuum/tket2/blob/main/LICENCE
  [CHANGELOG]: https://github.com/quantinuum/tket2/blob/main/hugr-passes/CHANGELOG.md
