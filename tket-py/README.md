# tket

[![pypi][]](https://pypi.org/project/tket/)
[![codecov][]](https://codecov.io/gh/quantinuum/tket2)
[![py-version][]](https://pypi.org/project/tket/)

  [codecov]: https://img.shields.io/codecov/c/gh/quantinuum/tket2?logo=codecov
  [py-version]: https://img.shields.io/pypi/pyversions/tket
  [pypi]: https://img.shields.io/pypi/v/tket

TKET is an open source quantum compiler developed by Quantinuum. Central to
TKET's design is its hardware agnosticism which allows researchers and quantum
software developers to take advantage of its powerful compilation for many
different quantum architectures.

This is the Python front-end for TKET, providing a high-level interface for working with quantum circuits.
See also the Rust library with the same name on [crates.io](https://crates.io/crates/tket).


## Install

TKET can be installed via `pip`. Requires Python >= 3.10.

```sh
pip install tket
```

## Usage

See the [Getting Started][getting-started] guide and the other [examples].

  [getting-started]: https://github.com/quantinuum/tket2/blob/main/tket-py/examples/1-Getting-Started.ipynb
  [examples]: https://github.com/quantinuum/tket2/tree/main/tket-py/examples

The API documentation for `tket-py` can be found at https://quantinuum.github.io/tket2/.

## Development

This package uses [pyo3](https://pyo3.rs/latest/) and
[maturin](https://github.com/PyO3/maturin) to bind TKET functionality to
python as the `tket` package.

Recommended:

A clean python 3.10 environment with `maturin` installed. At which point running
`maturin develop` in this directory should build and install the package in the
environment. Run `pytest` in this directory to test everything is working.

Don't forget to use the `--release` flag when using Badger and other heavy
computational workloads.

See [DEVELOPMENT.md] for more information.

  [DEVELOPMENT.md]: https://github.com/quantinuum/tket2/blob/main/DEVELOPMENT.md


## License

This project is licensed under Apache License, Version 2.0 ([LICENCE][] or http://www.apache.org/licenses/LICENSE-2.0).

  [LICENCE]: https://github.com/quantinuum/tket2/blob/main/LICENCE
