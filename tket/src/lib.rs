//! TKET: The Hardware Agnostic Quantum Compiler
//!
//! TKET is an open source quantum compiler developed by Quantinuum. Central to
//! TKET's design is its hardware agnosticism which allows researchers and
//! quantum software developers to take advantage of its state of the art
//! compilation for many different quantum architectures.
//!
//! TKET circuits are represented using the HUGR IR defined in the
//! [quantinuum-hugr] crate. The [`Circuit`] trait provides a high level
//! interface for working with HUGRs representing quantum circuits, and defines
//! a HUGR extension with quantum operations.
//!
//! This crate includes a number of optimisation passes and rewrite utilities
//! for circuits, as well as interoperability with `tket1` circuits via its
//! serial encoding.
//!
//! Python bindings for TKET are available in the `tket` package on PyPi.
//!
//! # Example
//!
#![cfg_attr(not(miri), doc = "```")] // this doctest reads from the filesystem, so it fails with miri
#![cfg_attr(miri, doc = "```ignore")]
//! use tket::Circuit;
//! use tket::serialize::pytket::DecodeOptions;
//! use hugr::HugrView;
//!
//! // Load a tket1 circuit.
//! let mut circ: Circuit = tket::serialize::load_tk1_json_file("../test_files/pytket/barenco_tof_5.json", DecodeOptions::new()).unwrap();
//!
//! assert_eq!(circ.qubit_count(), 9);
//! assert_eq!(circ.num_operations(), 170);
//!
//! // Traverse the circuit and print the gates.
//! for command in circ.commands() {
//!     println!("{:?}", command.optype());
//! }
//!
//! // Render the circuit as a mermaid diagram.
//! println!("{}", circ.mermaid_string());
//!
//! // Optimise the circuit.
//! tket::passes::apply_greedy_commutation(&mut circ);
//! ```
//!
//! [quantinuum-hugr]: https://lib.rs/crates/quantinuum-hugr
#![cfg_attr(coverage_nightly, feature(coverage_attribute))]

pub mod circuit;
pub mod extension;
pub mod modifier;
pub(crate) mod ops;
pub mod optimiser;
pub mod passes;
pub mod resource;
pub mod rewrite;
pub mod serialize;
pub mod subcircuit;

#[cfg(feature = "portmatching")]
pub mod portmatching;

#[cfg(feature = "llvm")]
pub mod llvm;

mod utils;

pub use circuit::{Circuit, CircuitError, CircuitMutError};
pub use hugr;
pub use hugr::Hugr;
pub use ops::{Pauli, TketOp, op_matches, symbolic_constant_op};
