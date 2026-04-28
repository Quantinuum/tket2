//! HUGR extension for logical operations on the
//! [Iceberg code](https://arxiv.org/abs/2211.06703).
//!
//! The extension `tket.qec.iceberg.types` provides one new type: this is the
//! code block for the [[k+2, k, 2]] Iceberg code (parametrized by an even
//! integer k >= 2). This is a linear (non-copyable) type.
//!
//! ```
//! use tket_qec::iceberg::types::block_type;
//!
//! let block = block_type(6);
//! assert!(!block.copyable());
//! ```
//!
//! The extension `tket.qec.iceberg.ops` provides operations that act on this
//! block type.
//!
//! To allocate a new block, with all (logical) qubits initialized to zero, use
//! an `alloc_zero` operation. This is parametrized by k (the number of logical
//! qubits):
//!
//! ```
//! use tket_qec::iceberg::ops::EXTENSION;
//!
//! let alloczero = EXTENSION
//!     .instantiate_extension_op("alloc_zero", [8.into()])
//!     .unwrap();
//! ```
//!
//! Use the `free` operation to free a previously allocated block.
//!
//! Some of the operations (such as `all_x` or `cx_transverse`) operate purely
//! at the block level. Others (such as `x` or `cx`) take one or more indices
//! as parameters: these are (64-bit unsigned) integers less than `k`, which
//! address logical qubits within the block. Some operations also take an angle,
//! represented as a float in radians.
//!
//! All operations that take indices come in two versions, "static" and
//! "dynamic", depending on whether the indices are parameters to the operation
//! itself (in which case they must be statically known) or dynamic (in which
//! case they are integer inputs to the operation). The dynamic versions have
//! names ending in `_d`. For example, `x_d` takes a block and an integer as
//! inputs, and outputs the block. Note that whereas the static versions are
//! infallible (validity of indices is checked on construction), the dynamic
//! versions will panic if the indices are invalid. When frontends generate
//! dynamic operations compilers should attempt to transform them to static
//! operations where possible.

pub mod ops;
pub mod types;
