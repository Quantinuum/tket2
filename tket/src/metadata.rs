//! Collection of metadata keys used throughout tket.
//!
//! # Example
//!
//! ```rust
//! use tket::metadata;
//! use hugr::{Hugr, HugrView};
//! # use hugr::hugr::hugrmut::HugrMut;
//! # use hugr::types::Signature;
//! # use tket_json_rs::register::{ElementId, Qubit};
//!
//! let mut hugr = Hugr::new();
//! let node = hugr.entrypoint();
//!
//! hugr.set_metadata::<metadata::MaxQubitsHint>(node, 3);
//! hugr.set_metadata::<metadata::PytketInputParameters>(node, vec!["theta".to_string()]);
//! hugr.set_metadata::<metadata::PytketQubitRegisterNames>(
//!     node,
//!     vec![Qubit::from(ElementId("q".to_string(), vec![0]))],
//! );
//!
//! assert_eq!(hugr.get_metadata::<metadata::MaxQubitsHint>(node), Some(3));
//! assert_eq!(
//!     hugr.get_metadata::<metadata::PytketInputParameters>(node),
//!     Some(vec!["theta".to_string()]),
//! );
//! ```
//
// Changes to this file **SHOULD** be reflected in `tket-py/tket/metadata.py`.

use crate::rewrite::trace::RewriteTrace;
use hugr_core::metadata::Metadata;
use tket_json_rs::register::{Bit, Qubit};

/// Metadata key for the number of qubits that a HUGR node expects to be required for execution.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ExpectedQubitsHint;
impl Metadata for ExpectedQubitsHint {
    const KEY: &'static str = "tket.hint.expected_qubits";
    const ALIASES: &'static [&'static str] = &["tket.hint.max_qubits"];
    type Type<'hugr> = u32;
}

/// Metadata hinting the compiler that a function declaration should be inlined at its call sites.
///
/// When a function is not annotated, we use a heuristic to determine whether to inline.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
#[serde(rename_all = "snake_case")]
#[non_exhaustive]
pub enum InlineAnnotation {
    /// Inline the function if we know it won't produce an invalid Hugr.
    ///
    /// This is a best effort option; the compiler may choose not to inline
    /// functions with this annotation.
    BestEffort,
    /// Never inline the function.
    Never,
}
impl Metadata for InlineAnnotation {
    const KEY: &'static str = "tket.inline";
    type Type<'hugr> = Self;
}

/// Metadata key for traced rewrites that were applied during circuit transformation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CircuitRewriteTraces;
impl Metadata for CircuitRewriteTraces {
    const KEY: &'static str = "tket.rewrites";
    type Type<'hugr> = Vec<RewriteTrace>;
}

/// Metadata key for flagging unitarity constraints / modifiers on a HUGR node
///
/// See crate::modifier::ModifierFlags
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct UnitaryFlags;
impl Metadata for UnitaryFlags {
    const KEY: &'static str = "tket.unitary";
    const ALIASES: &'static [&'static str] = &["unitary"];
    type Type<'hugr> = u8;
}

// Metadata keys used for pytket compatibility.

/// Metadata key for explicit names for the input parameter wires.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PytketInputParameters;
impl Metadata for PytketInputParameters {
    const KEY: &'static str = "TKET1.input_parameters";
    type Type<'hugr> = Vec<String>;
}

/// Metadata key for a tket1 operation "opgroup" field.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PytketOpGroup;
impl Metadata for PytketOpGroup {
    const KEY: &'static str = "TKET1.opgroup";
    type Type<'hugr> = &'hugr str;
}

/// Metadata key for explicit names for the input bit registers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PytketBitRegisterNames;
impl Metadata for PytketBitRegisterNames {
    const KEY: &'static str = "TKET1.bit_registers";
    type Type<'hugr> = Vec<Bit>;
}

/// Metadata key for explicit names for the input qubit registers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PytketQubitRegisterNames;
impl Metadata for PytketQubitRegisterNames {
    const KEY: &'static str = "TKET1.qubit_registers";
    type Type<'hugr> = Vec<Qubit>;
}

/// Metadata key for the global phase
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PytketPhaseExpr;
impl Metadata for PytketPhaseExpr {
    const KEY: &'static str = "TKET1.phase";
    type Type<'hugr> = &'hugr str;
}

/// Deprecated alias for [`ExpectedQubitsHint`].
#[deprecated(
    since = "0.21.0",
    note = "use ExpectedQubitsHint instead; this alias will be removed"
)]
pub type MaxQubitsHint = ExpectedQubitsHint;
