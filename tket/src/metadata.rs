//! Collection of metadata keys used throughout tket.

use crate::rewrite::trace::RewriteTrace;
use hugr_core::metadata::Metadata;
use tket_json_rs::register::{Bit, Qubit};

/// Metadata key for the number of qubits that a HUGR node expects to be required for execution.
///
/// This value is only valid when set at the entrypoint function node. TODO discuss this
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct MaxQubits;
impl Metadata for MaxQubits {
    const KEY: &'static str = "TKET.expected_qubits"; // TODO think about this name really hard
    type Type<'hugr> = u32;
}

/// Metadata key for traced rewrites that were applied during circuit transformation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CircuitRewriteTraces;
impl Metadata for CircuitRewriteTraces {
    const KEY: &'static str = "TKET.rewrites";
    type Type<'hugr> = Vec<RewriteTrace>;
}

/// Metadata key for flagging unitarity constraints on a HUGR node
///
/// See crate::modifier::ModifierFlags TODO discuss this
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Unitary;
impl Metadata for Unitary {
    const KEY: &'static str = "TKET.unitary"; // TODO think about this name really hard
    type Type<'hugr> = u8;
}

// Metadata keys used for TKET1 compatibility

/// Metadata key for explicit names for the input parameter wires.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct InputParameters;
impl Metadata for InputParameters {
    const KEY: &'static str = "TKET1.input_parameters";
    type Type<'hugr> = Vec<String>;
}

/// Metadata key for a tket1 operation "opgroup" field.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct OpGroup;
impl Metadata for OpGroup {
    const KEY: &'static str = "TKET1.opgroup";
    type Type<'hugr> = &'hugr str;
}

/// Metadata key for explicit names for the input bit registers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BitRegisters;
impl Metadata for BitRegisters {
    const KEY: &'static str = "TKET1.bit_registers";
    type Type<'hugr> = Vec<Bit>;
}

/// Metadata key for explicit names for the input qubit registers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct QubitRegisters;
impl Metadata for QubitRegisters {
    const KEY: &'static str = "TKET1.qubit_registers";
    type Type<'hugr> = Vec<Qubit>;
}

/// Metadata key for the global phase
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Phase;
impl Metadata for Phase {
    const KEY: &'static str = "TKET1.phase";
    type Type<'hugr> = &'hugr str;
}
