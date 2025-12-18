//! (Incomplete) Collection of metadata keys used throughout tket.

use hugr_core::metadata::Metadata;

/// Metadata key for the number of qubits that a HUGR node expects to be required for execution.
///
/// This value is only valid when set at the entrypoint function node. TODO discuss this
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct MaxQubits;
impl Metadata for MaxQubits {
    const KEY: &'static str = "tket.hint.max_qubits";
    type Type<'hugr> = u32; // TODO think about this name really hard
}
