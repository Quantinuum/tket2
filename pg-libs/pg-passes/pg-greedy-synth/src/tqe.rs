/// A Pauli encoded as a `u8`.
pub(crate) type PauliU8 = u8;
/// A TQE gate type encoded as a `u8`.
pub(crate) type TQEType = u8;

/// A TQE gate acting on an ordered pair of qubits.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) struct TQE {
    pub(crate) gate_type: TQEType,
    pub(crate) q0: usize,
    pub(crate) q1: usize,
}
