use serde::{Deserialize, Serialize};

/// The supported gate types that can appear in a Pauli graph.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq, Eq, Hash)]
pub enum GateType {
    // TQE gates
    /// An $X$-controlled $X$ gate, i.e. H(0);CX(0,1);H(0)
    XX,
    /// An $X$-controlled $Y$ gate, i.e. H(0);CY(0,1);H(0)
    XY,
    /// An $X$-controlled $Z$ gate, i.e. CX(1,0)
    XZ,
    /// A $Y$-controlled $X$ gate, i.e. V(0);CX(0,1);Vdg(0)
    YX,
    /// A $Y$-controlled $Y$ gate, i.e. V(0);CY(0,1);Vdg(0)
    YY,
    /// A $Y$-controlled $Z$ gate, i.e. CY(1,0)
    YZ,
    /// A $Z$-controlled $X$ gate, i.e. CX(0,1)
    ZX,
    /// A $Z$-controlled $Y$ gate, i.e. CY(0,1)
    ZY,
    /// A $Z$-controlled $Z$ gate, i.e. CZ(0,1)
    ZZ,
    // Single-qubit Clifford gates
    /// The Hadamard gate.
    H,
    /// The phase gate.
    S,
    /// The inverse phase gate.
    Sdg,
    /// The square-root-of-$X$ gate.
    V,
    /// The inverse square-root-of-$X$ gate.
    Vdg,
    /// The Pauli-$X$ gate.
    X,
    /// The Pauli-$Y$ gate.
    Y,
    /// The Pauli-$Z$ gate.
    Z,
    // Special gates
    /// A computational-basis measurement gate.
    /// args: [qubit, cbit]
    Measure,
    /// A qubit reset gate.
    Reset,
    /// A swap gate.
    SWAP,
    /// An opaque gate with implementation-defined behaviour.
    BlackBox,
    // Rotation gates
    /// A rotation around the $X$ axis.
    RX,
    /// A rotation around the $Y$ axis.
    RY,
    /// A rotation around the $Z$ axis.
    RZ,
    /// A $ZZ$ phase gate.
    ZZPHASE,
    /// A phased-$X$ gate.
    PHASEDX,
}

/// Returns the number of arguments for a given gate type.
///
/// # Arguments
///
/// - `gate_type` (`&GateType`) - The gate type for which to determine the number of arguments.
///
/// # Returns
///
/// - `Option<usize>` - The number of arguments for the given gate type. Returns `None` if the gate type has an undefined number of arguments.
///
pub(crate) fn gate_type_n_args(gate_type: &GateType) -> Option<usize> {
    match gate_type {
        GateType::XX
        | GateType::XY
        | GateType::XZ
        | GateType::YX
        | GateType::YY
        | GateType::YZ
        | GateType::ZX
        | GateType::ZY
        | GateType::ZZ
        | GateType::SWAP
        | GateType::ZZPHASE => Some(2),
        GateType::H
        | GateType::S
        | GateType::Sdg
        | GateType::V
        | GateType::Vdg
        | GateType::X
        | GateType::Y
        | GateType::Z
        | GateType::RX
        | GateType::RY
        | GateType::RZ
        | GateType::PHASEDX => Some(1),
        GateType::Measure => Some(2),
        GateType::Reset => Some(1),
        GateType::BlackBox => None,
    }
}

/// Describes the number of parameters for a given gate type.
///
/// # Arguments
///
/// - `gate_type` (`&GateType`) - The gate type for which to determine the number of parameters.
///
/// # Returns
///
/// - `usize` - The number of parameters for the given gate type.
///
pub(crate) fn gate_type_n_params(gate_type: &GateType) -> usize {
    match gate_type {
        GateType::XX
        | GateType::XY
        | GateType::XZ
        | GateType::YX
        | GateType::YY
        | GateType::YZ
        | GateType::ZX
        | GateType::ZY
        | GateType::ZZ
        | GateType::SWAP
        | GateType::H
        | GateType::S
        | GateType::Sdg
        | GateType::V
        | GateType::Vdg
        | GateType::X
        | GateType::Y
        | GateType::Z => 0,
        GateType::RX | GateType::RY | GateType::RZ | GateType::ZZPHASE => 1,
        GateType::PHASEDX => 2,
        GateType::Measure => 0,
        GateType::Reset => 0,
        GateType::BlackBox => 0,
    }
}
