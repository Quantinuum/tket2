/// U8 encoding for Pauli X
pub const X_U8: u8 = 0;
/// U8 encoding for Pauli Y
pub const Y_U8: u8 = 1;
/// U8 encoding for Pauli Z
pub const Z_U8: u8 = 2;
/// U8 encoding for Pauli I
pub const I_U8: u8 = 3;

// U8 encoding for TQE gates
// base 3 encoding 3a+b
/// U8 encoding for TQE gate XX
pub const XX_U8: u8 = 0;
/// U8 encoding for TQE gate XY
pub const XY_U8: u8 = 1;
/// U8 encoding for TQE gate XZ
pub const XZ_U8: u8 = 2;
/// U8 encoding for TQE gate YX
pub const YX_U8: u8 = 3;
/// U8 encoding for TQE gate YY
pub const YY_U8: u8 = 4;
/// U8 encoding for TQE gate YZ
pub const YZ_U8: u8 = 5;
/// U8 encoding for TQE gate ZX
pub const ZX_U8: u8 = 6;
/// U8 encoding for TQE gate ZY
pub const ZY_U8: u8 = 7;
/// U8 encoding for TQE gate ZZ
pub const ZZ_U8: u8 = 8;

/// Converts a pair of bits (z, x) into a u8 representing the corresponding Pauli operator.
///
/// # Arguments
///
/// - `z` (`u8`) - The Z bit of the Pauli operator.
/// - `x` (`u8`) - The X bit of the Pauli operator.
///
/// # Returns
///
/// - `u8` - The U8 encoding of the corresponding Pauli operator.
///
#[inline(always)]
pub fn bits_to_u8_pauli(z: u8, x: u8) -> u8 {
    BITS_TO_U8_PAULI[z as usize * 2 + x as usize]
}

// 2*zbit + xbit
const BITS_TO_U8_PAULI: [u8; 4] = [
    I_U8, // 00: I
    X_U8, // 01: X
    Z_U8, // 10: Z
    Y_U8, // 11: Y
];

/// Converts a U8 encoding of a TQE gate into a tuple of four u64 values.
///
/// # Arguments
///
/// - `gate` (`u8`) - The U8 encoding of the TQE gate.
///
/// # Returns
///
/// - `(u64, u64, u64, u64)` - The tuple of four u64 values representing the TQE gate.
///
#[inline(always)]
pub fn u8_tqe_to_u64(gate: u8) -> (u64, u64, u64, u64) {
    TQE_TO_U64[gate as usize]
}
const TQE_TO_U64: [(u64, u64, u64, u64); 9] = [
    // XX
    (0, u64::MAX, 0, u64::MAX),
    // XY
    (0, u64::MAX, u64::MAX, u64::MAX),
    // XZ
    (0, u64::MAX, u64::MAX, 0),
    // YX
    (u64::MAX, u64::MAX, 0, u64::MAX),
    // YY
    (u64::MAX, u64::MAX, u64::MAX, u64::MAX),
    // YZ
    (u64::MAX, u64::MAX, u64::MAX, 0),
    // ZX
    (u64::MAX, 0, 0, u64::MAX),
    // ZY
    (u64::MAX, 0, u64::MAX, u64::MAX),
    // ZZ
    (u64::MAX, 0, u64::MAX, 0),
];
