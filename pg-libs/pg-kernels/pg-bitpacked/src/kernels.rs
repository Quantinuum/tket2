use std::ops::{BitAnd, BitOr, BitXor, Not};

/// A trait for types that support bitwise operations and can be used in the conjugation functions.
pub trait BitOps:
    BitAnd<Output = Self>
    + BitXor<Output = Self>
    + BitOr<Output = Self>
    + Not<Output = Self>
    + Copy
    + Sized
{
}

// Blanket impl — anything satisfying all bounds gets BitOps for free
impl<T> BitOps for T where
    T: BitAnd<Output = T> + BitXor<Output = T> + BitOr<Output = T> + Not<Output = T> + Copy
{
}

use crate::u8_encodings::{XX_U8, XY_U8, XZ_U8, YX_U8, YY_U8, YZ_U8, ZX_U8, ZY_U8, ZZ_U8};

// =============================================================================
//        Clifford conjugation functions on bit-packed Pauli operators
// =============================================================================

/// Apply X gate (X P X) on bit-packed Pauli operators.
///
/// This function transforms Pauli operators:
/// - X -> X
/// - Y -> -Y
/// - Z -> -Z
///
/// # Arguments
///
/// * `z` - A `T` containing the Z component bits of the packed Pauli operators.
/// * `x` - A `T` containing the X component bits of the packed Pauli operators.
///
/// # Returns
/// The sign flip bits.
///
#[inline(always)]
pub fn x_gate<T: BitOps>(z: T, _x: T) -> T {
    // flip the sign if the letter is Z or Y
    z
}

/// Apply Y gate (Y P Y) on bit-packed Pauli operators.
///
/// This function transforms Pauli operators:
/// - X -> -X
/// - Y -> Y
/// - Z -> -Z
///
/// # Arguments
///
/// * `z` - A `T` containing the Z component bits of the packed Pauli operators.
/// * `x` - A `T` containing the X component bits of the packed Pauli operators.
///
/// # Returns
/// The sign flip bits.
///
#[inline(always)]
pub fn y_gate<T: BitOps>(z: T, x: T) -> T {
    // flip the sign if the letter is X or Z
    z ^ x
}

/// Apply Z gate (Z P Z) on bit-packed Pauli operators.
///
/// This function transforms Pauli operators:
/// - X -> -X
/// - Y -> -Y
/// - Z -> Z
///
/// # Arguments
///
/// * `z` - A `T` containing the Z component bits of the packed Pauli operators.
/// * `x` - A `T` containing the X component bits of the packed Pauli operators.
///
/// # Returns
/// The sign flip bits.
///
#[inline(always)]
pub fn z_gate<T: BitOps>(_z: T, x: T) -> T {
    // flip the sign if the letter is X or Y
    x
}

/// Apply H gate (H P H) on bit-packed Pauli operators.
///
/// This function transforms Pauli operators:
/// - X -> Z
/// - Z -> X
/// - Y -> -Y
///
/// # Arguments
///
/// * `z` - A `T` containing the Z component bits of the packed Pauli operators.
/// * `x` - A `T` containing the X component bits of the packed Pauli operators.
///
/// # Returns
///
/// A tuple containing:
/// - `T`: The new Z component bits.
/// - `T`: The new X component bits.
/// - `T`: The sign flip bits.
///
#[inline(always)]
pub fn h_gate<T: BitOps>(z: T, x: T) -> (T, T, T) {
    // swap bits
    // sign flip if both bits are 1
    (x, z, z & x)
}

/// Apply Sdg gate (Sdg P S) on bit-packed Pauli operators.
///
/// This function transforms Pauli operators:
/// - X -> -Y
/// - Y -> X
/// - Z -> Z
///
/// # Arguments
///
/// * `z` - A `T` containing the Z component bits of the packed Pauli operators.
/// * `x` - A `T` containing the X component bits of the packed Pauli operators.
///
/// # Returns
///
/// A tuple containing:
/// - `T`: The new Z component bits.
/// - `T`: The new X component bits.
/// - `T`: The sign flip bits.
///
#[inline(always)]
pub fn sdg_gate<T: BitOps>(z: T, x: T) -> (T, T, T) {
    // toggle z bits with x bits
    // sign flip if z bit is 0 and x bit is 1
    (z ^ x, x, !z & x)
}

/// Apply S gate (S P Sdg) on bit-packed Pauli operators.
///
/// This function transforms Pauli operators:
/// - X -> Y
/// - Y -> -X
/// - Z -> Z
///
/// # Arguments
///
/// * `z` - A `T` containing the Z component bits of the packed Pauli operators.
/// * `x` - A `T` containing the X component bits of the packed Pauli operators.
///
/// # Returns
///
/// A tuple containing:
/// - `T`: The new Z component bits.
/// - `T`: The new X component bits.
/// - `T`: The sign flip bits.
///
#[inline(always)]
pub fn s_gate<T: BitOps>(z: T, x: T) -> (T, T, T) {
    // toggle z bits with x bits
    (z ^ x, x, z & x)
}

/// Apply Vdg gate (Vdg P V) on bit-packed Pauli operators.
///
/// This function transforms Pauli operators:
/// - X -> X
/// - Y -> -Z
/// - Z -> Y
///
/// # Arguments
///
/// * `z` - A `T` containing the Z component bits of the packed Pauli operators.
/// * `x` - A `T` containing the X component bits of the packed Pauli operators.
///
/// # Returns
///
/// A tuple containing:
/// - `T`: The new Z component bits.
/// - `T`: The new X component bits.
/// - `T`: The sign flip bits.
///
#[inline(always)]
pub fn vdg_gate<T: BitOps>(z: T, x: T) -> (T, T, T) {
    // toggle x bits with z bits
    // sign flip if both bits are 1
    (z, x ^ z, z & x)
}

/// Apply V gate (V P Vdg) on bit-packed Pauli operators.
///
/// This function transforms Pauli operators:
/// - X -> X
/// - Y -> Z
/// - Z -> -Y
///
/// # Arguments
///
/// * `z` - A `T` containing the Z component bits of the packed Pauli operators.
/// * `x` - A `T` containing the X component bits of the packed Pauli operators.
///
/// # Returns
///
/// A tuple containing:
/// - `T`: The new Z component bits.
/// - `T`: The new X component bits.
/// - `T`: The sign flip bits.
///
#[inline(always)]
pub fn v_gate<T: BitOps>(z: T, x: T) -> (T, T, T) {
    // toggle x bits with z bits
    (z, x ^ z, z & !x)
}

/// Apply Ry(-pi/2) gate (Ry(-pi/2) P Ry(pi/2)) on bit-packed Pauli operators.
///
/// This function transforms Pauli operators:
/// - X -> Z
/// - Y -> Y
/// - Z -> -X
///
/// # Arguments
///
/// * `z` - A `T` containing the Z component bits of the packed Pauli operators.
/// * `x` - A `T` containing the X component bits of the packed Pauli operators.
///
/// # Returns
///
/// A tuple containing:
/// - `T`: The new Z component bits.
/// - `T`: The new X component bits.
/// - `T`: The sign flip bits.
///
#[inline(always)]
pub fn ry270_gate<T: BitOps>(z: T, x: T) -> (T, T, T) {
    // swap bits
    (x, z, z & !x)
}

/// Apply Ry(pi/2) gate (Ry(pi/2) P Ry(-pi/2)) on bit-packed Pauli operators.
///
/// This function transforms Pauli operators:
/// - X -> -Z
/// - Y -> Y
/// - Z -> X
///
/// # Arguments
///
/// * `z` - A `T` containing the Z component bits of the packed Pauli operators.
/// * `x` - A `T` containing the X component bits of the packed Pauli operators.
///
/// # Returns
///
/// A tuple containing:
/// - `T`: The new Z component bits.
/// - `T`: The new X component bits.
/// - `T`: The sign flip bits.
///
#[inline(always)]
pub fn ry90_gate<T: BitOps>(z: T, x: T) -> (T, T, T) {
    // swap bits
    (x, z, !z & x)
}

/// Apply Rz(-pi/2) gate (Rz(-pi/2) P Rz(pi/2)) on bit-packed Pauli operators. Equivalent to S gate.
#[inline(always)]
pub fn rz270_gate<T: BitOps>(z: T, x: T) -> (T, T, T) {
    sdg_gate(z, x)
}

/// Apply Rz(pi/2) gate (Rz(pi/2) P Rz(-pi/2)) on bit-packed Pauli operators. Equivalent to Sdg gate.
#[inline(always)]
pub fn rz90_gate<T: BitOps>(z: T, x: T) -> (T, T, T) {
    s_gate(z, x)
}

/// Apply Rx(-pi/2) gate (Rx(-pi/2) P Rx(pi/2)) on bit-packed Pauli operators. Equivalent to V gate.
#[inline(always)]
pub fn rx270_gate<T: BitOps>(z: T, x: T) -> (T, T, T) {
    vdg_gate(z, x)
}

/// Apply Rx(pi/2) gate (Rx(pi/2) P Rx(-pi/2)) on bit-packed Pauli operators. Equivalent to Vdg gate.
#[inline(always)]
pub fn rx90_gate<T: BitOps>(z: T, x: T) -> (T, T, T) {
    v_gate(z, x)
}

/// Apply XX gate (XX P XX) on bit-packed Pauli operators.
///
/// # Arguments
///
/// * `z0` - A `T` containing the Z component bits of the packed Pauli operators
///   on the first qubit.
/// * `x0` - A `T` containing the X component bits of the packed Pauli operators
///   on the first qubit.
/// * `z1` - A `T` containing the Z component bits of the packed Pauli operators
///   on the second qubit.
/// * `x1` - A `T` containing the X component bits of the packed Pauli operators
///   on the second qubit.
///
/// # Returns
///
/// A tuple containing:
/// - `T`: The new Z component bits on the first qubit.
/// - `T`: The new X component bits on the first qubit.
/// - `T`: The new Z component bits on the second qubit.
/// - `T`: The new X component bits on the second qubit.
/// - `T`: The sign flip bits.
///
#[inline(always)]
pub fn xx_gate<T: BitOps>(z0: T, x0: T, z1: T, x1: T) -> (T, T, T, T, T) {
    // we flip sign only for YZ, and ZY
    // (z0&x0&z1&!x1)|(z0&!x0&z1&x1) = z0&z1&(x0^x1)
    (z0, x0 ^ z1, z1, x1 ^ z0, z0 & z1 & (x0 ^ x1))
}

/// Apply XY (XY P XY) gate on bit-packed Pauli operators.
///
/// # Arguments
///
/// * `z0` - A `T` containing the Z component bits of the packed Pauli operators
///   on the first qubit.
/// * `x0` - A `T` containing the X component bits of the packed Pauli operators
///   on the first qubit.
/// * `z1` - A `T` containing the Z component bits of the packed Pauli operators
///   on the second qubit.
/// * `x1` - A `T` containing the X component bits of the packed Pauli operators
///   on the second qubit.
///
/// # Returns
///
/// A tuple containing:
/// - `T`: The new Z component bits on the first qubit.
/// - `T`: The new X component bits on the first qubit.
/// - `T`: The new Z component bits on the second qubit.
/// - `T`: The new X component bits on the second qubit.
/// - `T`: The sign flip bits.
///
#[inline(always)]
pub fn xy_gate<T: BitOps>(z0: T, x0: T, z1: T, x1: T) -> (T, T, T, T, T) {
    // we flip sign only for YX, and ZZ
    // (z0&x0&!z1&x1)|(z0&!x0&z1&!x1) = z0&(x0==x1&x0!=z1)
    (
        z0,
        x0 ^ z1 ^ x1,
        z1 ^ z0,
        x1 ^ z0,
        z0 & !(x0 ^ x1) & (x0 ^ z1),
    )
}

/// Apply XZ (XZ P XZ) gate on bit-packed Pauli operators.
///
/// # Arguments
///
/// * `z0` - A `T` containing the Z component bits of the packed Pauli operators
///   on the first qubit.
/// * `x0` - A `T` containing the X component bits of the packed Pauli operators
///   on the first qubit.
/// * `z1` - A `T` containing the Z component bits of the packed Pauli operators
///   on the second qubit.
/// * `x1` - A `T` containing the X component bits of the packed Pauli operators
///   on the second qubit.
///
/// # Returns
///
/// A tuple containing:
/// - `T`: The new Z component bits on the first qubit.
/// - `T`: The new X component bits on the first qubit.
/// - `T`: The new Z component bits on the second qubit.
/// - `T`: The new X component bits on the second qubit.
/// - `T`: The sign flip bits.
///
#[inline(always)]
pub fn xz_gate<T: BitOps>(z0: T, x0: T, z1: T, x1: T) -> (T, T, T, T, T) {
    // we flip sign only for YY, and ZX
    (z0, x0 ^ x1, z1 ^ z0, x1, z0 & x1 & !(x0 ^ z1))
}

/// Apply YY (YY P YY) gate on bit-packed Pauli operators.
///
/// # Arguments
///
/// * `z0` - A `T` containing the Z component bits of the packed Pauli operators
///   on the first qubit.
/// * `x0` - A `T` containing the X component bits of the packed Pauli operators
///   on the first qubit.
/// * `z1` - A `T` containing the Z component bits of the packed Pauli operators
///   on the second qubit.
/// * `x1` - A `T` containing the X component bits of the packed Pauli operators
///   on the second qubit.
///
/// # Returns
///
/// A tuple containing:
/// - `T`: The new Z component bits on the first qubit.
/// - `T`: The new X component bits on the first qubit.
/// - `T`: The new Z component bits on the second qubit.
/// - `T`: The new X component bits on the second qubit.
/// - `T`: The sign flip bits.
///
#[inline(always)]
pub fn yy_gate<T: BitOps>(z0: T, x0: T, z1: T, x1: T) -> (T, T, T, T, T) {
    // toggle both bits, if the other two bits has parity 1
    // we flip sign only for XZ, and ZX
    // z0!=x0 & z1!=x1 & x0==z1
    let toggle0 = z1 ^ x1;
    let toggle1 = z0 ^ x0;
    (
        z0 ^ toggle0,
        x0 ^ toggle0,
        z1 ^ toggle1,
        x1 ^ toggle1,
        toggle1 & toggle0 & !(x0 ^ z1),
    )
}

/// Apply YZ (YZ P YZ) gate on bit-packed Pauli operators.
///
/// # Arguments
///
/// * `z0` - A `T` containing the Z component bits of the packed Pauli operators
///   on the first qubit.
/// * `x0` - A `T` containing the X component bits of the packed Pauli operators
///   on the first qubit.
/// * `z1` - A `T` containing the Z component bits of the packed Pauli operators
///   on the second qubit.
/// * `x1` - A `T` containing the X component bits of the packed Pauli operators
///   on the second qubit.
///
/// # Returns
///
/// A tuple containing:
/// - `T`: The new Z component bits on the first qubit.
/// - `T`: The new X component bits on the first qubit.
/// - `T`: The new Z component bits on the second qubit.
/// - `T`: The new X component bits on the second qubit.
/// - `T`: The sign flip bits.
///
#[inline(always)]
pub fn yz_gate<T: BitOps>(z0: T, x0: T, z1: T, x1: T) -> (T, T, T, T, T) {
    // we flip sign only for ZY, and XX
    // x1 & z0==z1 & x0!=z0
    (
        z0 ^ x1,
        x0 ^ x1,
        z1 ^ z0 ^ x0,
        x1,
        x1 & !(z0 ^ z1) & (x0 ^ z0),
    )
}

/// Apply ZZ (ZZ P ZZ) gate on bit-packed Pauli operators.
///
/// # Arguments
///
/// * `z0` - A `T` containing the Z component bits of the packed Pauli operators
///   on the first qubit.
/// * `x0` - A `T` containing the X component bits of the packed Pauli operators
///   on the first qubit.
/// * `z1` - A `T` containing the Z component bits of the packed Pauli operators
///   on the second qubit.
/// * `x1` - A `T` containing the X component bits of the packed Pauli operators
///   on the second qubit.
///
/// # Returns
///
/// A tuple containing:
/// - `T`: The new Z component bits on the first qubit.
/// - `T`: The new X component bits on the first qubit.
/// - `T`: The new Z component bits on the second qubit.
/// - `T`: The new X component bits on the second qubit.
/// - `T`: The sign flip bits.
///
#[inline(always)]
pub fn zz_gate<T: BitOps>(z0: T, x0: T, z1: T, x1: T) -> (T, T, T, T, T) {
    // we flip sign only for XY, and YX
    // x0&x1&z0!=z1
    (z0 ^ x1, x0, z1 ^ x0, x1, x0 & x1 & (z0 ^ z1))
}

/// Apply TQE gate where the gate type is base-3 encoded in a u8.
/// The encoding is as follows:
/// - 0: XX
/// - 1: XY
/// - 2: XZ
/// - 3: YX
/// - 4: YY
/// - 5: YZ
/// - 6: ZX
/// - 7: ZY
/// - 8: ZZ
///
/// # Arguments
///
/// - `z0` (`&mut T`) - z bits on the first qubit.
/// - `x0` (`&mut T`) - x bits on the first qubit.
/// - `z1` (`&mut T`) - z bits on the second qubit.
/// - `x1` (`&mut T`) - x bits on the second qubit.
/// - `signs` (`&mut T`) - sign bits.
/// - `mask` (`T`) - mask to apply the gate.
/// - `gate` (`u8`) - gate type encoded in base-3.
///
pub fn apply_u8_tqe<T: BitOps>(
    z0: &mut T,
    x0: &mut T,
    z1: &mut T,
    x1: &mut T,
    signs: &mut T,
    mask: T,
    gate: u8,
) {
    let (new_z0, new_x0, new_z1, new_x1, sign_flips) = match gate {
        XX_U8 => xx_gate(*z0, *x0, *z1, *x1),
        XY_U8 => xy_gate(*z0, *x0, *z1, *x1),
        XZ_U8 => xz_gate(*z0, *x0, *z1, *x1),
        YX_U8 => {
            let (a, b, c, d, e) = xy_gate(*z1, *x1, *z0, *x0);
            (c, d, a, b, e)
        }
        YY_U8 => yy_gate(*z0, *x0, *z1, *x1),
        YZ_U8 => yz_gate(*z0, *x0, *z1, *x1),
        ZX_U8 => {
            let (a, b, c, d, e) = xz_gate(*z1, *x1, *z0, *x0);
            (c, d, a, b, e)
        }
        ZY_U8 => {
            let (a, b, c, d, e) = yz_gate(*z1, *x1, *z0, *x0);
            (c, d, a, b, e)
        }
        ZZ_U8 => zz_gate(*z0, *x0, *z1, *x1),
        _ => panic!("Invalid TQE gate"),
    };

    let new_signs = sign_flips ^ *signs;
    let mask_ng = !mask;
    *z0 = (*z0 & mask_ng) | (new_z0 & mask);
    *x0 = (*x0 & mask_ng) | (new_x0 & mask);
    *z1 = (*z1 & mask_ng) | (new_z1 & mask);
    *x1 = (*x1 & mask_ng) | (new_x1 & mask);
    *signs = (*signs & mask_ng) | (new_signs & mask);
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::test_utils::*;
    use pg_core::{GateData, GateType, Op, Pauli, PauliGraph, RotationData};
    use pg_tk::compare_unitaries_via_tk;

    type SqGateFn = fn(bool, bool) -> (bool, bool, bool);
    type PauliGateFn = fn(bool, bool) -> bool;
    type TqeGateFn = fn(bool, bool, bool, bool) -> (bool, bool, bool, bool, bool);

    pub(crate) const SQ_GATES: [&str; 11] = [
        "H", "S", "SDG", "V", "VDG", "RX90", "RX270", "RY90", "RY270", "RZ90", "RZ270",
    ];
    pub(crate) const PAULI_GATES: [&str; 3] = ["X", "Y", "Z"];
    pub(crate) const TQ_GATES: [&str; 6] = ["XX", "XY", "XZ", "YY", "YZ", "ZZ"];

    pub(crate) fn sq_gate_fn(name: &str) -> SqGateFn {
        match name {
            "H" => h_gate,
            "S" => s_gate,
            "SDG" => sdg_gate,
            "V" => v_gate,
            "VDG" => vdg_gate,
            "RX90" => rx90_gate,
            "RX270" => rx270_gate,
            "RY90" => ry90_gate,
            "RY270" => ry270_gate,
            "RZ90" => rz90_gate,
            "RZ270" => rz270_gate,
            _ => panic!("Invalid gate name"),
        }
    }

    pub(crate) fn pauli_gate_fn(name: &str) -> PauliGateFn {
        match name {
            "X" => x_gate,
            "Y" => y_gate,
            "Z" => z_gate,
            _ => panic!("Invalid gate name"),
        }
    }

    pub(crate) fn tq_gate_fn(name: &str) -> TqeGateFn {
        match name {
            "XX" => xx_gate,
            "XY" => xy_gate,
            "XZ" => xz_gate,
            "YY" => yy_gate,
            "YZ" => yz_gate,
            "ZZ" => zz_gate,
            _ => panic!("Invalid gate name"),
        }
    }

    fn get_gate(name: &str) -> Op {
        let gate_data = match name {
            "H" => GateData::new(GateType::H, vec![0]),
            "S" => GateData::new(GateType::S, vec![0]),
            "SDG" => GateData::new(GateType::Sdg, vec![0]),
            "V" => GateData::new(GateType::V, vec![0]),
            "VDG" => GateData::new(GateType::Vdg, vec![0]),
            "RX90" => GateData::new(GateType::RX, vec![0]).with_params(vec![0.5]),
            "RX270" => GateData::new(GateType::RX, vec![0]).with_params(vec![1.5]),
            "RY90" => GateData::new(GateType::RY, vec![0]).with_params(vec![0.5]),
            "RY270" => GateData::new(GateType::RY, vec![0]).with_params(vec![1.5]),
            "RZ90" => GateData::new(GateType::RZ, vec![0]).with_params(vec![0.5]),
            "RZ270" => GateData::new(GateType::RZ, vec![0]).with_params(vec![1.5]),
            "X" => GateData::new(GateType::X, vec![0]),
            "Y" => GateData::new(GateType::Y, vec![0]),
            "Z" => GateData::new(GateType::Z, vec![0]),
            "XX" => GateData::new(GateType::XX, vec![0, 1]),
            "XY" => GateData::new(GateType::XY, vec![0, 1]),
            "XZ" => GateData::new(GateType::XZ, vec![0, 1]),
            "YY" => GateData::new(GateType::YY, vec![0, 1]),
            "YZ" => GateData::new(GateType::YZ, vec![0, 1]),
            "ZZ" => GateData::new(GateType::ZZ, vec![0, 1]),
            _ => panic!("Invalid gate name"),
        };
        Op::Gate { data: gate_data }
    }

    #[test]
    fn test_sq_gates() {
        for gate in SQ_GATES.iter() {
            let gate_fn = sq_gate_fn(gate);
            for p in [Pauli::X, Pauli::Y, Pauli::Z].iter() {
                let (z, x) = pauli_to_bits(p);
                let (new_z, new_x, sign_flip) = gate_fn(z, x);
                let p_prime = bits_to_pauli(new_z, new_x);
                let cliff = get_gate(gate);
                let p_exp = Op::Rotation {
                    data: RotationData::new(vec![*p], 1.0),
                };
                let new_p_exp = Op::Rotation {
                    data: RotationData::new(vec![p_prime], if sign_flip { -1.0 } else { 1.0 }),
                };
                let pg = PauliGraph::new(1).with_ops(vec![p_exp, cliff.clone()]);
                let pg1 = PauliGraph::new(1).with_ops(vec![cliff, new_p_exp]);
                assert!(compare_unitaries_via_tk(&pg, &pg1));
            }
        }
    }

    #[test]
    fn test_pauli_gates() {
        for gate in PAULI_GATES.iter() {
            let gate_fn = pauli_gate_fn(gate);
            for p in [Pauli::X, Pauli::Y, Pauli::Z].iter() {
                let (z, x) = pauli_to_bits(p);
                let sign_flip = gate_fn(z, x);
                let cliff = get_gate(gate);
                let p_exp = Op::Rotation {
                    data: RotationData::new(vec![*p], 1.0),
                };
                let new_p_exp = Op::Rotation {
                    data: RotationData::new(vec![*p], if sign_flip { -1.0 } else { 1.0 }),
                };
                let pg = PauliGraph::new(1).with_ops(vec![p_exp, cliff.clone()]);
                let pg1 = PauliGraph::new(1).with_ops(vec![cliff, new_p_exp]);
                assert!(compare_unitaries_via_tk(&pg, &pg1));
            }
        }
    }

    #[test]
    fn test_tq_gates() {
        for gate in TQ_GATES.iter() {
            let gate_fn = tq_gate_fn(gate);
            for p0 in [Pauli::X, Pauli::Y, Pauli::Z].iter() {
                for p1 in [Pauli::X, Pauli::Y, Pauli::Z].iter() {
                    let (z0, x0) = pauli_to_bits(p0);
                    let (z1, x1) = pauli_to_bits(p1);
                    let (new_z0, new_x0, new_z1, new_x1, sign_flip) = gate_fn(z0, x0, z1, x1);
                    let p0_prime = bits_to_pauli(new_z0, new_x0);
                    let p1_prime = bits_to_pauli(new_z1, new_x1);
                    let cliff = get_gate(gate);
                    let p_exp = Op::Rotation {
                        data: RotationData::new(vec![*p0, *p1], 1.0),
                    };
                    let new_p_exp = Op::Rotation {
                        data: RotationData::new(
                            vec![p0_prime, p1_prime],
                            if sign_flip { -1.0 } else { 1.0 },
                        ),
                    };
                    let pg = PauliGraph::new(2).with_ops(vec![p_exp, cliff.clone()]);
                    let pg1 = PauliGraph::new(2).with_ops(vec![cliff, new_p_exp]);
                    assert!(compare_unitaries_via_tk(&pg, &pg1));
                }
            }
        }
    }
}
