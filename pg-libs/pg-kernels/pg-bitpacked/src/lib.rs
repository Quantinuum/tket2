//! This crate implements Clifford conjugation on bit-packed representations of Pauli operators.
//! The simd feature enables the use of the nightly portable_simd API.

#![cfg_attr(feature = "simd", feature(portable_simd))]

use pg_core::Pauli;

mod kernels;
mod slice;
mod u8_encodings;

pub use kernels::*;
pub use slice::*;
pub use u8_encodings::*;

#[cfg(feature = "simd")]
mod simd;
#[cfg(feature = "simd")]
pub use simd::*;

#[cfg(test)]
mod test_utils;

/// Converts a slice of boolean values into a vector of `u64` values, where each `u64`
/// represents up to 64 boolean values packed as bits.
///
/// Booleans are packed **little-endian** within each `u64`.
///
/// # Arguments
///
/// - `bits` (`&[bool]`) - A slice of boolean values to be converted.
///
/// # Returns
///
/// - `Vec<u64>` - A vector of `u64` values representing the packed boolean values.
///
/// # Example
///
/// ```
/// use pg_bitpacked::bools_to_u64_vec;
/// // indices 0 and 2 are true → bits 0 and 2 set → 0b0101 = 5
/// let bits = vec![true, false, true, false];
/// let packed = bools_to_u64_vec(&bits);
/// assert_eq!(packed, vec![0b0101_u64]);
/// ```
pub fn bools_to_u64_vec(bits: &[bool]) -> Vec<u64> {
    bits.chunks(64)
        .map(|chunk| {
            chunk
                .iter()
                .enumerate()
                .fold(0u64, |acc, (i, &b)| if b { acc | (1 << i) } else { acc })
        })
        .collect()
}

/// Converts a vector of `Pauli` values into two vectors of `u64` bit-packed representations.
///
/// Each Pauli is encoded into two bits:
/// - Z component: 0 for `I`/`X`, 1 for `Z`/`Y`
/// - X component: 0 for `I`/`Z`, 1 for `X`/`Y`
///
/// Paulis are packed **little-endian** within each `u64`.
///
/// # Arguments
///
/// * `paulis` - A slice of `Pauli` values to be encoded.
///
/// # Returns
///
/// A tuple `(zs, xs)` where:
/// - `zs`: Z component bits packed into `u64` values.
/// - `xs`: X component bits packed into `u64` values.
///
/// # Example
///
/// ```
/// use pg_core::Pauli;
/// use pg_bitpacked::paulis_to_u64s;
/// // [I, X, Y, Z] → Z bits: [0,0,1,1] = 0b1100 = 12
/// //               → X bits: [0,1,1,0] = 0b0110 = 6
/// let paulis = vec![Pauli::I, Pauli::X, Pauli::Y, Pauli::Z];
/// let (zs, xs) = paulis_to_u64s(&paulis);
/// assert_eq!(zs, vec![0b1100_u64]);
/// assert_eq!(xs, vec![0b0110_u64]);
/// ```
pub fn paulis_to_u64s(paulis: &[Pauli]) -> (Vec<u64>, Vec<u64>) {
    let n_u64s = paulis.len().div_ceil(64);
    let mut zs = Vec::with_capacity(n_u64s);
    let mut xs = Vec::with_capacity(n_u64s);
    let mut z_current: u64 = 0;
    let mut x_current: u64 = 0;
    let mut bits_filled = 0;
    for pauli in paulis {
        let z_bit = match pauli {
            Pauli::I => 0,
            Pauli::X => 0,
            Pauli::Z => 1,
            Pauli::Y => 1,
        };
        let x_bit = match pauli {
            Pauli::I => 0,
            Pauli::X => 1,
            Pauli::Z => 0,
            Pauli::Y => 1,
        };
        z_current |= (z_bit as u64) << bits_filled;
        x_current |= (x_bit as u64) << bits_filled;
        bits_filled += 1;
        if bits_filled == 64 {
            zs.push(z_current);
            xs.push(x_current);
            z_current = 0;
            x_current = 0;
            bits_filled = 0;
        }
    }
    if bits_filled > 0 {
        zs.push(z_current);
        xs.push(x_current);
    }
    (zs, xs)
}

/// Converts two vectors of `u64` bit-packed representations back into a vector of `Pauli` values.
///
/// This function is the inverse of `paulis_to_u64s`. It decodes the Z and X component bits
/// from the packed `u64` values to reconstruct the original Pauli operators.
///
/// # Arguments
///
/// * `packed_z_bits` - A reference to a vector of `u64` values containing the Z component bits.
/// * `packed_x_bits` - A reference to a vector of `u64` values containing the X component bits.
/// * `n_paulis` - The exact number of Pauli operators to decode (prevents decoding padding bits).
///
/// # Returns
///
/// A `Vec<Pauli>` containing the decoded Pauli operators.
///
/// # Example
///
/// ```
/// use pg_core::Pauli;
/// use pg_bitpacked::u64s_to_paulis;
/// let zs = vec![0b1100_u64]; // Z bits for [I, X, Y, Z]
/// let xs = vec![0b0110_u64]; // X bits for [I, X, Y, Z]
/// let paulis = vec![Pauli::I, Pauli::X, Pauli::Y, Pauli::Z];
/// let decoded = u64s_to_paulis(&zs, &xs, paulis.len());
/// assert_eq!(paulis, decoded);
/// ```
pub fn u64s_to_paulis(packed_z_bits: &[u64], packed_x_bits: &[u64], n_paulis: usize) -> Vec<Pauli> {
    let mut paulis = Vec::with_capacity(n_paulis);
    let mut decoded = 0;
    for (z_pack, x_pack) in packed_z_bits.iter().zip(packed_x_bits) {
        for i in 0..64 {
            if decoded == n_paulis {
                return paulis;
            }
            let z_bit = (z_pack >> i) & 0b1;
            let x_bit = (x_pack >> i) & 0b1;
            let pauli = match (z_bit, x_bit) {
                (0, 0) => Pauli::I,
                (0, 1) => Pauli::X,
                (1, 0) => Pauli::Z,
                (1, 1) => Pauli::Y,
                _ => unreachable!(),
            };
            paulis.push(pauli);
            decoded += 1;
        }
    }
    paulis
}

/// Converts a vector of boolean sign flip values into a bit-packed `u64` representation.
///
/// Each boolean is encoded as a single bit (`true` → 1, `false` → 0), packed
/// **little-endian** within each `u64`.
///
/// # Arguments
///
/// * `sign_flips` - A slice of `bool` values to be bit-packed.
///
/// # Returns
///
/// A `Vec<u64>` containing the bit-packed representation of the boolean values.
///
/// # Example
///
/// ```
/// use pg_bitpacked::sign_flips_to_u64s;
/// // indices 0 and 2 are true → bits 0 and 2 set → 0b0101 = 5
/// let signs = vec![true, false, true, false];
/// let packed = sign_flips_to_u64s(&signs);
/// assert_eq!(packed, vec![0b0101_u64]);
/// ```
pub fn sign_flips_to_u64s(sign_flips: &[bool]) -> Vec<u64> {
    let n_u64s = sign_flips.len().div_ceil(64);
    let mut encodings = Vec::with_capacity(n_u64s);
    let mut current: u64 = 0;
    let mut bits_filled = 0;
    for &sign in sign_flips {
        if sign {
            current |= 1 << bits_filled;
        }
        bits_filled += 1;
        if bits_filled == 64 {
            encodings.push(current);
            current = 0;
            bits_filled = 0;
        }
    }
    if bits_filled > 0 {
        encodings.push(current);
    }
    encodings
}

/// Converts a vector of `u64` bit-packed representations back into a vector of boolean sign flip values.
///
/// This function is the inverse of `sign_flips_to_u64s`. It decodes the packed bits from the `u64`
/// values to reconstruct the original boolean values.
///
/// # Arguments
///
/// * `packed_sign_flip_bits` - A slice of `u64` values containing the bit-packed sign flips.
/// * `n_signs` - The exact number of boolean values to decode (prevents decoding padding bits).
///
/// # Returns
///
/// A `Vec<bool>` containing the decoded boolean sign flip values.
///
/// # Example
///
/// ```
/// use pg_bitpacked::u64s_to_sign_flips;
/// let packed = vec![0b0101_u64]; // bits for [true, false, true, false]
/// let decoded = u64s_to_sign_flips(&packed, 4);
/// assert_eq!(decoded, vec![true, false, true, false]);
/// ```
pub fn u64s_to_sign_flips(packed_sign_flip_bits: &[u64], n_signs: usize) -> Vec<bool> {
    let mut signs = Vec::with_capacity(n_signs);
    let mut decoded = 0;
    for encoding in packed_sign_flip_bits {
        for i in 0..64 {
            if decoded == n_signs {
                return signs;
            }
            let bits = (encoding >> i) & 0b1;
            signs.push(bits == 1);
            decoded += 1;
        }
    }
    signs
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bools_to_u64_vec() {
        // indices 0 and 2 set → bits 0 and 2 → 0b0101 = 5
        let bits = vec![true, false, true, false];
        assert_eq!(bools_to_u64_vec(&bits), vec![0b0101_u64]);

        // empty input
        assert_eq!(bools_to_u64_vec(&[]), Vec::<u64>::new());

        // 65 booleans span two u64s; index 64 maps to bit 0 of the second u64
        let mut long_bits = vec![false; 65];
        long_bits[64] = true;
        assert_eq!(bools_to_u64_vec(&long_bits), vec![0_u64, 1_u64]);

        // 128 booleans across two full u64s: every even index is true
        // each u64 gets alternating bits set: 0b010101...01 = 0x5555_5555_5555_5555
        let bits_128: Vec<bool> = (0..128).map(|i| i % 2 == 0).collect();
        let packed = bools_to_u64_vec(&bits_128);
        assert_eq!(packed.len(), 2);
        assert_eq!(packed[0], 0x5555_5555_5555_5555_u64);
        assert_eq!(packed[1], 0x5555_5555_5555_5555_u64);
    }

    #[test]
    fn test_paulis_to_u64s() {
        // [I, X, Y, Z] → Z bits [0,0,1,1] = 0b1100 = 12, X bits [0,1,1,0] = 0b0110 = 6
        let paulis = vec![Pauli::I, Pauli::X, Pauli::Y, Pauli::Z];
        let (zs, xs) = paulis_to_u64s(&paulis);
        assert_eq!(zs, vec![0b1100_u64]);
        assert_eq!(xs, vec![0b0110_u64]);

        // empty input
        let (zs, xs) = paulis_to_u64s(&[]);
        assert!(zs.is_empty());
        assert!(xs.is_empty());
    }

    #[test]
    fn test_paulis_to_u64s_long() {
        // 65 Paulis: first 64 are I, last one is X
        // → first u64: zs=0, xs=0; second u64: zs=0, xs=1 (bit 0 of second word)
        let mut paulis = vec![Pauli::I; 65];
        paulis[64] = Pauli::X;
        let (zs, xs) = paulis_to_u64s(&paulis);
        assert_eq!(zs, vec![0_u64, 0_u64]);
        assert_eq!(xs, vec![0_u64, 1_u64]);

        // 128 Paulis: roundtrip
        let paulis: Vec<Pauli> = (0..128)
            .map(|i| match i % 4 {
                0 => Pauli::I,
                1 => Pauli::X,
                2 => Pauli::Y,
                _ => Pauli::Z,
            })
            .collect();
        let (zs, xs) = paulis_to_u64s(&paulis);
        assert_eq!(zs.len(), 2);
        assert_eq!(xs.len(), 2);
        assert_eq!(u64s_to_paulis(&zs, &xs, paulis.len()), paulis);
    }

    #[test]
    fn test_u64s_to_paulis() {
        // roundtrip
        let paulis = vec![Pauli::I, Pauli::X, Pauli::Y, Pauli::Z];
        let (zs, xs) = paulis_to_u64s(&paulis);
        assert_eq!(u64s_to_paulis(&zs, &xs, paulis.len()), paulis);

        // n_paulis truncates padding bits
        let (zs, xs) = paulis_to_u64s(&[Pauli::X, Pauli::Z]);
        assert_eq!(u64s_to_paulis(&zs, &xs, 2), vec![Pauli::X, Pauli::Z]);
    }

    #[test]
    fn test_sign_flips_to_u64s() {
        // indices 0 and 2 true → bits 0 and 2 → 0b0101 = 5
        let signs = vec![true, false, true, false];
        assert_eq!(sign_flips_to_u64s(&signs), vec![0b0101_u64]);

        // empty input
        assert_eq!(sign_flips_to_u64s(&[]), Vec::<u64>::new());
    }

    #[test]
    fn test_sign_flips_to_u64s_long() {
        // 65 booleans: first 64 false, last true
        // → first u64 = 0, second u64 = 1
        let mut signs = vec![false; 65];
        signs[64] = true;
        assert_eq!(sign_flips_to_u64s(&signs), vec![0_u64, 1_u64]);

        // 128 booleans: roundtrip
        let signs: Vec<bool> = (0..128).map(|i| i % 3 == 0).collect();
        let packed = sign_flips_to_u64s(&signs);
        assert_eq!(packed.len(), 2);
        assert_eq!(u64s_to_sign_flips(&packed, signs.len()), signs);
    }

    #[test]
    fn test_u64s_to_sign_flips() {
        // roundtrip
        let signs = vec![true, false, true, false];
        let packed = sign_flips_to_u64s(&signs);
        assert_eq!(u64s_to_sign_flips(&packed, signs.len()), signs);

        // n_signs truncates padding bits
        let packed = sign_flips_to_u64s(&[true, true, false]);
        assert_eq!(u64s_to_sign_flips(&packed, 3), vec![true, true, false]);
    }
}
