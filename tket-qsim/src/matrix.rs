use num_complex::Complex64;
use std::fmt;

/// A dense unitary matrix of dimension `2^num_qubits × 2^num_qubits`.
///
/// Stored in row-major order.
#[derive(Clone)]
pub struct UnitaryMatrix {
    /// Number of qubits this matrix acts on.
    num_qubits: usize,
    /// Flat row-major storage: entry (i, j) is at index `i * dim + j`.
    data: Vec<Complex64>,
}

impl UnitaryMatrix {
    /// Create an identity matrix for `n` qubits.
    pub fn identity(n: usize) -> Self {
        let dim = 1 << n;
        let mut data = vec![Complex64::ZERO; dim * dim];
        for i in 0..dim {
            data[i * dim + i] = Complex64::new(1.0, 0.0);
        }
        Self {
            num_qubits: n,
            data,
        }
    }

    /// Create a matrix from a flat row-major slice.
    ///
    /// # Panics
    /// Panics if `data.len() != (2^num_qubits)^2`.
    pub fn from_row_major(num_qubits: usize, data: Vec<Complex64>) -> Self {
        let dim = 1 << num_qubits;
        assert_eq!(
            data.len(),
            dim * dim,
            "Expected {} elements for {}-qubit matrix, got {}",
            dim * dim,
            num_qubits,
            data.len()
        );
        Self { num_qubits, data }
    }

    /// The number of qubits.
    pub fn num_qubits(&self) -> usize {
        self.num_qubits
    }

    /// The dimension (2^num_qubits).
    pub fn dim(&self) -> usize {
        1 << self.num_qubits
    }

    /// Get entry (row, col).
    pub fn get(&self, row: usize, col: usize) -> Complex64 {
        let dim = self.dim();
        self.data[row * dim + col]
    }

    /// Set entry (row, col).
    pub fn set(&mut self, row: usize, col: usize, value: Complex64) {
        let dim = self.dim();
        self.data[row * dim + col] = value;
    }

    /// Multiply two matrices: `self * other`.
    ///
    /// # Panics
    /// Panics if dimensions don't match.
    pub fn matmul(&self, other: &Self) -> Self {
        assert_eq!(self.num_qubits, other.num_qubits);
        let dim = self.dim();
        let mut result = vec![Complex64::ZERO; dim * dim];
        for i in 0..dim {
            for k in 0..dim {
                let a_ik = self.data[i * dim + k];
                if a_ik == Complex64::ZERO {
                    continue;
                }
                for j in 0..dim {
                    result[i * dim + j] += a_ik * other.data[k * dim + j];
                }
            }
        }
        Self {
            num_qubits: self.num_qubits,
            data: result,
        }
    }

    /// Tensor product: `self ⊗ other`.
    pub fn tensor(&self, other: &Self) -> Self {
        let n = self.num_qubits + other.num_qubits;
        let dim_a = self.dim();
        let dim_b = other.dim();
        let dim = dim_a * dim_b;
        let mut data = vec![Complex64::ZERO; dim * dim];
        for i_a in 0..dim_a {
            for j_a in 0..dim_a {
                let a = self.data[i_a * dim_a + j_a];
                if a == Complex64::ZERO {
                    continue;
                }
                for i_b in 0..dim_b {
                    for j_b in 0..dim_b {
                        let row = i_a * dim_b + i_b;
                        let col = j_a * dim_b + j_b;
                        data[row * dim + col] = a * other.data[i_b * dim_b + j_b];
                    }
                }
            }
        }
        Self {
            num_qubits: n,
            data,
        }
    }

    /// Expand a gate unitary acting on `target_qubits` (indices within a
    /// system of `total_qubits`) into the full 2^total × 2^total matrix.
    ///
    /// `target_qubits` lists the qubit indices this gate acts on, in order.
    /// For example, a CNOT on qubits [1, 2] in a 3-qubit system.
    pub fn expand_to_system(&self, target_qubits: &[usize], total_qubits: usize) -> Self {
        assert_eq!(target_qubits.len(), self.num_qubits);
        let full_dim = 1usize << total_qubits;
        let gate_dim = self.dim();
        let mut result = vec![Complex64::ZERO; full_dim * full_dim];

        for col in 0..full_dim {
            for gate_row in 0..gate_dim {
                // Determine the full row index by replacing target qubit bits
                // in `col` with the bits from `gate_row`.
                let mut full_row = col;
                for (gate_bit_pos, &sys_qubit) in target_qubits.iter().enumerate() {
                    let bit = (gate_row >> (self.num_qubits - 1 - gate_bit_pos)) & 1;
                    if bit == 1 {
                        full_row |= 1 << (total_qubits - 1 - sys_qubit);
                    } else {
                        full_row &= !(1 << (total_qubits - 1 - sys_qubit));
                    }
                }

                // Extract the target qubit bits from `col` to get `gate_col`.
                let mut gate_col = 0usize;
                for (gate_bit_pos, &sys_qubit) in target_qubits.iter().enumerate() {
                    let bit = (col >> (total_qubits - 1 - sys_qubit)) & 1;
                    gate_col |= bit << (self.num_qubits - 1 - gate_bit_pos);
                }

                let val = self.data[gate_row * gate_dim + gate_col];
                if val != Complex64::ZERO {
                    result[full_row * full_dim + col] += val;
                }
            }
        }

        Self {
            num_qubits: total_qubits,
            data: result,
        }
    }

    /// Check if two matrices are approximately equal (entry-wise) within `tol`.
    pub fn approx_eq(&self, other: &Self, tol: f64) -> bool {
        if self.num_qubits != other.num_qubits {
            return false;
        }
        self.data
            .iter()
            .zip(other.data.iter())
            .all(|(a, b)| (a - b).norm() < tol)
    }

    /// Check if two unitaries are equal up to a global phase, i.e.,
    /// if for unitaries U and V, there exists a complex number e^{iθ}
    /// such that U = e^{iθ} V.
    pub fn approx_eq_up_to_global_phase(&self, other: &Self, tol: f64) -> bool {
        if self.num_qubits != other.num_qubits {
            return false;
        }
        // Find the first non-negligible entry to determine the phase factor.
        // If only one matrix is nonzero at that entry, the matrices cannot be
        // related by a global phase. This also avoids treating matrices with
        // disjoint support (for example I and X) as equivalent.
        let mut phase = None;
        for (a, b) in self.data.iter().zip(other.data.iter()) {
            match (a.norm() > tol, b.norm() > tol) {
                (true, true) => {
                    phase = Some(a / b);
                    break;
                }
                (true, false) | (false, true) => return false,
                (false, false) => {}
            }
        }

        let Some(phase) = phase else {
            // Both matrices are zero within tolerance.
            return true;
        };

        // Normalize phase to unit magnitude.
        let phase = phase / Complex64::new(phase.norm(), 0.0);

        // Check all entries: self[i,j] ≈ phase * other[i,j]
        self.data
            .iter()
            .zip(other.data.iter())
            .all(|(a, b)| (a - phase * b).norm() < tol)
    }
}

impl fmt::Debug for UnitaryMatrix {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let dim = self.dim();
        writeln!(
            f,
            "UnitaryMatrix({} qubits, {}×{}):",
            self.num_qubits, dim, dim
        )?;
        for i in 0..dim {
            write!(f, "  [")?;
            for j in 0..dim {
                let c = self.get(i, j);
                if j > 0 {
                    write!(f, ", ")?;
                }
                write!(f, "{:.4}{:+.4}i", c.re, c.im)?;
            }
            writeln!(f, "]")?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_identity() {
        let id = UnitaryMatrix::identity(2);
        assert_eq!(id.dim(), 4);
        for i in 0..4 {
            for j in 0..4 {
                if i == j {
                    assert_eq!(id.get(i, j), Complex64::new(1.0, 0.0));
                } else {
                    assert_eq!(id.get(i, j), Complex64::ZERO);
                }
            }
        }
    }

    #[test]
    fn test_matmul_identity() {
        let id = UnitaryMatrix::identity(1);
        let h = UnitaryMatrix::from_row_major(
            1,
            vec![
                Complex64::new(1.0 / 2.0_f64.sqrt(), 0.0),
                Complex64::new(1.0 / 2.0_f64.sqrt(), 0.0),
                Complex64::new(1.0 / 2.0_f64.sqrt(), 0.0),
                Complex64::new(-1.0 / 2.0_f64.sqrt(), 0.0),
            ],
        );
        let result = id.matmul(&h);
        assert!(result.approx_eq(&h, 1e-10));
    }

    #[test]
    fn test_tensor_identity() {
        let id1 = UnitaryMatrix::identity(1);
        let id2 = UnitaryMatrix::identity(2);
        let tensor = id1.tensor(&id2);
        let expected = UnitaryMatrix::identity(3);
        assert!(tensor.approx_eq(&expected, 1e-10));
    }

    #[test]
    fn test_expand_single_qubit() {
        // X gate on qubit 1 of a 2-qubit system
        let x = UnitaryMatrix::from_row_major(
            1,
            vec![
                Complex64::ZERO,
                Complex64::new(1.0, 0.0),
                Complex64::new(1.0, 0.0),
                Complex64::ZERO,
            ],
        );
        let expanded = x.expand_to_system(&[1], 2);
        // Should be I ⊗ X
        let id = UnitaryMatrix::identity(1);
        let expected = id.tensor(&x);
        assert!(expanded.approx_eq(&expected, 1e-10));
    }

    #[test]
    fn global_phase_comparison_rejects_disjoint_support() {
        let id = UnitaryMatrix::identity(1);
        let x = UnitaryMatrix::from_row_major(
            1,
            vec![
                Complex64::ZERO,
                Complex64::new(1.0, 0.0),
                Complex64::new(1.0, 0.0),
                Complex64::ZERO,
            ],
        );

        assert!(!id.approx_eq_up_to_global_phase(&x, 1e-10));
    }
}
