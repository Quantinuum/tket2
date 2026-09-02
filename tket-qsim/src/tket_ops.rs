use std::f64::consts::{FRAC_1_SQRT_2, PI};

use num_complex::Complex64;
use tket::TketOp;

use crate::{Simulatable, UnitaryMatrix};

impl Simulatable for TketOp {
    fn num_qubits(&self) -> usize {
        match self {
            TketOp::H
            | TketOp::X
            | TketOp::Y
            | TketOp::Z
            | TketOp::S
            | TketOp::Sdg
            | TketOp::T
            | TketOp::Tdg
            | TketOp::V
            | TketOp::Vdg
            | TketOp::Rx
            | TketOp::Ry
            | TketOp::Rz => 1,
            TketOp::CX | TketOp::CY | TketOp::CZ | TketOp::CRz => 2,
            TketOp::Toffoli => 3,
            // Non-unitary ops — shouldn't be simulated
            TketOp::Measure
            | TketOp::MeasureFree
            | TketOp::QAlloc
            | TketOp::TryQAlloc
            | TketOp::QFree
            | TketOp::Reset => {
                panic!("Non-unitary op {self:?} cannot be simulated as a unitary");
            }
            &_ => {
                panic!("Unknown TketOp {self:?}");
            }
        }
    }

    fn unitary(&self, params: &[f64]) -> UnitaryMatrix {
        match self {
            TketOp::H => gate_h(),
            TketOp::X => gate_x(),
            TketOp::Y => gate_y(),
            TketOp::Z => gate_z(),
            TketOp::S => gate_s(),
            TketOp::Sdg => gate_sdg(),
            TketOp::T => gate_t(),
            TketOp::Tdg => gate_tdg(),
            TketOp::V => gate_v(),
            TketOp::Vdg => gate_vdg(),
            TketOp::Rx => gate_rx(params[0]),
            TketOp::Ry => gate_ry(params[0]),
            TketOp::Rz => gate_rz(params[0]),
            TketOp::CX => gate_cx(),
            TketOp::CY => gate_cy(),
            TketOp::CZ => gate_cz(),
            TketOp::CRz => gate_crz(params[0]),
            TketOp::Toffoli => gate_toffoli(),
            _ => panic!("Non-unitary op {self:?} cannot be simulated"),
        }
    }
}

// Single-qubit gates

fn gate_h() -> UnitaryMatrix {
    let s = FRAC_1_SQRT_2;
    UnitaryMatrix::from_row_major(
        1,
        vec![
            Complex64::new(s, 0.0),
            Complex64::new(s, 0.0),
            Complex64::new(s, 0.0),
            Complex64::new(-s, 0.0),
        ],
    )
}

fn gate_x() -> UnitaryMatrix {
    UnitaryMatrix::from_row_major(
        1,
        vec![
            Complex64::new(0.0, 0.0),
            Complex64::new(1.0, 0.0),
            Complex64::new(1.0, 0.0),
            Complex64::new(0.0, 0.0),
        ],
    )
}

fn gate_y() -> UnitaryMatrix {
    UnitaryMatrix::from_row_major(
        1,
        vec![
            Complex64::new(0.0, 0.0),
            Complex64::new(0.0, -1.0),
            Complex64::new(0.0, 1.0),
            Complex64::new(0.0, 0.0),
        ],
    )
}

fn gate_z() -> UnitaryMatrix {
    UnitaryMatrix::from_row_major(
        1,
        vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(0.0, 0.0),
            Complex64::new(0.0, 0.0),
            Complex64::new(-1.0, 0.0),
        ],
    )
}

fn gate_s() -> UnitaryMatrix {
    UnitaryMatrix::from_row_major(
        1,
        vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(0.0, 0.0),
            Complex64::new(0.0, 0.0),
            Complex64::new(0.0, 1.0),
        ],
    )
}

fn gate_sdg() -> UnitaryMatrix {
    UnitaryMatrix::from_row_major(
        1,
        vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(0.0, 0.0),
            Complex64::new(0.0, 0.0),
            Complex64::new(0.0, -1.0),
        ],
    )
}

fn gate_t() -> UnitaryMatrix {
    let phase = Complex64::from_polar(1.0, PI / 4.0);
    UnitaryMatrix::from_row_major(
        1,
        vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(0.0, 0.0),
            Complex64::new(0.0, 0.0),
            phase,
        ],
    )
}

fn gate_tdg() -> UnitaryMatrix {
    let phase = Complex64::from_polar(1.0, -PI / 4.0);
    UnitaryMatrix::from_row_major(
        1,
        vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(0.0, 0.0),
            Complex64::new(0.0, 0.0),
            phase,
        ],
    )
}

/// V = Rx(π/2) = [[1, -i], [-i, 1]] / √2
/// V is also known as sqrt(X).
fn gate_v() -> UnitaryMatrix {
    let s = FRAC_1_SQRT_2;
    UnitaryMatrix::from_row_major(
        1,
        vec![
            Complex64::new(s, 0.0),
            Complex64::new(0.0, -s),
            Complex64::new(0.0, -s),
            Complex64::new(s, 0.0),
        ],
    )
}

/// Vdg = Rx(-π/2) = [[1, i], [i, 1]] / √2
fn gate_vdg() -> UnitaryMatrix {
    let s = FRAC_1_SQRT_2;
    UnitaryMatrix::from_row_major(
        1,
        vec![
            Complex64::new(s, 0.0),
            Complex64::new(0.0, s),
            Complex64::new(0.0, s),
            Complex64::new(s, 0.0),
        ],
    )
}

/// Rx(θ) where θ is in half-turns (i.e., angle_radians = θ * π).
///
/// Rx(θ) = [[cos(θπ/2), -i·sin(θπ/2)], [-i·sin(θπ/2), cos(θπ/2)]]
fn gate_rx(half_turns: f64) -> UnitaryMatrix {
    let theta = half_turns * PI / 2.0;
    let c = Complex64::new(theta.cos(), 0.0);
    let s = Complex64::new(0.0, -theta.sin());
    UnitaryMatrix::from_row_major(1, vec![c, s, s, c])
}

/// Ry(θ) where θ is in half-turns.
///
/// Ry(θ) = [[cos(θπ/2), -sin(θπ/2)], [sin(θπ/2), cos(θπ/2)]]
fn gate_ry(half_turns: f64) -> UnitaryMatrix {
    let theta = half_turns * PI / 2.0;
    let c = Complex64::new(theta.cos(), 0.0);
    let s_neg = Complex64::new(-theta.sin(), 0.0);
    let s_pos = Complex64::new(theta.sin(), 0.0);
    UnitaryMatrix::from_row_major(1, vec![c, s_neg, s_pos, c])
}

/// Rz(θ) where θ is in half-turns.
///
/// Rz(θ) = [[e^{-iθπ/2}, 0], [0, e^{iθπ/2}]]
fn gate_rz(half_turns: f64) -> UnitaryMatrix {
    let theta = half_turns * PI / 2.0;
    let e_neg = Complex64::from_polar(1.0, -theta);
    let e_pos = Complex64::from_polar(1.0, theta);
    let zero = Complex64::new(0.0, 0.0);
    UnitaryMatrix::from_row_major(1, vec![e_neg, zero, zero, e_pos])
}

// Two-qubit gates

fn gate_cx() -> UnitaryMatrix {
    let o = Complex64::new(0.0, 0.0);
    let i = Complex64::new(1.0, 0.0);
    // |00⟩→|00⟩, |01⟩→|01⟩, |10⟩→|11⟩, |11⟩→|10⟩
    #[rustfmt::skip]
    let data = vec![
        i, o, o, o,
        o, i, o, o,
        o, o, o, i,
        o, o, i, o,
    ];
    UnitaryMatrix::from_row_major(2, data)
}

fn gate_cy() -> UnitaryMatrix {
    let o = Complex64::new(0.0, 0.0);
    let i = Complex64::new(1.0, 0.0);
    let mi = Complex64::new(0.0, -1.0);
    let pi = Complex64::new(0.0, 1.0);
    // CY = |0⟩⟨0| ⊗ I + |1⟩⟨1| ⊗ Y
    #[rustfmt::skip]
    let data = vec![
        i, o,  o, o,
        o, i,  o, o,
        o, o,  o, mi,
        o, o, pi,  o,
    ];
    UnitaryMatrix::from_row_major(2, data)
}

fn gate_cz() -> UnitaryMatrix {
    let o = Complex64::new(0.0, 0.0);
    let i = Complex64::new(1.0, 0.0);
    let m = Complex64::new(-1.0, 0.0);
    #[rustfmt::skip]
    let data = vec![
        i, o, o, o,
        o, i, o, o,
        o, o, i, o,
        o, o, o, m,
    ];
    UnitaryMatrix::from_row_major(2, data)
}

/// CRz(θ) where θ is in half-turns.
/// CRz = |0⟩⟨0| ⊗ I + |1⟩⟨1| ⊗ Rz(θ)
fn gate_crz(half_turns: f64) -> UnitaryMatrix {
    let o = Complex64::new(0.0, 0.0);
    let i = Complex64::new(1.0, 0.0);
    let theta = half_turns * PI / 2.0;
    let e_neg = Complex64::from_polar(1.0, -theta);
    let e_pos = Complex64::from_polar(1.0, theta);
    #[rustfmt::skip]
    let data = vec![
        i, o,     o, o,
        o, i,     o, o,
        o, o, e_neg, o,
        o, o,     o, e_pos,
    ];
    UnitaryMatrix::from_row_major(2, data)
}

// Three-qubit gates

fn gate_toffoli() -> UnitaryMatrix {
    let dim = 8;
    let zero = Complex64::new(0.0, 0.0);
    let mut data = vec![zero; dim * dim];
    // Identity on all states except |110⟩↔|111⟩
    for i in 0..dim {
        match i {
            6 => data[6 * dim + 7] = Complex64::new(1.0, 0.0), // |110⟩→|111⟩
            7 => data[7 * dim + 6] = Complex64::new(1.0, 0.0), // |111⟩→|110⟩
            _ => data[i * dim + i] = Complex64::new(1.0, 0.0),
        }
    }
    UnitaryMatrix::from_row_major(3, data)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_h_squared_is_identity() {
        let h = TketOp::H.unitary(&[]);
        let hh = h.matmul(&h);
        let id = UnitaryMatrix::identity(1);
        assert!(hh.approx_eq(&id, 1e-10));
    }

    #[test]
    fn test_x_squared_is_identity() {
        let x = TketOp::X.unitary(&[]);
        let xx = x.matmul(&x);
        let id = UnitaryMatrix::identity(1);
        assert!(xx.approx_eq(&id, 1e-10));
    }

    #[test]
    fn test_s_squared_is_z() {
        let s = TketOp::S.unitary(&[]);
        let ss = s.matmul(&s);
        let z = TketOp::Z.unitary(&[]);
        assert!(ss.approx_eq(&z, 1e-10));
    }

    #[test]
    fn test_t_squared_is_s() {
        let t = TketOp::T.unitary(&[]);
        let tt = t.matmul(&t);
        let s = TketOp::S.unitary(&[]);
        assert!(tt.approx_eq(&s, 1e-10));
    }

    #[test]
    fn test_v_squared_is_x() {
        let v = TketOp::V.unitary(&[]);
        let vv = v.matmul(&v);
        let x = TketOp::X.unitary(&[]);
        // V is Rx(pi/2), so V^2 = -i X.
        assert!(vv.approx_eq_up_to_global_phase(&x, 1e-10));
    }

    #[test]
    fn test_rx_pi_is_x() {
        // Rx(1 half-turn) = -i X (equal up to global phase)
        let rx = TketOp::Rx.unitary(&[1.0]);
        let x = TketOp::X.unitary(&[]);
        assert!(rx.approx_eq_up_to_global_phase(&x, 1e-10));
    }

    #[test]
    fn test_ry_pi_is_y() {
        let ry = TketOp::Ry.unitary(&[1.0]);
        let y = TketOp::Y.unitary(&[]);
        assert!(ry.approx_eq_up_to_global_phase(&y, 1e-10));
    }

    #[test]
    fn test_rz_pi_is_z() {
        let rz = TketOp::Rz.unitary(&[1.0]);
        let z = TketOp::Z.unitary(&[]);
        assert!(rz.approx_eq_up_to_global_phase(&z, 1e-10));
    }
}
