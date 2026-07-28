use num_complex::Complex64;
use tket_qsystem::extension::qsystem::helios::HeliosOp;

use crate::{Simulatable, UnitaryMatrix};

impl Simulatable for HeliosOp {
    fn num_qubits(&self) -> usize {
        match self {
            HeliosOp::PhasedX | HeliosOp::Rz => 1,
            HeliosOp::ZZPhase => 2,
            // Non-unitary ops
            HeliosOp::LazyMeasure
            | HeliosOp::LazyMeasureReset
            | HeliosOp::TryQAlloc
            | HeliosOp::QFree
            | HeliosOp::Reset
            | HeliosOp::LazyMeasureLeaked
            | HeliosOp::FutureToMeasurement => {
                panic!("Non-unitary Helios op {self:?} cannot be simulated as a unitary");
            }
            &_ => {
                panic!("Unknown Helios op {self:?}");
            }
        }
    }

    fn unitary(&self, params: &[f64]) -> UnitaryMatrix {
        match self {
            HeliosOp::PhasedX => gate_phased_x(params[0], params[1]),
            HeliosOp::Rz => gate_rz_radians(params[0]),
            HeliosOp::ZZPhase => gate_zz_phase(params[0]),
            _ => panic!("Non-unitary Helios op {self:?} cannot be simulated"),
        }
    }
}

/// PhasedX(θ, φ) gate.
///
/// PhasedX(θ, φ) = Rz(φ) · Rx(θ) · Rz(-φ)
///
/// where θ and φ are in radians (these are the raw f64 parameters as used
/// in the Helios platform, not half-turns).
///
/// Matrix:
/// ```text
/// [[cos(θ/2),          -i·e^{-iφ}·sin(θ/2)],
///  [-i·e^{iφ}·sin(θ/2), cos(θ/2)           ]]
/// ```
fn gate_phased_x(theta: f64, phi: f64) -> UnitaryMatrix {
    let ct = (theta / 2.0).cos();
    let st = (theta / 2.0).sin();
    let e_neg_phi = Complex64::from_polar(1.0, -phi);
    let e_pos_phi = Complex64::from_polar(1.0, phi);

    let a = Complex64::new(ct, 0.0);
    let b = Complex64::new(0.0, -1.0) * e_neg_phi * Complex64::new(st, 0.0);
    let c = Complex64::new(0.0, -1.0) * e_pos_phi * Complex64::new(st, 0.0);
    let d = Complex64::new(ct, 0.0);

    UnitaryMatrix::from_row_major(1, vec![a, b, c, d])
}

/// Rz(θ) where θ is in radians (the raw f64 parameter).
///
/// Rz(θ) = [[e^{-iθ/2}, 0], [0, e^{iθ/2}]]
fn gate_rz_radians(theta: f64) -> UnitaryMatrix {
    let e_neg = Complex64::from_polar(1.0, -theta / 2.0);
    let e_pos = Complex64::from_polar(1.0, theta / 2.0);
    UnitaryMatrix::from_row_major(1, vec![e_neg, Complex64::ZERO, Complex64::ZERO, e_pos])
}

/// ZZPhase(θ) gate where θ is in radians.
///
/// ZZPhase(θ) = exp(-i·θ/2·Z⊗Z)
///           = diag(e^{-iθ/2}, e^{iθ/2}, e^{iθ/2}, e^{-iθ/2})
fn gate_zz_phase(theta: f64) -> UnitaryMatrix {
    let e_neg = Complex64::from_polar(1.0, -theta / 2.0);
    let e_pos = Complex64::from_polar(1.0, theta / 2.0);
    let o = Complex64::ZERO;
    #[rustfmt::skip]
    let data = vec![
        e_neg, o,     o,     o,
        o,     e_pos, o,     o,
        o,     o,     e_pos, o,
        o,     o,     o,     e_neg,
    ];
    UnitaryMatrix::from_row_major(2, data)
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use super::*;

    #[test]
    fn test_phased_x_theta_zero_is_identity() {
        let u = HeliosOp::PhasedX.unitary(&[0.0, 1.23]);
        let id = UnitaryMatrix::identity(1);
        assert!(u.approx_eq(&id, 1e-10));
    }

    #[test]
    fn test_rz_zero_is_identity() {
        let u = HeliosOp::Rz.unitary(&[0.0]);
        let id = UnitaryMatrix::identity(1);
        assert!(u.approx_eq(&id, 1e-10));
    }

    #[test]
    fn test_zz_phase_zero_is_identity() {
        let u = HeliosOp::ZZPhase.unitary(&[0.0]);
        let id = UnitaryMatrix::identity(2);
        assert!(u.approx_eq(&id, 1e-10));
    }

    #[test]
    fn test_phased_x_pi_phi_zero_is_x() {
        // PhasedX(π, 0) should be -iX (up to global phase, same as X)
        let u = HeliosOp::PhasedX.unitary(&[PI, 0.0]);
        let x = tket::TketOp::X.unitary(&[]);
        assert!(u.approx_eq_up_to_global_phase(&x, 1e-10));
    }
}
