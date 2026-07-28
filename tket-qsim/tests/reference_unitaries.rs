//! Gate-unitary tests using roqoqo as an independent reference implementation.
//!
//! tket parameters are expressed in half-turns, while roqoqo, Helios, and Sol
//! use radians. Gates without a direct roqoqo equivalent are constructed from
//! roqoqo gate matrices using standard circuit identities.

use std::f64::consts::PI;

use ndarray::Array2;
use ndarray::linalg::kron;
use num_complex::Complex64;
use roqoqo::operations::{
    CNOT, ControlledPauliY, ControlledPauliZ, Hadamard, InvSGate, InvSqrtPauliX, InvTGate,
    OperateGate, PauliX, PauliY, PauliZ, RotateX, RotateXY, RotateY, RotateZ, SGate, SqrtPauliX,
    TGate, Toffoli, VariableMSXX,
};
use rstest::rstest;
use tket::TketOp;
use tket_qsim::{Simulatable, UnitaryMatrix};
use tket_qsystem::extension::qsystem::{helios::HeliosOp, sol::SolOp};

const TOLERANCE: f64 = 1e-12;

fn assert_roqoqo_matrix(actual: UnitaryMatrix, expected: &Array2<Complex64>) {
    assert_eq!(expected.shape(), &[actual.dim(), actual.dim()]);

    for row in 0..actual.dim() {
        for col in 0..actual.dim() {
            let got = actual.get(row, col);
            let reference = expected[[row, col]];
            assert!(
                (got - reference).norm() < TOLERANCE,
                "mismatch at ({row}, {col}): got {got}, expected {reference}"
            );
        }
    }
}

fn roqoqo_unitary<G: OperateGate>(gate: &G) -> Array2<Complex64> {
    gate.unitary_matrix()
        .expect("roqoqo gate parameters should be numeric")
}

fn assert_matches_roqoqo<G: OperateGate>(actual: UnitaryMatrix, reference: &G) {
    assert_roqoqo_matrix(actual, &roqoqo_unitary(reference));
}

fn identity(dim: usize) -> Array2<Complex64> {
    Array2::from_diag_elem(dim, Complex64::new(1.0, 0.0))
}

fn one_qubit_on_two(gate: &Array2<Complex64>, qubit: usize) -> Array2<Complex64> {
    let id = identity(2);
    match qubit {
        0 => kron(gate, &id),
        1 => kron(&id, gate),
        _ => panic!("two-qubit system has no qubit {qubit}"),
    }
}

/// Compose matrices listed in circuit execution order.
fn compose_in_time_order(gates: &[Array2<Complex64>]) -> Array2<Complex64> {
    let dim = gates
        .first()
        .expect("at least one gate is required")
        .nrows();
    gates
        .iter()
        .fold(identity(dim), |unitary, gate| gate.dot(&unitary))
}

#[test]
fn fixed_single_qubit_tket_gates_match_roqoqo() {
    assert_matches_roqoqo(TketOp::H.unitary(&[]), &Hadamard::new(0));
    assert_matches_roqoqo(TketOp::X.unitary(&[]), &PauliX::new(0));
    assert_matches_roqoqo(TketOp::Y.unitary(&[]), &PauliY::new(0));
    assert_matches_roqoqo(TketOp::Z.unitary(&[]), &PauliZ::new(0));
    assert_matches_roqoqo(TketOp::S.unitary(&[]), &SGate::new(0));
    assert_matches_roqoqo(TketOp::Sdg.unitary(&[]), &InvSGate::new(0));
    assert_matches_roqoqo(TketOp::T.unitary(&[]), &TGate::new(0));
    assert_matches_roqoqo(TketOp::Tdg.unitary(&[]), &InvTGate::new(0));
    assert_matches_roqoqo(TketOp::V.unitary(&[]), &SqrtPauliX::new(0));
    assert_matches_roqoqo(TketOp::Vdg.unitary(&[]), &InvSqrtPauliX::new(0));
}

#[test]
fn fixed_multi_qubit_tket_gates_match_roqoqo() {
    // These also check the local MSB/LSB qubit order.
    assert_matches_roqoqo(TketOp::CX.unitary(&[]), &CNOT::new(0, 1));
    assert_matches_roqoqo(TketOp::CY.unitary(&[]), &ControlledPauliY::new(0, 1));
    assert_matches_roqoqo(TketOp::CZ.unitary(&[]), &ControlledPauliZ::new(0, 1));
    assert_matches_roqoqo(TketOp::Toffoli.unitary(&[]), &Toffoli::new(0, 1, 2));
}

#[rstest]
#[case(0.0)]
#[case(0.125)]
#[case(0.5)]
#[case(-0.7)]
#[case(1.234)]
fn parametric_tket_gates_match_roqoqo(#[case] half_turns: f64) {
    let radians = half_turns * PI;

    assert_matches_roqoqo(
        TketOp::Rx.unitary(&[half_turns]),
        &RotateX::new(0, radians.into()),
    );
    assert_matches_roqoqo(
        TketOp::Ry.unitary(&[half_turns]),
        &RotateY::new(0, radians.into()),
    );
    assert_matches_roqoqo(
        TketOp::Rz.unitary(&[half_turns]),
        &RotateZ::new(0, radians.into()),
    );
}

#[rstest]
#[case(0.0)]
#[case(0.125)]
#[case(0.5)]
#[case(-0.7)]
#[case(1.234)]
fn tket_crz_matches_roqoqo_decomposition(#[case] half_turns: f64) {
    let radians = half_turns * PI;
    let positive_half = roqoqo_unitary(&RotateZ::new(1, (radians / 2.0).into()));
    let negative_half = roqoqo_unitary(&RotateZ::new(1, (-radians / 2.0).into()));
    let cnot = roqoqo_unitary(&CNOT::new(0, 1));

    let reference = compose_in_time_order(&[
        one_qubit_on_two(&positive_half, 1),
        cnot.clone(),
        one_qubit_on_two(&negative_half, 1),
        cnot,
    ]);
    assert_roqoqo_matrix(TketOp::CRz.unitary(&[half_turns]), &reference);
}

#[rstest]
#[case(0.0, 0.0)]
#[case(PI / 2.0, PI / 4.0)]
#[case(-0.7, 0.3)]
#[case(1.234, -0.91)]
fn native_single_qubit_gates_match_roqoqo(#[case] theta: f64, #[case] phi: f64) {
    assert_matches_roqoqo(
        HeliosOp::PhasedX.unitary(&[theta, phi]),
        &RotateXY::new(0, theta.into(), phi.into()),
    );
    assert_matches_roqoqo(
        SolOp::PhasedX.unitary(&[theta, phi]),
        &RotateXY::new(0, theta.into(), phi.into()),
    );
    assert_matches_roqoqo(
        HeliosOp::Rz.unitary(&[theta]),
        &RotateZ::new(0, theta.into()),
    );
    assert_matches_roqoqo(SolOp::Rz.unitary(&[theta]), &RotateZ::new(0, theta.into()));
}

#[rstest]
#[case(0.0)]
#[case(PI / 2.0)]
#[case(-0.7)]
#[case(1.234)]
fn helios_zzphase_matches_roqoqo_decomposition(#[case] theta: f64) {
    let cnot = roqoqo_unitary(&CNOT::new(0, 1));
    let rz = roqoqo_unitary(&RotateZ::new(1, theta.into()));
    let reference = compose_in_time_order(&[cnot.clone(), one_qubit_on_two(&rz, 1), cnot]);

    assert_roqoqo_matrix(HeliosOp::ZZPhase.unitary(&[theta]), &reference);
}

#[rstest]
#[case(0.0, 0.0)]
#[case(PI / 2.0, PI / 4.0)]
#[case(-0.7, 0.3)]
#[case(1.234, -0.91)]
fn sol_phased_xx_matches_roqoqo_decomposition(#[case] theta: f64, #[case] phi: f64) {
    let rz_negative = roqoqo_unitary(&RotateZ::new(0, (-phi).into()));
    let rz_positive = roqoqo_unitary(&RotateZ::new(0, phi.into()));
    let xx = roqoqo_unitary(&VariableMSXX::new(0, 1, theta.into()));
    let negative_basis_change = kron(&rz_negative, &rz_negative);
    let positive_basis_change = kron(&rz_positive, &rz_positive);

    let reference = compose_in_time_order(&[negative_basis_change, xx, positive_basis_change]);
    assert_roqoqo_matrix(SolOp::PhasedXX.unitary(&[theta, phi]), &reference);
}
