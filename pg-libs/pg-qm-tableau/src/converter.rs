use crate::Tableau;
use ::pg_bitpacked::{bools_to_u64_vec, paulis_to_u64s, u64s_to_paulis};
use ::pg_core::{GateData, GateType, Op, Pauli, RotationData, TableauData};
use ::pg_ir_kernels::{PGTableau, is_clifford};
use ::pg_utils::{cliff_angle, equiv_0};

impl PGTableau for Tableau {
    fn eye(n_qubits: usize) -> Self {
        Tableau::eye(n_qubits)
    }
    fn get_n_qubits(&self) -> usize {
        self.get_n_qubits()
    }
    fn x(&self, qubit: usize) -> (Vec<Pauli>, bool) {
        let (paulis, sign) = self.get_x_col_paulis(qubit);
        (paulis, !sign)
    }
    fn z(&self, qubit: usize) -> (Vec<Pauli>, bool) {
        let (paulis, sign) = self.get_z_col_paulis(qubit);
        (paulis, !sign)
    }
    fn get_dagger(&self) -> Self {
        self.invert()
    }

    fn precompose_op(&mut self, op: &Op) {
        if !is_clifford(op) {
            panic!("Cannot precompose non-Clifford operation: {:?}", op);
        }
        match op {
            Op::Gate { data } => {
                if !data.get_conditional_bits().is_empty() {
                    panic!("Conditional gates cannot be precomposed.");
                }
                match data.get_gate_type() {
                    GateType::H => {
                        self.precompose_basis_change(Pauli::Y, data.get_args()[0], false)
                    }
                    GateType::S => {
                        self.precompose_basis_change(Pauli::Z, data.get_args()[0], false)
                    }
                    GateType::V => {
                        self.precompose_basis_change(Pauli::X, data.get_args()[0], false)
                    }
                    GateType::Sdg => {
                        self.precompose_basis_change(Pauli::Z, data.get_args()[0], true)
                    }
                    GateType::Vdg => {
                        self.precompose_basis_change(Pauli::X, data.get_args()[0], true)
                    }
                    GateType::X => self.precompose_pauli(Pauli::X, data.get_args()[0]),
                    GateType::Y => self.precompose_pauli(Pauli::Y, data.get_args()[0]),
                    GateType::Z => self.precompose_pauli(Pauli::Z, data.get_args()[0]),
                    GateType::XX => self.precompose_tqe(
                        Pauli::X,
                        Pauli::X,
                        data.get_args()[0],
                        data.get_args()[1],
                    ),
                    GateType::XY => self.precompose_tqe(
                        Pauli::X,
                        Pauli::Y,
                        data.get_args()[0],
                        data.get_args()[1],
                    ),
                    GateType::XZ => self.precompose_tqe(
                        Pauli::X,
                        Pauli::Z,
                        data.get_args()[0],
                        data.get_args()[1],
                    ),
                    GateType::YX => self.precompose_tqe(
                        Pauli::Y,
                        Pauli::X,
                        data.get_args()[0],
                        data.get_args()[1],
                    ),
                    GateType::YY => self.precompose_tqe(
                        Pauli::Y,
                        Pauli::Y,
                        data.get_args()[0],
                        data.get_args()[1],
                    ),
                    GateType::YZ => self.precompose_tqe(
                        Pauli::Y,
                        Pauli::Z,
                        data.get_args()[0],
                        data.get_args()[1],
                    ),
                    GateType::ZX => self.precompose_tqe(
                        Pauli::Z,
                        Pauli::X,
                        data.get_args()[0],
                        data.get_args()[1],
                    ),
                    GateType::ZY => self.precompose_tqe(
                        Pauli::Z,
                        Pauli::Y,
                        data.get_args()[0],
                        data.get_args()[1],
                    ),
                    GateType::ZZ => self.precompose_tqe(
                        Pauli::Z,
                        Pauli::Z,
                        data.get_args()[0],
                        data.get_args()[1],
                    ),
                    GateType::SWAP => self.precompose_swap(data.get_args()[0], data.get_args()[1]),
                    GateType::RX
                    | GateType::RY
                    | GateType::RZ
                    | GateType::ZZPHASE
                    | GateType::PHASEDX => {
                        // TODO: Replace with direct precomposition implementations.
                        let mut tab = Tableau::eye(self.get_n_qubits());
                        tab.postcompose_op(op);
                        self.precompose_tableau(&tab);
                    }
                    _ => panic!("Unexpected gate type: {:?}", data.get_gate_type()),
                }
            }
            Op::Rotation { .. } => {
                // TODO: Replace with a direct precomposition implementation.
                let mut tab = Tableau::eye(self.get_n_qubits());
                tab.postcompose_op(op);
                self.precompose_tableau(&tab);
            }
            Op::Tableau { data } => {
                self.precompose_tableau(&data.clone().into());
            }
            _ => {
                panic!("Unexpected operation type: {:?}", op);
            }
        }
    }

    fn postcompose_op(&mut self, op: &Op) {
        if !is_clifford(op) {
            panic!("Cannot postcompose non-Clifford operation: {:?}", op);
        }
        match op {
            Op::Gate { data } => {
                if !data.get_conditional_bits().is_empty() {
                    panic!("Conditional gates cannot be postcomposed.");
                }
                match data.get_gate_type() {
                    GateType::H => {
                        self.postcompose_basis_change(Pauli::Y, data.get_args()[0], false)
                    }
                    GateType::S => {
                        self.postcompose_basis_change(Pauli::Z, data.get_args()[0], false)
                    }
                    GateType::V => {
                        self.postcompose_basis_change(Pauli::X, data.get_args()[0], false)
                    }
                    GateType::Sdg => {
                        self.postcompose_basis_change(Pauli::Z, data.get_args()[0], true)
                    }
                    GateType::Vdg => {
                        self.postcompose_basis_change(Pauli::X, data.get_args()[0], true)
                    }
                    GateType::X => self.postcompose_pauli(Pauli::X, data.get_args()[0]),
                    GateType::Y => self.postcompose_pauli(Pauli::Y, data.get_args()[0]),
                    GateType::Z => self.postcompose_pauli(Pauli::Z, data.get_args()[0]),
                    GateType::XX => self.postcompose_tqe(
                        Pauli::X,
                        Pauli::X,
                        data.get_args()[0],
                        data.get_args()[1],
                    ),
                    GateType::XY => self.postcompose_tqe(
                        Pauli::X,
                        Pauli::Y,
                        data.get_args()[0],
                        data.get_args()[1],
                    ),
                    GateType::XZ => self.postcompose_tqe(
                        Pauli::X,
                        Pauli::Z,
                        data.get_args()[0],
                        data.get_args()[1],
                    ),
                    GateType::YX => self.postcompose_tqe(
                        Pauli::Y,
                        Pauli::X,
                        data.get_args()[0],
                        data.get_args()[1],
                    ),
                    GateType::YY => self.postcompose_tqe(
                        Pauli::Y,
                        Pauli::Y,
                        data.get_args()[0],
                        data.get_args()[1],
                    ),
                    GateType::YZ => self.postcompose_tqe(
                        Pauli::Y,
                        Pauli::Z,
                        data.get_args()[0],
                        data.get_args()[1],
                    ),
                    GateType::ZX => self.postcompose_tqe(
                        Pauli::Z,
                        Pauli::X,
                        data.get_args()[0],
                        data.get_args()[1],
                    ),
                    GateType::ZY => self.postcompose_tqe(
                        Pauli::Z,
                        Pauli::Y,
                        data.get_args()[0],
                        data.get_args()[1],
                    ),
                    GateType::ZZ => self.postcompose_tqe(
                        Pauli::Z,
                        Pauli::Z,
                        data.get_args()[0],
                        data.get_args()[1],
                    ),
                    GateType::SWAP => self.postcompose_swap(data.get_args()[0], data.get_args()[1]),
                    GateType::RX => {
                        // TODO not efficient
                        let mut string = vec![Pauli::I; self.get_n_qubits()];
                        string[data.get_args()[0]] = Pauli::X;
                        self.postcompose_op(&Op::Rotation {
                            data: (RotationData::new(string, data.get_params()[0])),
                        })
                    }
                    GateType::RY => {
                        let mut string = vec![Pauli::I; self.get_n_qubits()];
                        string[data.get_args()[0]] = Pauli::Y;
                        self.postcompose_op(&Op::Rotation {
                            data: (RotationData::new(string, data.get_params()[0])),
                        })
                    }
                    GateType::RZ => {
                        let mut string = vec![Pauli::I; self.get_n_qubits()];
                        string[data.get_args()[0]] = Pauli::Z;
                        self.postcompose_op(&Op::Rotation {
                            data: (RotationData::new(string, data.get_params()[0])),
                        })
                    }
                    GateType::ZZPHASE => {
                        let mut string = vec![Pauli::I; self.get_n_qubits()];
                        string[data.get_args()[0]] = Pauli::Z;
                        string[data.get_args()[1]] = Pauli::Z;
                        self.postcompose_op(&Op::Rotation {
                            data: (RotationData::new(string, data.get_params()[0])),
                        })
                    }
                    GateType::PHASEDX => {
                        let alpha = data.get_params()[0];
                        let beta = data.get_params()[1];
                        // handle special cases
                        // if alpha is a multiple of 2pi, then this is identity
                        if equiv_0(alpha, 2.0) {
                            return;
                        }
                        // if alpha is a multiple of pi and beta is a multiple of pi/4, then this is a Clifford gate
                        if equiv_0(alpha, 1.0) && equiv_0(beta, 0.25) {
                            self.postcompose_op(&Op::Gate {
                                data: GateData::new(GateType::RX, data.get_args().clone())
                                    .with_params(vec![alpha]),
                            });
                            self.postcompose_op(&Op::Gate {
                                data: GateData::new(GateType::RZ, data.get_args().clone())
                                    .with_params(vec![2.0 * beta]),
                            });
                            return;
                        }
                        self.postcompose_op(&Op::Gate {
                            data: GateData::new(GateType::RZ, data.get_args().clone())
                                .with_params(vec![-beta]),
                        });
                        self.postcompose_op(&Op::Gate {
                            data: GateData::new(GateType::RX, data.get_args().clone())
                                .with_params(vec![alpha]),
                        });
                        self.postcompose_op(&Op::Gate {
                            data: GateData::new(GateType::RZ, data.get_args().clone())
                                .with_params(vec![beta]),
                        });
                    }
                    _ => panic!("Unexpected gate type: {:?}", data.get_gate_type()),
                }
            }
            Op::Rotation { data } => {
                let (z_vec, x_vec) = paulis_to_u64s(data.get_string());
                self.postcompose_pauli_gadget(
                    &z_vec,
                    &x_vec,
                    cliff_angle(data.get_angle()).expect("Pauli gadget has non clifford angle"),
                );
            }
            Op::Tableau { data } => {
                self.postcompose_tableau(&data.clone().into());
            }
            _ => {
                panic!("Unexpected operation type: {:?}", op);
            }
        }
    }

    fn precompose_tableau(&mut self, other: &Self) {
        let mut tab = other.clone();
        tab.compose(self);
        *self = tab;
    }

    fn postcompose_tableau(&mut self, other: &Self) {
        self.compose(other);
    }

    fn conjugate_string(&self, paulis: &[Pauli]) -> (Vec<Pauli>, bool) {
        let (z_vec, x_vec) = paulis_to_u64s(paulis);
        let (new_z_vec, new_x_vec, neg_sign) = self.apply_to_pauli(&z_vec, &x_vec);
        (
            u64s_to_paulis(&new_z_vec, &new_x_vec, self.get_n_qubits()),
            !neg_sign,
        )
    }
}

impl From<TableauData> for Tableau {
    fn from(data: TableauData) -> Self {
        let n_qubits = data.get_x_outputs().len();
        let mut zb_rows = Vec::with_capacity(n_qubits);
        let mut xb_rows = Vec::with_capacity(n_qubits);
        for row_idx in 0..n_qubits {
            let mut row = vec![Pauli::I; 2 * n_qubits];
            for col_idx in 0..n_qubits {
                row[2 * col_idx] = data.get_z_outputs()[col_idx].0[row_idx];
                row[2 * col_idx + 1] = data.get_x_outputs()[col_idx].0[row_idx];
            }
            let (zb_row, xb_row) = paulis_to_u64s(&row);
            zb_rows.push(zb_row);
            xb_rows.push(xb_row);
        }
        let mut neg_signs = vec![false; 2 * n_qubits];
        for col_idx in 0..n_qubits {
            neg_signs[2 * col_idx] = !data.get_z_outputs()[col_idx].1;
            neg_signs[2 * col_idx + 1] = !data.get_x_outputs()[col_idx].1;
        }
        Tableau::new(zb_rows, xb_rows, bools_to_u64_vec(&neg_signs), n_qubits)
    }
}

impl From<Tableau> for TableauData {
    fn from(tab: Tableau) -> Self {
        let n_qubits = tab.get_n_qubits();
        let mut z_outputs = Vec::with_capacity(n_qubits);
        let mut x_outputs = Vec::with_capacity(n_qubits);
        for col_idx in 0..n_qubits {
            z_outputs.push(tab.z(col_idx));
            x_outputs.push(tab.x(col_idx));
        }
        TableauData::new(z_outputs, x_outputs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pg_core::{PauliGraph, TableauData};
    use pg_tk::compare_unitaries_via_tk;
    use rand::{Rng, SeedableRng};
    use rstest::rstest;

    fn random_paulis(length: usize, seed: u64) -> Vec<Pauli> {
        let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
        (0..length)
            .map(|_| match rng.random_range(0..4) {
                0 => Pauli::I,
                1 => Pauli::X,
                2 => Pauli::Y,
                _ => Pauli::Z,
            })
            .collect()
    }

    #[test]
    fn test_conversion() {
        let z_outputs = vec![
            (vec![Pauli::X, Pauli::I, Pauli::Z], false),
            (vec![Pauli::I, Pauli::Y, Pauli::I], true),
            (vec![Pauli::Z, Pauli::I, Pauli::X], false),
        ];
        let x_outputs = vec![
            (vec![Pauli::I, Pauli::X, Pauli::I], true),
            (vec![Pauli::Y, Pauli::I, Pauli::Z], false),
            (vec![Pauli::I, Pauli::Z, Pauli::I], true),
        ];
        let tableau_data = TableauData::new(z_outputs.clone(), x_outputs.clone());
        let row_major_tableau: Tableau = tableau_data.into();
        let converted_back = TableauData::from(row_major_tableau);
        assert_eq!(converted_back.get_z_outputs(), &z_outputs);
        assert_eq!(converted_back.get_x_outputs(), &x_outputs);
    }

    #[test]
    fn test_identity_tableau_conversion() {
        let identity_tableau = Tableau::eye(2);
        let tableau_data: TableauData = identity_tableau.into();
        assert_eq!(
            tableau_data.get_z_outputs(),
            &vec![
                (vec![Pauli::Z, Pauli::I], true),
                (vec![Pauli::I, Pauli::Z], true),
            ]
        );
        assert_eq!(
            tableau_data.get_x_outputs(),
            &vec![
                (vec![Pauli::X, Pauli::I], true),
                (vec![Pauli::I, Pauli::X], true),
            ]
        );
    }

    #[test]
    fn test_random_pg_qm_tableau_round_trip() {
        for seed in 0..10 {
            let random_tableau = Tableau::random(5, seed, 100, 100);
            let tableau_data = TableauData::from(random_tableau.clone());
            let converted_back: Tableau = tableau_data.into();
            assert_eq!(random_tableau.get_zb_rows(), converted_back.get_zb_rows());
            assert_eq!(random_tableau.get_xb_rows(), converted_back.get_xb_rows());
            assert_eq!(random_tableau.get_n_qubits(), converted_back.get_n_qubits());
            assert_eq!(random_tableau.get_signs(), converted_back.get_signs());
        }
    }
    #[test]
    fn test_conjugation() {
        let mut tableau = Tableau::eye(2);
        let input_paulis = vec![Pauli::X, Pauli::Z];
        let rotation = Op::Rotation {
            data: RotationData::new(input_paulis.clone(), 0.1),
        };
        let results = tableau.conjugate(&rotation);
        assert_eq!(results.len(), 1);
        match &results[0] {
            Op::Rotation { data } => {
                assert_eq!(data.get_string(), &vec![Pauli::X, Pauli::Z]);
                assert_eq!(data.get_angle(), 0.1);
            }
            _ => panic!("Expected a rotation op"),
        }
        // Conjugate by H on the first qubit
        tableau.precompose_op(&Op::Gate {
            data: GateData::new(GateType::H, vec![0]),
        });
        let results = tableau.conjugate(&rotation);
        assert_eq!(results.len(), 1);
        match &results[0] {
            Op::Rotation { data } => {
                assert_eq!(data.get_string(), &vec![Pauli::Z, Pauli::Z]);
                assert_eq!(data.get_angle(), 0.1);
            }
            _ => panic!("Expected a rotation op"),
        }
    }

    #[test]
    fn test_identity_tableau() {
        let tableau = Tableau::eye(2);
        let z0 = tableau.z(0);
        assert_eq!(z0, (vec![Pauli::Z, Pauli::I], true));
        let x0 = tableau.x(0);
        assert_eq!(x0, (vec![Pauli::X, Pauli::I], true));
        let z1 = tableau.z(1);
        assert_eq!(z1, (vec![Pauli::I, Pauli::Z], true));
        let x1 = tableau.x(1);
        assert_eq!(x1, (vec![Pauli::I, Pauli::X], true));
    }

    #[rstest]
    #[case(0.0, 0.0)]
    #[case(2.0, 1.7)]
    #[case(0.0, 1.7)]
    #[case(-2.0, 1.7)]
    #[case(-1.0, 0.5)]
    #[case(3.0, 0.25)]
    #[case(1.0, -0.25)]
    fn test_postcompose_phasedx(#[case] alpha: f64, #[case] beta: f64) {
        let op = Op::Gate {
            data: GateData::new(GateType::PHASEDX, vec![0]).with_params(vec![alpha, beta]),
        };
        let mut tab = Tableau::eye(1);
        tab.postcompose_op(&op);
        let tab_pg = PauliGraph::new(1).with_ops(vec![Op::Tableau { data: tab.into() }]);
        let gate_pg = PauliGraph::new(1).with_ops(vec![op]);
        assert!(
            compare_unitaries_via_tk(&gate_pg, &tab_pg),
            "Postcomposition failed for alpha: {}, beta: {}",
            alpha,
            beta
        );
    }

    #[rstest]
    #[case(GateType::H, vec![0])]
    #[case(GateType::X, vec![0])]
    #[case(GateType::Y, vec![0])]
    #[case(GateType::Z, vec![0])]
    #[case(GateType::S, vec![0])]
    #[case(GateType::Sdg, vec![0])]
    #[case(GateType::V, vec![0])]
    #[case(GateType::Vdg, vec![0])]
    #[case(GateType::XX, vec![0, 1])]
    #[case(GateType::XY, vec![0, 1])]
    #[case(GateType::XZ, vec![0, 1])]
    #[case(GateType::YX, vec![0, 1])]
    #[case(GateType::YY, vec![0, 1])]
    #[case(GateType::YZ, vec![0, 1])]
    #[case(GateType::ZX, vec![0, 1])]
    #[case(GateType::ZY, vec![0, 1])]
    #[case(GateType::ZZ, vec![0, 1])]
    #[case(GateType::SWAP, vec![0, 1])]
    fn test_tableau_gate_correctness(#[case] gate_type: GateType, #[case] qubits: Vec<usize>) {
        let op = Op::Gate {
            data: GateData::new(gate_type.clone(), qubits),
        };
        let gate_pg = PauliGraph::new(2).with_ops(vec![op.clone()]);
        // precompose
        let mut tab = Tableau::eye(2);
        tab.precompose_op(&op);
        let tab_pg = PauliGraph::new(2).with_ops(vec![Op::Tableau { data: tab.into() }]);
        assert!(
            compare_unitaries_via_tk(&gate_pg, &tab_pg),
            "Precomposition failed for gate type: {:?}",
            gate_type
        );
        // postcompose
        let mut tab = Tableau::eye(2);
        tab.postcompose_op(&op);
        let tab_pg = PauliGraph::new(2).with_ops(vec![Op::Tableau { data: tab.into() }]);
        assert!(
            compare_unitaries_via_tk(&gate_pg, &tab_pg),
            "Postcomposition failed for gate type: {:?}",
            gate_type
        );
    }

    #[rstest]
    #[case(Op::Gate {
        data: GateData::new(GateType::RX, vec![0]).with_params(vec![0.5]),
    })]
    #[case(Op::Gate {
        data: GateData::new(GateType::RY, vec![1]).with_params(vec![0.5]),
    })]
    #[case(Op::Gate {
        data: GateData::new(GateType::RZ, vec![2]).with_params(vec![0.5]),
    })]
    #[case(Op::Gate {
        data: GateData::new(GateType::ZZPHASE, vec![0, 2]).with_params(vec![0.5]),
    })]
    #[case(Op::Gate {
        data: GateData::new(GateType::PHASEDX, vec![0]).with_params(vec![0.5, 0.5]),
    })]
    #[case(Op::Gate {
        data: GateData::new(GateType::PHASEDX, vec![1]).with_params(vec![3.0, 0.25]),
    })]
    #[case(Op::Gate {
        data: GateData::new(GateType::PHASEDX, vec![2]).with_params(vec![2.0, 1.7]),
    })]
    #[case(Op::Rotation {
        data: RotationData::new(vec![Pauli::X, Pauli::Y, Pauli::Z], 0.5),
    })]
    fn test_parameterized_clifford_op_correctness(#[case] op: Op) {
        let op_pg = PauliGraph::new(3).with_ops(vec![op.clone()]);

        let mut tab = Tableau::eye(3);
        tab.precompose_op(&op);
        let tab_pg = PauliGraph::new(3).with_ops(vec![Op::Tableau { data: tab.into() }]);
        assert!(
            compare_unitaries_via_tk(&op_pg, &tab_pg),
            "Precomposition failed for operation: {op:?}"
        );

        let mut tab = Tableau::eye(3);
        tab.postcompose_op(&op);
        let tab_pg = PauliGraph::new(3).with_ops(vec![Op::Tableau { data: tab.into() }]);
        assert!(
            compare_unitaries_via_tk(&op_pg, &tab_pg),
            "Postcomposition failed for operation: {op:?}"
        );
    }

    fn panic_message(payload: Box<dyn std::any::Any + Send>) -> String {
        if let Some(message) = payload.downcast_ref::<String>() {
            message.clone()
        } else if let Some(message) = payload.downcast_ref::<&str>() {
            (*message).to_owned()
        } else {
            "non-string panic payload".to_owned()
        }
    }

    #[rstest]
    #[case(Op::Gate {
        data: GateData::new(GateType::RX, vec![0]).with_params(vec![0.25]),
    })]
    #[case(Op::Gate {
        data: GateData::new(GateType::RY, vec![1]).with_params(vec![0.25]),
    })]
    #[case(Op::Gate {
        data: GateData::new(GateType::RZ, vec![2]).with_params(vec![0.25]),
    })]
    #[case(Op::Gate {
        data: GateData::new(GateType::ZZPHASE, vec![0, 2]).with_params(vec![0.25]),
    })]
    #[case(Op::Gate {
        data: GateData::new(GateType::PHASEDX, vec![1]).with_params(vec![0.5, 0.25]),
    })]
    #[case(Op::Rotation {
        data: RotationData::new(vec![Pauli::X, Pauli::Y, Pauli::Z], 0.25),
    })]
    fn test_non_clifford_parameterized_ops_are_rejected(#[case] op: Op) {
        let mut tab = Tableau::eye(3);
        let panic = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            tab.precompose_op(&op);
        }))
        .expect_err("precomposing a non-Clifford operation should panic");
        assert!(
            panic_message(panic).contains("Cannot precompose non-Clifford operation"),
            "unexpected precomposition panic for {op:?}"
        );

        let mut tab = Tableau::eye(3);
        let panic = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            tab.postcompose_op(&op);
        }))
        .expect_err("postcomposing a non-Clifford operation should panic");
        assert!(
            panic_message(panic).contains("Cannot postcompose non-Clifford operation"),
            "unexpected postcomposition panic for {op:?}"
        );
    }

    #[test]
    fn test_conjugation_correctness() {
        let tab = Tableau::random(5, 0, 100, 100);
        let tab_op = Op::Tableau {
            data: tab.clone().into(),
        };
        for seed in 0..10 {
            let paulis = random_paulis(5, seed);
            let rotation = Op::Rotation {
                data: RotationData::new(paulis.clone(), 0.1),
            };
            let results = tab.conjugate(&rotation);
            let pg_before = PauliGraph::new(5).with_ops(vec![rotation, tab_op.clone()]);
            let pg_after = PauliGraph::new(5).with_ops([vec![tab_op.clone()], results].concat());
            assert!(
                compare_unitaries_via_tk(&pg_before, &pg_after),
                "Conjugation failed for paulis: {:?}",
                paulis
            );
        }
    }
}
