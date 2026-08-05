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
    fn x_image(&self, qubit: usize) -> (Vec<Pauli>, bool) {
        Tableau::x_image(self, qubit)
    }
    fn z_image(&self, qubit: usize) -> (Vec<Pauli>, bool) {
        Tableau::z_image(self, qubit)
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
                    GateType::H => self.precompose_h(data.get_args()[0]),
                    GateType::S => self.precompose_half_pi(Pauli::Z, data.get_args()[0], false),
                    GateType::V => self.precompose_half_pi(Pauli::X, data.get_args()[0], false),
                    GateType::Sdg => self.precompose_half_pi(Pauli::Z, data.get_args()[0], true),
                    GateType::Vdg => self.precompose_half_pi(Pauli::X, data.get_args()[0], true),
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
                    GateType::H => self.postcompose_h(data.get_args()[0]),
                    GateType::S => self.postcompose_half_pi(Pauli::Z, data.get_args()[0], false),
                    GateType::V => self.postcompose_half_pi(Pauli::X, data.get_args()[0], false),
                    GateType::Sdg => self.postcompose_half_pi(Pauli::Z, data.get_args()[0], true),
                    GateType::Vdg => self.postcompose_half_pi(Pauli::X, data.get_args()[0], true),
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
                        if equiv_0(alpha, 1.0) {
                            assert!(equiv_0(beta, 0.25), "is_clifford should guarantee this");
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
                let (pauli_z_bits, pauli_x_bits) = paulis_to_u64s(data.get_string());
                self.postcompose_pauli_gadget(
                    &pauli_z_bits,
                    &pauli_x_bits,
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
        let (input_z_bits, input_x_bits) = paulis_to_u64s(paulis);
        let (result_z_bits, result_x_bits, result_sign_bit) =
            self.apply_to_pauli(&input_z_bits, &input_x_bits);
        (
            u64s_to_paulis(&result_z_bits, &result_x_bits, self.get_n_qubits()),
            result_sign_bit,
        )
    }
}

impl From<TableauData> for Tableau {
    fn from(data: TableauData) -> Self {
        let n_qubits = data.get_x_images().len();
        let mut qubit_slices_z_bits = Vec::with_capacity(n_qubits);
        let mut qubit_slices_x_bits = Vec::with_capacity(n_qubits);
        for output_q in 0..n_qubits {
            let mut slice = vec![Pauli::I; 2 * n_qubits];
            for input_q in 0..n_qubits {
                slice[2 * input_q] = data.get_z_images()[input_q].0[output_q];
                slice[2 * input_q + 1] = data.get_x_images()[input_q].0[output_q];
            }
            let (slice_z_bits, slice_x_bits) = paulis_to_u64s(&slice);
            qubit_slices_z_bits.push(slice_z_bits);
            qubit_slices_x_bits.push(slice_x_bits);
        }
        let mut sign_bits = vec![false; 2 * n_qubits];
        for input_q in 0..n_qubits {
            sign_bits[2 * input_q] = data.get_z_images()[input_q].1;
            sign_bits[2 * input_q + 1] = data.get_x_images()[input_q].1;
        }
        Tableau::from_packed_qubit_slices(
            qubit_slices_z_bits,
            qubit_slices_x_bits,
            bools_to_u64_vec(&sign_bits),
            n_qubits,
        )
    }
}

impl From<Tableau> for TableauData {
    fn from(tab: Tableau) -> Self {
        let n_qubits = tab.get_n_qubits();
        let mut z_images = Vec::with_capacity(n_qubits);
        let mut x_images = Vec::with_capacity(n_qubits);
        for input_q in 0..n_qubits {
            z_images.push(tab.z_image(input_q));
            x_images.push(tab.x_image(input_q));
        }
        TableauData::new(z_images, x_images)
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
        let z_images = vec![
            (vec![Pauli::X, Pauli::I, Pauli::Z], false),
            (vec![Pauli::I, Pauli::Y, Pauli::I], true),
            (vec![Pauli::Z, Pauli::I, Pauli::X], false),
        ];
        let x_images = vec![
            (vec![Pauli::I, Pauli::X, Pauli::I], true),
            (vec![Pauli::Y, Pauli::I, Pauli::Z], false),
            (vec![Pauli::I, Pauli::Z, Pauli::I], true),
        ];
        let tableau_data = TableauData::new(z_images.clone(), x_images.clone());
        let qm_tableau: Tableau = tableau_data.into();
        let converted_back = TableauData::from(qm_tableau);
        assert_eq!(converted_back.get_z_images(), &z_images);
        assert_eq!(converted_back.get_x_images(), &x_images);
    }

    #[test]
    fn test_identity_tableau_conversion() {
        let identity_tableau = Tableau::eye(2);
        let tableau_data: TableauData = identity_tableau.into();
        assert_eq!(
            tableau_data.get_z_images(),
            &vec![
                (vec![Pauli::Z, Pauli::I], false),
                (vec![Pauli::I, Pauli::Z], false),
            ]
        );
        assert_eq!(
            tableau_data.get_x_images(),
            &vec![
                (vec![Pauli::X, Pauli::I], false),
                (vec![Pauli::I, Pauli::X], false),
            ]
        );
    }

    #[test]
    fn test_random_pg_qm_tableau_round_trip() {
        for (n_qubits, seed) in [(5, 0), (63, 1), (64, 2), (65, 3)] {
            let random_tableau = Tableau::random(n_qubits, seed, 20, 20);
            let tableau_data = TableauData::from(random_tableau.clone());
            let converted_back: Tableau = tableau_data.into();
            assert_eq!(
                random_tableau.qubit_slices_z_bits(),
                converted_back.qubit_slices_z_bits()
            );
            assert_eq!(
                random_tableau.qubit_slices_x_bits(),
                converted_back.qubit_slices_x_bits()
            );
            assert_eq!(random_tableau.get_n_qubits(), converted_back.get_n_qubits());
            assert_eq!(
                random_tableau.get_sign_bits(),
                converted_back.get_sign_bits()
            );
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
        let z0 = tableau.z_image(0);
        assert_eq!(z0, (vec![Pauli::Z, Pauli::I], false));
        let x0 = tableau.x_image(0);
        assert_eq!(x0, (vec![Pauli::X, Pauli::I], false));
        let z1 = tableau.z_image(1);
        assert_eq!(z1, (vec![Pauli::I, Pauli::Z], false));
        let x1 = tableau.x_image(1);
        assert_eq!(x1, (vec![Pauli::I, Pauli::X], false));
    }

    #[rstest]
    #[case(0.0, 0.0)]
    #[case(2.0, 1.7)]
    #[case(0.0, 1.7)]
    #[case(-2.0, 1.7)]
    #[case(-1.0, 0.5)]
    #[case(3.0, 0.25)]
    #[case(1.0, -0.25)]
    #[case(0.5, -0.5)]
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
    #[case(GateType::XX)]
    #[case(GateType::XY)]
    #[case(GateType::XZ)]
    #[case(GateType::YX)]
    #[case(GateType::YY)]
    #[case(GateType::YZ)]
    #[case(GateType::ZX)]
    #[case(GateType::ZY)]
    #[case(GateType::ZZ)]
    fn test_tableau_tqe_precompose_correctness_on_random_tableau(#[case] gate_type: GateType) {
        let op = Op::Gate {
            data: GateData::new(gate_type.clone(), vec![0, 1]),
        };
        for seed in 0..10 {
            let mut tab = Tableau::random(2, seed, 10, 10);
            let random_tab_op = Op::Tableau {
                data: tab.clone().into(),
            };
            let expected_pg = PauliGraph::new(2).with_ops(vec![op.clone(), random_tab_op]);

            tab.precompose_op(&op);
            let tab_pg = PauliGraph::new(2).with_ops(vec![Op::Tableau { data: tab.into() }]);

            assert!(
                compare_unitaries_via_tk(&expected_pg, &tab_pg),
                "Precomposition onto a random tableau failed for gate type: {:?}, seed: {}",
                gate_type,
                seed
            );
        }
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
