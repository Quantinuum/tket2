use pg_core::Pauli;
use rand::{Rng, SeedableRng};

pub(crate) fn pauli_to_bits(pauli: &Pauli) -> (bool, bool) {
    match pauli {
        Pauli::X => (false, true),
        Pauli::Y => (true, true),
        Pauli::Z => (true, false),
        Pauli::I => (false, false),
    }
}

pub(crate) fn bits_to_pauli(z: bool, x: bool) -> Pauli {
    match (z, x) {
        (false, false) => Pauli::I,
        (false, true) => Pauli::X,
        (true, false) => Pauli::Z,
        (true, true) => Pauli::Y,
    }
}

pub(crate) fn random_paulis(length: usize, seed: u64) -> Vec<Pauli> {
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

pub(crate) fn random_u64s(len: usize, seed: u64) -> Vec<u64> {
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    (0..len).map(|_| rng.random()).collect()
}
