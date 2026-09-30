use crate::simd_vector::SIMDVector;
use std::collections::{HashMap, hash_map::Entry};
use tk_pg_core::{Op, Pauli, PauliGraph, RotationData};

/// Parity table representation of a phase polynomial.
type PhasePolynomial = Vec<(SIMDVector, i32)>;

// Applies the todd algorithm to resynthesize the phase polynomial
// within the given pauli graph.
//
// graph must only be comprised of rotations with I/Z letters and
// the angles must be 0.25
pub fn t_optimization(graph: &mut PauliGraph) {
    let phase_polynomial = rotations_to_simd(graph);
    let reduced = apply_todd(phase_polynomial, graph.get_n_qubits());
    *graph = simd_to_rotations(reduced, graph.get_n_qubits());
}

/// Converts pauli graph rotations into SIMD vectors.
fn rotations_to_simd(graph: &PauliGraph) -> PhasePolynomial {
    let n = graph.get_n_qubits();
    let mut phase_polynomial = Vec::new();
    for op in graph.get_ops() {
        let Op::Rotation { data } = op else {
            panic!("phase polynomial resynthesis can only be applied to rotations");
        };
        let turns = 4.0 * data.get_angle();
        let coefficient = turns.round().rem_euclid(8.0) as i32;
        let mut parity = SIMDVector::new(n);

        for (q, p) in data.get_string().iter().enumerate() {
            match p {
                Pauli::Z => parity.flip_bit(q),
                Pauli::I => (),
                _ => panic!("phase resynthesis requires only I/Z rotations"),
            }
        }
        phase_polynomial.push((parity, coefficient));
    }

    phase_polynomial
}

// Converts a simd vector representation of a phase polynomial
// into its equivalent pauli graph representation.
fn simd_to_rotations(result: PhasePolynomial, n: usize) -> PauliGraph {
    let ops = result
        .into_iter()
        .filter(|(_, c)| c.rem_euclid(8) != 0)
        .map(|(p, c)| Op::Rotation {
            data: RotationData::new(
                (0..n)
                    .map(|q| if p.get(q) { Pauli::Z } else { Pauli::I })
                    .collect(),
                c.rem_euclid(8) as f64 / 4.0,
            ),
        })
        .collect();
    PauliGraph::new(n).with_ops(ops)
}

// Applies the todd algorithm to resynthesize the phase polynomial
fn apply_todd(mut phase_polynomial: PhasePolynomial, n: usize) -> PhasePolynomial {
    let table = phase_polynomial
        .iter()
        .filter(|(_, c)| c.rem_euclid(2) != 0)
        .map(|(p, _)| p.clone())
        .collect();

    let mut result: Vec<_> = todd(table, n).into_iter().map(|p| (p, 1)).collect();
    phase_polynomial.extend(result.iter().map(|(p, c)| (p.clone(), -c)));

    let mut linear: Vec<i32> = (0..n)
        .map(|q| {
            phase_polynomial
                .iter()
                .filter(|(p, _)| p.get(q))
                .map(|(_, c)| c)
                .sum()
        })
        .collect();

    for q in 0..n {
        for r in q + 1..n {
            let quadratic = phase_polynomial
                .iter()
                .filter(|(p, _)| p.get(q) && p.get(r))
                .map(|(_, c)| c)
                .sum::<i32>()
                .rem_euclid(4);

            if quadratic == 0 {
                continue;
            }

            let mut parity = SIMDVector::new(n);
            parity.flip_bit(q);
            parity.flip_bit(r);
            result.push((parity, quadratic));
            linear[q] -= quadratic;
            linear[r] -= quadratic;
        }

        let mut parity = SIMDVector::new(n);
        parity.flip_bit(q);
        result.push((parity, linear[q]));
    }

    result
}

/// Removes zero columns and cancels duplicate parity columns in pairs.
fn proper(mut table: Vec<SIMDVector>) -> Vec<SIMDVector> {
    let mut map = HashMap::with_capacity(table.len());
    let mut to_remove = Vec::new();
    for (i, col) in table.iter().enumerate() {
        if col.first_one().is_none() {
            to_remove.push(i);
        } else {
            match map.entry(col.packed_words()) {
                Entry::Occupied(entry) => {
                    to_remove.push(entry.remove());
                    to_remove.push(i);
                }
                Entry::Vacant(entry) => {
                    entry.insert(i);
                }
            }
        }
    }
    to_remove.sort_unstable_by(|a, b| b.cmp(a));
    for i in to_remove {
        table.swap_remove(i);
    }
    table
}

/// Reduces a row against the binary basis using XOR, inserting it if independent.
/// Returns whether the insertion increased the rank of the basis.
fn insert_row(basis: &mut [Option<SIMDVector>], mut row: SIMDVector) -> bool {
    while let Some(pivot) = row.first_one() {
        if let Some(existing) = &basis[pivot] {
            row.xor(existing);
        } else {
            basis[pivot] = Some(row);
            return true;
        }
    }
    false
}

/// Find a vector orthogonal to the basis with different bits at `i` and `j`.
fn separating_kernel(basis: &[Option<SIMDVector>], i: usize, j: usize) -> Option<SIMDVector> {
    let mut difference = SIMDVector::new(basis.len());
    difference.flip_bit(i);
    difference.flip_bit(j);
    for (pivot, row) in basis.iter().enumerate() {
        if difference.get(pivot)
            && let Some(row) = row
        {
            difference.xor(row);
        }
    }
    let free = difference.first_one()?;
    let mut y = SIMDVector::new(basis.len());
    y.flip_bit(free);
    for (pivot, row) in basis.iter().enumerate().rev() {
        if let Some(row) = row
            && row.dot_parity(&y)
        {
            y.flip_bit(pivot);
        }
    }
    Some(y)
}

/// Looks up the cached bitwise AND of two distinct rows, regardless of index order.
fn product(products: &[Vec<SIMDVector>], a: usize, b: usize) -> &SIMDVector {
    let (a, b) = (a.min(b), a.max(b));
    &products[a][b - a - 1]
}

/// Reduce the number of parity columns using TODD, preserving the phase polynomial
/// up to a Clifford correction.
pub fn todd(table: Vec<SIMDVector>, nb_qubits: usize) -> Vec<SIMDVector> {
    let mut table = proper(table);
    loop {
        let m = table.len();
        if m < 2 {
            return table;
        }
        let rows: Vec<_> = (0..nb_qubits)
            .filter_map(|q| {
                let mut row = SIMDVector::new(m);
                for (k, col) in table.iter().enumerate() {
                    if col.get(q) {
                        row.flip_bit(k);
                    }
                }
                row.first_one().map(|_| row)
            })
            .collect();
        let mut base = vec![None; m];
        let mut base_rank = 0;
        for row in &rows {
            base_rank += usize::from(insert_row(&mut base, row.clone()));
        }
        if base_rank == m {
            return table;
        }
        let products: Vec<Vec<_>> = (0..rows.len())
            .map(|a| {
                (a + 1..rows.len())
                    .map(|b| {
                        let mut row = rows[a].clone();
                        row.and(&rows[b]);
                        row
                    })
                    .collect()
            })
            .collect();
        let mut replacement = None;
        'pairs: for i in 0..m {
            for j in i + 1..m {
                let z: Vec<_> = rows.iter().map(|r| r.get(i) ^ r.get(j)).collect();
                let pivot = z
                    .iter()
                    .position(|&b| b)
                    .expect("proper columns are distinct");
                let mut basis = base.clone();
                let mut rank = base_rank;
                'constraints: for a in 0..rows.len() {
                    if a == pivot {
                        continue;
                    }
                    for b in a + 1..rows.len() {
                        if b == pivot {
                            continue;
                        }
                        let mut row = product(&products, a, b).clone();
                        if z[a] {
                            row.xor(product(&products, pivot, b));
                        }
                        if z[b] {
                            row.xor(product(&products, pivot, a));
                        }
                        rank += usize::from(insert_row(&mut basis, row));
                        if rank == m {
                            break 'constraints;
                        }
                    }
                }
                if rank == m {
                    continue;
                }
                if let Some(y) = separating_kernel(&basis, i, j) {
                    let mut z = table[i].clone();
                    z.xor(&table[j]);
                    let mut candidate = table.clone();
                    for (k, col) in candidate.iter_mut().enumerate() {
                        if y.get(k) {
                            col.xor(&z);
                        }
                    }
                    if y.popcount() % 2 == 1 {
                        candidate.push(z);
                    }
                    candidate = proper(candidate);
                    if candidate.len() < m {
                        replacement = Some(candidate);
                        break 'pairs;
                    }
                }
            }
        }
        match replacement {
            Some(next) => table = next,
            None => return table,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn signature_tensor(table: &[SIMDVector], nb_qubits: usize) -> Vec<Vec<Vec<bool>>> {
        let mut tensor = vec![vec![vec![false; nb_qubits]; nb_qubits]; nb_qubits];

        for alpha in 0..nb_qubits {
            for beta in alpha..nb_qubits {
                for gamma in beta..nb_qubits {
                    let mut count = 0u32;
                    for col in table {
                        if col.get(alpha) && col.get(beta) && col.get(gamma) {
                            count += 1;
                        }
                    }
                    let val = count % 2 == 1;
                    tensor[alpha][beta][gamma] = val;
                    tensor[alpha][gamma][beta] = val;
                    tensor[beta][alpha][gamma] = val;
                    tensor[beta][gamma][alpha] = val;
                    tensor[gamma][alpha][beta] = val;
                    tensor[gamma][beta][alpha] = val;
                }
            }
        }

        tensor
    }

    struct Lcg {
        state: u64,
    }

    impl Lcg {
        fn new(seed: u64) -> Self {
            Lcg { state: seed }
        }

        fn next_u64(&mut self) -> u64 {
            self.state = self.state.wrapping_mul(6364136223846793005).wrapping_add(1);
            self.state
        }

        fn next_bool(&mut self) -> bool {
            self.next_u64() >> 63 != 0
        }
    }

    fn random_parity_table(nb_qubits: usize, nb_columns: usize, seed: u64) -> Vec<SIMDVector> {
        let mut rng = Lcg::new(seed);
        (0..nb_columns)
            .map(|_| {
                let mut col = SIMDVector::new(nb_qubits);
                for q in 0..nb_qubits {
                    if rng.next_bool() {
                        col.flip_bit(q);
                    }
                }
                col
            })
            .collect()
    }

    #[test]
    fn signature_tensor_two_duplicate_columns() {
        let nb_qubits = 2;
        let mut col = SIMDVector::new(nb_qubits);
        col.flip_bit(0);
        col.flip_bit(1);

        let table = vec![col.clone(), col];
        let s = signature_tensor(&table, nb_qubits);

        assert!(!s[0][0][0]);
        assert!(!s[0][0][1]);
        assert!(!s[0][1][1]);
        assert!(!s[1][1][1]);
    }

    #[test]
    fn todd_preserves_signature_tensor() {
        let nb_qubits = 4;
        let nb_columns = 12;

        for seed in 0..20 {
            let table = random_parity_table(nb_qubits, nb_columns, seed);
            let original = signature_tensor(&table, nb_qubits);

            let reduced = todd(table, nb_qubits);
            let after = signature_tensor(&reduced, nb_qubits);

            assert_eq!(original, after, "signature tensor changed for seed {seed}");
        }
    }

    #[test]
    fn todd_preserves_signature_tensor_larger() {
        let nb_qubits = 6;
        let nb_columns = 30;

        let table = random_parity_table(nb_qubits, nb_columns, 99);
        let original = signature_tensor(&table, nb_qubits);

        let reduced = todd(table, nb_qubits);
        let after = signature_tensor(&reduced, nb_qubits);

        assert_eq!(original, after);
    }

    #[test]
    fn todd_reduces_column_count() {
        let nb_qubits = 5;
        let nb_columns = 20;

        let table = random_parity_table(nb_qubits, nb_columns, 7);
        let original_len = table.len();

        let reduced = todd(table, nb_qubits);

        assert!(reduced.len() <= original_len);
    }

    #[test]
    fn todd_no_duplicates_in_output() {
        let nb_qubits = 5;
        let nb_columns = 20;

        let table = random_parity_table(nb_qubits, nb_columns, 13);
        let reduced = todd(table, nb_qubits);

        for i in 0..reduced.len() {
            for j in (i + 1)..reduced.len() {
                let mut diff = reduced[i].clone();
                diff.xor(&reduced[j]);
                assert!(
                    diff.first_one().is_some(),
                    "duplicate columns {i} and {j} in output"
                );
            }
        }
    }
}
