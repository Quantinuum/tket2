use crate::simd_vector::SIMDVector;
use std::collections::{HashMap, hash_map::Entry};
use tk_pg_core::{Op, Pauli, PauliGraph, RotationData};

type PhasePolynomial = Vec<(SIMDVector, i32)>;

pub fn t_optimization(graph: &mut PauliGraph) {
    let polynomial = rotations_to_simd(graph);
    let reduced = apply_todd(polynomial, graph.get_n_qubits());
    *graph = simd_to_rotations(reduced, graph.get_n_qubits());
}

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

fn apply_todd(mut weighted: PhasePolynomial, n: usize) -> PhasePolynomial {
    let table = weighted
        .iter()
        .filter(|(_, c)| c.rem_euclid(2) != 0)
        .map(|(p, _)| p.clone())
        .collect();

    let mut result: Vec<_> = todd(table, n).into_iter().map(|p| (p, 1)).collect();
    weighted.extend(result.iter().map(|(p, c)| (p.clone(), -c)));

    let mut linear: Vec<i32> = (0..n)
        .map(|q| {
            weighted
                .iter()
                .filter(|(p, _)| p.get(q))
                .map(|(_, c)| c)
                .sum()
        })
        .collect();

    for q in 0..n {
        for r in q + 1..n {
            let quadratic = weighted
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

fn product(products: &[Vec<SIMDVector>], a: usize, b: usize) -> &SIMDVector {
    let (a, b) = (a.min(b), a.max(b));
    &products[a][b - a - 1]
}

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
