use crate::clifford_propagation::{collect_cliffords, normalize, tableau_op};
use crate::hadamard_optimizer::{gadgetize, synthesize};
use crate::t_optimizer::t_optimization;

use bitvec::prelude::*;
use tk_pg_core::{GateData, GateType, Op, Pauli, PauliGraph, TableauData};
use tk_pg_ir_kernels::{PGRewrite, PGTableau};
use tk_pg_qm_tableau::Tableau;

pub fn optimize(pauli_graph: &PauliGraph, budget: usize) -> PauliGraph {
    validate_ancillas(pauli_graph, budget);

    let n_qubits = pauli_graph.get_n_qubits();
    let ancillas = n_qubits - budget..n_qubits;
    let (rotations, mut final_clifford) = normalize(pauli_graph);
    let mut output = PauliGraph::new(n_qubits);

    for a in ancillas {
        let h = GateData::new(GateType::H, vec![a]);
        final_clifford.precompose_op(&Op::Gate { data: h.clone() });
        output.add_op(Op::Gate { data: h });
    }

    let mut next_bit = 0;
    for sub_graph in batch_rotations(&rotations, budget) {
        let rotations: Vec<_> = sub_graph
            .get_ops()
            .iter()
            .map(|op| match op {
                Op::Rotation { data } => data.clone(),
                _ => unreachable!(),
            })
            .collect();
        let region = synthesize(&rotations, n_qubits);
        let (prefix, mut diagonal, suffix) = gadgetize(region, budget, &mut next_bit);

        t_optimization(&mut diagonal);

        output.extend(prefix);
        output.extend(diagonal);
        output.extend(suffix);
    }
    output.add_op(tableau_op(final_clifford));

    let output = collect_cliffords(output);
    output
        .try_validate()
        .expect("invalid T-optimization output");

    output
}

fn validate_ancillas(graph: &PauliGraph, budget: usize) {
    if budget == 0 {
        return;
    }

    let n_qubits = graph.get_n_qubits();
    assert!(
        budget <= n_qubits,
        "ancilla budget exceeds input qubit count"
    );
    let identity = TableauData::from(Tableau::eye(n_qubits));
    for q in n_qubits - budget..n_qubits {
        let idle = graph.get_ops().iter().all(|op| match op {
            Op::Rotation { data } => data.get_string()[q] == Pauli::I,
            Op::Gate { data } => !data.get_args().contains(&q),
            Op::Tableau { data } => {
                data.get_x_images()[q] == identity.get_x_images()[q]
                    && data.get_z_images()[q] == identity.get_z_images()[q]
            }
            _ => true,
        });
        assert!(idle, "Reserved ancilla qubit {q} is not idle");
    }
}

fn batch_rotations(pauli_graph: &PauliGraph, budget: usize) -> Vec<PauliGraph> {
    batch_indices(pauli_graph, budget)
        .iter()
        .map(|batch| subgraph_from(pauli_graph, batch))
        .collect()
}

pub fn batch_indices(pauli_graph: &PauliGraph, budget: usize) -> Vec<Vec<usize>> {
    let mut matrix = get_commutation_matrix(pauli_graph);
    let mut index_map: Vec<usize> = (0..matrix.len()).collect();
    let mut batches: Vec<Vec<usize>> = Vec::new();

    while !matrix.is_empty() {
        for _ in 0..budget {
            let roots = get_roots_mask(&matrix);
            if roots.count_ones() == matrix.len() {
                break;
            }

            if let Some(node) = get_root_child(&matrix, &roots) {
                pivot(&mut matrix, node);
            } else {
                break;
            }
        }

        let roots = get_roots_mask(&matrix);
        let batch: Vec<usize> = roots.iter_ones().map(|i| index_map[i]).collect();
        batches.push(batch);

        let root_indices: Vec<usize> = roots.iter_ones().collect();
        remove_nodes(&mut matrix, &mut index_map, &root_indices);
    }

    batches
}

/// Build a symmetric matrix whose set bits mark anticommuting rotations.
fn get_commutation_matrix(pauli_graph: &PauliGraph) -> Vec<BitVec<usize>> {
    let n = pauli_graph.get_ops().len();
    let mut matrix = vec![BitVec::repeat(false, n); n];
    for i in 0..n {
        for j in i + 1..n {
            let anticommutes = !pauli_graph.do_commute::<Tableau>(i, j);
            matrix[i].set(j, anticommutes);
            matrix[j].set(i, anticommutes);
        }
    }
    matrix
}

/// Returns a subgraph of the PauliGraph that contains only the specified nodes.
fn subgraph_from(pauli_graph: &PauliGraph, nodes: &[usize]) -> PauliGraph {
    let ops = pauli_graph.get_ops();
    let mut subgraph = PauliGraph::new(pauli_graph.get_n_qubits());
    for &i in nodes {
        subgraph.add_op(ops[i].clone());
    }
    subgraph
}

/// Returns a bitmask of all root nodes (nodes with no predecessors).
fn get_roots_mask(matrix: &[BitVec<usize>]) -> BitVec<usize> {
    let n = matrix.len();
    let mut roots = BitVec::<usize>::repeat(false, n);

    for (i, row) in matrix.iter().enumerate() {
        let has_predecessor = row[..i].any();
        if !has_predecessor {
            roots.set(i, true);
        }
    }
    roots
}

/// Returns a node whose predecessors are all roots, if one exists.
fn get_root_child(matrix: &[BitVec<usize>], roots: &BitVec<usize>) -> Option<usize> {
    for (i, row) in matrix.iter().enumerate() {
        if roots[i] {
            continue;
        }
        let predecessors = &row[..i];

        let roots_prefix = &roots[..i];
        let non_root_predecessors = predecessors
            .iter()
            .zip(roots_prefix.iter())
            .any(|(pred, root)| *pred && !*root);

        if !non_root_predecessors {
            return Some(i);
        }
    }
    None
}

/// Performs edge complementation to flatten the graph at the given node.
/// This corresponds to using one ancilla to make `node` a root.
fn pivot(matrix: &mut [BitVec<usize>], node: usize) {
    let n = matrix.len();
    let predecessors: Vec<usize> = matrix[node][..node].iter_ones().collect();
    if predecessors.is_empty() {
        return;
    }

    let v = predecessors[0];
    let mut successor_mask = BitVec::<usize>::repeat(false, n);
    for (j, successor) in matrix[v].iter().enumerate().skip(v + 1) {
        if *successor {
            successor_mask.set(j, true);
        }
    }

    for &pred in &predecessors {
        for j in successor_mask.iter_ones() {
            let current = matrix[pred][j];
            matrix[pred].set(j, !current);
            matrix[j].set(pred, !current);
        }
    }
}

/// Removes the specified nodes from the matrix and updates the index mapping.
fn remove_nodes(
    matrix: &mut Vec<BitVec<usize>>,
    index_map: &mut Vec<usize>,
    nodes_to_remove: &[usize],
) {
    let mut sorted: Vec<usize> = nodes_to_remove.to_vec();
    sorted.sort_by(|a, b| b.cmp(a));

    for &node in &sorted {
        matrix.remove(node);
        index_map.remove(node);

        for row in matrix.iter_mut() {
            if node < row.len() {
                row.remove(node);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;
    use tk_pg_core::RotationData;

    fn get_batch_indices(pauli_graph: &PauliGraph, budget: usize) -> Vec<Vec<usize>> {
        batch_indices(pauli_graph, budget)
    }

    #[test]
    fn test_batch_rotations_covers_all_ops() {
        let pg = PauliGraph::new(8).with_ops(
            (0..50)
                .map(|i| Op::Rotation {
                    data: RotationData::new(Tableau::random(8, 42 + i, 20, 20).z_image(0).0, 0.25),
                })
                .collect(),
        );
        let batches = get_batch_indices(&pg, 3);

        let mut all_indices: Vec<usize> = batches.iter().flatten().copied().collect();
        all_indices.sort();

        let expected: Vec<usize> = (0..pg.get_ops().len()).collect();
        assert_eq!(all_indices, expected);
    }

    #[test]
    fn test_batch_rotations_no_duplicates() {
        let pg = PauliGraph::new(8).with_ops(
            (0..50)
                .map(|i| Op::Rotation {
                    data: RotationData::new(Tableau::random(8, 123 + i, 20, 20).z_image(0).0, 0.25),
                })
                .collect(),
        );
        let batches = get_batch_indices(&pg, 3);

        let all_indices: Vec<usize> = batches.iter().flatten().copied().collect();
        let unique: HashSet<usize> = all_indices.iter().copied().collect();

        assert_eq!(all_indices.len(), unique.len());
    }

    #[test]
    fn test_batch_rotations_respects_dependencies() {
        let pg = PauliGraph::new(8).with_ops(
            (0..50)
                .map(|i| Op::Rotation {
                    data: RotationData::new(Tableau::random(8, 456 + i, 20, 20).z_image(0).0, 0.25),
                })
                .collect(),
        );
        let batches = get_batch_indices(&pg, 0);
        let matrix = get_commutation_matrix(&pg);

        let mut op_to_batch: Vec<usize> = vec![0; pg.get_ops().len()];
        for (batch_idx, batch) in batches.iter().enumerate() {
            for &op_idx in batch {
                op_to_batch[op_idx] = batch_idx;
            }
        }

        for i in 0..matrix.len() {
            for j in (i + 1)..matrix.len() {
                if matrix[i][j] {
                    assert!(op_to_batch[j] >= op_to_batch[i]);
                }
            }
        }
    }

    #[test]
    fn test_batch_rotations_empty_graph() {
        let pg = PauliGraph::new(4);
        let batches = batch_rotations(&pg, 3);
        assert!(batches.is_empty());
    }

    #[test]
    fn test_batch_rotations_single_op() {
        use tk_pg_core::{Op, Pauli, RotationData};

        let mut pg = PauliGraph::new(2);
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::Z], 0.5),
        });

        let batches = batch_rotations(&pg, 3);
        assert_eq!(batches.len(), 1);
        assert_eq!(batches[0].get_ops().len(), 1);
    }

    #[test]
    fn test_batch_rotations_commuting() {
        use tk_pg_core::{Op, Pauli, RotationData};

        let mut pg = PauliGraph::new(4);
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::Z, Pauli::I, Pauli::I, Pauli::I], 0.1),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::I, Pauli::Z, Pauli::I, Pauli::I], 0.2),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::I, Pauli::I, Pauli::Z, Pauli::I], 0.3),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::I, Pauli::I, Pauli::I, Pauli::Z], 0.4),
        });

        let batches = get_batch_indices(&pg, 0);

        assert_eq!(batches.len(), 1);
        assert_eq!(batches[0].len(), 4);
    }

    #[test]
    fn test_batch_rotations_anticommuting() {
        use tk_pg_core::{Op, Pauli, RotationData};

        let mut pg = PauliGraph::new(2);
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::I], 0.1),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::Z, Pauli::I], 0.2),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::I], 0.3),
        });

        let batches = get_batch_indices(&pg, 0);

        assert_eq!(batches.len(), 3);
    }

    #[test]
    fn test_batch_rotations_anticommuting_with_budget() {
        use tk_pg_core::{Op, Pauli, RotationData};

        let mut pg = PauliGraph::new(2);
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::I], 0.1),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::Z, Pauli::I], 0.2),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::I], 0.3),
        });

        let batches = get_batch_indices(&pg, 1);

        assert_eq!(batches.len(), 2);
        assert_eq!(batches[0].len(), 2);
        assert_eq!(batches[1].len(), 1);
    }
}
