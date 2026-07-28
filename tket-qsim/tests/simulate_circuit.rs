//! End-to-end tests for HUGR traversal and QSystem lowering equivalence.

use std::collections::HashMap;

use hugr::{
    builder::{Container, DFGBuilder, Dataflow, DataflowHugr},
    extension::prelude::qb_t,
    ops::Value,
    types::Signature,
};
use num_complex::Complex64;
use tket::{
    TketOp,
    extension::rotation::ConstRotation,
    passes::{
        ComposablePass, InlineDFGsPass, InlineFunctionsPass, inline_funcs::InlineFuncsHeuristic,
    },
};
use tket_qsim::simulate_circuit;
use tket_qsystem::extension::qsystem::{LowerTketToQSystemPass, QSystemPlatform};

fn reference_circuit() -> hugr::Hugr {
    let mut builder = DFGBuilder::new(Signature::new_endo([qb_t(), qb_t()])).unwrap();
    let [q0, q1] = builder.input_wires_arr();
    let [q0] = builder
        .add_dataflow_op(TketOp::H, [q0])
        .unwrap()
        .outputs_arr();
    let [q0, q1] = builder
        .add_dataflow_op(TketOp::CX, [q0, q1])
        .unwrap()
        .outputs_arr();
    let angle = builder.add_constant(Value::extension(ConstRotation::new(0.5).unwrap()));
    let angle = builder.load_const(&angle);
    let [q1] = builder
        .add_dataflow_op(TketOp::Rz, [q1, angle])
        .unwrap()
        .outputs_arr();
    builder.finish_hugr_with_outputs([q0, q1]).unwrap()
}

#[test]
fn composed_circuit_matches_pytket_reference() {
    // Equivalent pytket circuit:
    // Circuit(2).H(0).CX(0, 1).Rz(0.5, 1)
    let hugr = reference_circuit();

    let unitary = simulate_circuit(&hugr, &HashMap::new()).unwrap();
    let p = Complex64::new(0.5, 0.5);
    let m = Complex64::new(0.5, -0.5);
    let z = Complex64::ZERO;
    let expected = [
        m, z, m, z, // row 0
        z, p, z, p, // row 1
        z, m, z, -m, // row 2
        p, z, -p, z, // row 3
    ];

    for row in 0..4 {
        for col in 0..4 {
            let delta = unitary.get(row, col) - expected[row * 4 + col];
            assert!(
                delta.norm() < 1e-12,
                "mismatch at ({row}, {col}): got {}, expected {}",
                unitary.get(row, col),
                expected[row * 4 + col]
            );
        }
    }
}

#[test]
fn lowering_to_each_qsystem_platform_preserves_the_unitary() {
    let original = reference_circuit();
    let expected = simulate_circuit(&original, &HashMap::new()).unwrap();

    for platform in [QSystemPlatform::Helios, QSystemPlatform::Sol] {
        let mut lowered = original.clone();
        LowerTketToQSystemPass::new(platform)
            .run(&mut lowered)
            .unwrap();
        // Multi-operation lowerings are represented as calls. The lightweight
        // simulator operates on a flat dataflow graph, so normalize those
        // calls and their nested DFGs before comparing semantics.
        InlineFunctionsPass::default()
            .with_heuristic(InlineFuncsHeuristic::All)
            .run(&mut lowered)
            .unwrap();
        InlineDFGsPass::default().run(&mut lowered).unwrap();
        let actual = simulate_circuit(&lowered, &HashMap::new()).unwrap();
        assert!(
            actual.approx_eq_up_to_global_phase(&expected, 1e-10),
            "lowering to {platform:?} changed the circuit unitary\nexpected: {expected:?}\nactual: {actual:?}"
        );
    }
}
