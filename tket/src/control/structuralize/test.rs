use crate::control::nest_cfgs::test::build_conditional_in_loop_cfg;
use crate::passes::composable::WithScope;
use crate::passes::structuralize_cfgs::StructuralizeCfgsPass;
use crate::passes::{ComposablePass, PassScope};
use hugr::builder::{
    BuildError, CFGBuilder, Container, Dataflow, DataflowSubContainer, HugrBuilder, ModuleBuilder,
    endo_sig,
};
use hugr::extension::prelude::usize_t;
use hugr::hugr::hugrmut::HugrMut;
use hugr::ops::handle::{BasicBlockID, ConstID, NodeHandle};
use hugr::ops::{OpTrait, OpType, Value};
use hugr::types::Signature;
use hugr::{Hugr, HugrView, Node};
use hugr_core::ops::OpTag;
use itertools::Itertools;
use rstest::{fixture, rstest};
use std::io::BufReader;

use super::{
    StructuralizationAnalysisReport, StructuralizationStrategy, StructuredLoopKind, StructuredNode,
    StructuredRegionBody, analyze_hugr_cfgs, structurize_cfgs,
};

fn n_identity<T: DataflowSubContainer>(
    mut dataflow_builder: T,
    pred_const: &ConstID,
) -> Result<T::ContainerHandle, BuildError> {
    let wires = dataflow_builder.input_wires();
    let unit = dataflow_builder.load_const(pred_const);
    dataflow_builder.finish_with_outputs([unit].into_iter().chain(wires))
}

/// Builds the classic diamond subgraph used by several structuralization tests.
///
/// ```text
///        split
///       /     \
///    left    right
///       \     /
///        merge
/// ```
///
/// The helper returns the merge block so larger fixtures can continue from the
/// common join point.
fn build_then_else_merge_from_if<T: AsMut<Hugr> + AsRef<Hugr>>(
    cfg: &mut CFGBuilder<T>,
    unit_const: &ConstID,
    split: BasicBlockID,
) -> Result<BasicBlockID, BuildError> {
    let merge = n_identity(
        cfg.simple_block_builder(endo_sig([usize_t()]), 1)?,
        unit_const,
    )?;
    let left = n_identity(
        cfg.simple_block_builder(endo_sig([usize_t()]), 1)?,
        unit_const,
    )?;
    let right = n_identity(
        cfg.simple_block_builder(endo_sig([usize_t()]), 1)?,
        unit_const,
    )?;
    cfg.branch(&split, 0, &left)?;
    cfg.branch(&split, 1, &right)?;
    cfg.branch(&left, 0, &merge)?;
    cfg.branch(&right, 0, &merge)?;
    Ok(merge)
}

/// Builds a CFG with a branch followed by a tail-controlled loop.
///
/// ```text
///  entry -> split -> left ---\
///                  \          > merge -> head -> tail -> exit
///                   -> right -/                  ^      /
///                                                |_____/
/// ```
///
/// This is the simplest fixture where both strategies must recognize sibling
/// structured regions in sequence: first a branch, then a loop.
fn build_cond_then_loop_cfg() -> Result<Hugr, BuildError> {
    let mut cfg_builder = CFGBuilder::new(Signature::new_endo([usize_t()]))?;
    let pred_const = cfg_builder.add_constant(Value::unit_sum(0, 2).expect("0 < 2"));
    let const_unit = cfg_builder.add_constant(Value::unary_unit_sum());

    let entry = n_identity(
        cfg_builder.simple_entry_builder(vec![usize_t()].into(), 1)?,
        &const_unit,
    )?;
    let split = n_identity(
        cfg_builder.simple_block_builder(endo_sig([usize_t()]), 2)?,
        &pred_const,
    )?;
    cfg_builder.branch(&entry, 0, &split)?;
    let merge = build_then_else_merge_from_if(&mut cfg_builder, &const_unit, split)?;
    let head = n_identity(
        cfg_builder.simple_block_builder(endo_sig([usize_t()]), 1)?,
        &const_unit,
    )?;
    let tail = n_identity(
        cfg_builder.simple_block_builder(endo_sig([usize_t()]), 2)?,
        &pred_const,
    )?;
    cfg_builder.branch(&merge, 0, &head)?;
    cfg_builder.branch(&head, 0, &tail)?;
    cfg_builder.branch(&tail, 1, &head)?;
    let exit = cfg_builder.exit_block();
    cfg_builder.branch(&tail, 0, &exit)?;

    Ok(cfg_builder.finish_hugr()?)
}

/// Builds a pre-tested `while` loop.
///
/// ```text
///  entry -> header -> after -> exit
///            ^  |
///            |  v
///            body
/// ```
///
/// The break edge and the backedge come from different blocks, so this fixture
/// specifically exercises header-controlled loop analysis and lowering.
fn build_header_controlled_loop_cfg() -> Result<Hugr, BuildError> {
    let mut cfg_builder = CFGBuilder::new(Signature::new_endo([usize_t()]))?;
    let pred_const = cfg_builder.add_constant(Value::unit_sum(0, 2).expect("0 < 2"));
    let const_unit = cfg_builder.add_constant(Value::unary_unit_sum());

    let entry = n_identity(
        cfg_builder.simple_entry_builder(vec![usize_t()].into(), 1)?,
        &const_unit,
    )?;
    let header = n_identity(
        cfg_builder.simple_block_builder(endo_sig([usize_t()]), 2)?,
        &pred_const,
    )?;
    let body = n_identity(
        cfg_builder.simple_block_builder(endo_sig([usize_t()]), 1)?,
        &const_unit,
    )?;
    let after = n_identity(
        cfg_builder.simple_block_builder(endo_sig([usize_t()]), 1)?,
        &const_unit,
    )?;
    let exit = cfg_builder.exit_block();

    cfg_builder.branch(&entry, 0, &header)?;
    cfg_builder.branch(&header, 0, &after)?;
    cfg_builder.branch(&header, 1, &body)?;
    cfg_builder.branch(&body, 0, &header)?;
    cfg_builder.branch(&after, 0, &exit)?;

    Ok(cfg_builder.finish_hugr()?)
}

/// Builds an irreducible CFG with a two-entry cycle.
///
/// ```text
///             /-> a -> c -\
///            /    ^   /    \
///           /      \ /      \
///  entry ---        x        +--> exit
///           \      / \      /
///            \    v   \    /
///             \-> b -> d -/
/// ```
///
/// Neither `a` nor `b` is the unique header of the cyclic SCC, so the
/// Beyond-Relooper implementation must currently reject this fixture until the
/// Appendix A preprocessing step is implemented.
fn build_irreducible_cfg() -> Result<Hugr, BuildError> {
    let mut cfg_builder = CFGBuilder::new(Signature::new_endo([usize_t()]))?;
    let pred_const = cfg_builder.add_constant(Value::unit_sum(0, 2).expect("0 < 2"));
    let const_unit = cfg_builder.add_constant(Value::unary_unit_sum());

    let entry = n_identity(
        cfg_builder.simple_entry_builder(vec![usize_t()].into(), 2)?,
        &pred_const,
    )?;
    let a = n_identity(
        cfg_builder.simple_block_builder(endo_sig([usize_t()]), 1)?,
        &const_unit,
    )?;
    let b = n_identity(
        cfg_builder.simple_block_builder(endo_sig([usize_t()]), 1)?,
        &const_unit,
    )?;
    let c = n_identity(
        cfg_builder.simple_block_builder(endo_sig([usize_t()]), 2)?,
        &pred_const,
    )?;
    let d = n_identity(
        cfg_builder.simple_block_builder(endo_sig([usize_t()]), 2)?,
        &pred_const,
    )?;
    let exit = cfg_builder.exit_block();

    cfg_builder.branch(&entry, 0, &a)?;
    cfg_builder.branch(&entry, 1, &b)?;
    cfg_builder.branch(&a, 0, &c)?;
    cfg_builder.branch(&b, 0, &d)?;
    cfg_builder.branch(&c, 0, &b)?;
    cfg_builder.branch(&c, 1, &exit)?;
    cfg_builder.branch(&d, 0, &a)?;
    cfg_builder.branch(&d, 1, &exit)?;

    Ok(cfg_builder.finish_hugr()?)
}

/// Loads the checked-in Guppy fixture used for end-to-end structuralization tests.
///
/// The fixture contains real Guppy-emitted branches and loops, so it is a good
/// regression target for differences between the RVSDG and Beyond-Relooper
/// strategies.
fn load_guppy_complex_control_fixture() -> Hugr {
    let reader = BufReader::new(
        include_bytes!(
            "../../../../test_files/guppy_optimization/complex_control/complex_control.hugr"
        )
        .as_slice(),
    );
    Hugr::load(reader, None).unwrap()
}

#[fixture]
fn cond_then_loop() -> Hugr {
    build_cond_then_loop_cfg().unwrap()
}

#[fixture]
fn nested_branch_loop() -> Hugr {
    let (h, _, _) = build_conditional_in_loop_cfg(true).unwrap();
    h
}

#[fixture]
fn combined_headers() -> Hugr {
    let (h, _, _) = build_conditional_in_loop_cfg(false).unwrap();
    h
}

#[fixture]
fn header_loop() -> Hugr {
    build_header_controlled_loop_cfg().unwrap()
}

#[fixture]
fn irreducible() -> Hugr {
    build_irreducible_cfg().unwrap()
}

#[fixture]
fn complex_control() -> Hugr {
    load_guppy_complex_control_fixture()
}

fn single_cfg_analysis(h: &Hugr) -> StructuredRegionBody {
    let report = analyze_hugr_cfgs(h, StructuralizationStrategy::Rvsdg).unwrap();
    let StructuralizationAnalysisReport { cfg_regions } = report;
    let (_, region) = cfg_regions.into_iter().exactly_one().unwrap();
    region.body
}

fn cfgs(h: &Hugr) -> Vec<Node> {
    h.nodes()
        .filter(|n| h.get_optype(*n).tag() == OpTag::Cfg)
        .collect_vec()
}

fn run_structurize(h: &mut Hugr, strategy: StructuralizationStrategy) {
    let cfgs = cfgs(h);
    structurize_cfgs(h, &cfgs, strategy).unwrap();
}

fn conditional_count(h: &Hugr) -> usize {
    h.nodes()
        .filter(|n| matches!(h.get_optype(*n), OpType::Conditional(_)))
        .count()
}

fn tail_loop_count(h: &Hugr) -> usize {
    h.nodes()
        .filter(|n| matches!(h.get_optype(*n), OpType::TailLoop(_)))
        .count()
}

fn assert_lowered_counts(h: &Hugr, expected_conditionals: usize, expected_loops: usize) {
    assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_cfg()).count(), 0);
    assert_eq!(conditional_count(h), expected_conditionals);
    assert_eq!(tail_loop_count(h), expected_loops);
}

#[rstest]
fn branch_then_loop_io(cond_then_loop: Hugr) {
    let body = single_cfg_analysis(&cond_then_loop);
    let StructuredRegionBody::Sequence(items) = body else {
        panic!("expected root sequence");
    };
    assert_eq!(items.len(), 4);
    let StructuredNode::Region(branch) = &items[1] else {
        panic!("expected branch region");
    };
    let StructuredRegionBody::Branch { split, join, .. } = &branch.body else {
        panic!("expected branch body");
    };
    assert_eq!(split.inputs().len(), 1);
    assert_eq!(join.inputs().len(), 1);

    let StructuredNode::Region(loop_region) = &items[2] else {
        panic!("expected loop region");
    };
    let StructuredRegionBody::Loop {
        continue_inputs,
        break_outputs,
        ..
    } = &loop_region.body
    else {
        panic!("expected loop body");
    };
    assert_eq!(continue_inputs.len(), 1);
    assert_eq!(break_outputs.len(), 1);
}

#[rstest]
fn nested_branch_io(nested_branch_loop: Hugr) {
    let body = single_cfg_analysis(&nested_branch_loop);
    let StructuredRegionBody::Sequence(items) = body else {
        panic!("expected root sequence");
    };
    let StructuredNode::Region(loop_region) = &items[1] else {
        panic!("expected loop region");
    };
    let StructuredRegionBody::Loop { body, .. } = &loop_region.body else {
        panic!("expected loop body");
    };
    let StructuredNode::Region(branch_region) = &body[1] else {
        panic!("expected nested branch");
    };
    let StructuredRegionBody::Branch { arms, .. } = &branch_region.body else {
        panic!("expected branch body");
    };
    assert_eq!(arms.len(), 2);
}

#[rstest]
fn header_loop_io(header_loop: Hugr) {
    let body = single_cfg_analysis(&header_loop);
    let StructuredRegionBody::Sequence(items) = body else {
        panic!("expected root sequence");
    };
    let StructuredNode::Region(loop_region) = &items[1] else {
        panic!("expected loop region");
    };
    let StructuredRegionBody::Loop {
        kind,
        body,
        continue_inputs,
        break_outputs,
        ..
    } = &loop_region.body
    else {
        panic!("expected loop body");
    };
    assert_eq!(*kind, StructuredLoopKind::HeaderControlled);
    assert_eq!(body.len(), 1);
    assert_eq!(continue_inputs.len(), 1);
    assert_eq!(break_outputs.len(), 1);
}

#[rstest]
fn lowers_branch_then_loop(mut cond_then_loop: Hugr) {
    let original_cfgs = cfgs(&cond_then_loop);
    let report = structurize_cfgs(
        &mut cond_then_loop,
        &original_cfgs,
        StructuralizationStrategy::Rvsdg,
    )
    .unwrap();
    assert_eq!(report.rewrites.len(), 1);
    assert_eq!(report.rewrites[0], original_cfgs[0]);
    assert_lowered_counts(&cond_then_loop, 2, 1);
}

#[rstest]
fn lowers_nested_branch(mut nested_branch_loop: Hugr) {
    run_structurize(&mut nested_branch_loop, StructuralizationStrategy::Rvsdg);
    assert_lowered_counts(&nested_branch_loop, 2, 1);
}

#[rstest]
fn lowers_header_loop(mut header_loop: Hugr) {
    run_structurize(&mut header_loop, StructuralizationStrategy::Rvsdg);
    assert_lowered_counts(&header_loop, 2, 1);
}

#[rstest]
#[case::rvsdg(StructuralizationStrategy::Rvsdg)]
#[case::relooper(StructuralizationStrategy::BeyondRelooper)]
fn pass_rewrites_supported_cfg(
    #[case] strategy: StructuralizationStrategy,
    nested_branch_loop: Hugr,
) {
    let mut h = nested_branch_loop;
    let report = StructuralizeCfgsPass::default()
        .with_strategy(strategy)
        .run(&mut h)
        .unwrap();
    assert_eq!(report.rewrites.len(), 1);
    assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_cfg()).count(), 0);
}

#[rstest]
fn relooper_handles_combined_headers(mut combined_headers: Hugr) {
    let cfgs = cfgs(&combined_headers);
    let report = structurize_cfgs(
        &mut combined_headers,
        &cfgs,
        StructuralizationStrategy::BeyondRelooper,
    )
    .unwrap();
    assert_eq!(report.rewrites.len(), 1);
    assert_eq!(
        combined_headers
            .nodes()
            .filter(|n| combined_headers.get_optype(*n).is_cfg())
            .count(),
        0
    );
    assert!(
        combined_headers
            .nodes()
            .any(|n| matches!(combined_headers.get_optype(n), OpType::TailLoop(_)))
    );
    assert!(
        combined_headers
            .nodes()
            .filter(|n| matches!(combined_headers.get_optype(*n), OpType::Conditional(_)))
            .count()
            >= 1
    );
}

#[rstest]
fn rvsdg_rejects_combined_headers(mut combined_headers: Hugr) {
    let cfgs = cfgs(&combined_headers);
    assert!(
        structurize_cfgs(
            &mut combined_headers,
            &cfgs,
            StructuralizationStrategy::Rvsdg
        )
        .is_err()
    );
}

#[rstest]
fn relooper_rejects_irreducible(mut irreducible: Hugr) {
    let cfgs = cfgs(&irreducible);
    let err = structurize_cfgs(
        &mut irreducible,
        &cfgs,
        StructuralizationStrategy::BeyondRelooper,
    )
    .unwrap_err();
    assert!(matches!(
        err,
        super::StructuralizationError::UnsupportedIrreducibleCfg { .. }
    ));
}

#[rstest]
fn relooper_is_deterministic(combined_headers: Hugr) {
    let mut a = combined_headers;
    let mut b = {
        let (h, _, _) = build_conditional_in_loop_cfg(false).unwrap();
        h
    };
    StructuralizeCfgsPass::default()
        .with_strategy(StructuralizationStrategy::BeyondRelooper)
        .run(&mut a)
        .unwrap();
    StructuralizeCfgsPass::default()
        .with_strategy(StructuralizationStrategy::BeyondRelooper)
        .run(&mut b)
        .unwrap();
    assert_eq!(a.mermaid_string(), b.mermaid_string());
}

#[rstest]
#[case::default(true, 1)]
#[case::disabled(false, 2)]
fn pass_inlining(#[case] inline_dfgs: bool, #[case] minimum_dfgs: usize, header_loop: Hugr) {
    let mut h = header_loop;
    StructuralizeCfgsPass::default()
        .inline_dfgs(inline_dfgs)
        .run(&mut h)
        .unwrap();
    assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_cfg()).count(), 0);
    if inline_dfgs {
        assert_eq!(
            h.nodes().filter(|n| h.get_optype(*n).is_dfg()).count(),
            minimum_dfgs
        );
    } else {
        assert!(h.nodes().filter(|n| h.get_optype(*n).is_dfg()).count() >= minimum_dfgs);
    }
}

#[rstest]
fn scope_preserves_other_cfgs(cond_then_loop: Hugr) -> Result<(), BuildError> {
    let cfg_a = cond_then_loop;
    let cfg_b = build_cond_then_loop_cfg()?;
    let mut module = ModuleBuilder::new();
    let mut func_a = module.define_function("a", Signature::new_endo([usize_t()]))?;
    let [a_in] = func_a.input_wires_arr();
    let ins_a = func_a.add_hugr_with_wires(cfg_a, [a_in])?;
    func_a.finish_with_outputs(ins_a.outputs())?;
    let mut func_b = module.define_function("b", Signature::new_endo([usize_t()]))?;
    let [b_in] = func_b.input_wires_arr();
    let ins_b = func_b.add_hugr_with_wires(cfg_b, [b_in])?;
    func_b.finish_with_outputs(ins_b.outputs())?;
    let mut h = module.finish_hugr()?;

    let report = StructuralizeCfgsPass::default()
        .with_scope(PassScope::EntrypointRecursive)
        .run(&mut h.with_entrypoint_mut(ins_a.node()))
        .unwrap();
    assert_eq!(report.rewrites.len(), 1);
    assert_eq!(h.get_optype(ins_a.node()).tag(), OpTag::Dfg);
    assert!(h.get_optype(ins_b.node()).is_cfg());
    assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_cfg()).count(), 1);
    Ok(())
}

#[rstest]
#[case::rvsdg(StructuralizationStrategy::Rvsdg)]
#[case::relooper(StructuralizationStrategy::BeyondRelooper)]
fn structurizes_complex_control(
    #[case] strategy: StructuralizationStrategy,
    complex_control: Hugr,
) {
    let mut h = complex_control;
    let report = StructuralizeCfgsPass::default()
        .with_strategy(strategy)
        .run(&mut h)
        .unwrap();
    assert!(!report.rewrites.is_empty());
    assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_cfg()).count(), 0);
    assert!(tail_loop_count(&h) >= 1);
    assert!(conditional_count(&h) >= 1);
}
