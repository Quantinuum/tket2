use rstest::rstest;

use super::*;

fn build_dead_branch_cfg() -> Hugr {
    let mut cfg_builder = CFGBuilder::new(Signature::new_endo([usize_t()])).unwrap();
    let pred_const = cfg_builder.add_constant(Value::unit_sum(0, 2).expect("0 < 2"));
    let const_unit = cfg_builder.add_constant(Value::unary_unit_sum());

    let entry = n_identity(
        cfg_builder
            .simple_entry_builder(vec![usize_t()].into(), 2)
            .unwrap(),
        &pred_const,
    )
    .unwrap();
    let live = n_identity(
        cfg_builder
            .simple_block_builder(endo_sig([usize_t()]), 1)
            .unwrap(),
        &const_unit,
    )
    .unwrap();
    let dead = n_identity(
        cfg_builder
            .simple_block_builder(endo_sig([usize_t()]), 1)
            .unwrap(),
        &const_unit,
    )
    .unwrap();
    let exit = cfg_builder.exit_block();

    cfg_builder.branch(&entry, 0, &live).unwrap();
    cfg_builder.branch(&entry, 1, &dead).unwrap();
    cfg_builder.branch(&live, 0, &exit).unwrap();
    cfg_builder.branch(&dead, 0, &dead).unwrap();

    cfg_builder.finish_hugr().unwrap()
}

#[rstest]
#[case::rvsdg(StructuralizationStrategy::Rvsdg)]
#[case::relooper(StructuralizationStrategy::Relooper)]
fn pass_rewrites_supported_cfg(
    #[case] strategy: StructuralizationStrategy,
    #[from(build_nested_branch_loop_cfg)] nested_branch_loop: Hugr,
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
#[case::rvsdg(StructuralizationStrategy::Rvsdg)]
#[case::relooper(StructuralizationStrategy::Relooper)]
fn strategy_is_deterministic(
    #[case] strategy: StructuralizationStrategy,
    #[from(build_combined_headers_cfg)] combined_headers: Hugr,
) {
    let mut a = combined_headers;
    let mut b = {
        let (h, _, _) = build_conditional_in_loop_cfg(false).unwrap();
        h
    };
    StructuralizeCfgsPass::default()
        .with_strategy(strategy)
        .run(&mut a)
        .unwrap();
    StructuralizeCfgsPass::default()
        .with_strategy(strategy)
        .run(&mut b)
        .unwrap();
    assert_eq!(a.mermaid_string(), b.mermaid_string());
}

#[rstest]
fn pass_inlining(#[from(build_header_controlled_loop_cfg)] header_loop: Hugr) {
    let mut h = header_loop;
    StructuralizeCfgsPass::default().run(&mut h).unwrap();
    assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_cfg()).count(), 0);
    assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_dfg()).count(), 1);
}

#[rstest]
fn scope_preserves_other_cfgs(
    #[from(build_cond_then_loop_cfg)] cond_then_loop: Hugr,
) -> Result<(), BuildError> {
    let cfg_a = cond_then_loop;
    let cfg_b = build_cond_then_loop_cfg();
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
#[case::loop_and_branch_rvsdg("loop_and_branch", StructuralizationStrategy::Rvsdg, 1)]
#[case::loop_and_branch_relooper("loop_and_branch", StructuralizationStrategy::Relooper, 1)]
#[case::complex_control_rvsdg("complex_control", StructuralizationStrategy::Rvsdg, 1)]
#[case::complex_control_relooper("complex_control", StructuralizationStrategy::Relooper, 1)]
#[case::nested_loops_rvsdg("nested_loops", StructuralizationStrategy::Rvsdg, 1)]
#[case::nested_loops_relooper("nested_loops", StructuralizationStrategy::Relooper, 1)]
#[case::shortcircuit_rvsdg("shortcircuit", StructuralizationStrategy::Rvsdg, 0)]
#[case::shortcircuit_relooper("shortcircuit", StructuralizationStrategy::Relooper, 0)]
#[case::multi_level_exit_rvsdg("multi_level_exit", StructuralizationStrategy::Rvsdg, 2)]
#[case::multi_level_exit_relooper("multi_level_exit", StructuralizationStrategy::Relooper, 2)]
fn structurizes_checked_in_fixture(
    #[case] example: &str,
    #[case] strategy: StructuralizationStrategy,
    #[case] min_tail_loops: usize,
) {
    let mut h = load_guppy_example(example);
    let report = StructuralizeCfgsPass::default()
        .with_strategy(strategy)
        .run(&mut h)
        .unwrap();
    assert!(!report.rewrites.is_empty());
    h.validate().unwrap();
    assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_cfg()).count(), 0);
    assert!(tail_loop_count(&h) >= min_tail_loops);
    assert!(conditional_count(&h) >= 1);
}

#[rstest]
#[case::rvsdg(StructuralizationStrategy::Rvsdg)]
#[case::relooper(StructuralizationStrategy::Relooper)]
fn pass_skips_unstructuralizable_cfgs_by_default(#[case] strategy: StructuralizationStrategy) {
    let mut h = build_dead_branch_cfg();

    let report = StructuralizeCfgsPass::default()
        .with_strategy(strategy)
        .run(&mut h)
        .unwrap();

    assert!(report.rewrites.is_empty());
    assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_cfg()).count(), 1);
}

#[rstest]
#[case::rvsdg(StructuralizationStrategy::Rvsdg)]
#[case::relooper(StructuralizationStrategy::Relooper)]
fn pass_can_error_on_unstructuralizable_cfgs(#[case] strategy: StructuralizationStrategy) {
    let mut h = build_dead_branch_cfg();

    let err = StructuralizeCfgsPass::default()
        .with_strategy(strategy)
        .with_skip_unstructuralizable_cfgs(false)
        .run(&mut h)
        .unwrap_err();

    assert!(err.is_unstructuralizable());
}
