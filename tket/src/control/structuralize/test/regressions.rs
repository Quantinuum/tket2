use rstest::rstest;

use super::*;

#[rstest]
#[case::rvsdg(StructuralizationStrategy::Rvsdg)]
#[case::relooper(StructuralizationStrategy::Relooper)]
fn handles_combined_headers(
    #[case] strategy: StructuralizationStrategy,
    #[from(build_combined_headers_cfg)] mut combined_headers: Hugr,
) {
    let report = structurize_report(&mut combined_headers, strategy);
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
#[case::rvsdg(StructuralizationStrategy::Rvsdg)]
#[case::relooper(StructuralizationStrategy::Relooper)]
fn lowers_irreducible_cfg(
    #[case] strategy: StructuralizationStrategy,
    #[from(build_irreducible_cfg)] mut irreducible: Hugr,
) {
    let report = structurize_report(&mut irreducible, strategy);
    assert_eq!(report.rewrites.len(), 1);
    assert_eq!(
        irreducible
            .nodes()
            .filter(|n| irreducible.get_optype(*n).is_cfg())
            .count(),
        0
    );
}

#[rstest]
#[case::two_level_escape(build_two_level_loop_escape_cfg as TestCfgBuilder)]
#[case::multi_exit_loop(build_multi_exit_loop_cfg as TestCfgBuilder)]
fn lowers_reducible_cfgs_with_multiple_loop_exit_targets(
    #[case] build_cfg: TestCfgBuilder,
    #[values(StructuralizationStrategy::Rvsdg, StructuralizationStrategy::Relooper)]
    strategy: StructuralizationStrategy,
) {
    let mut h = build_cfg();
    let report = structurize_report(&mut h, strategy);
    assert_eq!(report.rewrites.len(), 1);
    assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_cfg()).count(), 0);
}

#[rstest]
fn lowers_multi_continue_header_loop(
    #[from(build_multi_continue_header_loop_cfg)] mut multi_continue_header_loop: Hugr,
) {
    let report = structurize_report(
        &mut multi_continue_header_loop,
        StructuralizationStrategy::Relooper,
    );
    assert_eq!(report.rewrites.len(), 1);
    assert_eq!(
        multi_continue_header_loop
            .nodes()
            .filter(|n| multi_continue_header_loop.get_optype(*n).is_cfg())
            .count(),
        0
    );
}

#[rstest]
fn lowers_non_unique_backedge_loop(
    #[from(build_non_unique_backedge_loop_cfg)] mut non_unique_backedge_loop: Hugr,
) {
    let report = structurize_report(
        &mut non_unique_backedge_loop,
        StructuralizationStrategy::Relooper,
    );
    assert_eq!(report.rewrites.len(), 1);
    assert_eq!(
        non_unique_backedge_loop
            .nodes()
            .filter(|n| non_unique_backedge_loop.get_optype(*n).is_cfg())
            .count(),
        0
    );
}

#[rstest]
#[case::rvsdg(StructuralizationStrategy::Rvsdg)]
#[case::relooper(StructuralizationStrategy::Relooper)]
fn preprocesses_three_entry_irreducible_cfg(
    #[case] strategy: StructuralizationStrategy,
    #[from(build_three_entry_irreducible_cfg)] three_entry_irreducible: Hugr,
) {
    let h = structurize_result(three_entry_irreducible, strategy)
        .unwrap_or_else(|err| panic!("strategy {strategy:?} failed on three-entry SCC: {err}"));
    assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_cfg()).count(), 0);
}

#[rstest]
#[case::rvsdg(StructuralizationStrategy::Rvsdg)]
#[case::relooper(StructuralizationStrategy::Relooper)]
fn bell_test_preserves_container_io_wiring(
    bell_test: Hugr,
    #[case] strategy: StructuralizationStrategy,
) {
    let raw_block = bell_test
        .nodes()
        .find(|node| bell_test.get_optype(*node).is_dataflow_block())
        .unwrap();
    let (raw_input_links, raw_output_links) = container_order_link_counts(&bell_test, raw_block);
    assert!(raw_input_links.iter().all(|count| *count > 0));
    assert!(raw_output_links.iter().all(|count| *count > 0));

    let mut structured = bell_test;
    let report = StructuralizeCfgsPass::default()
        .with_strategy(strategy)
        .run(&mut structured)
        .unwrap();
    assert!(!report.rewrites.is_empty());
    structured.validate().unwrap();

    let structured_container = structured.entrypoint();
    let (structured_input_links, structured_output_links) =
        container_order_link_counts(&structured, structured_container);
    assert_eq!(structured_input_links, raw_input_links);
    assert_eq!(structured_output_links, raw_output_links);
}
