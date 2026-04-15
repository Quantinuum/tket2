use super::*;

pub(super) fn container_order_link_counts(h: &Hugr, container: Node) -> (Vec<usize>, Vec<usize>) {
    let children = h.children(container).collect_vec();
    let input = children
        .iter()
        .copied()
        .find(|child| matches!(h.get_optype(*child), OpType::Input(_)))
        .unwrap();
    let output = children
        .iter()
        .copied()
        .find(|child| matches!(h.get_optype(*child), OpType::Output(_)))
        .unwrap();
    let input_links = (0..h.get_optype(input).output_count())
        .filter(|&port| {
            h.get_optype(input).port_kind(OutgoingPort::from(port)) == Some(EdgeKind::StateOrder)
        })
        .map(|port| h.linked_inputs(input, OutgoingPort::from(port)).count())
        .collect();
    let output_links = (0..h.get_optype(output).input_count())
        .filter(|&port| {
            h.get_optype(output).port_kind(IncomingPort::from(port)) == Some(EdgeKind::StateOrder)
        })
        .map(|port| h.linked_outputs(output, IncomingPort::from(port)).count())
        .collect();
    (input_links, output_links)
}

pub(super) fn single_cfg_analysis(h: &Hugr) -> StructuredRegionBody {
    let report = analyze_hugr_cfgs(h, StructuralizationStrategy::Rvsdg).unwrap();
    let StructuralizationAnalysisReport { cfg_regions } = report;
    let (_, region) = cfg_regions.into_iter().exactly_one().unwrap();
    region.body
}

pub(super) fn cfgs(h: &Hugr) -> Vec<Node> {
    h.nodes()
        .filter(|n| h.get_optype(*n).tag() == OpTag::Cfg)
        .collect_vec()
}

pub(super) fn run_structurize(h: &mut Hugr, strategy: StructuralizationStrategy) {
    let cfgs = cfgs(h);
    structurize_cfgs(h, &cfgs, strategy).unwrap();
}

pub(super) fn structurize_result(
    mut h: Hugr,
    strategy: StructuralizationStrategy,
) -> Result<Hugr, String> {
    let cfgs = cfgs(&h);
    structurize_cfgs(&mut h, &cfgs, strategy)
        .map(|_| h)
        .map_err(|err| err.to_string())
}

pub(super) fn conditional_count(h: &Hugr) -> usize {
    h.nodes()
        .filter(|n| matches!(h.get_optype(*n), OpType::Conditional(_)))
        .count()
}

pub(super) fn tail_loop_count(h: &Hugr) -> usize {
    h.nodes()
        .filter(|n| matches!(h.get_optype(*n), OpType::TailLoop(_)))
        .count()
}

pub(super) fn assert_lowered_counts(h: &Hugr, expected_conditionals: usize, expected_loops: usize) {
    assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_cfg()).count(), 0);
    assert_eq!(conditional_count(h), expected_conditionals);
    assert_eq!(tail_loop_count(h), expected_loops);
}
