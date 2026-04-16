use rstest::rstest;

use super::*;

#[rstest]
fn branch_then_loop_io(#[from(build_cond_then_loop_cfg)] cond_then_loop: Hugr) {
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
        continue_edges,
        break_outputs,
        ..
    } = &loop_region.body
    else {
        panic!("expected loop body");
    };
    assert_eq!(continue_edges[0].payload.len(), 1);
    assert_eq!(break_outputs.len(), 1);
}

#[rstest]
fn nested_branch_io(#[from(build_nested_branch_loop_cfg)] nested_branch_loop: Hugr) {
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
fn header_loop_io(#[from(build_header_controlled_loop_cfg)] header_loop: Hugr) {
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
        continue_edges,
        break_outputs,
        ..
    } = &loop_region.body
    else {
        panic!("expected loop body");
    };
    assert_eq!(*kind, StructuredLoopKind::HeaderControlled);
    assert_eq!(body.len(), 1);
    assert_eq!(continue_edges[0].payload.len(), 1);
    assert_eq!(break_outputs.len(), 1);
}

#[rstest]
fn lowers_branch_then_loop(#[from(build_cond_then_loop_cfg)] mut cond_then_loop: Hugr) {
    let original_cfgs = cfgs(&cond_then_loop);
    let report = structurize_report(&mut cond_then_loop, StructuralizationStrategy::Rvsdg);
    assert_eq!(report.rewrites.len(), 1);
    assert_eq!(report.rewrites[0], original_cfgs[0]);
    assert_lowered_counts(&cond_then_loop, 2, 1);
}

#[rstest]
fn lowers_nested_branch(#[from(build_nested_branch_loop_cfg)] mut nested_branch_loop: Hugr) {
    run_structurize(&mut nested_branch_loop, StructuralizationStrategy::Rvsdg);
    assert_lowered_counts(&nested_branch_loop, 2, 1);
}

#[rstest]
fn lowers_header_loop(#[from(build_header_controlled_loop_cfg)] mut header_loop: Hugr) {
    run_structurize(&mut header_loop, StructuralizationStrategy::Rvsdg);
    assert_lowered_counts(&header_loop, 2, 1);
}
