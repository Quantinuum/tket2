use hugr::HugrView;
use hugr::builder::{
    BuildError, CFGBuilder, Container, DataflowSubContainer, HugrBuilder, endo_sig,
};
use hugr::extension::prelude::usize_t;
use hugr::ops::Value;
use hugr::ops::handle::ConstID;
use hugr::types::Signature;
use rstest::{fixture, rstest};

use crate::control::IdentityCfgMap;
use crate::control::nest_cfgs::test::build_conditional_in_loop_cfg;
use crate::control::structuralize::{StructuredNode, StructuredRegionBody};

use super::ast::{RelooperLabel, RelooperStmt};
use super::construct::build_cfg_ast;
use super::lower::lower_region;

fn n_identity<T: DataflowSubContainer>(
    mut dataflow_builder: T,
    pred_const: &ConstID,
) -> Result<T::ContainerHandle, BuildError> {
    let wires = dataflow_builder.input_wires();
    let unit = dataflow_builder.load_const(pred_const);
    dataflow_builder.finish_with_outputs([unit].into_iter().chain(wires))
}

#[fixture]
fn combined_headers() -> hugr::Hugr {
    let (h, _, _) = build_conditional_in_loop_cfg(false).unwrap();
    h
}

#[fixture]
fn multi_exit_loop() -> hugr::Hugr {
    let mut cfg_builder = CFGBuilder::new(Signature::new_endo([usize_t()])).unwrap();
    let pred_const = cfg_builder.add_constant(Value::unit_sum(0, 2).expect("0 < 2"));
    let const_unit = cfg_builder.add_constant(Value::unary_unit_sum());

    let entry = n_identity(
        cfg_builder
            .simple_entry_builder(vec![usize_t()].into(), 1)
            .unwrap(),
        &const_unit,
    )
    .unwrap();
    let header = n_identity(
        cfg_builder
            .simple_block_builder(endo_sig([usize_t()]), 2)
            .unwrap(),
        &pred_const,
    )
    .unwrap();
    let body = n_identity(
        cfg_builder
            .simple_block_builder(endo_sig([usize_t()]), 1)
            .unwrap(),
        &const_unit,
    )
    .unwrap();
    let split = n_identity(
        cfg_builder
            .simple_block_builder(endo_sig([usize_t()]), 2)
            .unwrap(),
        &pred_const,
    )
    .unwrap();
    let cont = n_identity(
        cfg_builder
            .simple_block_builder(endo_sig([usize_t()]), 1)
            .unwrap(),
        &const_unit,
    )
    .unwrap();
    let after_a = n_identity(
        cfg_builder
            .simple_block_builder(endo_sig([usize_t()]), 1)
            .unwrap(),
        &const_unit,
    )
    .unwrap();
    let after_b = n_identity(
        cfg_builder
            .simple_block_builder(endo_sig([usize_t()]), 1)
            .unwrap(),
        &const_unit,
    )
    .unwrap();
    let join = n_identity(
        cfg_builder
            .simple_block_builder(endo_sig([usize_t()]), 1)
            .unwrap(),
        &const_unit,
    )
    .unwrap();
    let exit = cfg_builder.exit_block();

    cfg_builder.branch(&entry, 0, &header).unwrap();
    cfg_builder.branch(&header, 0, &after_a).unwrap();
    cfg_builder.branch(&header, 1, &body).unwrap();
    cfg_builder.branch(&body, 0, &split).unwrap();
    cfg_builder.branch(&split, 0, &after_b).unwrap();
    cfg_builder.branch(&split, 1, &cont).unwrap();
    cfg_builder.branch(&cont, 0, &header).unwrap();
    cfg_builder.branch(&after_a, 0, &join).unwrap();
    cfg_builder.branch(&after_b, 0, &join).unwrap();
    cfg_builder.branch(&join, 0, &exit).unwrap();

    cfg_builder.finish_hugr().unwrap()
}

#[rstest]
fn ast_is_deterministic(combined_headers: hugr::Hugr) {
    let cfg_node = combined_headers
        .nodes()
        .find(|node| combined_headers.get_optype(*node).is_cfg())
        .unwrap();
    let base_a = combined_headers.clone();
    let base_b = combined_headers;
    let cfg_a = base_a.with_entrypoint(cfg_node);
    let cfg_b = base_b.with_entrypoint(cfg_node);
    let a = build_cfg_ast(&cfg_a, &IdentityCfgMap::new(cfg_a.clone())).unwrap();
    let b = build_cfg_ast(&cfg_b, &IdentityCfgMap::new(cfg_b.clone())).unwrap();
    assert_eq!(a, b);
}

#[rstest]
fn combined_headers_contains_loop_and_branch(combined_headers: hugr::Hugr) {
    let cfg_node = combined_headers
        .nodes()
        .find(|node| combined_headers.get_optype(*node).is_cfg())
        .unwrap();
    let cfg_view = combined_headers.with_entrypoint(cfg_node);
    let ast = build_cfg_ast(&cfg_view, &IdentityCfgMap::new(cfg_view.clone())).unwrap();

    let RelooperStmt::Seq(items) = ast.body else {
        panic!("expected a sequence root");
    };
    assert!(items.iter().any(contains_loop));
    assert!(items.iter().any(contains_case));
    assert!(items.iter().any(contains_block));
    assert!(items.iter().any(contains_br));
    assert!(items.iter().any(contains_return));
}

#[rstest]
fn combined_headers_uses_cfg_backed_labels(combined_headers: hugr::Hugr) {
    let cfg_node = combined_headers
        .nodes()
        .find(|node| combined_headers.get_optype(*node).is_cfg())
        .unwrap();
    let cfg_view = combined_headers.with_entrypoint(cfg_node);
    let ast = build_cfg_ast(&cfg_view, &IdentityCfgMap::new(cfg_view.clone())).unwrap();

    let RelooperStmt::Seq(items) = ast.body else {
        panic!("expected a sequence root");
    };
    assert!(items.iter().any(contains_original_label));
}

#[rstest]
fn multi_exit_loop_wraps_exits_in_labelled_blocks(multi_exit_loop: hugr::Hugr) {
    let cfg_node = multi_exit_loop
        .nodes()
        .find(|node| multi_exit_loop.get_optype(*node).is_cfg())
        .unwrap();
    let cfg_view = multi_exit_loop.with_entrypoint(cfg_node);
    let ast = build_cfg_ast(&cfg_view, &IdentityCfgMap::new(cfg_view.clone())).unwrap();

    let labels = collect_block_labels(&ast.body);
    assert!(labels.len() >= 2);
    let edge_counts = collect_loop_exit_edge_counts(&ast.body);
    assert!(edge_counts.iter().any(|count| *count > 0), "{ast:#?}");
}

#[rstest]
fn lowered_combined_headers_loop_has_body(combined_headers: hugr::Hugr) {
    let cfg_node = combined_headers
        .nodes()
        .find(|node| combined_headers.get_optype(*node).is_cfg())
        .unwrap();
    let cfg_view = combined_headers.with_entrypoint(cfg_node);
    let ast = build_cfg_ast(&cfg_view, &IdentityCfgMap::new(cfg_view.clone())).unwrap();
    let lowered = lower_region(&cfg_view, &ast).unwrap();

    let loops = collect_lowered_loops(&lowered.body);
    assert!(
        loops
            .iter()
            .any(|(body, exits)| !body.is_empty() && *exits == 1),
        "{lowered:#?}"
    );
}

#[rstest]
fn lowered_multi_exit_loop_preserves_exit_count(multi_exit_loop: hugr::Hugr) {
    let cfg_node = multi_exit_loop
        .nodes()
        .find(|node| multi_exit_loop.get_optype(*node).is_cfg())
        .unwrap();
    let cfg_view = multi_exit_loop.with_entrypoint(cfg_node);
    let ast = build_cfg_ast(&cfg_view, &IdentityCfgMap::new(cfg_view.clone())).unwrap();
    let lowered = lower_region(&cfg_view, &ast).unwrap();

    let loops = collect_lowered_loops(&lowered.body);
    assert!(loops.iter().any(|(_, exits)| *exits == 2), "{lowered:#?}");
}

fn contains_loop(stmt: &RelooperStmt) -> bool {
    match stmt {
        RelooperStmt::Seq(items) => items.iter().any(contains_loop),
        RelooperStmt::Region(region) => contains_loop(&region.body),
        RelooperStmt::Exec(_) => false,
        RelooperStmt::Block { body, .. } => contains_loop(body),
        RelooperStmt::Case { arms, .. } => arms.iter().any(contains_loop),
        RelooperStmt::Loop { .. } => true,
        RelooperStmt::Br(_) | RelooperStmt::Return(_) => false,
    }
}

fn contains_case(stmt: &RelooperStmt) -> bool {
    match stmt {
        RelooperStmt::Seq(items) => items.iter().any(contains_case),
        RelooperStmt::Region(region) => contains_case(&region.body),
        RelooperStmt::Exec(_) => false,
        RelooperStmt::Block { body, .. } => contains_case(body),
        RelooperStmt::Case { .. } => true,
        RelooperStmt::Loop { body, .. } => contains_case(body),
        RelooperStmt::Br(_) | RelooperStmt::Return(_) => false,
    }
}

fn contains_block(stmt: &RelooperStmt) -> bool {
    match stmt {
        RelooperStmt::Seq(items) => items.iter().any(contains_block),
        RelooperStmt::Region(region) => contains_block(&region.body),
        RelooperStmt::Exec(_) => false,
        RelooperStmt::Block { .. } => true,
        RelooperStmt::Case { arms, .. } => arms.iter().any(contains_block),
        RelooperStmt::Loop { body, .. } => contains_block(body),
        RelooperStmt::Br(_) | RelooperStmt::Return(_) => false,
    }
}

fn contains_br(stmt: &RelooperStmt) -> bool {
    match stmt {
        RelooperStmt::Seq(items) => items.iter().any(contains_br),
        RelooperStmt::Region(region) => contains_br(&region.body),
        RelooperStmt::Exec(_) => false,
        RelooperStmt::Block { body, .. } => contains_br(body),
        RelooperStmt::Case { arms, .. } => arms.iter().any(contains_br),
        RelooperStmt::Loop { body, .. } => contains_br(body),
        RelooperStmt::Br(_) => true,
        RelooperStmt::Return(_) => false,
    }
}

fn contains_return(stmt: &RelooperStmt) -> bool {
    match stmt {
        RelooperStmt::Seq(items) => items.iter().any(contains_return),
        RelooperStmt::Region(region) => contains_return(&region.body),
        RelooperStmt::Exec(_) => false,
        RelooperStmt::Block { body, .. } => contains_return(body),
        RelooperStmt::Case { arms, .. } => arms.iter().any(contains_return),
        RelooperStmt::Loop { body, .. } => contains_return(body),
        RelooperStmt::Br(_) => false,
        RelooperStmt::Return(_) => true,
    }
}

fn contains_original_label(stmt: &RelooperStmt) -> bool {
    match stmt {
        RelooperStmt::Seq(items) => items.iter().any(contains_original_label),
        RelooperStmt::Region(region) => contains_original_label(&region.body),
        RelooperStmt::Exec(_) => false,
        RelooperStmt::Block { label, body, .. } => {
            matches!(label, RelooperLabel::Original(_)) || contains_original_label(body)
        }
        RelooperStmt::Case { arms, .. } => arms.iter().any(contains_original_label),
        RelooperStmt::Loop { label, body, .. } => {
            matches!(label, RelooperLabel::Original(_)) || contains_original_label(body)
        }
        RelooperStmt::Br(label) => matches!(label, RelooperLabel::Original(_)),
        RelooperStmt::Return(_) => false,
    }
}

fn collect_block_labels(stmt: &RelooperStmt) -> Vec<RelooperLabel> {
    match stmt {
        RelooperStmt::Seq(items) => items.iter().flat_map(collect_block_labels).collect(),
        RelooperStmt::Region(region) => collect_block_labels(&region.body),
        RelooperStmt::Exec(_)
        | RelooperStmt::Case { .. }
        | RelooperStmt::Br(_)
        | RelooperStmt::Return(_) => Vec::new(),
        RelooperStmt::Block { label, body, .. } => {
            let mut labels = vec![*label];
            labels.extend(collect_block_labels(body));
            labels
        }
        RelooperStmt::Loop { body, .. } => collect_block_labels(body),
    }
}

fn collect_loop_exit_edge_counts(stmt: &RelooperStmt) -> Vec<usize> {
    match stmt {
        RelooperStmt::Seq(items) => items
            .iter()
            .flat_map(collect_loop_exit_edge_counts)
            .collect(),
        RelooperStmt::Region(region) => collect_loop_exit_edge_counts(&region.body),
        RelooperStmt::Exec(_) | RelooperStmt::Br(_) | RelooperStmt::Return(_) => Vec::new(),
        RelooperStmt::Block { lowering, body, .. } => {
            let mut counts = vec![lowering.loop_exit_edges.len()];
            counts.extend(collect_loop_exit_edge_counts(body));
            counts
        }
        RelooperStmt::Case { arms, .. } => arms
            .iter()
            .flat_map(collect_loop_exit_edge_counts)
            .collect(),
        RelooperStmt::Loop { body, .. } => collect_loop_exit_edge_counts(body),
    }
}

fn collect_lowered_loops(body: &StructuredRegionBody) -> Vec<(&[StructuredNode], usize)> {
    match body {
        StructuredRegionBody::Sequence(items) => {
            items.iter().flat_map(collect_loops_from_node).collect()
        }
        StructuredRegionBody::Branch { arms, .. } => arms
            .iter()
            .flat_map(|arm| arm.iter().flat_map(collect_loops_from_node))
            .collect(),
        StructuredRegionBody::Loop { body, exits, .. } => vec![(body.as_slice(), exits.len())],
    }
}

fn collect_loops_from_node(node: &StructuredNode) -> Vec<(&[StructuredNode], usize)> {
    match node {
        StructuredNode::Block(_) => Vec::new(),
        StructuredNode::Region(region) => collect_lowered_loops(&region.body),
    }
}
