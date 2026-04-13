use hugr::builder::{
    BuildError, CFGBuilder, Container, DataflowSubContainer, HugrBuilder, endo_sig,
};
use hugr::extension::prelude::usize_t;
use hugr::ops::Value;
use hugr::ops::handle::ConstID;
use hugr::types::Signature;
use hugr::{Hugr, HugrView};
use rstest::{fixture, rstest};

use crate::control::IdentityCfgMap;
use crate::control::nest_cfgs::test::build_conditional_in_loop_cfg;

use crate::control::structuralize::{StructuredNode, StructuredRegionBody};

use super::{LoopKind, RvsdgBuildError, RvsdgNode, analyze_cfg, build_cfg_rvsdg};

fn find_gamma(nodes: &[RvsdgNode]) -> Option<&super::GammaNode> {
    nodes.iter().find_map(|node| match node {
        RvsdgNode::Gamma(gamma) => Some(gamma.as_ref()),
        RvsdgNode::Theta(theta) => find_gamma(&theta.body.body),
        RvsdgNode::Block(_) => None,
    })
}

fn n_identity<T: DataflowSubContainer>(
    mut dataflow_builder: T,
    pred_const: &ConstID,
) -> Result<T::ContainerHandle, BuildError> {
    let wires = dataflow_builder.input_wires();
    let unit = dataflow_builder.load_const(pred_const);
    dataflow_builder.finish_with_outputs([unit].into_iter().chain(wires))
}

#[fixture]
fn header_loop() -> Hugr {
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
    let after = n_identity(
        cfg_builder
            .simple_block_builder(endo_sig([usize_t()]), 1)
            .unwrap(),
        &const_unit,
    )
    .unwrap();
    let exit = cfg_builder.exit_block();

    cfg_builder.branch(&entry, 0, &header).unwrap();
    cfg_builder.branch(&header, 0, &after).unwrap();
    cfg_builder.branch(&header, 1, &body).unwrap();
    cfg_builder.branch(&body, 0, &header).unwrap();
    cfg_builder.branch(&after, 0, &exit).unwrap();

    cfg_builder.finish_hugr().unwrap()
}

#[fixture]
fn combined_headers() -> Hugr {
    let (h, _, _) = build_conditional_in_loop_cfg(false).unwrap();
    h
}

#[rstest]
fn gamma_entry_exit_vars(combined_headers: Hugr) {
    let base = combined_headers.clone();
    let cfg_node = combined_headers
        .nodes()
        .find(|node| combined_headers.get_optype(*node).is_cfg())
        .unwrap();
    let cfg_view = base.with_entrypoint(cfg_node);
    let cfg = IdentityCfgMap::new(cfg_view.clone());
    let rvsdg = build_cfg_rvsdg(&cfg_view, &cfg).unwrap();
    let gamma = find_gamma(&rvsdg.root.body).unwrap();

    assert_eq!(gamma.match_rows.len(), 2);
    assert_eq!(gamma.entry_vars.len(), 1);
    assert_eq!(gamma.entry_vars[0].branch_arguments.len(), 2);
    assert_eq!(gamma.outputs.len(), 1);
    assert_eq!(gamma.outputs[0].branch_results.len(), 2);
}

#[rstest]
fn theta_loop_vars(header_loop: Hugr) {
    let base = header_loop.clone();
    let cfg_node = header_loop
        .nodes()
        .find(|node| header_loop.get_optype(*node).is_cfg())
        .unwrap();
    let cfg_view = base.with_entrypoint(cfg_node);
    let cfg = IdentityCfgMap::new(cfg_view.clone());
    let rvsdg = build_cfg_rvsdg(&cfg_view, &cfg).unwrap();
    let theta = rvsdg
        .root
        .body
        .iter()
        .find_map(|node| match node {
            RvsdgNode::Theta(theta) => Some(theta.as_ref()),
            _ => None,
        })
        .unwrap();

    assert_eq!(theta.kind, LoopKind::HeaderControlled);
    assert_eq!(theta.loop_vars.len(), 1);
    assert_eq!(theta.body.arguments.len(), 1);
    assert_eq!(theta.body.results.len(), 1);
}

#[rstest]
fn deterministic_order(combined_headers: Hugr) {
    let cfg_node = combined_headers
        .nodes()
        .find(|node| combined_headers.get_optype(*node).is_cfg())
        .unwrap();
    let base_a = combined_headers.clone();
    let base_b = combined_headers;
    let cfg_a = base_a.with_entrypoint(cfg_node);
    let cfg_b = base_b.with_entrypoint(cfg_node);
    let a = build_cfg_rvsdg(&cfg_a, &IdentityCfgMap::new(cfg_a.clone())).unwrap();
    let b = build_cfg_rvsdg(&cfg_b, &IdentityCfgMap::new(cfg_b.clone())).unwrap();
    assert_eq!(a, b);
}

#[rstest]
fn lowered_region_contains_loop_and_branch(combined_headers: Hugr) {
    let cfg_node = combined_headers
        .nodes()
        .find(|node| combined_headers.get_optype(*node).is_cfg())
        .unwrap();
    let cfg_view = combined_headers.with_entrypoint(cfg_node);
    let lowered = analyze_cfg(&cfg_view, &IdentityCfgMap::new(cfg_view.clone())).unwrap();

    let StructuredRegionBody::Sequence(items) = lowered.body else {
        panic!("expected sequence root");
    };
    assert!(items.iter().any(contains_loop));
    assert!(items.iter().any(contains_branch));
}

fn contains_loop(node: &StructuredNode) -> bool {
    match node {
        StructuredNode::Block(_) => false,
        StructuredNode::Region(region) => match &region.body {
            StructuredRegionBody::Sequence(items) => items.iter().any(contains_loop),
            StructuredRegionBody::Branch { arms, .. } => {
                arms.iter().flat_map(|arm| arm.iter()).any(contains_loop)
            }
            StructuredRegionBody::Loop { .. } => true,
        },
    }
}

fn contains_branch(node: &StructuredNode) -> bool {
    match node {
        StructuredNode::Block(_) => false,
        StructuredNode::Region(region) => match &region.body {
            StructuredRegionBody::Sequence(items) => items.iter().any(contains_branch),
            StructuredRegionBody::Branch { .. } => true,
            StructuredRegionBody::Loop { body, .. } => body.iter().any(contains_branch),
        },
    }
}

#[rstest]
fn rejects_irreducible() {
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
    let a = n_identity(
        cfg_builder
            .simple_block_builder(endo_sig([usize_t()]), 1)
            .unwrap(),
        &const_unit,
    )
    .unwrap();
    let b = n_identity(
        cfg_builder
            .simple_block_builder(endo_sig([usize_t()]), 1)
            .unwrap(),
        &const_unit,
    )
    .unwrap();
    let c = n_identity(
        cfg_builder
            .simple_block_builder(endo_sig([usize_t()]), 2)
            .unwrap(),
        &pred_const,
    )
    .unwrap();
    let d = n_identity(
        cfg_builder
            .simple_block_builder(endo_sig([usize_t()]), 2)
            .unwrap(),
        &pred_const,
    )
    .unwrap();
    let exit = cfg_builder.exit_block();

    cfg_builder.branch(&entry, 0, &a).unwrap();
    cfg_builder.branch(&entry, 1, &b).unwrap();
    cfg_builder.branch(&a, 0, &c).unwrap();
    cfg_builder.branch(&b, 0, &d).unwrap();
    cfg_builder.branch(&c, 0, &b).unwrap();
    cfg_builder.branch(&c, 1, &exit).unwrap();
    cfg_builder.branch(&d, 0, &a).unwrap();
    cfg_builder.branch(&d, 1, &exit).unwrap();

    let h = cfg_builder.finish_hugr().unwrap();
    let cfg_node = h.nodes().find(|node| h.get_optype(*node).is_cfg()).unwrap();
    let base = h.clone();
    let cfg_view = base.with_entrypoint(cfg_node);
    let err = build_cfg_rvsdg(&cfg_view, &IdentityCfgMap::new(cfg_view.clone())).unwrap_err();
    assert!(matches!(err, RvsdgBuildError::IrreducibleCfg { .. }));
}
