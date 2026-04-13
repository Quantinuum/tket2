use hugr::builder::{
    BuildError, CFGBuilder, Container, DataflowSubContainer, HugrBuilder, endo_sig,
};
use hugr::extension::prelude::usize_t;
use hugr::ops::Value;
use hugr::ops::handle::ConstID;
use hugr::types::Signature;
use hugr::{Hugr, HugrView};
use itertools::Itertools;
use rstest::{fixture, rstest};

use crate::control::nest_cfgs::test::build_conditional_in_loop_cfg;
use crate::control::{CfgNodeMap, IdentityCfgMap};

use super::CfgFacts;
use super::CfgFactsError;
use super::preprocess::{NormalizedCfg, PreprocessedCfg, PreprocessedNode};

#[fixture]
fn combined_headers_cfg() -> Hugr {
    let (h, _, _) = build_conditional_in_loop_cfg(false).unwrap();
    h
}

#[fixture]
fn irreducible_cfg() -> Hugr {
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

    cfg_builder.finish_hugr().unwrap()
}

#[fixture]
fn dead_branch_cfg() -> Hugr {
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
fn normalization_preserves_reducible_cfg(combined_headers_cfg: Hugr) {
    let cfg_root = combined_headers_cfg
        .nodes()
        .find(|node| combined_headers_cfg.get_optype(*node).is_cfg())
        .unwrap();
    let cfg_view = combined_headers_cfg.with_entrypoint(cfg_root);
    let cfg = IdentityCfgMap::new(cfg_view.clone());

    let normalized = NormalizedCfg::new(cfg_root, &cfg).unwrap();

    assert_eq!(normalized.entry_node(), cfg.entry_node());
    assert_eq!(normalized.exit_node(), cfg.exit_node());
    assert!(normalized.scope().contains(&normalized.entry_node()));
    assert!(normalized.scope().contains(&normalized.exit_node()));
    assert_eq!(
        normalized.successors(normalized.entry_node()).collect_vec(),
        cfg.successors(cfg.entry_node()).collect_vec()
    );
}

#[rstest]
fn normalization_rejects_irreducible_cfg(irreducible_cfg: Hugr) {
    let cfg_root = irreducible_cfg
        .nodes()
        .find(|node| irreducible_cfg.get_optype(*node).is_cfg())
        .unwrap();
    let cfg_view = irreducible_cfg.with_entrypoint(cfg_root);

    let err = match NormalizedCfg::new(cfg_root, &IdentityCfgMap::new(cfg_view)) {
        Ok(_) => panic!("expected irreducible normalization to fail"),
        Err(err) => err,
    };
    assert!(matches!(
        err,
        CfgFactsError::Irreducible { cfg, entries }
            if cfg == cfg_root && entries.len() == 2
    ));
}

#[rstest]
fn preprocessing_makes_irreducible_cfg_reducible(irreducible_cfg: Hugr) {
    let cfg_root = irreducible_cfg
        .nodes()
        .find(|node| irreducible_cfg.get_optype(*node).is_cfg())
        .unwrap();
    let cfg_view = irreducible_cfg.with_entrypoint(cfg_root);
    let cfg = IdentityCfgMap::new(cfg_view);

    let preprocessed = PreprocessedCfg::new(cfg_root, &cfg).unwrap();
    let facts = CfgFacts::new(preprocessed.entry_node(), &preprocessed).unwrap();

    assert!(facts.scope.len() > 6);
    assert!(
        preprocessed
            .scope()
            .iter()
            .any(|node| matches!(node, PreprocessedNode::Duplicate { .. }))
    );
}

#[rstest]
fn normalization_rejects_reachable_dead_branch(dead_branch_cfg: Hugr) {
    let cfg_root = dead_branch_cfg
        .nodes()
        .find(|node| dead_branch_cfg.get_optype(*node).is_cfg())
        .unwrap();
    let cfg_view = dead_branch_cfg.with_entrypoint(cfg_root);

    let err = match NormalizedCfg::new(cfg_root, &IdentityCfgMap::new(cfg_view)) {
        Ok(_) => panic!("expected dead branch normalization to fail"),
        Err(err) => err,
    };
    assert!(matches!(
        err,
        CfgFactsError::ReachableNodesDoNotReachExit { dropped } if dropped.len() == 1
    ));
}

fn n_identity<T: DataflowSubContainer>(
    mut dataflow_builder: T,
    pred_const: &ConstID,
) -> Result<T::ContainerHandle, BuildError> {
    let wires = dataflow_builder.input_wires();
    let unit = dataflow_builder.load_const(pred_const);
    dataflow_builder.finish_with_outputs([unit].into_iter().chain(wires))
}
