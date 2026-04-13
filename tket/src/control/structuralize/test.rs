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
use hugr::{Hugr, HugrView};
use hugr_core::ops::OpTag;
use itertools::Itertools;
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

fn single_cfg_analysis(h: &Hugr) -> StructuredRegionBody {
    let report = analyze_hugr_cfgs(h, StructuralizationStrategy::Rvsdg).unwrap();
    let StructuralizationAnalysisReport { cfg_regions } = report;
    let (_, region) = cfg_regions.into_iter().exactly_one().unwrap();
    region.body
}

#[test]
fn analyzes_branch_then_loop_io() -> Result<(), BuildError> {
    let h = build_cond_then_loop_cfg()?;
    let body = single_cfg_analysis(&h);
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
    Ok(())
}

#[test]
fn analyzes_nested_branch_inside_loop_io() -> Result<(), BuildError> {
    let (h, _, _) = build_conditional_in_loop_cfg(true)?;
    let body = single_cfg_analysis(&h);
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
    Ok(())
}

#[test]
fn analyzes_header_controlled_loop_io() -> Result<(), BuildError> {
    let h = build_header_controlled_loop_cfg()?;
    let body = single_cfg_analysis(&h);
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
    Ok(())
}

#[test]
fn lowers_conditional_and_loop_cfgs() -> Result<(), BuildError> {
    let mut h = build_cond_then_loop_cfg()?;
    let cfgs = h
        .nodes()
        .filter(|n| h.get_optype(*n).tag() == OpTag::Cfg)
        .collect_vec();
    let report = structurize_cfgs(&mut h, &cfgs, StructuralizationStrategy::Rvsdg).unwrap();
    assert_eq!(report.rewrites.len(), 1);
    assert_eq!(report.rewrites[0], cfgs[0]);
    assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_cfg()).count(), 0);
    assert_eq!(
        h.nodes()
            .filter(|n| matches!(h.get_optype(*n), OpType::Conditional(_)))
            .count(),
        2
    );
    assert_eq!(
        h.nodes()
            .filter(|n| matches!(h.get_optype(*n), OpType::TailLoop(_)))
            .count(),
        1
    );
    Ok(())
}

#[test]
fn lowers_nested_branch_inside_loop() -> Result<(), BuildError> {
    let (mut h, _, _) = build_conditional_in_loop_cfg(true)?;
    let cfgs = h
        .nodes()
        .filter(|n| h.get_optype(*n).tag() == OpTag::Cfg)
        .collect_vec();
    structurize_cfgs(&mut h, &cfgs, StructuralizationStrategy::Rvsdg).unwrap();
    assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_cfg()).count(), 0);
    assert_eq!(
        h.nodes()
            .filter(|n| matches!(h.get_optype(*n), OpType::Conditional(_)))
            .count(),
        2
    );
    assert_eq!(
        h.nodes()
            .filter(|n| matches!(h.get_optype(*n), OpType::TailLoop(_)))
            .count(),
        1
    );
    Ok(())
}

#[test]
fn lowers_header_controlled_loop() -> Result<(), BuildError> {
    let mut h = build_header_controlled_loop_cfg()?;
    let cfgs = h
        .nodes()
        .filter(|n| h.get_optype(*n).tag() == OpTag::Cfg)
        .collect_vec();
    structurize_cfgs(&mut h, &cfgs, StructuralizationStrategy::Rvsdg).unwrap();
    assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_cfg()).count(), 0);
    assert_eq!(
        h.nodes()
            .filter(|n| matches!(h.get_optype(*n), OpType::Conditional(_)))
            .count(),
        2
    );
    assert_eq!(
        h.nodes()
            .filter(|n| matches!(h.get_optype(*n), OpType::TailLoop(_)))
            .count(),
        1
    );
    Ok(())
}

#[test]
fn pass_rewrites_cfgs_and_keeps_strategy_error() -> Result<(), BuildError> {
    let (mut h, _, _) = build_conditional_in_loop_cfg(true)?;
    let report = StructuralizeCfgsPass::default().run(&mut h).unwrap();
    assert_eq!(report.rewrites.len(), 1);

    let (mut h, _, _) = build_conditional_in_loop_cfg(true)?;
    let err = StructuralizeCfgsPass::default()
        .with_strategy(StructuralizationStrategy::BeyondRelooper)
        .run(&mut h)
        .unwrap_err();
    assert!(matches!(
        err,
        super::StructuralizationError::UnsupportedStrategy { .. }
    ));
    Ok(())
}

#[test]
fn pass_inlines_helper_dfgs_by_default() -> Result<(), BuildError> {
    let mut h = build_header_controlled_loop_cfg()?;
    StructuralizeCfgsPass::default().run(&mut h).unwrap();
    assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_cfg()).count(), 0);
    assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_dfg()).count(), 1);

    let mut h = build_header_controlled_loop_cfg()?;
    StructuralizeCfgsPass::default()
        .inline_dfgs(false)
        .run(&mut h)
        .unwrap();
    assert!(h.nodes().filter(|n| h.get_optype(*n).is_dfg()).count() > 1);
    Ok(())
}

#[test]
fn pass_respects_scope_without_touching_other_cfgs() -> Result<(), BuildError> {
    let cfg_a = build_cond_then_loop_cfg()?;
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

#[test]
fn structurizes_guppy_complex_control_fixture() {
    let reader = BufReader::new(
        include_bytes!(
            "../../../../test_files/guppy_optimization/complex_control/complex_control.hugr"
        )
        .as_slice(),
    );
    let mut h = Hugr::load(reader, None).unwrap();

    let report = StructuralizeCfgsPass::default().run(&mut h).unwrap();
    assert!(!report.rewrites.is_empty());
    assert_eq!(h.nodes().filter(|n| h.get_optype(*n).is_cfg()).count(), 0);
    assert!(
        h.nodes()
            .any(|n| matches!(h.get_optype(n), OpType::TailLoop(_)))
    );
    assert!(
        h.nodes()
            .filter(|n| matches!(h.get_optype(*n), OpType::Conditional(_)))
            .count()
            >= 1
    );
}
