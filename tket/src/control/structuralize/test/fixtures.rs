use rstest::fixture;

use super::*;

pub(super) fn n_identity<T: DataflowSubContainer>(
    mut dataflow_builder: T,
    pred_const: &ConstID,
) -> Result<T::ContainerHandle, BuildError> {
    let wires = dataflow_builder.input_wires();
    let unit = dataflow_builder.load_const(pred_const);
    dataflow_builder.finish_with_outputs([unit].into_iter().chain(wires))
}

/// Builds the classic diamond subgraph used by several structuralization tests.
pub(super) fn build_then_else_merge_from_if<T: AsMut<Hugr> + AsRef<Hugr>>(
    cfg: &mut CFGBuilder<T>,
    unit_const: &ConstID,
    split: BasicBlockID,
) -> BasicBlockID {
    let merge = n_identity(
        cfg.simple_block_builder(endo_sig([usize_t()]), 1).unwrap(),
        unit_const,
    )
    .unwrap();
    let left = n_identity(
        cfg.simple_block_builder(endo_sig([usize_t()]), 1).unwrap(),
        unit_const,
    )
    .unwrap();
    let right = n_identity(
        cfg.simple_block_builder(endo_sig([usize_t()]), 1).unwrap(),
        unit_const,
    )
    .unwrap();
    cfg.branch(&split, 0, &left).unwrap();
    cfg.branch(&split, 1, &right).unwrap();
    cfg.branch(&left, 0, &merge).unwrap();
    cfg.branch(&right, 0, &merge).unwrap();
    merge
}

#[fixture]
pub(super) fn build_cond_then_loop_cfg() -> Hugr {
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
    let split = n_identity(
        cfg_builder
            .simple_block_builder(endo_sig([usize_t()]), 2)
            .unwrap(),
        &pred_const,
    )
    .unwrap();
    cfg_builder.branch(&entry, 0, &split).unwrap();
    let merge = build_then_else_merge_from_if(&mut cfg_builder, &const_unit, split);
    let head = n_identity(
        cfg_builder
            .simple_block_builder(endo_sig([usize_t()]), 1)
            .unwrap(),
        &const_unit,
    )
    .unwrap();
    let tail = n_identity(
        cfg_builder
            .simple_block_builder(endo_sig([usize_t()]), 2)
            .unwrap(),
        &pred_const,
    )
    .unwrap();
    cfg_builder.branch(&merge, 0, &head).unwrap();
    cfg_builder.branch(&head, 0, &tail).unwrap();
    cfg_builder.branch(&tail, 1, &head).unwrap();
    let exit = cfg_builder.exit_block();
    cfg_builder.branch(&tail, 0, &exit).unwrap();

    cfg_builder.finish_hugr().unwrap()
}

#[fixture]
pub(super) fn build_header_controlled_loop_cfg() -> Hugr {
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
pub(super) fn build_irreducible_cfg() -> Hugr {
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
pub(super) fn build_two_level_loop_escape_cfg() -> Hugr {
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
    let break_mid = n_identity(
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
    let join = n_identity(
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
    cfg_builder.branch(&body, 0, &split).unwrap();
    cfg_builder.branch(&split, 0, &break_mid).unwrap();
    cfg_builder.branch(&split, 1, &cont).unwrap();
    cfg_builder.branch(&cont, 0, &header).unwrap();
    cfg_builder.branch(&break_mid, 0, &join).unwrap();
    cfg_builder.branch(&after, 0, &join).unwrap();
    cfg_builder.branch(&join, 0, &exit).unwrap();

    cfg_builder.finish_hugr().unwrap()
}

#[fixture]
pub(super) fn build_multi_exit_loop_cfg() -> Hugr {
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

pub(super) fn load_guppy_example(name: &str) -> Hugr {
    let one_file = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../test_files/guppy_examples")
        .join(format!("{name}.hugr"));
    let nested = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../test_files/guppy_optimization")
        .join(name)
        .join(format!("{name}.hugr"));
    let file = if one_file.exists() { one_file } else { nested };
    let reader = BufReader::new(fs::File::open(file).unwrap());
    Hugr::load(reader, None).unwrap()
}

#[fixture]
pub(super) fn build_nested_branch_loop_cfg() -> Hugr {
    let (h, _, _) = build_conditional_in_loop_cfg(true).unwrap();
    h
}

#[fixture]
pub(super) fn build_combined_headers_cfg() -> Hugr {
    let (h, _, _) = build_conditional_in_loop_cfg(false).unwrap();
    h
}

#[fixture]
pub(super) fn bell_test() -> Hugr {
    load_guppy_example("bell_test")
}

pub(super) type TestCfgBuilder = fn() -> Hugr;

#[fixture]
pub(super) fn build_non_unique_backedge_loop_cfg() -> Hugr {
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
    let latch_a = n_identity(
        cfg_builder
            .simple_block_builder(endo_sig([usize_t()]), 1)
            .unwrap(),
        &const_unit,
    )
    .unwrap();
    let latch_b = n_identity(
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
    cfg_builder.branch(&body, 0, &split).unwrap();
    cfg_builder.branch(&split, 0, &latch_a).unwrap();
    cfg_builder.branch(&split, 1, &latch_b).unwrap();
    cfg_builder.branch(&latch_a, 0, &header).unwrap();
    cfg_builder.branch(&latch_b, 0, &header).unwrap();
    cfg_builder.branch(&after, 0, &exit).unwrap();

    cfg_builder.finish_hugr().unwrap()
}

#[fixture]
pub(super) fn build_multi_continue_header_loop_cfg() -> Hugr {
    let mut cfg_builder = CFGBuilder::new(Signature::new_endo([usize_t()])).unwrap();
    let tri_const = cfg_builder.add_constant(Value::unit_sum(0, 3).expect("0 < 3"));
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
            .simple_block_builder(endo_sig([usize_t()]), 3)
            .unwrap(),
        &tri_const,
    )
    .unwrap();
    let left = n_identity(
        cfg_builder
            .simple_block_builder(endo_sig([usize_t()]), 1)
            .unwrap(),
        &const_unit,
    )
    .unwrap();
    let right = n_identity(
        cfg_builder
            .simple_block_builder(endo_sig([usize_t()]), 1)
            .unwrap(),
        &const_unit,
    )
    .unwrap();
    let latch = n_identity(
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
    cfg_builder.branch(&header, 1, &left).unwrap();
    cfg_builder.branch(&header, 2, &right).unwrap();
    cfg_builder.branch(&left, 0, &latch).unwrap();
    cfg_builder.branch(&right, 0, &latch).unwrap();
    cfg_builder.branch(&latch, 0, &header).unwrap();
    cfg_builder.branch(&after, 0, &exit).unwrap();

    cfg_builder.finish_hugr().unwrap()
}

#[fixture]
pub(super) fn build_three_entry_irreducible_cfg() -> Hugr {
    let mut cfg_builder = CFGBuilder::new(Signature::new_endo([usize_t()])).unwrap();
    let tri_const = cfg_builder.add_constant(Value::unit_sum(0, 3).expect("0 < 3"));
    let pred_const = cfg_builder.add_constant(Value::unit_sum(0, 2).expect("0 < 2"));
    let const_unit = cfg_builder.add_constant(Value::unary_unit_sum());

    let entry = n_identity(
        cfg_builder
            .simple_entry_builder(vec![usize_t()].into(), 1)
            .unwrap(),
        &const_unit,
    )
    .unwrap();
    let p = n_identity(
        cfg_builder
            .simple_block_builder(endo_sig([usize_t()]), 3)
            .unwrap(),
        &tri_const,
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
            .simple_block_builder(endo_sig([usize_t()]), 1)
            .unwrap(),
        &const_unit,
    )
    .unwrap();
    let x = n_identity(
        cfg_builder
            .simple_block_builder(endo_sig([usize_t()]), 1)
            .unwrap(),
        &const_unit,
    )
    .unwrap();
    let y = n_identity(
        cfg_builder
            .simple_block_builder(endo_sig([usize_t()]), 2)
            .unwrap(),
        &pred_const,
    )
    .unwrap();
    let z = n_identity(
        cfg_builder
            .simple_block_builder(endo_sig([usize_t()]), 1)
            .unwrap(),
        &const_unit,
    )
    .unwrap();
    let exit = cfg_builder.exit_block();

    cfg_builder.branch(&entry, 0, &p).unwrap();
    cfg_builder.branch(&p, 0, &a).unwrap();
    cfg_builder.branch(&p, 1, &b).unwrap();
    cfg_builder.branch(&p, 2, &c).unwrap();
    cfg_builder.branch(&a, 0, &x).unwrap();
    cfg_builder.branch(&b, 0, &y).unwrap();
    cfg_builder.branch(&c, 0, &z).unwrap();
    cfg_builder.branch(&x, 0, &y).unwrap();
    cfg_builder.branch(&y, 0, &c).unwrap();
    cfg_builder.branch(&y, 1, &exit).unwrap();
    cfg_builder.branch(&z, 0, &a).unwrap();

    cfg_builder.finish_hugr().unwrap()
}
