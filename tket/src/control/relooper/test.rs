use hugr::HugrView;
use rstest::{fixture, rstest};

use crate::control::IdentityCfgMap;
use crate::control::nest_cfgs::test::build_conditional_in_loop_cfg;

use super::ast::{RelooperLabel, RelooperStmt};
use super::construct::build_cfg_ast;

#[fixture]
fn combined_headers() -> hugr::Hugr {
    let (h, _, _) = build_conditional_in_loop_cfg(false).unwrap();
    h
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
        RelooperStmt::Br(exit) => matches!(exit.target, RelooperLabel::Original(_)),
        RelooperStmt::Return(_) => false,
    }
}
