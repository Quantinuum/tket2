use hugr::HugrView;
use rstest::{fixture, rstest};

use crate::control::IdentityCfgMap;
use crate::control::nest_cfgs::test::build_conditional_in_loop_cfg;

use super::ast::{RelooperBody, RelooperNode};
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

    let RelooperBody::Sequence(items) = ast.body else {
        panic!("expected a sequence root");
    };
    assert!(items.iter().any(contains_loop));
    assert!(items.iter().any(contains_branch));
}

fn contains_loop(node: &RelooperNode) -> bool {
    match node {
        RelooperNode::Block(_) => false,
        RelooperNode::Region(region) => match &region.body {
            RelooperBody::Sequence(items) => items.iter().any(contains_loop),
            RelooperBody::Branch { arms, .. } => {
                arms.iter().flat_map(|arm| arm.iter()).any(contains_loop)
            }
            RelooperBody::Loop { .. } => true,
        },
    }
}

fn contains_branch(node: &RelooperNode) -> bool {
    match node {
        RelooperNode::Block(_) => false,
        RelooperNode::Region(region) => match &region.body {
            RelooperBody::Sequence(items) => items.iter().any(contains_branch),
            RelooperBody::Branch { .. } => true,
            RelooperBody::Loop { body, .. } => body.iter().any(contains_branch),
        },
    }
}
