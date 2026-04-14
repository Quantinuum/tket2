//! Lowering from the Beyond-Relooper AST into the shared rewrite-oriented form.
//!
//! The pass pipeline currently rewrites CFGs through the structured HUGR IR
//! used by `control::structuralize`. This module isolates the translation from
//! the Beyond-Relooper AST into that representation so the analysis phase does
//! not depend on the shared lowering types directly.

use crate::control::structuralize::{
    StructuralizationError, StructuredLoopExit, StructuredNode, StructuredRegion,
    StructuredRegionBody,
};

use super::ast::{RelooperBody, RelooperLoopExit, RelooperNode, RelooperRegion};

/// Lowers one Beyond-Relooper region into the shared structuralization region.
pub(super) fn lower_region(
    region: &RelooperRegion,
) -> Result<StructuredRegion, StructuralizationError> {
    Ok(StructuredRegion {
        io: region.io.clone(),
        body: lower_body(&region.body)?,
    })
}

/// Lowers one Beyond-Relooper body into the shared structuralization body.
fn lower_body(body: &RelooperBody) -> Result<StructuredRegionBody, StructuralizationError> {
    Ok(match body {
        RelooperBody::Sequence(items) => StructuredRegionBody::Sequence(
            items
                .iter()
                .map(lower_node)
                .collect::<Result<Vec<_>, _>>()?,
        ),
        RelooperBody::Branch {
            split,
            arms,
            join,
            join_kind,
        } => StructuredRegionBody::Branch {
            split: split.clone(),
            arms: arms
                .iter()
                .map(|arm| arm.iter().map(lower_node).collect::<Result<Vec<_>, _>>())
                .collect::<Result<Vec<_>, _>>()?,
            join: join.clone(),
            join_kind: *join_kind,
        },
        RelooperBody::Loop {
            kind,
            header,
            body,
            backedge_source,
            continue_edge,
            exits,
            break_outputs,
        } => StructuredRegionBody::Loop {
            kind: *kind,
            header: header.clone(),
            body: body.iter().map(lower_node).collect::<Result<Vec<_>, _>>()?,
            backedge_source: *backedge_source,
            continue_edge: continue_edge.clone(),
            exits: exits
                .iter()
                .map(lower_exit)
                .collect::<Result<Vec<_>, _>>()?,
            break_outputs: break_outputs.clone(),
        },
    })
}

/// Lowers one Beyond-Relooper sequence item into the shared structuralization node.
fn lower_node(node: &RelooperNode) -> Result<StructuredNode, StructuralizationError> {
    Ok(match node {
        RelooperNode::Block(block) => StructuredNode::Block(block.clone()),
        RelooperNode::Region(region) => StructuredNode::Region(Box::new(lower_region(region)?)),
    })
}

/// Lowers one Beyond-Relooper loop exit into the shared structural form.
fn lower_exit(exit: &RelooperLoopExit) -> Result<StructuredLoopExit, StructuralizationError> {
    Ok(StructuredLoopExit {
        edges: exit.edges.clone(),
        outputs: exit.outputs.clone(),
        continuation: exit
            .continuation
            .iter()
            .map(lower_node)
            .collect::<Result<Vec<_>, _>>()?,
    })
}
