//! Shared lowering pipeline for structured CFG rewrites.
//!
//! Strategy-specific analysis produces a private lowering-oriented IR in
//! [`crate::control::structuralize::types`]. This module turns that IR into a
//! structured replacement by:
//!
//! 1. building a detached template containing `Conditional`, `TailLoop`, and
//!    placeholder `DFG` nodes, and
//! 2. materializing the original CFG basic blocks into those placeholder
//!    locations after the template has been inserted into the destination HUGR.
//!
//! The split keeps the control-node construction reusable across strategies
//! while preserving original block subtrees, including any static references to
//! module-level constants or function definitions.

mod materialize;
mod template;

use std::collections::BTreeSet;

use hugr::HugrView;
use hugr::Node;
use hugr::hugr::hugrmut::HugrMut;

use crate::passes::NormalizeCFGPass;
use crate::passes::composable::ComposablePass;
use crate::passes::{PassScope, composable::WithScope};

use super::analyze::analyze_cfg;
use super::types::{
    StructuralizationError, StructuralizationRewriteReport, StructuralizationStrategy,
};
use crate::control::{IdentityCfgMap, relooper};
use materialize::materialize_cfg_rewrite;
pub(crate) use template::{LoweredCfgTemplate, prepare_cfg_replacement};

/// Structuralize the specified CFGs and rewrite them in place.
///
/// # Errors
///
/// Returns an error when the selected strategy is unsupported, when CFG
/// structural analysis or lowering fails for any targeted CFG, or when local
/// post-rewrite normalization produces an invalid HUGR.
pub fn structurize_cfgs<H: HugrMut<Node = Node>>(
    hugr: &mut H,
    cfgs: &[Node],
    strategy: StructuralizationStrategy,
) -> Result<StructuralizationRewriteReport, StructuralizationError> {
    let cfgs = outermost_cfgs(hugr, cfgs);
    let mut rewrites = Vec::with_capacity(cfgs.len());
    for cfg in cfgs {
        let cfg_view = hugr.with_entrypoint(cfg);
        let replacement = match strategy {
            StructuralizationStrategy::Rvsdg => {
                let analyzed = analyze_cfg(cfg_view.clone(), strategy)?;
                prepare_cfg_replacement(&cfg_view, &analyzed)?
            }
            StructuralizationStrategy::BeyondRelooper => {
                let id_cfg = IdentityCfgMap::new(cfg_view.clone());
                relooper::prepare_cfg_rewrite(&cfg_view, &id_cfg)?
            }
        };
        materialize_cfg_rewrite(hugr, cfg, replacement)?;
        NormalizeCFGPass::default()
            .with_scope(PassScope::EntrypointRecursive)
            .run(&mut hugr.with_entrypoint_mut(cfg))?;
        rewrites.push(cfg);
    }

    Ok(StructuralizationRewriteReport { rewrites })
}

/// Filters a set of CFG roots down to the outermost ones.
///
/// Rewriting a parent CFG subsumes rewriting nested CFGs beneath it, so the
/// pass only prepares top-level rewrite targets.
fn outermost_cfgs<H: HugrView<Node = Node>>(hugr: &H, cfgs: &[Node]) -> Vec<Node> {
    let cfg_set = cfgs.iter().copied().collect::<BTreeSet<_>>();
    cfgs.iter()
        .copied()
        .filter(|cfg| {
            let mut parent = hugr.get_parent(*cfg);
            while let Some(node) = parent {
                if cfg_set.contains(&node) {
                    return false;
                }
                parent = hugr.get_parent(node);
            }
            true
        })
        .collect()
}
