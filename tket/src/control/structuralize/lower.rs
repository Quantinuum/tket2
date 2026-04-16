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

use crate::passes::normalize_cfgs::normalize_cfg;

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
        let Some(replacement) = prepare_replacement_with_normalize_retry(hugr, cfg, strategy)?
        else {
            continue;
        };
        materialize_cfg_rewrite(hugr, cfg, replacement)?;
        normalize_nested_cfgs(hugr, cfg)?;
        rewrites.push(cfg);
    }

    Ok(StructuralizationRewriteReport { rewrites })
}

/// Prepares one CFG rewrite, retrying after local CFG normalization when raw
/// branch shapes are not yet directly structuralizable.
fn prepare_replacement_with_normalize_retry<H: HugrMut<Node = Node>>(
    hugr: &mut H,
    cfg: Node,
    strategy: StructuralizationStrategy,
) -> Result<Option<LoweredCfgTemplate>, StructuralizationError> {
    match prepare_replacement(hugr, cfg, strategy) {
        Ok(replacement) => Ok(Some(replacement)),
        Err(StructuralizationError::UnsupportedBranch { .. }) => {
            normalize_cfg(&mut hugr.with_entrypoint_mut(cfg))?;
            if !hugr.get_optype(cfg).is_cfg() {
                return Ok(None);
            }
            prepare_replacement(hugr, cfg, strategy).map(Some)
        }
        Err(err) => Err(err),
    }
}

/// Prepares the detached structured replacement for one CFG without mutating it.
fn prepare_replacement<H: HugrMut<Node = Node>>(
    hugr: &H,
    cfg: Node,
    strategy: StructuralizationStrategy,
) -> Result<LoweredCfgTemplate, StructuralizationError> {
    let cfg_view = hugr.with_entrypoint(cfg);
    match strategy {
        StructuralizationStrategy::Rvsdg => {
            let analyzed = analyze_cfg(cfg_view.clone(), strategy)?;
            prepare_cfg_replacement(&cfg_view, &analyzed)
        }
        StructuralizationStrategy::Relooper => {
            let id_cfg = IdentityCfgMap::new(cfg_view.clone());
            relooper::prepare_cfg_rewrite(&cfg_view, &id_cfg)
        }
    }
}

/// Normalizes nested CFGs beneath one rewritten region root.
///
/// Structuralization can materialize helper CFGs inside the rewritten region,
/// for example when original block subgraphs already contained nested control
/// flow. The structuralization pipeline only owns one rewritten root at a
/// time, so it normalizes exactly the nested CFG descendants of that root
/// rather than invoking the general pass layer again.
fn normalize_nested_cfgs<H: HugrMut<Node = Node>>(
    hugr: &mut H,
    root: Node,
) -> Result<(), StructuralizationError> {
    let mut cfgs = hugr
        .descendants(root)
        .filter(|node| hugr.get_optype(*node).is_cfg())
        .collect::<Vec<_>>();
    cfgs.reverse();
    for cfg in cfgs {
        normalize_cfg(&mut hugr.with_entrypoint_mut(cfg))?;
    }
    Ok(())
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
