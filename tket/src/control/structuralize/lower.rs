//! Shared lowering pipeline for one structured CFG rewrite.
//!
//! Strategy-specific analysis produces a private lowering-oriented IR in
//! [`crate::control::structuralize::types`]. This module turns one analyzed CFG
//! into a structured replacement by:
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

use hugr::Node;
use hugr::hugr::hugrmut::HugrMut;

use crate::passes::normalize_cfgs::normalize_cfg;

use super::analyze::analyze_cfg;
use super::types::{StructuralizationError, StructuralizationStrategy};
use crate::control::{IdentityCfgMap, relooper};
use materialize::materialize_cfg_rewrite;
pub(crate) use template::{LoweredCfgTemplate, prepare_cfg_replacement};

/// Structuralizes one CFG and rewrites it in place.
///
/// # Errors
///
/// Returns an error when the selected strategy is unsupported, when CFG
/// structural analysis or lowering fails for the targeted CFG, or when the
/// local normalization retry produces an invalid HUGR.
///
/// Returns `Ok(true)` when a CFG was rewritten, and `Ok(false)` when the local
/// normalization retry simplified the CFG away before structuralization.
pub(crate) fn structurize_cfg<H: HugrMut<Node = Node>>(
    hugr: &mut H,
    cfg: Node,
    strategy: StructuralizationStrategy,
) -> Result<bool, StructuralizationError> {
    let Some(replacement) = prepare_replacement_with_normalize_retry(hugr, cfg, strategy)? else {
        return Ok(true);
    };
    materialize_cfg_rewrite(hugr, cfg, replacement)?;
    Ok(true)
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
