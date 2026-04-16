//! Structuralization tests split by fixture, analysis, pass, and regression.

mod analysis;
mod fixtures;
mod pass;
mod regressions;
mod support;

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
use hugr::types::{EdgeKind, Signature};
use hugr::{Hugr, HugrView, IncomingPort, Node, OutgoingPort};
use hugr_core::ops::OpTag;
use itertools::Itertools;
use std::fs;
use std::io::BufReader;
use std::path::Path;

use super::{
    StructuralizationAnalysisReport, StructuralizationRewriteReport, StructuralizationStrategy,
    StructuredLoopKind, StructuredNode, StructuredRegionBody, analyze_hugr_cfgs,
};

use fixtures::*;
use support::*;
