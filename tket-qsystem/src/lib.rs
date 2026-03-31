//! Provides a preparation and validation workflow for Hugrs targeting
//! Quantinuum H-series quantum computers.

#[cfg(feature = "cli")]
pub mod cli;
pub mod extension;
#[cfg(feature = "llvm")]
pub mod llvm;
pub mod lower_drops;
pub mod pytket;
pub mod replace_bools;

use derive_more::{Display, Error, From};
use hugr::hugr::{HugrError, hugrmut::HugrMut};
use hugr::{HugrView, Node, core::Visibility, ops::OpType};
use itertools::Itertools as _;
use std::collections::HashSet;
use tket::passes::composable::WithScope;
use tket::passes::const_fold::{ConstFoldError, ConstantFoldPass};
use tket::passes::{
    ComposablePass, MonomorphizePass, PassScope, RemoveDeadFuncsError, RemoveDeadFuncsPass,
    force_order, replace_types::ReplaceTypesError,
};

use lower_drops::LowerDropsPass;
use replace_bools::{ReplaceBoolPass, ReplaceBoolPassError};
use tket::TketOp;

use extension::{
    futures::FutureOpDef,
    qsystem::{LowerTk2Error, LowerTketToQSystemPass, QSystemOp},
};

/// Modify a [hugr::Hugr] into a form that is acceptable for ingress into a
/// Q-System. Returns an error if this cannot be done.
///
/// This pass should only be applied with [`PassScope::Global`] scopes on HUGRs
/// with function entrypoints. An error will be returned if this is not the
/// case.
///
/// To construct a `QSystemPass` use [Default::default].
#[derive(Debug, Clone)]
pub struct QSystemPass {
    constant_fold: bool,
    monomorphize: bool,
    force_order: bool,
    lazify: bool,
    hide_funcs: bool,

    /// Where to apply the pass.
    ///
    /// Configurable via [`WithScope::with_scope`].
    scope: PassScope,
}

impl Default for QSystemPass {
    fn default() -> Self {
        Self {
            constant_fold: true,
            monomorphize: true,
            force_order: true,
            lazify: true,
            hide_funcs: true,
            scope: PassScope::default(),
        }
    }
}

#[derive(Error, Debug, Display, From)]
#[non_exhaustive]
/// An error reported from [QSystemPass].
pub enum QSystemPassError<N = Node> {
    /// An error from the component [ReplaceBoolPass].
    ReplaceBoolError(ReplaceBoolPassError<N>),
    /// An error from the component [force_order()] pass.
    ForceOrderError(HugrError),
    /// An error from the component [LowerTketToQSystemPass] pass.
    LowerTk2Error(LowerTk2Error),
    /// An error from the component [ConstantFoldPass] pass.
    ConstantFoldError(ConstFoldError),
    /// An error from the component [LowerDropsPass] pass.
    LinearizeArrayError(ReplaceTypesError),
    /// An error when running [RemoveDeadFuncsPass] after the monomorphisation
    /// pass.
    ///
    ///  [RemoveDeadFuncsPass]: tket::passes::RemoveDeadFuncsError
    DCEError(RemoveDeadFuncsError),
    /// No [FuncDefn] named "main" in [Module].
    ///
    /// [FuncDefn]: hugr::ops::FuncDefn
    /// [Module]: hugr::ops::Module
    #[display("No function named 'main' in module.")]
    NoMain,
    /// QSystemPass was applied with a local scope.
    #[display("QSystemPass was applied with a local scope {scope}")]
    LocalScopeError {
        /// The scope that was applied.
        scope: PassScope,
    },
}

impl QSystemPass {
    /// Returns a new `QSystemPass` with constant folding enabled according to
    /// `constant_fold`.
    ///
    /// On by default
    pub fn with_constant_fold(mut self, constant_fold: bool) -> Self {
        self.constant_fold = constant_fold;
        self
    }

    /// Returns a new `QSystemPass` with monomorphization enabled according to
    /// `monomorphize`.
    ///
    /// On by default.
    pub fn with_monomorphize(mut self, monomorphize: bool) -> Self {
        self.monomorphize = monomorphize;
        self
    }

    /// Changes whether we force a total ordering on all ops in the Hugr.
    ///
    /// On by default.
    ///
    /// When enabled, we push quantum ops as early as possible, and we push
    /// `tket.futures.read` ops as late as possible.
    pub fn with_force_order(mut self, force_order: bool) -> Self {
        self.force_order = force_order;
        self
    }

    /// Enables or disables lazification of quantum measurement ops.
    ///
    /// On by default.
    ///
    /// When enabled we replace strict measurement ops with lazy equivalents
    /// from `tket.qsystem`.
    pub fn with_lazify(mut self, lazify: bool) -> Self {
        self.lazify = lazify;
        self
    }

    /// Add order edges in the HUGR regions to force qubit frees to be as early
    /// as possible, quantum ops to be as early as possible, and Future::Reads
    /// to be as late as possible.
    fn force_order(&self, hugr: &mut impl HugrMut<Node = Node>) -> Result<(), QSystemPassError> {
        let Some(root) = self.scope.root(hugr) else {
            // Scope tells us not to modify any node.
            return Ok(());
        };

        force_order(hugr, root, |hugr, node| {
            let optype = hugr.get_optype(node);

            let is_quantum =
                optype.cast::<TketOp>().is_some() || optype.cast::<QSystemOp>().is_some();
            let is_qalloc = matches!(
                optype.cast(),
                Some(TketOp::QAlloc) | Some(TketOp::TryQAlloc)
            ) || optype.cast() == Some(QSystemOp::TryQAlloc);
            let is_qfree =
                optype.cast() == Some(TketOp::QFree) || optype.cast() == Some(QSystemOp::QFree);
            let is_read = optype.cast() == Some(FutureOpDef::Read);

            // HACK: for now qallocs and qfrees are not adequately ordered,
            // see <https://github.com/quantinuum/guppylang/issues/778>. To
            // mitigate this we push qfrees as early as possible and qallocs
            // as late as possible
            //
            // To maximise laziness we push quantum ops (including
            // LazyMeasure) as early as possible and Future::Read as late as
            // possible.
            if is_qfree {
                -3
            } else if is_quantum && !is_qalloc {
                // non-qalloc quantum ops
                -2
            } else if is_qalloc {
                -1
            } else if !is_read {
                // all other ops
                0
            } else {
                // Future::Read ops
                1
            }
        })?;
        Ok(())
    }

    /// Find a function named "main" in the HUGR.
    ///
    /// This is used for backwards compatibility with HUGRs that have a module as
    /// the entrypoint.
    ///
    /// Returns [`QSystemPassError::NoMain`] if there is no function named "main".
    fn find_main(&self, hugr: &impl HugrView<Node = Node>) -> Result<Node, QSystemPassError> {
        hugr.children(hugr.module_root())
            .find(|&n| {
                hugr.get_optype(n)
                    .as_func_defn()
                    .is_some_and(|fd| fd.func_name() == "main")
            })
            .ok_or(QSystemPassError::NoMain)
    }

    /// Collect the set of public function definitions in the HUGR, if `hide_funcs` is
    /// enabled. These will be made private at the end of the pass to avoid
    /// forcing LLVM to compile them as callable.
    fn collect_pub_funcs(&self, hugr: &impl HugrView<Node = Node>) -> Option<HashSet<Node>> {
        self.hide_funcs.then(|| {
            hugr.children(hugr.module_root())
                .filter(|n| {
                    hugr.get_optype(*n)
                        .as_func_defn()
                        .is_some_and(|fd| fd.visibility() == &Visibility::Public)
                })
                .collect::<HashSet<_>>()
        })
    }

    /// Mark non-whitelisted function definitions as private to avoid forcing LLVM to compile them as callable.
    ///
    /// Use [`Self::collect_pub_funcs`] to get the set of whitelisted public functions before running the main passes.
    fn hide_non_pub_funcs(&self, hugr: &mut impl HugrMut<Node = Node>, pub_funcs: HashSet<Node>) {
        for n in hugr.children(hugr.module_root()).collect_vec() {
            if !pub_funcs.contains(&n)
                && let OpType::FuncDefn(fd) = hugr.optype_mut(n)
            {
                *fd.visibility_mut() = Visibility::Private;
            }
        }
    }
}

impl WithScope for QSystemPass {
    fn with_scope(mut self, scope: impl Into<PassScope>) -> Self {
        self.scope = scope.into();
        self
    }
}

impl<H: HugrMut<Node = Node> + 'static> ComposablePass<H> for QSystemPass {
    type Error = QSystemPassError;
    type Result = ();

    /// Run `QSystemPass` on the given Hugr. `registry` is used for
    /// validation, if enabled.
    /// Expects the HUGR to have a function entrypoint.
    fn run(&self, hugr: &mut H) -> Result<(), QSystemPassError> {
        if !matches!(self.scope, PassScope::Global(_)) {
            return Err(QSystemPassError::LocalScopeError {
                scope: self.scope.clone(),
            });
        }

        if self.monomorphize {
            MonomorphizePass::default_with_scope(self.scope.clone())
                .run(hugr)
                .unwrap_or_else(|never| match never {});
            RemoveDeadFuncsPass::default_with_scope(self.scope.clone()).run(hugr)?
        }

        // ReplaceTypes steps (there are several below) can introduce new helper
        // functions that are public to enable linking/sharing. We'll make these private
        // once we're done so that LLVM is not forced to compile them as callable.
        let pub_funcs = self.collect_pub_funcs(hugr);

        LowerTketToQSystemPass::default_with_scope(self.scope.clone()).run(hugr)?;
        if self.lazify {
            ReplaceBoolPass::default_with_scope(self.scope.clone()).run(hugr)?;
        }

        LowerDropsPass::default_with_scope(self.scope.clone()).run(hugr)?;

        // Mark any new helper functions as private.
        if let Some(pub_funcs) = pub_funcs {
            self.hide_non_pub_funcs(hugr, pub_funcs);
        }

        if self.constant_fold {
            ConstantFoldPass::default().run(hugr)?;
        }
        if self.force_order {
            self.force_order(hugr)?;
        }

        // Backwards compatibility: If the entrypoint is a module, find a function named "main" and set that as
        // entrypoint instead.
        if hugr.entrypoint() == hugr.module_root() {
            let main_n = self.find_main(hugr)?;
            hugr.set_entrypoint(main_n);
        }

        Ok(())
    }
}

#[cfg(test)]
mod test {
    use super::*;

    use hugr::{
        Hugr,
        builder::{Dataflow, DataflowHugr, DataflowSubContainer, FunctionBuilder, HugrBuilder},
        core::Visibility,
        extension::prelude::qb_t,
        hugr::hugrmut::HugrMut,
        ops::{ExtensionOp, OpType, handle::NodeHandle},
        std_extensions::arithmetic::float_types::ConstF64,
        std_extensions::collections::array::{ArrayOpBuilder, array_type},
        type_row,
        types::Signature,
    };

    use hugr_core::hugr::internal::{HugrInternals, PortgraphNodeMap};
    use petgraph::visit::{Topo, Walker as _};
    use rstest::rstest;
    use tket::extension::{
        bool::bool_type,
        guppy::{DROP_OP_NAME, GUPPY_EXTENSION},
    };

    use crate::extension::{futures::FutureOpDef, qsystem::QSystemOp};

    #[rstest]
    #[case(false)]
    #[case(true)]
    fn qsystem_pass(#[case] set_entrypoint: bool) {
        let mut mb = hugr::builder::ModuleBuilder::new();
        let func = mb
            .define_function("func", Signature::new_endo(type_row![]))
            .unwrap()
            .finish_with_outputs([])
            .unwrap();

        let (mut hugr, [call_node, h_node, f_node, rx_node, main_node]) = {
            let mut builder = mb
                .define_function_vis(
                    "main",
                    Signature::new(vec![qb_t()], vec![bool_type(), bool_type()]),
                    Visibility::Public,
                )
                .unwrap();
            let [qb] = builder.input_wires_arr();

            // This call node has no dependencies, so it should be lifted above
            // Future Reads and sunk below quantum ops.
            let call_node = builder.call(func.handle(), &[], []).unwrap().node();

            // this LoadConstant should be pushed below the quantum ops where possible
            let angle = builder.add_load_value(ConstF64::new(0.0));
            let f_node = angle.node();

            // with no dependencies, this Reset should be lifted to the start
            let [qb] = builder
                .add_dataflow_op(QSystemOp::Reset, [qb])
                .unwrap()
                .outputs_arr();
            let h_node = qb.node();

            // depending on the angle means this op can't be lifted above the angle ops
            let [qb] = builder
                .add_dataflow_op(QSystemOp::Rz, [qb, angle])
                .unwrap()
                .outputs_arr();
            let rx_node = qb.node();

            // the Measure node will be removed. A Lazy Measure and two Future
            // Reads will be added.  The Lazy Measure will be lifted and the
            // reads will be sunk.
            let [measure_result] = builder
                .add_dataflow_op(QSystemOp::Measure, [qb])
                .unwrap()
                .outputs_arr();

            let main_n = builder
                .finish_with_outputs([measure_result, measure_result])
                .unwrap()
                .node();
            let hugr = mb.finish_hugr().unwrap();
            (hugr, [call_node, h_node, f_node, rx_node, main_n])
        };
        if set_entrypoint {
            // set the entrypoint to the main function
            // if this is not done the "backwards compatibility" code is triggered
            hugr.set_entrypoint(main_node);
        }
        QSystemPass::default().run(&mut hugr).unwrap();

        let (pg, node_map) = hugr.region_portgraph(main_node);
        let topo_sorted = Topo::new(&pg).iter(&pg).collect_vec();

        let get_pos = |x| {
            topo_sorted
                .iter()
                .position(|&y| y == node_map.to_portgraph(x))
                .unwrap()
        };
        assert!(get_pos(h_node) < get_pos(f_node));
        assert!(get_pos(h_node) < get_pos(call_node));
        assert!(get_pos(rx_node) < get_pos(call_node));

        for n in topo_sorted
            .iter()
            .map(|&pg_n| node_map.from_portgraph(pg_n))
            .filter(|&n| FutureOpDef::try_from(hugr.get_optype(n)) == Ok(FutureOpDef::Read))
        {
            assert!(get_pos(call_node) < get_pos(n));
        }
    }

    #[test]
    fn hide_funcs() {
        let orig = {
            let arr_t = || array_type(4, bool_type());
            let mut dfb = FunctionBuilder::new("main", Signature::new_endo(vec![arr_t()])).unwrap();
            let [arr] = dfb.input_wires_arr();
            let (arr1, arr2) = dfb.add_array_clone(bool_type(), 4, arr).unwrap();
            let dop = GUPPY_EXTENSION.get_op(&DROP_OP_NAME).unwrap();
            dfb.add_dataflow_op(
                ExtensionOp::new(dop.clone(), [arr_t().into()]).unwrap(),
                [arr1],
            )
            .unwrap();
            dfb.finish_hugr_with_outputs([arr2]).unwrap()
        };

        let count_pub_funcs = |hugr: &Hugr| {
            hugr.children(hugr.module_root())
                .filter(|n| match hugr.get_optype(*n) {
                    OpType::FuncDefn(fd) => fd.visibility() == &Visibility::Public,
                    OpType::FuncDecl(fd) => fd.visibility() == &Visibility::Public,
                    _ => false,
                })
                .count()
        };

        // Check there are no public funcs (after hiding)
        let mut hugr = orig.clone();
        QSystemPass::default().run(&mut hugr).unwrap();
        assert_eq!(count_pub_funcs(&hugr), 0);

        // Run again without hiding...
        let mut hugr_public = orig;
        QSystemPass {
            hide_funcs: false,
            ..Default::default()
        }
        .run(&mut hugr_public)
        .unwrap();

        assert_eq!(count_pub_funcs(&hugr_public), 4);
        assert_eq!(
            hugr.children(hugr.module_root()).count(),
            hugr_public.children(hugr_public.module_root()).count()
        );
        assert_eq!(hugr.num_nodes(), hugr_public.num_nodes());
    }
}
