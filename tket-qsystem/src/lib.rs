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
use hugr::{Hugr, HugrView, Node, core::Visibility, ops::OpType};
use hugr_core::hugr::internal::HugrMutInternals;
use hugr_passes::PassScope;
use hugr_passes::composable::{Preserve, WithScope};
use hugr_passes::const_fold::{ConstFoldError, ConstantFoldPass};
use hugr_passes::{
    ComposablePass as _, MonomorphizePass, RemoveDeadFuncsError, RemoveDeadFuncsPass, force_order,
    replace_types::ReplaceTypesError,
};
use std::collections::HashSet;

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
/// To construct a `QSystemPass` use [Default::default].
#[derive(Debug, Clone, Copy)]
pub struct QSystemPass {
    constant_fold: bool,
    monomorphize: bool,
    force_order: bool,
    lazify: bool,
    hide_funcs: bool,
}

impl Default for QSystemPass {
    fn default() -> Self {
        Self {
            constant_fold: false,
            monomorphize: true,
            force_order: true,
            lazify: true,
            hide_funcs: true,
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
    ///  [RemoveDeadFuncsPass]: hugr_passes::RemoveDeadFuncsError
    DCEError(RemoveDeadFuncsError),
    /// No [FuncDefn] named "main" in [Module].
    ///
    /// [FuncDefn]: hugr::ops::FuncDefn
    /// [Module]: hugr::ops::Module
    #[display("No function named 'main' in module.")]
    NoMain,
}

impl QSystemPass {
    /// Run `QSystemPass` on the given [Hugr]. `registry` is used for
    /// validation, if enabled.
    /// Expects the HUGR to have a function entrypoint.
    pub fn run(&self, hugr: &mut Hugr) -> Result<(), QSystemPassError> {
        let entrypoint = if hugr.entrypoint_optype().is_module() {
            // backwards compatibility: if the entrypoint is a module, we look for
            // a function named "main" in the module and use that as the entrypoint.
            hugr.children(hugr.entrypoint())
                .find(|&n| {
                    hugr.get_optype(n)
                        .as_func_defn()
                        .is_some_and(|fd| fd.func_name() == "main")
                })
                .ok_or(QSystemPassError::NoMain)?
        } else {
            hugr.entrypoint()
        };

        // passes that run on whole module
        hugr.set_entrypoint(hugr.module_root());
        if self.monomorphize {
            self.monomorphization().run(hugr).unwrap();

            // Will move to pass scopes in <https://github.com/Quantinuum/tket2/pull/1429>
            let rdfp = RemoveDeadFuncsPass::default_with_scope(PassScope::Global(Preserve::All));
            rdfp.run(hugr)?
        }

        // ReplaceTypes steps (there are several below) can introduce new helper
        // functions that are public to enable linking/sharing. We'll make these private
        // once we're done so that LLVM is not forced to compile them as callable.
        let pubfuncs = self.hide_funcs.then(|| {
            hugr.children(hugr.module_root())
                .filter(|n| {
                    hugr.get_optype(*n)
                        .as_func_defn()
                        .is_some_and(|fd| fd.visibility() == &Visibility::Public)
                })
                .collect::<HashSet<_>>()
        });

        self.lower_tk2().run(hugr)?;
        if self.lazify {
            self.replace_bools().run(hugr)?;
        }
        self.lower_drops().run(hugr)?;

        if let Some(pubfuncs) = pubfuncs {
            for n in hugr
                .children(hugr.module_root())
                .filter(|n| !pubfuncs.contains(n))
                .collect::<Vec<_>>()
            {
                if let OpType::FuncDefn(fd) = hugr.optype_mut(n) {
                    *fd.visibility_mut() = Visibility::Private;
                }
            }
        }

        if self.constant_fold {
            self.constant_fold().run(hugr)?;
        }
        if self.force_order {
            self.force_order(hugr)?;
        }
        // restore the entrypoint
        hugr.set_entrypoint(entrypoint);
        Ok(())
    }

    fn force_order(&self, hugr: &mut Hugr) -> Result<(), QSystemPassError> {
        force_order(hugr, hugr.entrypoint(), |hugr, node| {
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
        Ok::<_, QSystemPassError>(())
    }

    fn lower_tk2(&self) -> LowerTketToQSystemPass {
        LowerTketToQSystemPass
    }

    fn replace_bools(&self) -> ReplaceBoolPass {
        ReplaceBoolPass
    }

    fn constant_fold(&self) -> ConstantFoldPass {
        ConstantFoldPass::default()
    }

    fn monomorphization(&self) -> MonomorphizePass {
        MonomorphizePass::default()
    }

    fn lower_drops(&self) -> LowerDropsPass {
        LowerDropsPass
    }

    /// Returns a new `QSystemPass` with constant folding enabled according to
    /// `constant_fold`.
    ///
    /// Off by default.
    pub fn with_constant_fold(mut self, constant_fold: bool) -> Self {
        self.constant_fold = constant_fold;
        self
    }

    /// Returns a new `QSystemPass` with monomorphization enabled according to
    /// `monomorphize`.
    ///
    /// On by default.
    pub fn with_monormophize(mut self, monomorphize: bool) -> Self {
        self.monomorphize = monomorphize;
        self
    }

    /// Returns a new `QSystemPass` with forcing the HUGR to have
    /// totally-ordered ops enabled according to `force_order`.
    ///
    /// On by default.
    ///
    /// When enabled, we push quantum ops as early as possible, and we push
    /// `tket.futures.read` ops as late as possible.
    pub fn with_force_order(mut self, force_order: bool) -> Self {
        self.force_order = force_order;
        self
    }

    /// Returns a new `QSystemPass` with lazification enabled according to
    /// `lazify`.
    ///
    /// On by default.
    ///
    /// When enabled we replace strict measurement ops with lazy equivalents
    /// from `tket.qsystem`.
    pub fn with_lazify(mut self, lazify: bool) -> Self {
        self.lazify = lazify;
        self
    }
}

#[cfg(test)]
mod test {
    use hugr::{
        Hugr, HugrView as _,
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
    use itertools::Itertools as _;
    use petgraph::visit::{Topo, Walker as _};
    use rstest::rstest;
    use tket::extension::{
        bool::bool_type,
        guppy::{DROP_OP_NAME, GUPPY_EXTENSION},
    };

    use crate::{
        QSystemPass,
        extension::{futures::FutureOpDef, qsystem::QSystemOp},
    };

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
                .define_function(
                    "main",
                    Signature::new(vec![qb_t()], vec![bool_type(), bool_type()]),
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
