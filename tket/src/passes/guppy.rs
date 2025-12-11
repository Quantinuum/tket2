//! A pass that normalizes the structure of Guppy-generated circuits into something that can be optimized by tket.

use hugr::algorithms::const_fold::{ConstFoldError, ConstantFoldPass};
use hugr::algorithms::inline_dfgs::InlineDFGsPass;
use hugr::algorithms::normalize_cfgs::{NormalizeCFGError, NormalizeCFGPass};
use hugr::algorithms::untuple::{UntupleError, UntupleRecursive};
use hugr::algorithms::{
    inline_acyclic, ComposablePass, RemoveDeadFuncsError, RemoveDeadFuncsPass, UntuplePass,
};
use hugr::hugr::hugrmut::HugrMut;
use hugr::hugr::patch::inline_dfg::InlineDFGError;
use hugr::Node;

use crate::passes::{unpack_container::TypeUnpacker, BorrowSquashPass};

/// Normalize the structure of a Guppy-generated circuit into something that can be optimized by tket.
///
/// This is a mixture of global optimization passes, and operations that optimize the entrypoint.
#[derive(Clone, Copy)]
pub struct NormalizeGuppy {
    /// Whether to simplify CFG control flow.
    simplify_cfgs: bool,
    /// Whether to remove tuple/untuple operations.
    untuple: bool,
    /// Whether to constant fold the program.
    constant_fold: bool,
    /// Whether to remove dead functions.
    dead_funcs: bool,
    /// Whether to inline DFG operations.
    inline_dfgs: bool,
    /// Whether to squash BorrowArray borrow/return ops
    squash_borrows: bool,
}

impl NormalizeGuppy {
    /// Set whether to simplify CFG control flow.
    pub fn simplify_cfgs(&mut self, simplify_cfgs: bool) -> &mut Self {
        self.simplify_cfgs = simplify_cfgs;
        self
    }
    /// Set whether to remove tuple/untuple operations.
    pub fn remove_tuple_untuple(&mut self, untuple: bool) -> &mut Self {
        self.untuple = untuple;
        self
    }
    /// Set whether to constant fold the program.
    pub fn constant_folding(&mut self, constant_fold: bool) -> &mut Self {
        self.constant_fold = constant_fold;
        self
    }
    /// Set whether to remove dead functions.
    pub fn remove_dead_funcs(&mut self, dead_funcs: bool) -> &mut Self {
        self.dead_funcs = dead_funcs;
        self
    }
    /// Set whether to inline DFG operations.
    pub fn inline_dfgs(&mut self, inline: bool) -> &mut Self {
        self.inline_dfgs = inline;
        self
    }
    /// Set whether to squash BorrowArray borrow/return ops
    pub fn squash_borrows(&mut self, squash: bool) -> &mut Self {
        self.squash_borrows = squash;
        self
    }
}

impl Default for NormalizeGuppy {
    fn default() -> Self {
        Self {
            simplify_cfgs: true,
            constant_fold: true,
            untuple: true,
            dead_funcs: true,
            inline_dfgs: true,
            squash_borrows: true,
        }
    }
}

impl<H: HugrMut<Node = Node> + 'static> ComposablePass<H> for NormalizeGuppy {
    type Error = NormalizeGuppyErrors;
    type Result = ();
    fn run(&self, hugr: &mut H) -> Result<Self::Result, Self::Error> {
        // We probably shouldn't inline quite as aggressively as this, but
        // the results demonstrate how much we need to do at least some of it:
        let qubit_finder = TypeUnpacker::for_qubits();
        inline_acyclic(hugr, |h, call| {
            // Look for qubits. Use instantiated type so that we inline generic
            // (e.g. container) functions as this might enable better qubit tracking.
            let inst = &h.get_optype(call).as_call().unwrap().instantiation;
            inst.input_types()
                .iter()
                .chain(inst.output_types())
                .any(|ty| qubit_finder.contains_element_type(ty))
        })
        .unwrap();
        // Many functions now unreachable, so remove - this may improve compilation speed,
        // although not if all remaining phases operate only beneath the entrypoint.
        // Shouldn't be affected by anything else until we start removing untaken branches.
        if self.dead_funcs {
            RemoveDeadFuncsPass::default().run(hugr)?;
        }

        if self.simplify_cfgs {
            NormalizeCFGPass::default().run(hugr)?;
        }
        // When we do function inlining, do this after, to sort out argument marshalling
        if self.untuple {
            UntuplePass::new(UntupleRecursive::Recursive).run(hugr)?;
        }
        // Should propagate through untuple, so could do earlier, and must be before BorrowSquash
        if self.constant_fold {
            ConstantFoldPass::default().run(hugr)?;
        }
        // Do earlier? Nothing creates DFGs
        if self.inline_dfgs {
            InlineDFGsPass.run(hugr).unwrap_or_else(|e| match e {})
        }
        // Potentially, could (need to) do fixpoint here with untuple,
        // as both create opportunities for the other
        if self.squash_borrows {
            BorrowSquashPass::default()
                .run(hugr)
                .unwrap_or_else(|e| match e {});
        }

        Ok(())
    }
}

/// Errors that can occur during the guppy-program normalization process.
#[derive(derive_more::Error, Debug, derive_more::Display, derive_more::From)]
pub enum NormalizeGuppyErrors {
    /// Error while simplifying CFG control flow.
    SimplifyCFG(NormalizeCFGError),
    /// Error while removing tuple/untuple operations.
    Untuple(UntupleError),
    /// Error while constant folding.
    ConstantFold(ConstFoldError),
    /// Error while removing dead functions.
    DeadFuncs(RemoveDeadFuncsError),
    /// Error while inlining DFG operations.
    Inline(InlineDFGError),
}

#[cfg(test)]
mod test {
    use hugr::builder::{Dataflow, DataflowHugr, FunctionBuilder};
    use hugr::extension::prelude::qb_t;
    use hugr::types::Signature;

    use crate::TketOp;

    use super::*;

    /// Running the pass with all options disabled should still work (and do nothing).
    #[test]
    fn test_guppy_pass_noop() {
        let mut b = FunctionBuilder::new("main", Signature::new_endo(vec![qb_t()])).unwrap();
        let [q] = b.input_wires_arr();
        let [q] = b.add_dataflow_op(TketOp::H, [q]).unwrap().outputs_arr();
        let hugr = b.finish_hugr_with_outputs([q]).unwrap();

        let mut hugr2 = hugr.clone();
        NormalizeGuppy::default()
            .simplify_cfgs(false)
            .remove_tuple_untuple(false)
            .constant_folding(false)
            .remove_dead_funcs(false)
            .inline_dfgs(false)
            .run(&mut hugr2)
            .unwrap();

        assert_eq!(hugr2, hugr);
    }
}
