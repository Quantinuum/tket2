//! Pass to inline calls to functions, controlled by [InlineAnnotation] metadata.
use std::collections::{HashSet, VecDeque};

use crate::metadata::InlineAnnotation;
use crate::passes::{ComposablePass, PassScope, composable::WithScope};
use hugr::hugr::patch::inline_call::InlineCall;
use hugr_core::module_graph::{ModuleGraph, StaticNode};
use hugr_core::{Node, hugr::hugrmut::HugrMut};

use itertools::Itertools;
use petgraph::algo::tarjan_scc;
use petgraph::data::DataMap;
use petgraph::visit::{
    Dfs, IntoNeighbors, IntoNodeIdentifiers, NodeFiltered, NodeIndexable, Visitable, Walker,
};

/// Errors that may be raised by [InlinePass]
#[derive(Clone, Debug, PartialEq, Eq, derive_more::Display)]
pub enum InlineError<N = Node> {
    /// Functions annotated with [InlineAnnotation::Always] form a cycle
    /// so inlining would produce an infinitely-big program
    #[display("Cycle detected in functions marked to Always inline: {_0:?}")]
    AlwaysCycle(Vec<N>),
}

impl<N: std::fmt::Debug> std::error::Error for InlineError<N> {}

/// A [ComposablePass] that inlines `Call`s to functions
/// according to [InlineAnnotation]s.
#[derive(Default, Clone, Debug)]
pub struct InlinePass {
    scope: PassScope,
}

impl WithScope for InlinePass {
    fn with_scope(self, scope: impl Into<PassScope>) -> Self {
        Self {
            scope: scope.into(),
        }
    }
}

impl<H: HugrMut> ComposablePass<H> for InlinePass {
    type Error = InlineError<H::Node>;
    type Result = ();

    fn run(&self, hugr: &mut H) -> Result<(), InlineError<H::Node>> {
        let Some(root) = self.scope.root(hugr) else {
            return Ok(()); // Nothing to do
        };
        let cg = ModuleGraph::new(hugr);
        let always_funcs = hugr.children(hugr.module_root()).filter(|n| {
            hugr.get_optype(*n).is_func_defn()
                && hugr.get_metadata::<InlineAnnotation>(*n) == Some(InlineAnnotation::Always)
        });
        // We're going to object if there's a cycle of functions marked Always, as that would
        // lead to an infinitely big Hugr. However, don't object unless such a cycle is reachable
        // from the entrypoint...
        let reachable_always: HashSet<H::Node> = match &self.scope {
            PassScope::Global(_) => always_funcs.collect(),
            PassScope::EntrypointFlat | PassScope::EntrypointRecursive => {
                let reachable = Dfs::new(cg.graph(), cg.node_index(hugr.entrypoint()).unwrap())
                    .iter(&cg.graph())
                    .collect::<HashSet<_>>();
                always_funcs
                    .filter(|n| {
                        let ni = cg.node_index(*n).unwrap();
                        reachable.contains(&ni)
                    })
                    .collect()
            }
        };
        let always_cg =
            NodeFiltered::from_fn(cg.graph(), |n| match cg.graph().node_weight(n).unwrap() {
                StaticNode::FuncDefn(func) => reachable_always.contains(func),
                _ => false,
            });
        if let Some(cycle) = cycles(&always_cg).next() {
            return Err(InlineError::AlwaysCycle(cycle));
        }
        // Proceed with inlining. Do outermost first within the scope root, as we cannot
        // inline into functions that are outside the scope until they themselves are inlined
        // beneath the root.
        let mut parents = VecDeque::from([root]);
        let mut seen = HashSet::new();
        while let Some(parent) = parents.pop_front() {
            if hugr.get_optype(parent).is_func_defn() {
                seen.insert(parent);
            }
            let mut to_inline = Vec::new();
            for child in hugr.children(parent) {
                if hugr.first_child(child).is_some() {
                    parents.push_back(child);
                } else if hugr.get_optype(child).is_call()
                    && let Some(func) = hugr.static_source(child)
                    && reachable_always.contains(&func)
                {
                    to_inline.push((child, func));
                }
            }
            while let Some((call, func)) = to_inline.pop() {
                // We've already checked the error conditions.
                hugr.apply_patch(InlineCall::new(call)).unwrap();
                if !seen.contains(&func) {
                    // We have not inlined everything into `func` yet,
                    // so there may still be some work to do in the inlined copy.
                    parents.push_back(call);
                }
            }
        }
        // Remove the always-inlined functions themselves, as they are now unreachable.
        let funcs_to_preserve = self.scope.preserve_interface(hugr).collect::<HashSet<_>>();
        if root == hugr.module_root() {
            for func in reachable_always {
                debug_assert!(hugr.static_targets(func).unwrap().next().is_none());
                if !funcs_to_preserve.contains(&func) {
                    hugr.remove_subtree(func);
                }
            }
        }
        Ok(())
    }
}

fn cycles<'a, N: Copy>(
    g: impl Copy
    + Visitable
    + DataMap<NodeWeight = StaticNode<N>>
    + IntoNeighbors
    + IntoNodeIdentifiers
    + NodeIndexable
    + 'a,
) -> impl Iterator<Item = Vec<N>> + 'a {
    tarjan_scc(g)
        .into_iter()
        .filter(move |ns| {
            ns.iter()
                .exactly_one()
                .ok()
                .is_none_or(|n| // multi-node, or single-node cycle
            g.neighbors(*n).contains(n))
        })
        .map(move |cycle| {
            cycle
                .into_iter()
                .map(|n| match g.node_weight(n).unwrap() {
                    StaticNode::FuncDefn(fd) => *fd,
                    _ => panic!("Expected only FuncDefns in sccs"),
                })
                .collect()
        })
}

#[cfg(test)]
mod test {
    use rstest::rstest;
    use std::collections::HashSet;

    use crate::passes::{ComposablePass, RemoveDeadFuncsPass, inline_dfgs::InlineDFGsPass};
    use hugr::{
        HugrView,
        builder::{
            Container, Dataflow, DataflowHugr, DataflowSubContainer, FunctionBuilder, HugrBuilder,
        },
        extension::prelude::{qb_t, usize_t},
        hugr::hugrmut::HugrMut,
        ops::handle::NodeHandle,
        types::Signature,
    };

    use super::{InlineAnnotation, InlineError, InlinePass};

    #[test]
    fn test_single_cycle() {
        let mut main = FunctionBuilder::new("main", Signature::new_endo([qb_t(), qb_t()])).unwrap();
        let mut mb = main.module_root_builder();
        let mut fb = mb
            .define_function("self-call", Signature::new_endo([qb_t()]))
            .unwrap();
        let c = fb
            .call::<true>(&fb.container_node().into(), &[], fb.input_wires())
            .unwrap();
        let fb = fb.finish_with_outputs(c.outputs()).unwrap();
        let inputs = main.input_wires();
        let mut hugr = main.finish_hugr_with_outputs(inputs).unwrap();
        hugr.set_metadata::<InlineAnnotation>(fb.node(), InlineAnnotation::Always);
        let backup = hugr.clone();

        // We error even though the function is not called
        let e = InlinePass::default().run(&mut hugr).unwrap_err();
        assert_eq!(e, InlineError::AlwaysCycle(vec![fb.node()]));
        assert_eq!(hugr, backup);

        RemoveDeadFuncsPass::default().run(&mut hugr).unwrap();
        assert_eq!(
            hugr.children(hugr.module_root()).collect::<Vec<_>>(),
            [hugr.entrypoint()]
        );
        let backup = hugr.clone();
        InlinePass::default().run(&mut hugr).unwrap();
        assert_eq!(hugr, backup);
    }

    #[test]
    fn cycle() {
        let mut main = FunctionBuilder::new("main", Signature::new_endo([usize_t()])).unwrap();
        let main_h = main.container_node().into();
        let mut mb = main.module_root_builder();
        let mut fb1 = mb
            .define_function("f1", Signature::new_endo([usize_t()]))
            .unwrap();
        let c1 = fb1.call::<true>(&main_h, &[], fb1.input_wires()).unwrap();
        let fb1 = fb1.finish_with_outputs(c1.outputs()).unwrap();
        let c2 = main.call(fb1.handle(), &[], main.input_wires()).unwrap();
        let mut hugr = main.finish_hugr_with_outputs(c2.outputs()).unwrap();
        hugr.set_metadata::<InlineAnnotation>(hugr.entrypoint(), InlineAnnotation::Always);
        InlinePass::default().run(&mut hugr.clone()).unwrap(); // Ok

        hugr.set_metadata::<InlineAnnotation>(fb1.node(), InlineAnnotation::Always);
        let e = InlinePass::default().run(&mut hugr).unwrap_err();
        assert_eq!(
            e,
            InlineError::AlwaysCycle(vec![fb1.node(), hugr.entrypoint()])
        );
    }

    #[rstest]
    fn test_one_deep(#[values(1, 2, 5)] num_calls: usize) {
        let mut main =
            FunctionBuilder::new("main", Signature::new_endo([qb_t(), qb_t(), qb_t()])).unwrap();

        let mut mb = main.module_root_builder();
        let swap = mb
            .define_function("swap", Signature::new_endo([qb_t(), qb_t()]))
            .unwrap();
        let [a, b] = swap.input_wires_arr();
        let swap = swap.finish_with_outputs([b, a]).unwrap();

        let [mut a, mut b, c] = main.input_wires_arr();
        for _ in 0..num_calls {
            [a, b] = main.call(swap.handle(), &[], [a, b]).unwrap().outputs_arr();
        }
        let mut hugr = main.finish_hugr_with_outputs([a, b, c]).unwrap();
        hugr.set_metadata::<InlineAnnotation>(swap.node(), InlineAnnotation::Always);

        InlinePass::default().run(&mut hugr).unwrap();
        hugr.validate().unwrap();

        let swap_present =
            hugr.contains_node(swap.node()) && hugr.get_optype(swap.node()).is_func_defn();
        assert!(!swap_present);
        InlineDFGsPass::default().run(&mut hugr).unwrap();
        hugr.validate().unwrap();
        let [inp, outp] = hugr.get_io(hugr.entrypoint()).unwrap();
        assert_eq!(
            HashSet::from_iter(hugr.input_neighbours(outp)),
            HashSet::from([inp])
        );
    }

    #[test]
    fn entrypoint_scope() {
        // TODO
    }

    // TODO cycle of one always func and one not always, should be inlined to a self-recursive func

    // TODO cycle of two funcs where one has no other calls (and one has, or is preserved),
    // should be inlined to a self-recursive func
}
