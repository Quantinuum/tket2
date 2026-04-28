//! Pass to inline calls to functions, controlled by [InlineAnnotation] metadata.
use std::collections::{HashMap, HashSet, VecDeque};

use crate::metadata::InlineAnnotation;
use crate::passes::{ComposablePass, InScope, PassScope, composable::WithScope};
use hugr::hugr::patch::Patch;
use hugr::hugr::patch::inline_call::{InlineCall, InlineCallError};
use hugr_core::module_graph::{ModuleGraph, StaticNode};
use hugr_core::{Node, hugr::hugrmut::HugrMut};

use itertools::Itertools;
use petgraph::algo::tarjan_scc;
use petgraph::data::DataMap;
use petgraph::visit::{
    Dfs, IntoNeighbors, IntoNodeIdentifiers, IntoNodeReferences, NodeFiltered, NodeIndexable,
    Visitable, Walker,
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
        let reachable_always = {
            let filter_reachable = match &self.scope {
                PassScope::Global(_) => None,
                PassScope::EntrypointFlat | PassScope::EntrypointRecursive => Some(
                    Dfs::new(cg.graph(), cg.node_index(hugr.entrypoint()).unwrap())
                        .iter(&cg.graph())
                        .collect::<Vec<_>>(),
                ),
            };
            hugr.children(hugr.module_root())
                .filter_map(|n| cg.node_index(n).map(|ni| (n, ni)))
                .filter(|(_, ni)| {
                    filter_reachable
                        .as_ref()
                        .is_none_or(|reachable| reachable.contains(ni))
                })
                .filter(|(n, _)| {
                    hugr.get_optype(*n).is_func_defn()
                        && hugr.get_metadata::<InlineAnnotation>(*n)
                            == Some(InlineAnnotation::Always)
                })
                .collect::<HashMap<_, _>>()
        };

        // If we inverted the map, we'd save a little here, but it'd get much worse in the reverse lookup below
        if let Some(cycle) = cycles(&NodeFiltered::from_fn(cg.graph(), |n| {
            match cg.graph().node_weight(n).unwrap() {
                StaticNode::FuncDefn(func) => reachable_always.contains_key(func),
                _ => false,
            }
        }))
        .next()
        {
            return Err(InlineError::AlwaysCycle(cycle));
        }
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
                    && reachable_always.contains_key(&func)
                {
                    to_inline.push((child, func));
                }
            }
            while let Some((call, func)) = to_inline.pop() {
                do_inline(call, hugr);
                // We have not inlined everything into `func` yet, so there may still be some work to do in the inlined copy.
                // (Inlining in postorder traversal order would avoid this for PassScope::Global,
                // but we cannot do that for PassScope::EntrypointFlat/Recursive, as there we cannot
                // touch the functions until they are inlined into the entrypoint-subtree.)
                if !seen.contains(&func) {
                    parents.push_back(call);
                }
            }
        }
        // Also inline any function called only once.
        // First remove the always-inlined functions themselves, as they are now unreachable.
        let funcs_to_preserve = self.scope.preserve_interface(hugr).collect::<HashSet<_>>();
        if root == hugr.module_root() {
            for func in reachable_always.keys() {
                if !funcs_to_preserve.contains(func) {
                    hugr.remove_subtree(*func);
                }
            }
        }
        let cg = ModuleGraph::new(hugr);
        let funcs_in_cycles = cycles(cg.graph()).flatten().collect::<HashSet<_>>();

        let called_once = cg
            .graph()
            .node_references()
            .filter_map(|(_, sn)| match sn {
                StaticNode::FuncDefn(func)
                    if !funcs_to_preserve.contains(func) && !funcs_in_cycles.contains(func) =>
                {
                    hugr.static_targets(*func)
                        .unwrap()
                        .exactly_one()
                        .ok()
                        .map(|(call, _port)| (*func, call))
                }

                _ => None,
            })
            .collect::<Vec<_>>();
        for (func, call) in called_once {
            if hugr.get_optype(call).is_call() // skip LoadFunctions
                && self.scope.in_scope(hugr, call) != InScope::No
            {
                do_inline(call, hugr);
                if self.scope.in_scope(hugr, func) == InScope::Yes {
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

fn do_inline<H: HugrMut>(call: H::Node, hugr: &mut H) {
    match InlineCall::new(call).apply(hugr) {
        Ok(()) => (),
        Err(InlineCallError::NotCallNode(_, _) | InlineCallError::CallTargetNotFuncDefn(_, _)) => {
            unreachable!();
        }
        Err(e) => {
            todo!("Update to handle {e:?}")
        }
    }
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
    fn test_one_deep(#[values(1, 2, 5)] num_calls: usize, #[values(false, true)] always: bool) {
        let mut main =
            FunctionBuilder::new("main", Signature::new_endo([qb_t(), qb_t(), qb_t()])).unwrap();

        let mut mb = main.module_root_builder();
        let swap = mb
            .define_function("called-once", Signature::new_endo([qb_t(), qb_t()]))
            .unwrap();
        let [a, b] = swap.input_wires_arr();
        let swap = swap.finish_with_outputs([b, a]).unwrap();

        let [mut a, mut b, c] = main.input_wires_arr();
        for _ in 0..num_calls {
            [a, b] = main.call(swap.handle(), &[], [a, b]).unwrap().outputs_arr();
        }
        let mut hugr = main.finish_hugr_with_outputs([a, b, c]).unwrap();
        if always {
            hugr.set_metadata::<InlineAnnotation>(swap.node(), InlineAnnotation::Always);
        }
        let backup = hugr.clone();

        InlinePass::default().run(&mut hugr).unwrap();
        hugr.validate().unwrap();
        let should_be_inlined = always || num_calls == 1;
        if should_be_inlined {
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
        } else {
            assert_eq!(hugr, backup);
        }
    }

    #[test]
    fn entrypoint_scope() {
        // TODO
    }

    // TODO cycle of one always func and one not always, should be inlined to a self-recursive func

    // TODO cycle of two funcs where one has no other calls (and one has, or is preserved),
    // should be inlined to a self-recursive func
}
