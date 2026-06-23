//! Contains a pass to inline calls to selected functions in a Hugr.
use std::collections::{BTreeSet, HashMap, HashSet, VecDeque};

use hugr::HugrView;
use itertools::Itertools;
use petgraph::algo::tarjan_scc;
use petgraph::data::DataMap;
use petgraph::visit::{
    Dfs, IntoNeighbors, IntoNodeIdentifiers, NodeFiltered, NodeIndexable, Visitable, Walker,
};

use hugr_core::Node;
use hugr_core::hugr::{hugrmut::HugrMut, patch::inline_call::InlineCall};
use hugr_core::module_graph::{ModuleGraph, StaticNode};

use crate::metadata::InlineAnnotation;
use crate::passes::{ComposablePass, PassScope, WithScope};

/// Error raised by [InlineFunctionsPass]
#[derive(Clone, Debug, thiserror::Error, PartialEq, Eq)]
#[non_exhaustive]
pub enum InlineFuncsError<N = Node> {
    /// Functions annotated with [InlineAnnotation::Always] form a cycle.
    #[error("Cycle detected in functions marked to Always inline: {0:?}")]
    AlwaysCycle(Vec<N>),
}

/// Heuristic for deciding which functions to inline.
///
/// Note that recursive functions are never inlined.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub enum InlineFuncsHeuristic {
    /// Inline functions that contain at most the specified number of children
    /// nodes.
    MaxSize(usize),
    /// Inline all non-recursive functions.
    All,
    // TODO: Heuristic based on function signature. <https://github.com/Quantinuum/tket2/issues/1003>
}

impl InlineFuncsHeuristic {
    /// Returns `True` if the function definition should be inlined.
    fn should_inline<H: HugrView>(&self, func: H::Node, hugr: &H) -> bool {
        match self {
            InlineFuncsHeuristic::MaxSize(size) => hugr.descendants(func).count() <= *size,
            InlineFuncsHeuristic::All => true,
        }
    }
}

impl Default for InlineFuncsHeuristic {
    fn default() -> Self {
        Self::MaxSize(64)
    }
}

/// Inlines non-recursive function calls.
///
/// We use a heuristic to determine which functions to inline. Currently, we
/// inline all functions whose number of descendant nodes is at most
/// `max_inline_size` (defaults to 64).
#[derive(Debug, Default, Clone)]
pub struct InlineFunctionsPass {
    /// Heuristic for deciding which functions to inline.
    heuristic: InlineFuncsHeuristic,
    /// Where to apply the pass. See [PassScope] for details.
    scope: PassScope,
}

impl InlineFunctionsPass {
    /// Sets the heuristic for deciding which functions to inline.
    pub fn with_heuristic(mut self, heuristic: InlineFuncsHeuristic) -> Self {
        self.heuristic = heuristic;
        self
    }
}

impl<H: HugrMut> ComposablePass<H> for InlineFunctionsPass {
    type Error = InlineFuncsError<H::Node>;
    type Result = ();

    fn run(&self, h: &mut H) -> Result<(), Self::Error> {
        inline_always_scoped(h, &self.scope)?;
        let mut should_inline_cache: HashMap<H::Node, bool> = HashMap::new();
        inline_acyclic_scoped(h, self.scope.clone(), |h, call| {
            let Some(func) = h.static_source(call) else {
                return false;
            };
            *should_inline_cache.entry(func).or_insert_with(|| {
                match h.get_metadata::<InlineAnnotation>(func) {
                    Some(InlineAnnotation::Never) => false,
                    Some(InlineAnnotation::BestEffort) => true,
                    Some(InlineAnnotation::Always) => {
                        panic!("Always-inline funcs should have no calls remaining")
                    }
                    None => self.heuristic.should_inline(func, h),
                }
            })
        })
    }
}

impl WithScope for InlineFunctionsPass {
    fn with_scope(mut self, scope: impl Into<PassScope>) -> Self {
        self.scope = scope.into();
        self
    }
}

/// Inline (a subset of) [Call]s whose target [FuncDefn]s are not in cycles of the call
/// graph.
///
/// The function `call_predicate` is passed each such [Call] node and can return
/// `false` to prevent that Call from being inlined. (Note the [Call] may be created as
/// a result of previous inlinings so may not have existed in the original Hugr).
///
/// [Call]: hugr_core::ops::Call
/// [FuncDefn]: hugr_core::ops::FuncDefn
pub fn inline_acyclic_scoped<H: HugrMut>(
    h: &mut H,
    scope: impl Into<PassScope>,
    mut call_predicate: impl FnMut(&H, H::Node) -> bool,
) -> Result<(), InlineFuncsError<H::Node>> {
    let scope: PassScope = scope.into();
    let Some(scope_root) = scope.root(h) else {
        return Ok(());
    };

    let cg = ModuleGraph::new(&*h);
    let g = cg.graph();
    let all_funcs_in_cycles = tarjan_scc(g)
        .into_iter()
        .flat_map(|mut ns| {
            if let Ok(n) = ns.iter().exactly_one()
                && g.edges_connecting(*n, *n).next().is_none()
            {
                ns.clear(); // Single-node SCC has no self edge, so discard
            }
            ns.into_iter().map(|n| {
                let StaticNode::FuncDefn(fd) = g.node_weight(n).unwrap() else {
                    panic!("Expected only FuncDefns in sccs")
                };
                *fd
            })
        })
        .collect::<HashSet<_>>();
    let target_funcs: HashSet<H::Node> = h
        .children(h.module_root())
        .filter(|n| h.get_optype(*n).is_func_defn() && !all_funcs_in_cycles.contains(n))
        .collect();

    let mut q = VecDeque::from([scope_root]);
    while let Some(n) = q.pop_front() {
        if h.get_optype(n).is_call()
            && let Some(t) = h.static_source(n)
            && target_funcs.contains(&t)
            && call_predicate(h, n)
        {
            // We've already checked all error conditions
            h.apply_patch(InlineCall::new(n)).unwrap();
        }
        // Traverse children - including any resulting from turning Call into DFG
        if scope.recursive() {
            q.extend(h.children(n));
        }
    }
    Ok(())
}

fn inline_always_scoped<H: HugrMut>(
    hugr: &mut H,
    scope: &PassScope,
) -> Result<(), InlineFuncsError<H::Node>> {
    let Some(root) = scope.root(hugr) else {
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
    let reachable_always: BTreeSet<H::Node> = match &scope {
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
        return Err(InlineFuncsError::AlwaysCycle(cycle));
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
            // We've already checked all error conditions.
            hugr.apply_patch(InlineCall::new(call)).unwrap();
            if !seen.contains(&func) {
                // We have not inlined everything into `func` yet,
                // so there may still be some work to do in the inlined copy.
                parents.push_back(call);
            }
        }
    }

    if root == hugr.module_root() {
        // Remove the always-inlined functions themselves, as they are now unreachable.
        let funcs_to_preserve = scope.preserve_interface(hugr).collect::<HashSet<_>>();
        for func in reachable_always {
            debug_assert!(hugr.static_targets(func).unwrap().next().is_none());
            if !funcs_to_preserve.contains(&func) {
                hugr.remove_subtree(func);
            }
        }
    }
    Ok(())
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
                .is_none_or(|n| g.neighbors(*n).contains(n))
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
    use std::collections::HashSet;

    use hugr::builder::{Container, DataflowHugr, FunctionBuilder};
    use hugr::extension::prelude::usize_t;
    use hugr::ops::handle::NodeHandle;
    use itertools::Itertools;
    use rstest::rstest;

    use hugr_core::HugrView;
    use hugr_core::builder::{Dataflow, DataflowSubContainer, HugrBuilder, ModuleBuilder};
    use hugr_core::core::HugrNode;
    use hugr_core::hugr::hugrmut::HugrMut;
    use hugr_core::module_graph::{ModuleGraph, StaticNode};
    use hugr_core::ops::OpType;
    use hugr_core::{Hugr, extension::prelude::qb_t, types::Signature};

    use super::{
        InlineFuncsError, InlineFuncsHeuristic, InlineFunctionsPass, inline_acyclic_scoped,
    };
    use crate::TketOp;
    use crate::metadata::InlineAnnotation;
    use crate::passes::composable::{Preserve, test::run_validating};
    use crate::passes::{ComposablePass, InlineDFGsPass, PassScope, WithScope};

    ///          /->-\
    /// main -> f     g -> b -> c
    ///        / \-<-/
    ///       /
    ///       \-> a -> x
    fn make_test_hugr() -> Hugr {
        let sig = || Signature::new_endo([qb_t()]);
        let mut mb = ModuleBuilder::new();
        let x = mb.declare("x", sig().into()).unwrap();
        let a = {
            let mut fb = mb.define_function("a", sig()).unwrap();
            let ins = fb.input_wires();
            let res = fb.call(&x, &[], ins).unwrap();
            fb.finish_with_outputs(res.outputs()).unwrap()
        };
        let c = {
            let fb = mb.define_function("c", sig()).unwrap();
            let ins = fb.input_wires();
            fb.finish_with_outputs(ins).unwrap()
        };
        let b = {
            let mut fb = mb.define_function("b", sig()).unwrap();
            let ins = fb.input_wires();
            let res = fb.call(c.handle(), &[], ins).unwrap().outputs();
            fb.finish_with_outputs(res).unwrap()
        };
        let f = mb.declare("f", sig().into()).unwrap();
        let g = {
            let mut fb = mb.define_function("g", sig()).unwrap();
            let ins = fb.input_wires();
            let c1 = fb.call(&f, &[], ins).unwrap();
            let c2 = fb.call(b.handle(), &[], c1.outputs()).unwrap();
            fb.finish_with_outputs(c2.outputs()).unwrap()
        };
        let _f = {
            let mut fb = mb.define_declaration(&f).unwrap();
            let ins = fb.input_wires();
            let c1 = fb.call(g.handle(), &[], ins).unwrap();
            let c2 = fb.call(a.handle(), &[], c1.outputs()).unwrap();
            fb.finish_with_outputs(c2.outputs()).unwrap()
        };
        mb.finish_hugr().unwrap()
    }

    fn find_func<H: HugrView>(h: &H, name: &str) -> H::Node {
        h.children(h.module_root())
            .find(|n| {
                h.get_optype(*n)
                    .as_func_defn()
                    .is_some_and(|fd| fd.func_name() == name)
            })
            .unwrap()
    }

    #[rstest]
    #[case(["a", "b", "c"], ["a", "b", "c"], [vec!["g", "x"], vec!["f"], vec!["x"], vec![], vec![]])]
    #[case(["a", "b"], ["a", "b"], [vec!["g", "x"], vec!["f", "c"], vec!["x"], vec!["c"], vec![]])]
    #[case(["c"], ["c"], [vec!["g", "a"], vec!("f", "b"), vec!["x"], vec![], vec![]])]
    fn test_inline(
        #[case] req: impl IntoIterator<Item = &'static str>,
        #[case] check_not_called: impl IntoIterator<Item = &'static str>,
        #[case] calls_fgabc: [Vec<&'static str>; 5],
    ) {
        let mut h = make_test_hugr();
        let target_funcs = req
            .into_iter()
            .map(|name| find_func(&h, name))
            .collect::<HashSet<_>>();
        inline_acyclic_scoped(
            &mut h,
            PassScope::Global(Preserve::Entrypoint),
            |h, call| {
                let tgt = h.static_source(call).unwrap();
                // Check the callback is never asked about an impossible inlining
                assert!(["a", "b", "c"].contains(&func_name(h, tgt).as_str()));
                target_funcs.contains(&tgt)
            },
        )
        .unwrap();
        let cg = ModuleGraph::new(&h);
        for fname in check_not_called {
            let fnode = find_func(&h, fname);
            let fnode = cg.node_index(fnode).unwrap();
            assert_eq!(
                None,
                cg.graph()
                    .edges_directed(fnode, petgraph::Direction::Incoming)
                    .next()
            );
        }
        for (fname, tgts) in ["f", "g", "a", "b", "c"].into_iter().zip_eq(calls_fgabc) {
            let fnode = find_func(&h, fname);
            assert_eq!(
                outgoing_calls(&cg, fnode)
                    .into_iter()
                    .map(|n| func_name(&h, n).as_str())
                    .collect::<HashSet<_>>(),
                HashSet::from_iter(tgts),
                "Calls from {fname}"
            );
        }
    }

    fn outgoing_calls<N: HugrNode>(cg: &ModuleGraph<N>, src: N) -> Vec<N> {
        cg.out_edges(src).map(|(_, tgt)| func_node(tgt)).collect()
    }

    #[test]
    fn test_filter_caller() {
        let mut h = make_test_hugr();
        let [g, b, c] = ["g", "b", "c"].map(|n| find_func(&h, n));
        // Inline calls contained within `g`
        inline_acyclic_scoped(
            &mut h,
            PassScope::Global(Preserve::Entrypoint),
            |h, mut call| {
                loop {
                    if call == g {
                        return true;
                    };
                    let Some(parent) = h.get_parent(call) else {
                        return false;
                    };
                    call = parent;
                }
            },
        )
        .unwrap();
        let cg = ModuleGraph::new(&h);
        // b and then c should have been inlined into g, leaving only cyclic call to f
        assert_eq!(outgoing_calls(&cg, g), [find_func(&h, "f")]);
        // But c should not have been inlined into b:
        assert_eq!(outgoing_calls(&cg, b), [c]);
    }

    fn func_node<N: Copy>(cgn: &StaticNode<N>) -> N {
        match cgn {
            StaticNode::FuncDecl(n) | StaticNode::FuncDefn(n) => *n,
            _ => panic!(),
        }
    }

    fn func_name<H: HugrView>(h: &H, n: H::Node) -> &String {
        match h.get_optype(n) {
            OpType::FuncDecl(fd) => fd.func_name(),
            OpType::FuncDefn(fd) => fd.func_name(),
            _ => panic!(),
        }
    }

    #[rstest]
    #[case::size_zero(InlineFuncsHeuristic::MaxSize(0), vec!["f", "b"])]
    #[case::size_unlimited(InlineFuncsHeuristic::MaxSize(usize::MAX), vec!["f"])]
    #[case::all(InlineFuncsHeuristic::All, vec!["f"])]
    fn inline_functions_pass_heuristic(
        #[case] heuristic: InlineFuncsHeuristic,
        #[case] g_targets: Vec<&'static str>,
    ) {
        let mut h = make_test_hugr();
        run_validating(
            InlineFunctionsPass::default().with_heuristic(heuristic),
            &mut h,
        )
        .unwrap();

        let cg = ModuleGraph::new(&h);
        let g = find_func(&h, "g");
        assert_eq!(
            outgoing_calls(&cg, g)
                .into_iter()
                .map(|n| func_name(&h, n).as_str())
                .collect::<HashSet<_>>(),
            HashSet::from_iter(g_targets),
        );
    }

    #[rstest]
    fn inline_functions_pass_hints() {
        let g_targets = vec!["f", "c"];

        let mut h = make_test_hugr();
        let b = find_func(&h, "b");
        let c = find_func(&h, "c");
        let f = find_func(&h, "f");
        // This should be inlined
        h.set_metadata::<InlineAnnotation>(b, InlineAnnotation::BestEffort);
        // This should never be inlined, even if `follow_hints` is false.
        h.set_metadata::<InlineAnnotation>(c, InlineAnnotation::Never);
        // This should be ignored, as `f` is in a double-recursive loop with `g`.
        h.set_metadata::<InlineAnnotation>(f, InlineAnnotation::BestEffort);

        run_validating(
            InlineFunctionsPass::default().with_heuristic(InlineFuncsHeuristic::MaxSize(0)),
            &mut h,
        )
        .unwrap();

        let cg = ModuleGraph::new(&h);
        let g = find_func(&h, "g");
        assert_eq!(
            outgoing_calls(&cg, g)
                .into_iter()
                .map(|n| func_name(&h, n).as_str())
                .collect::<HashSet<_>>(),
            HashSet::from_iter(g_targets),
        );
    }

    #[test]
    fn unreached_always_cycle() {
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
        let e = InlineFunctionsPass::default().run(&mut hugr).unwrap_err();
        assert_eq!(e, InlineFuncsError::AlwaysCycle(vec![fb.node()]));
        assert_eq!(hugr, backup);

        hugr.remove_subtree(fb.node());
        assert_eq!(
            hugr.children(hugr.module_root()).collect::<Vec<_>>(),
            [hugr.entrypoint()]
        );
        let backup = hugr.clone();
        InlineFunctionsPass::default().run(&mut hugr).unwrap();
        assert_eq!(hugr, backup);
    }

    #[test]
    fn always_cycle() {
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
        InlineFunctionsPass::default()
            .run(&mut hugr.clone())
            .unwrap(); // Ok

        hugr.set_metadata::<InlineAnnotation>(fb1.node(), InlineAnnotation::Always);
        let e = InlineFunctionsPass::default().run(&mut hugr).unwrap_err();
        assert_eq!(
            e,
            InlineFuncsError::AlwaysCycle(vec![fb1.node(), hugr.entrypoint()])
        );
    }

    #[rstest]
    fn test_always(#[values(1, 2, 5)] num_calls: usize) {
        use crate::passes::InlineDFGsPass;

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

        InlineFunctionsPass::default()
            .with_heuristic(InlineFuncsHeuristic::MaxSize(0))
            .run(&mut hugr)
            .unwrap();

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

    #[rstest]
    #[case(PassScope::EntrypointFlat)]
    #[case(PassScope::EntrypointRecursive)]
    #[case(Preserve::All)]
    #[case(Preserve::Public)]
    #[case(Preserve::Entrypoint)]
    fn always_entrypoint_scope(#[case] ps: impl Into<PassScope>) {
        let mut entry = FunctionBuilder::new("entry", Signature::new_endo([qb_t()])).unwrap();
        let mut mb = entry.module_root_builder();
        let mut cyclic = mb
            .define_function("cyclic", Signature::new_endo([qb_t()]))
            .unwrap();
        let c = cyclic
            .call::<true>(&cyclic.container_node().into(), &[], cyclic.input_wires())
            .unwrap();
        let cyclic = cyclic.finish_with_outputs(c.outputs()).unwrap();
        let mut other = mb
            .define_function("other", Signature::new_endo([qb_t()]))
            .unwrap();
        let c = other
            .call(cyclic.handle(), &[], other.input_wires())
            .unwrap();
        other.finish_with_outputs(c.outputs()).unwrap();

        let id = mb
            .define_function("id", Signature::new_endo([qb_t()]))
            .unwrap();
        let inps = id.input_wires();
        let id = id.finish_with_outputs(inps).unwrap();
        let c = entry
            .call::<true>(id.handle(), &[], entry.input_wires())
            .unwrap();
        let mut h = entry.finish_hugr_with_outputs(c.outputs()).unwrap();
        assert_eq!(h.static_targets(cyclic.node()).unwrap().count(), 2); // cyclic and entry
        h.set_metadata::<InlineAnnotation>(cyclic.node(), InlineAnnotation::Always);
        h.set_metadata::<InlineAnnotation>(id.node(), InlineAnnotation::Always);
        let ps = ps.into();
        let e = InlineFunctionsPass::default_with_scope(ps.clone()).run(&mut h);
        if let PassScope::EntrypointFlat | PassScope::EntrypointRecursive = ps {
            assert_eq!(e, Ok(()));
            assert_eq!(h.static_targets(cyclic.node()).unwrap().count(), 2); // cyclic and entry
            assert_eq!(h.static_targets(id.node()).unwrap().collect_vec(), []); // No calls, but can't be removed as outside scope
            InlineDFGsPass::default_with_scope(ps).run(&mut h).unwrap();
            let [inp, out] = h.get_io(h.entrypoint()).unwrap();
            assert_eq!(h.output_neighbours(inp).collect_vec(), [out]);
        } else {
            assert_eq!(e, Err(InlineFuncsError::AlwaysCycle(vec![cyclic.node()])));
        };
    }

    #[test]
    fn cycle_part_always() {
        let mut main = FunctionBuilder::new("main", Signature::new_endo([qb_t()])).unwrap();
        let main_h = main.container_node().into();
        let mut mb = main.module_root_builder();
        let mut f = mb
            .define_function("f", Signature::new_endo([qb_t()]))
            .unwrap();
        let hada = f.add_dataflow_op(TketOp::H, f.input_wires()).unwrap();
        let c = f.call::<true>(&main_h, &[], hada.outputs()).unwrap();
        let f = f.finish_with_outputs(c.outputs()).unwrap();
        let c = main.call(f.handle(), &[], main.input_wires()).unwrap();
        let backup = main.finish_hugr_with_outputs(c.outputs()).unwrap();

        // 1. Mark private callee f as Always, so it can be inlined into main and removed
        let mut hugr = backup.clone();
        hugr.set_metadata::<InlineAnnotation>(f.node(), InlineAnnotation::Always);
        hugr.set_metadata::<InlineAnnotation>(main_h.node(), InlineAnnotation::Never);
        InlineFunctionsPass::default() //.with_heuristic(InlineFuncsHeuristic::MaxSize(0))
            .run(&mut hugr)
            .unwrap();
        assert_eq!(
            hugr.children(hugr.module_root()).collect_vec(),
            [main_h.node()]
        );
        InlineDFGsPass::default().run(&mut hugr).unwrap();
        hugr.validate().unwrap();
        let [_in, _out, h, call] = hugr.children(main_h.node()).collect_array().unwrap();
        assert_eq!(
            hugr.get_optype(h).as_extension_op(),
            Some(&TketOp::H.into_extension_op())
        );
        assert!(hugr.get_optype(call).is_call());
        assert_eq!(hugr.static_source(call), Some(main_h.node()));

        // 2. Mark entrypoint function main as Always; it can be inlined into f, but not removed.
        let mut hugr = backup.clone();
        hugr.set_metadata::<InlineAnnotation>(main_h.node(), InlineAnnotation::Always);
        InlineFunctionsPass::default().run(&mut hugr).unwrap();
        // No functions can be removed
        assert_eq!(
            hugr.children(hugr.module_root()).collect_vec(),
            backup.children(backup.module_root()).collect_vec()
        );
        for n in hugr.children(hugr.module_root()) {
            assert_eq!(hugr.get_optype(n), backup.get_optype(n));
        }

        // main inlined into f (inside DFG):
        let [_in, _out, h, dfg] = hugr.children(f.node()).collect_array().unwrap();
        assert_eq!(
            hugr.get_optype(h).as_extension_op(),
            Some(&TketOp::H.into_extension_op())
        );
        assert!(hugr.get_optype(dfg).is_dfg());
        let [_in, _out, call] = hugr.children(dfg).collect_array().unwrap();
        assert!(hugr.get_optype(call).is_call());
        assert_eq!(hugr.static_source(call), Some(f.node()));

        // No calls to main:
        assert_eq!(hugr.static_targets(main_h.node()).unwrap().next(), None);
    }
}
