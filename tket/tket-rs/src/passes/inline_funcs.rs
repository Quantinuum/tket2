//! Contains a pass to inline calls to selected functions in a Hugr.
use std::collections::{HashMap, HashSet, VecDeque};

use hugr::HugrView;
use itertools::Itertools;
use petgraph::algo::tarjan_scc;

use hugr_core::hugr::{hugrmut::HugrMut, patch::inline_call::InlineCall};
use hugr_core::module_graph::{ModuleGraph, StaticNode};

use crate::metadata::InlineAnnotation;
use crate::passes::{ComposablePass, PassScope, WithScope};

/// Error raised by [InlineFunctionsPass]
#[derive(Clone, Debug, thiserror::Error, PartialEq)]
#[non_exhaustive]
pub enum InlineFuncsError {}

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
    fn should_inline<H: HugrView>(
        &self,
        func: H::Node,
        hugr: &H,
        size_cache: &mut HashMap<H::Node, usize>,
    ) -> bool {
        match self {
            InlineFuncsHeuristic::MaxSize(size) => {
                estimated_inline_size(func, hugr, size_cache, *size) <= *size
            }
            InlineFuncsHeuristic::All => true,
        }
    }
}

/// Estimate the expanded size of inlining a function under a max-size heuristic.
///
/// The direct descendant count alone can underestimate wrappers that call other
/// inlineable helpers. Counting reachable static callees keeps those wrappers
/// from repeatedly expanding medium-sized helper graphs before later cleanup
/// passes get a chance to simplify them.
fn estimated_inline_size<H: HugrView>(
    func: H::Node,
    hugr: &H,
    cache: &mut HashMap<H::Node, usize>,
    limit: usize,
) -> usize {
    estimated_inline_size_inner(func, hugr, cache, &mut HashSet::new(), limit)
}

/// Recursive worker for [`estimated_inline_size`].
///
/// Arguments:
/// - `cache` memoizes completed function estimates across calls in the pass,
/// - `visiting` tracks the current recursion stack so cyclic call graphs are
///   treated as over budget.
/// - `limit` is only used to cut the estimation early if we already exceeded
///   the max size.
fn estimated_inline_size_inner<H: HugrView>(
    func: H::Node,
    hugr: &H,
    cache: &mut HashMap<H::Node, usize>,
    visiting: &mut HashSet<H::Node>,
    limit: usize,
) -> usize {
    if let Some(size) = cache.get(&func) {
        return *size;
    }
    if !visiting.insert(func) {
        return limit.saturating_add(1);
    }

    let mut size = hugr.descendants(func).count();
    for call in hugr
        .descendants(func)
        .filter(|node| hugr.get_optype(*node).is_call())
    {
        if let Some(callee) = hugr.static_source(call) {
            size = size.saturating_add(estimated_inline_size_inner(
                callee, hugr, cache, visiting, limit,
            ));
            if size > limit {
                break;
            }
        }
    }
    visiting.remove(&func);
    cache.insert(func, size);
    size
}

impl Default for InlineFuncsHeuristic {
    fn default() -> Self {
        Self::MaxSize(128)
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
    type Error = InlineFuncsError;
    type Result = ();

    fn run(&self, h: &mut H) -> Result<(), Self::Error> {
        let mut should_inline_cache: HashMap<H::Node, bool> = HashMap::new();
        let mut size_cache: HashMap<H::Node, usize> = HashMap::new();
        inline_acyclic_scoped(h, self.scope.clone(), |h, call| {
            let Some(func) = h.static_source(call) else {
                return false;
            };
            *should_inline_cache.entry(func).or_insert_with(|| {
                match h.get_metadata::<InlineAnnotation>(func) {
                    Some(InlineAnnotation::Never) => false,
                    Some(InlineAnnotation::BestEffort) => true,
                    None => self.heuristic.should_inline(func, h, &mut size_cache),
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
) -> Result<(), InlineFuncsError> {
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

#[cfg(test)]
mod test {
    use std::collections::{HashMap, HashSet};

    use itertools::Itertools;
    use rstest::rstest;

    use hugr_core::HugrView;
    use hugr_core::builder::{Dataflow, DataflowSubContainer, HugrBuilder, ModuleBuilder};
    use hugr_core::core::HugrNode;
    use hugr_core::hugr::hugrmut::HugrMut;
    use hugr_core::module_graph::{ModuleGraph, StaticNode};
    use hugr_core::ops::OpType;
    use hugr_core::{Hugr, extension::prelude::qb_t, types::Signature};

    use super::{InlineFunctionsPass, estimated_inline_size, inline_acyclic_scoped};
    use crate::metadata::InlineAnnotation;
    use crate::passes::composable::test::run_validating;
    use crate::passes::inline_funcs::InlineFuncsHeuristic;
    use crate::passes::{PassScope, composable::Preserve};

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

    #[test]
    fn max_size_heuristic_counts_transitive_callees() {
        let h = make_test_hugr();
        let b = find_func(&h, "b");
        let direct_size = h.descendants(b).count();
        let expanded_size = estimated_inline_size(b, &h, &mut HashMap::new(), usize::MAX);

        assert!(expanded_size > direct_size);
        assert!(!InlineFuncsHeuristic::MaxSize(direct_size).should_inline(
            b,
            &h,
            &mut HashMap::new()
        ));
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
}
