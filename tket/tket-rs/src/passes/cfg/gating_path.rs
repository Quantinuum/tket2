//! Analysis of Control-Flow Graphs using dominator-strong components decomposition
use itertools::Itertools;
use std::collections::HashMap;

use hugr::core::HugrNode;
use hugr::{OutgoingPort, PortIndex as _};

use super::dom_tree::Path;

/// A collection (0 or more) paths from some given CFG node, within that node's dominator tree.
/// `None` (rather than a `GatingPath`) represents the empty collection (no paths).
#[derive(Clone, Debug)]
pub enum GatingPath<N> {
    /// Source of an edge leaving the dominator tree
    Always(N, OutgoingPort),
    /// One element (None if no path via that port) for each outgoing port of the branch node
    Branch(N, Vec<Option<GatingPath<N>>>),
}

impl<N: HugrNode> Path<N> for GatingPath<N> {
    type OptPath = Option<GatingPath<N>>;

    fn opt_as_ref(opt: &Self::OptPath) -> Option<&Self> {
        opt.as_ref()
    }
    /// Merges `other` into the accumulated path `acc`, which may not have any path yet.
    fn union_opt(acc: &mut Option<GatingPath<N>>, other: &GatingPath<N>) {
        match acc {
            Some(existing) => existing.union(other),
            None => *acc = Some(other.clone()),
        }
    }

    fn new_edge(node: N, outport: OutgoingPort) -> Self {
        GatingPath::Always(node, outport)
    }

    fn new_branch(node: N, num_ports: usize, p: OutgoingPort, sub_path: GatingPath<N>) -> Self {
        let mut branches = vec![None; num_ports];
        branches[p.index()] = Some(sub_path);
        GatingPath::Branch(node, branches)
    }

    fn concat(&self, other: &GatingPath<N>) -> Self {
        fn has_none<N>(gp: &Option<GatingPath<N>>) -> bool {
            match gp {
                None => true,
                Some(GatingPath::Always(_, _)) => false,
                Some(GatingPath::Branch(_, opts)) => opts.iter().any(has_none),
            }
        }
        match self {
            GatingPath::Always(_, _) => other.clone(),
            GatingPath::Branch(node, opts) => {
                if opts.iter().any(has_none) {
                    GatingPath::Branch(
                        *node,
                        opts.iter()
                            .map(|v| v.as_ref().map(|v| v.concat(other)))
                            .collect(),
                    )
                } else {
                    other.clone()
                }
            }
        }
    }

    /// Splits this path into one path per distinct target reachable from its leaves.
    fn leaves(&self) -> impl Iterator<Item = ((N, OutgoingPort), Self)> {
        match self {
            GatingPath::Always(node, port) => HashMap::from([((*node, *port), self.clone())]),
            GatingPath::Branch(node, opts) => opts
                .iter()
                .enumerate()
                .filter_map(|(i, o)| Some((i, o.as_ref()?)))
                .flat_map(|(i, ch)| {
                    ch.leaves().map(move |(np, sub_path)| {
                        (
                            np,
                            GatingPath::new_branch(*node, opts.len(), i.into(), sub_path),
                        )
                    })
                })
                .into_grouping_map()
                .reduce(|mut p, _np, p2| {
                    p.union(&p2);
                    p
                }),
        }
        .into_iter()
    }

    fn union(&mut self, other: &GatingPath<N>) {
        match self {
            GatingPath::Always(_, _) => {
                panic!("Union of Always with {other:?}");
            }
            GatingPath::Branch(n, opts) => {
                if let GatingPath::Branch(n2, opts2) = other
                    && n == n2
                {
                    for (this, other) in opts.iter_mut().zip_eq(opts2.iter()) {
                        let Some(other) = other else { continue };
                        match this {
                            None => *this = Some(other.clone()),
                            Some(this) => this.union(other),
                        }
                    }
                } else {
                    panic!("Cannot union Branch({n:?}) with {other:?}");
                }
            }
        }
    }
}
