//! Elide pairs of return-borrow operations on a `BorrowArray` where possible.

use derive_more::{Display, Error};
use hugr::algorithms::ComposablePass;
use hugr::extension::prelude::ConstUsize;
use hugr::extension::simple_op::MakeExtensionOp;
use hugr::hugr::hugrmut::HugrMut;
use hugr::ops::{OpTag, OpTrait, Value};
use hugr::std_extensions::arithmetic::int_types::ConstInt;
use hugr::std_extensions::collections::borrow_array::{BArrayUnsafeOpDef, BORROW_ARRAY_TYPENAME};
use hugr::types::{EdgeKind, Type};
use hugr::{HugrView, IncomingPort, Node, OutgoingPort, Wire};

use std::collections::hash_map::Entry;
use std::collections::{HashMap, HashSet, VecDeque};

/// A pass for eliding pairs of return-borrow operations on [BorrowArray]s
/// where they have identical constant indices and there is a preceding borrow (not elided)
/// that guarantees the elided ops would not have panicked.
///
/// [BorrowArray]: hugr::std_extensions::collections::borrow_array::BorrowArray
#[derive(Clone, Debug, Default)]
pub struct BorrowSquashPass {
    regions: Option<Vec<Node>>,
}

impl BorrowSquashPass {
    /// Sets the regions (subgraphs) in which to perform the pass.
    ///
    /// Overrides effect of any previous call; if `regions` is empty, the pass will do nothing.
    ///
    /// If `with_regions` is not called, the default is for the pass to act on all dataflow regions
    /// beneath the entrypoint.
    pub fn set_regions(mut self, regions: impl IntoIterator<Item = Node>) -> Self {
        self.regions = Some(regions.into_iter().collect());
        self
    }
}

impl<H: HugrMut<Node = Node>> ComposablePass<H> for BorrowSquashPass {
    type Error = BorrowSquashError;
    /// Pairs of (Return node, Borrow node) that were elided.
    type Result = Vec<(Node, Node)>;

    /// Perform the pass on the given hugr.
    ///
    /// Note it is recommended to run [ConstantFoldPass] first to make as many indices
    /// constant as possible.
    ///
    /// [ConstantFoldPass]: hugr::algorithms::const_fold::ConstantFoldPass
    fn run(&self, hugr: &mut H) -> Result<Vec<(Node, Node)>, BorrowSquashError> {
        let mut temp = Vec::new(); // to keep alive
        let regions = self.regions.as_ref().unwrap_or_else(|| {
            temp.extend(
                hugr.entry_descendants()
                    .filter(|n| OpTag::DataflowParent.is_superset(hugr.get_optype(*n).tag())),
            );
            &temp
        });
        let mut results = Vec::new();
        for region in regions {
            let mut seen = HashSet::new();
            // Start with all nodes not reachable along dataflow edges from other nodes. (Includes Input.)
            let mut queue = VecDeque::from_iter(
                hugr.children(*region)
                    .filter(|n| hugr.in_value_types(*n).next().is_none())
                    .flat_map(|n| all_outs(hugr, n)),
            );

            while let Some(start) = queue.pop_front() {
                if !seen.insert(start) {
                    continue;
                }
                let elided = borrow_squash_traversal(hugr, &mut queue, start, true);
                results.extend(elided);
            }
        }
        Ok(results)
    }
}

/// Reasons a [BorrowSquashPass] may fail
#[derive(Clone, Debug, Display, Error, PartialEq)]
#[non_exhaustive]
pub enum BorrowSquashError {}

/// The ports by which the container array reaches and leaves a
/// particular borrow or return node.
#[derive(Clone, Debug)]
struct BorrowFromPorts {
    /// The port receiving the array before the borrow/return.
    inc: IncomingPort,
    /// The port returning the array after the borrow/return.
    out: OutgoingPort,
}

/// Whether a node is a borrow or return, along with the port for
/// the borrowed value.
#[derive(Debug, Clone, PartialEq, Eq)]
enum BRAction {
    /// A borrow action, containing the port on which the borrowed value is output.
    Borrow(OutgoingPort),
    /// A return action, containing the port on which the value to return is input.
    Return(IncomingPort),
}

/// Ports common to a borrow or return op
#[derive(Debug, Clone)]
struct BorrowReturnPorts {
    /// Whether this is a borrow or return, and the port for the borrowed value.
    action: BRAction,
    /// Port on which the index (of the element to borrow or return) is passed in
    elem_index: IncomingPort,
    /// The ports by which the container array reaches and leaves this node.
    borrow_from: BorrowFromPorts,
}

/// Determine if the given node is a borrow or return node, and if so, return
/// the ports identifying the operands.
///
/// If not, return `Ok(None)` - this will prevent elision for any Borrow-Return-Borrow
/// spanning this node.
fn is_borrow_return<H: HugrView>(node: H::Node, hugr: &H) -> Option<BorrowReturnPorts> {
    let op = hugr.get_optype(node);

    let ext_op = op.as_extension_op()?;

    match BArrayUnsafeOpDef::from_extension_op(ext_op) {
        Ok(BArrayUnsafeOpDef::borrow) => {
            let sig = op.dataflow_signature().unwrap();
            let counts = (sig.input_count(), sig.output_count());
            assert_eq!(counts, (2, 2), "Borrow node has incorrect signature");
            Some(BorrowReturnPorts {
                action: BRAction::Borrow(OutgoingPort::from(1)),
                borrow_from: BorrowFromPorts {
                    inc: IncomingPort::from(0),
                    out: OutgoingPort::from(0),
                },
                elem_index: IncomingPort::from(1),
            })
        }
        Ok(BArrayUnsafeOpDef::r#return) => {
            let sig = op.dataflow_signature().unwrap();
            let counts = (sig.input_count(), sig.output_count());
            assert_eq!(counts, (3, 1), "Return node has incorrect signature");
            Some(BorrowReturnPorts {
                action: BRAction::Return(IncomingPort::from(2)),
                borrow_from: BorrowFromPorts {
                    inc: IncomingPort::from(0),
                    out: OutgoingPort::from(0),
                },
                elem_index: IncomingPort::from(1),
            })
        }
        _ => None,
    }
}

/// Tells whether the specified type is an array that can be borrowed from.
fn is_borrow_array(ty: &Type) -> bool {
    ty.as_extension()
        .is_some_and(|ext| ext.name() == &BORROW_ARRAY_TYPENAME)
}

/// Elide return-borrow pairs for a single array.
///
/// If `recurse` is true, then also elide return-borrow pairs for sub-arrays
/// (i.e. if this is a nested array).
///
/// # Panics
///
/// if `start` does not exist in the Hugr
///
/// # Returns
///
/// * Pairs of (Return node, Borrow node) that were elided.
pub fn borrow_squash_array<H: HugrMut<Node = Node>>(
    hugr: &mut H,
    start: Wire,
    recurse: bool,
) -> Vec<(Node, Node)> {
    borrow_squash_traversal(hugr, &mut Vec::new(), start, recurse)
}

/// Internal method to keep traversal private.
/// Like [borrow_squash_array] but also pushes new Wires that may create arrays onto `candidates`.
fn borrow_squash_traversal<H: HugrMut<Node = Node>>(
    hugr: &mut H,
    candidates: &mut impl Extend<Wire>,
    start: Wire,
    recurse: bool,
) -> Vec<(Node, Node)> {
    let array_ty = wire_type(hugr, start);
    if !is_borrow_array(&array_ty) {
        for (n, _) in hugr.linked_inputs(start.node(), start.source()) {
            // Traverse successors until we find an array-creating op. Borrows/Returns
            // will be traversed when reached along their array input.
            if is_borrow_return(n, hugr).is_none() {
                candidates.extend(all_outs(hugr, n));
            }
        }
        return vec![];
    };

    struct Borrow(Node, OutgoingPort, BorrowFromPorts);
    struct Return(Node, IncomingPort, BorrowFromPorts);

    let mut borrowed: HashMap<u64, (Borrow, Option<Return>)> = HashMap::new(); // Key is elem index
    let mut rb_elisions: Vec<(Return, Borrow)> = Vec::new();

    let mut array = start;
    while let Some((node, index, action, borrow_from)) =
        next_array_op(hugr, candidates, array, &array_ty)
    {
        array = Wire::new(node, borrow_from.out); // for next iteration

        match (action, borrowed.entry(index)) {
            (BRAction::Borrow(borrowed_out), Entry::Vacant(ve)) => {
                // initial borrow - record
                ve.insert((Borrow(node, borrowed_out, borrow_from), None));
            }

            (BRAction::Return(inc), Entry::Occupied(mut oe)) => {
                // return after borrow - record
                let (_, ret) = &mut oe.get_mut();
                if ret.replace(Return(node, inc, borrow_from)).is_some() {
                    oe.remove_entry(); // double return - this will panic, don't bother optimizing
                }
            }

            (BRAction::Borrow(borrowed_out), Entry::Occupied(mut oe)) => {
                // Borrow after return - record both to be removed
                let (_, prev_return) = oe.get_mut();
                match prev_return.take() {
                    Some(prev_return) => {
                        rb_elisions.push((prev_return, Borrow(node, borrowed_out, borrow_from)));
                    }
                    None => {
                        oe.remove_entry(); // double borrow - this will panic, don't bother optimizing
                    }
                };
            }

            (BRAction::Return(_), Entry::Vacant(_)) => (), // return without borrow - will panic, don't optimize
        }
    }

    fn elide_node<H: HugrMut<Node = Node>>(hugr: &mut H, n: Node, ports: &BorrowFromPorts) {
        let in_array = hugr
            .single_linked_output(n, ports.inc)
            .expect("array is linear");
        let out_array = hugr
            .single_linked_input(n, ports.out)
            .expect("array is linear");
        hugr.connect(in_array.0, in_array.1, out_array.0, out_array.1);
        hugr.remove_node(n);
    }
    let mut elided: Vec<_> = rb_elisions
        .into_iter()
        .map(|(ret, bor)| {
            // Pass Return-ed value directly through to users of Borrow-ed value
            let src = hugr.single_linked_output(ret.0, ret.1).expect("input");
            for tgt in hugr.linked_inputs(bor.0, bor.1).collect::<Vec<_>>() {
                hugr.connect(src.0, src.1, tgt.0, tgt.1);
            }
            elide_node(hugr, bor.0, &bor.2);
            elide_node(hugr, ret.0, &ret.2);
            (ret.0, bor.0)
        })
        .collect();

    if recurse {
        // Elide any intervening borrows/returns on nested arrays
        for (bor, _) in borrowed.into_values() {
            let start = Wire::new(bor.0, bor.1);
            let new_elided = borrow_squash_traversal(hugr, candidates, start, true);
            elided.extend(new_elided);
            // TODO: it would be good to elide borrow-return (of the same value), but we would need to know
            // the index was not already borrowed, e.g. had just been returned before the borrow.
        }
    }

    elided
}

fn next_array_op(
    hugr: &impl HugrView<Node = Node>,
    candidates: &mut impl Extend<Wire>,
    array: Wire,
    array_ty: &Type,
) -> Option<(Node, u64, BRAction, BorrowFromPorts)> {
    let (node, inport) = hugr
        .single_linked_input(array.node(), array.source())
        .expect("array is linear");
    let Some(is_br) = is_borrow_return(node, hugr) else {
        // Not a borrow/return - stop processing this array;
        // any outports of the node are considered fresh arrays.
        candidates.extend(all_outs(hugr, node));
        return None;
    };

    if inport != is_br.borrow_from.inc {
        if let BRAction::Borrow(_) = is_br.action {
            panic!("Array fed into unexpected port of borrow")
        }
        // Op will be reachable along the incoming-array port, so some other call
        // (along that path) will process it.
        if let BRAction::Return(rv) = is_br.action {
            assert_eq!(rv, inport); // Nested array being returned into outer array.
        }
        return None;
    }

    assert_eq!(
        *array_ty,
        wire_type(hugr, Wire::new(node, is_br.borrow_from.out))
    );

    let Some(idx) = find_const(hugr, node, is_br.elem_index) else {
        // Unknown index.
        // Hence, we must preserve borrowedness of all indices for this op;
        // so we can't elide anything before this op with anything after.
        // Hence, op can be considered as beginning of fresh array(s) on each outport.
        candidates.extend(all_outs(hugr, node));
        return None;
    };
    Some((node, idx, is_br.action, is_br.borrow_from))
}

fn wire_type(h: &impl HugrView<Node = Node>, w: Wire) -> Type {
    let Some(EdgeKind::Value(ty)) = h.get_optype(w.node()).port_kind(w.source()) else {
        panic!("Invalid wire {w}")
    };
    ty.clone()
}

fn all_outs(h: &impl HugrView<Node = Node>, n: Node) -> impl Iterator<Item = Wire> + '_ {
    h.out_value_types(n).map(move |(p, _)| Wire::new(n, p))
}

fn find_const<H: HugrView>(hugr: &H, n: H::Node, inp: IncomingPort) -> Option<u64> {
    let (load_const, _) = hugr.single_linked_output(n, inp).expect("dataflow input");
    if !hugr.get_optype(load_const).is_load_constant() {
        return None;
    }

    let const_op = hugr
        .single_linked_output(load_const, 0)
        .and_then(|(n, _)| hugr.get_optype(n).as_const())
        .expect("LoadConstant input is constant");

    if let Value::Extension { e } = &const_op.value {
        if let Some(c) = e.value().downcast_ref::<ConstUsize>() {
            return Some(c.value());
        }
        if let Some(c) = e.value().downcast_ref::<ConstInt>() {
            return Some(c.value_u());
        }
    }
    panic!("Unexpected index {:?}", const_op.value)
}

#[cfg(test)]
mod test {
    use std::{collections::BTreeSet, io::BufReader};

    use super::{BorrowSquashPass, find_const};
    use crate::extension::REGISTRY;
    use hugr::{
        Hugr, HugrView,
        algorithms::{ComposablePass, const_fold::ConstantFoldPass},
        builder::{DFGBuilder, Dataflow, DataflowHugr, FunctionBuilder, endo_sig},
        extension::{
            prelude::{ConstUsize, qb_t, usize_t},
            simple_op::MakeExtensionOp,
        },
        hugr::hugrmut::HugrMut,
        ops::{OpTrait, handle::NodeHandle},
        std_extensions::collections::{
            array::ArrayKind,
            borrow_array::{BArrayOpBuilder, BArrayUnsafeOpDef, BorrowArray},
        },
        types::Signature,
    };
    use itertools::Itertools;
    use rstest::{fixture, rstest};

    #[rstest]
    fn simple() {
        let mut dfb = DFGBuilder::new(endo_sig(BorrowArray::ty(3, qb_t()))).unwrap();
        let [arr] = dfb.input_wires_arr();
        let idx = dfb.add_load_value(ConstUsize::new(1));
        let (arr, q) = dfb.add_borrow_array_borrow(qb_t(), 3, arr, idx).unwrap();
        let arr2 = dfb.add_borrow_array_return(qb_t(), 3, arr, idx, q).unwrap();
        let (arr3, q2) = dfb.add_borrow_array_borrow(qb_t(), 3, arr2, idx).unwrap();
        let arr4 = dfb
            .add_borrow_array_return(qb_t(), 3, arr3, idx, q2)
            .unwrap();
        let mut h = dfb.finish_hugr_with_outputs([arr4]).unwrap();
        h.validate().unwrap();

        // Hand-construct expected hugr after elision
        let mut h2 = h.clone();
        h2.remove_node(arr2.node());
        h2.remove_node(arr3.node());
        h2.connect(q.node(), q.source(), arr4.node(), 2);
        h2.connect(q.node(), 0, arr4.node(), 0);
        h2.validate().unwrap();

        let r = BorrowSquashPass::default().run(&mut h).unwrap();
        assert_eq!(r, vec![(arr2.node(), arr3.node())]);

        if h != h2 {
            // Hugr equality very under-approximates
            assert_eq!(h.nodes().collect_vec(), h2.nodes().collect_vec());
            for n in h.nodes() {
                assert_eq!(h.get_parent(n), h2.get_parent(n));
                assert_eq!(h.get_optype(n), h2.get_optype(n));
                for p in h.all_node_ports(n) {
                    let ins_h = h.linked_ports(n, p).collect_vec();
                    let ins_h2 = h2.linked_ports(n, p).collect_vec();
                    assert_eq!(ins_h, ins_h2);
                }
            }
        }
    }

    #[fixture]
    fn ranges_array() -> Hugr {
        let reader = BufReader::new(
            include_bytes!("../../../test_files/guppy_optimization/ranges/ranges.flat.array.hugr")
                .as_slice(),
        );
        Hugr::load(reader, Some(&REGISTRY)).unwrap()
    }

    #[rstest]
    fn test_borrow_squash(ranges_array: Hugr) {
        let counts = |h: &Hugr| {
            let mut brs = vec![(0, 0); 4];
            for n in find_borrows(h) {
                brs[get_index(h, n) as usize].0 += 1;
            }
            for n in find_returns(h) {
                brs[get_index(h, n) as usize].1 += 1;
            }
            brs
        };

        let f = ranges_array
            .children(ranges_array.module_root())
            .find(|n| {
                ranges_array
                    .get_optype(*n)
                    .as_func_defn()
                    .is_some_and(|fd| fd.func_name() == "f")
            })
            .unwrap();

        let mut h = ranges_array;
        h.set_entrypoint(f);

        ConstantFoldPass::default().run(&mut h).unwrap();
        assert_eq!(counts(&h), vec![(4, 4), (6, 6), (6, 6), (4, 4)]);
        let orig_num_nodes = h.num_nodes();
        let res = BorrowSquashPass::default().run(&mut h).unwrap();
        h.validate().unwrap();
        let expected_elisions = 16;
        assert_eq!(res.len(), expected_elisions);
        assert_eq!(h.num_nodes(), orig_num_nodes - 2 * expected_elisions);
        assert_eq!(counts(&h), vec![(1, 1); 4]);
    }

    fn find_borrows<H: HugrView>(h: &H) -> impl Iterator<Item = H::Node> + '_ {
        h.entry_descendants().filter(|n| {
            h.get_optype(*n).as_extension_op().is_some_and(|eop| {
                BArrayUnsafeOpDef::from_extension_op(eop) == Ok(BArrayUnsafeOpDef::borrow)
            })
        })
    }

    fn find_returns<H: HugrView>(h: &H) -> impl Iterator<Item = H::Node> + '_ {
        h.entry_descendants().filter(|n| {
            h.get_optype(*n).as_extension_op().is_some_and(|eop| {
                BArrayUnsafeOpDef::from_extension_op(eop) == Ok(BArrayUnsafeOpDef::r#return)
            })
        })
    }

    fn get_index<H: HugrView>(h: &H, n: H::Node) -> u64 {
        find_const(h, n, 1.into()).unwrap()
    }

    #[rstest]
    fn test_nested_array() {
        let inner_array_type = BorrowArray::ty(5, qb_t());
        let outer_array_type = BorrowArray::ty(3, inner_array_type.clone());
        let reader = BufReader::new(
            include_bytes!("../../../test_files/guppy_optimization/nested_array/nested_array.hugr")
                .as_slice(),
        );
        let mut h = Hugr::load(reader, Some(&REGISTRY)).unwrap();
        let array_func = h
            .children(h.module_root())
            .find(|n| {
                h.get_optype(*n)
                    .as_func_defn()
                    .is_some_and(|fd| fd.func_name() == "main")
            })
            .unwrap();
        h.set_entrypoint(array_func);
        ConstantFoldPass::default().run(&mut h).unwrap();
        // Sanity checks: all borrows are qs[0][1 or 2]
        for nodes in [find_borrows(&h).collect_vec(), find_returns(&h).collect()] {
            let mut outer_count = 0;
            for node in nodes {
                let expected_array_type = match get_index(&h, node) {
                    0 => {
                        outer_count += 1;
                        &outer_array_type
                    }
                    1 | 2 => &inner_array_type,
                    idx => panic!("Unexpected index {idx}"),
                };
                assert_eq!(
                    h.get_optype(node)
                        .dataflow_signature()
                        .unwrap()
                        .input_types()[0],
                    *expected_array_type
                );
            }
            // For each CX, two borrows or two returns of the outer array before the op, and two after
            assert_eq!(outer_count, 8);
        }
        // Two CXs, both borrowing their inputs and returning their outputs:
        let [cx1, cx2] = h
            .nodes()
            .filter(|n| {
                h.get_optype(*n)
                    .as_extension_op()
                    .is_some_and(|eop| eop.qualified_id() == "tket.quantum.CX")
            })
            .collect_array()
            .unwrap();
        for cx in [cx1, cx2] {
            assert!(
                BTreeSet::from_iter(find_returns(&h))
                    .is_superset(&h.output_neighbours(cx).collect())
            );
            assert!(
                BTreeSet::from_iter(find_borrows(&h))
                    .is_superset(&h.input_neighbours(cx).collect())
            );
        }

        let res = BorrowSquashPass::default().run(&mut h).unwrap();
        h.validate().unwrap();
        assert_eq!(res.len(), 9);
        // Now, one borrow and one return from the outer array
        assert_eq!(
            find_borrows(&h)
                .map(|n| get_index(&h, n))
                .sorted()
                .collect_vec(),
            [0, 1, 2]
        );
        assert_eq!(
            find_returns(&h)
                .map(|n| get_index(&h, n))
                .sorted()
                .collect_vec(),
            [0, 1, 2]
        );
        // CX's should still be there (in same place), but now directly connected:
        for cx in [cx1, cx2] {
            assert!(
                h.get_optype(cx)
                    .as_extension_op()
                    .is_some_and(|eop| eop.qualified_id() == "tket.quantum.CX")
            );
        }
        assert!(h.output_neighbours(cx1).all(|n| n == cx2));
    }

    /// Captures (somewhere) dynamic borrow, return or both together
    ///   coming at any position before/during/after borrow-return-reborrow
    #[rstest]
    fn test_dynamic(#[values(0, 1, 2)] dyn_pos: usize) {
        // element 1:               Borrow Return Reborrow
        // dyn_pos==0:  dyn Borrow                          dyn Return
        // element 2:                        Borrow Return                          Reborrow
        // dyn_pos==1:                                      dyn Borrow                                      dyn Return
        // element 3:                               Borrow                          Return Reborrow
        // dyn_pos==2:                                      dyn Borrow, dyn Return
        // element 4:                                                               Borrow Return Reborrow
        let ins = vec![usize_t(), BorrowArray::ty(10, usize_t())];
        let mut outs = vec![usize_t(); 3];
        outs.extend(ins.clone());
        let mut fb = FunctionBuilder::new("test", Signature::new(ins, outs)).unwrap();
        let [i, arr] = fb.input_wires_arr();
        let borrow = |fb: &mut FunctionBuilder<Hugr>, arr, i| {
            let op = BArrayUnsafeOpDef::borrow.to_concrete(usize_t(), 10);
            let h = fb.add_dataflow_op(op, [arr, i]).unwrap();
            (h.node(), h.outputs_arr::<2>())
        };
        let return_ = |fb: &mut FunctionBuilder<Hugr>, arr, i, val| {
            let op = BArrayUnsafeOpDef::r#return.to_concrete(usize_t(), 10);
            let h = fb.add_dataflow_op(op, [arr, i, val]).unwrap();
            (h.node(), h.outputs_arr::<1>()[0])
        };
        let [one, two, three, four] = [1, 2, 3, 4].map(|i| fb.add_load_value(ConstUsize::new(i)));
        let (arr, xi) = if dyn_pos == 0 {
            // Borrow dynamic index first
            let (_, [arr, xi]) = borrow(&mut fb, arr, i);
            (arr, Some(xi))
        } else {
            (arr, None)
        };
        // Element 1: all three operations together (either before any dynamic indexing, or between dynamic borrow and return)
        let (_, [arr, x1]) = borrow(&mut fb, arr, one);
        let (ret1, arr) = return_(&mut fb, arr, one, x1);
        let (rebo1, [arr, x1]) = borrow(&mut fb, arr, one);
        // Element 2: initial borrow and return together, final reborrow later
        let (_, [arr, x2]) = borrow(&mut fb, arr, two);
        let (_, arr) = return_(&mut fb, arr, two, x2);
        // Element 3: initial borrow first, return + reborrow together later
        let (_, [arr, x3]) = borrow(&mut fb, arr, three);
        // Element 4: all three operations together later

        // *Some* dynamic indexing ops midway through index 2 and index 3
        let (arr, xi) = if dyn_pos == 0 {
            let (_, arr) = return_(&mut fb, arr, i, xi.unwrap());
            (arr, None)
        } else {
            assert!(xi.is_none());
            let (_, [arr, xi]) = borrow(&mut fb, arr, i);
            if dyn_pos == 1 {
                (arr, Some(xi))
            } else {
                let (_, arr) = return_(&mut fb, arr, i, xi);
                (arr, None)
            }
        };

        // Element 1 finished; final borrow of element 2, return+reborrow of element 3
        let (_, [arr, x2]) = borrow(&mut fb, arr, two);
        let (_, arr) = return_(&mut fb, arr, three, x3);
        let (_, [arr, x3]) = borrow(&mut fb, arr, three);
        // Element 4: all three operations together (either after all dynamic indexing, or between dynamic borrow and return)
        let (_, [arr, x4]) = borrow(&mut fb, arr, four);
        let (ret4, arr) = return_(&mut fb, arr, four, x4);
        let (rebo4, [arr, x4]) = borrow(&mut fb, arr, four);

        // Possibly final dynamic return
        let arr = xi.map_or(arr, |xi| return_(&mut fb, arr, i, xi).1);
        let mut h = fb.finish_hugr_with_outputs([x1, x2, x3, x4, arr]).unwrap();

        let res = BorrowSquashPass::default().run(&mut h).unwrap();
        assert_eq!(res, [(ret1, rebo1), (ret4, rebo4)]);
        h.validate().unwrap();
    }
}
