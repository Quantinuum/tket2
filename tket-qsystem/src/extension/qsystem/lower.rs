use derive_more::{Display, Error, From};
use hugr::algorithms::replace_types::{NodeTemplate, ReplaceTypesError};
use hugr::algorithms::{ComposablePass, ReplaceTypes};
use hugr::extension::prelude::Barrier;
use hugr::extension::simple_op::MakeExtensionOp;
use hugr::hugr::patch::insert_cut::InsertCutError;
use hugr::{
    Hugr, HugrView, Node, Wire,
    builder::{BuildError, Dataflow, DataflowHugr, FunctionBuilder},
    extension::ExtensionRegistry,
    hugr::{HugrError, hugrmut::HugrMut},
    ops::{self, DataflowOpTrait},
    std_extensions::arithmetic::{float_ops::FloatOps, float_types::ConstF64},
    types::Signature,
};
use lazy_static::lazy_static;
use std::collections::BTreeMap;
use std::collections::btree_map::Entry;
use tket::{TketOp, extension::rotation::RotationOpBuilder};

use crate::extension::qsystem::{self, QSystemOp, QSystemOpBuilder};

use super::barrier::BarrierInserter;

lazy_static! {
    /// Extension registry including [crate::extension::qsystem::REGISTRY] and
    /// [tket::extension::rotation::ROTATION_EXTENSION].
    pub static ref REGISTRY: ExtensionRegistry = {
        let mut registry = qsystem::REGISTRY.to_owned();
        registry.register(tket::extension::rotation::ROTATION_EXTENSION.to_owned()).unwrap();
        registry
    };
}

pub(super) fn pi_mul_f64<T: Dataflow + ?Sized>(builder: &mut T, multiplier: f64) -> Wire {
    const_f64(builder, multiplier * std::f64::consts::PI)
}

fn const_f64<T: Dataflow + ?Sized>(builder: &mut T, value: f64) -> Wire {
    builder.add_load_const(ops::Const::new(ConstF64::new(value).into()))
}

/// Errors produced by lowering [TketOp]s.
#[derive(Debug, Display, Error, From)]
#[non_exhaustive]
pub enum LowerTk2Error {
    /// An error raised when building the circuit.
    #[display("Error when building the circuit: {_0}")]
    BuildError(BuildError),
    /// Found an unrecognised operation.
    #[display("Unrecognised operation: {} with {_1} inputs", _0.exposed_name())]
    UnknownOp(TketOp, usize),
    /// An error raised when replacing an operation.
    #[display("Error when replacing op: {_0}")]
    OpReplacement(HugrError),
    /// An error raised when lowering operations.
    #[display("Error when lowering ops: {_0}")]
    CircuitReplacement(hugr::algorithms::lower::LowerError),
    /// TketOps were not lowered after the pass.
    #[display("TketOps were not lowered: {missing_ops:?}")]
    #[from(ignore)]
    Unlowered {
        /// The list of nodes that were not lowered.
        missing_ops: Vec<Node>,
    },
    /// Non-module HUGR can't be lowered.
    #[display("HUGR root cannot have FuncDefn, has type: {}", _0)]
    InvalidFuncDefn(#[error(ignore)] hugr::ops::OpType),
    /// Error when using [`ReplaceTypes`] to lower operations.
    ReplaceTypesError(#[from] ReplaceTypesError),

    /// Error when inserting a runtime barrier.
    #[display("Error when inserting a runtime barrier: {_0}")]
    RuntimeBarrierError(#[from] InsertCutError),
}

enum ReplaceOps {
    Tk2(TketOp),
    Barrier(Barrier),
}

/// Lower [`TketOp`] operations to [`QSystemOp`] operations.
///
/// Single op replacements are done directly, while multi-op replacements are done
/// by lazily defining and calling functions that implement the decomposition.
/// Returns the nodes that were replaced.
///
/// # Errors
/// Returns an error if the replacement fails.
pub fn lower_tk2_op(hugr: &mut impl HugrMut<Node = Node>) -> Result<Vec<Node>, LowerTk2Error> {
    let mut funcs: BTreeMap<TketOp, Node> = BTreeMap::new();
    let mut lowerer = ReplaceTypes::new_empty();
    let mut barrier_funcs = BarrierInserter::new();

    let replacements: Vec<_> = hugr
        .nodes()
        .filter_map(|n| {
            let optype = hugr.get_optype(n);
            if let Some(op) = optype.cast::<TketOp>() {
                Some((n, ReplaceOps::Tk2(op)))
            } else {
                optype
                    .cast::<Barrier>()
                    .map(|op| (n, ReplaceOps::Barrier(op)))
            }
        })
        .collect();

    let mut replaced_nodes = Vec::with_capacity(replacements.len());
    for (node, op) in replacements {
        replaced_nodes.push(node);

        match op {
            ReplaceOps::Tk2(tket_op) => {
                // Handle TketOp replacements
                if let Some(direct) = direct_map(tket_op) {
                    lowerer.set_replace_op(
                        &tket_op.into_extension_op(),
                        NodeTemplate::SingleOp(direct.into()),
                    );
                    continue;
                }

                // Need to get or create function definition
                let func_node = match funcs.entry(tket_op) {
                    Entry::Occupied(e) => *e.get(),
                    Entry::Vacant(e) => {
                        let h = build_func(tket_op)?;
                        let inserted = hugr.insert_hugr(hugr.module_root(), h).inserted_entrypoint;
                        *e.insert(inserted)
                    }
                };
                lowerer.set_replace_op(
                    &tket_op.into_extension_op(),
                    NodeTemplate::Call(func_node, vec![]),
                );
            }
            ReplaceOps::Barrier(barrier) => {
                // Handle barrier replacements
                barrier_funcs.insert_runtime_barrier(hugr, node, barrier)?;
            }
        }
    }

    barrier_funcs.register_operation_replacements(hugr, &mut lowerer);

    // functions inserted at module root level so lowerer needs to be
    // run with module root as entrypoint
    let old_entrypoint = hugr.entrypoint();
    hugr.set_entrypoint(hugr.module_root());
    lowerer.run(hugr)?;
    // restore entrypoint
    hugr.set_entrypoint(old_entrypoint);

    Ok(replaced_nodes)
}

fn build_func(op: TketOp) -> Result<Hugr, LowerTk2Error> {
    let sig = op.into_extension_op().signature().into_owned();
    let sig = Signature::new(sig.input, sig.output); // ignore extension delta
    // TODO check generated names are namespaced enough
    let f_name = format!("__tk2_{}", op.op_id().to_lowercase());
    let mut b = FunctionBuilder::new(f_name, sig)?;
    let inputs: Vec<_> = b.input_wires().collect();
    let outputs = match (op, inputs.as_slice()) {
        (TketOp::H, [q]) => vec![b.build_h(*q)?],
        (TketOp::X, [q]) => vec![b.build_x(*q)?],
        (TketOp::Y, [q]) => vec![b.build_y(*q)?],
        (TketOp::Z, [q]) => vec![b.build_z(*q)?],
        (TketOp::S, [q]) => vec![b.build_s(*q)?],
        (TketOp::Sdg, [q]) => vec![b.build_sdg(*q)?],
        (TketOp::V, [q]) => vec![b.build_v(*q)?],
        (TketOp::Vdg, [q]) => vec![b.build_vdg(*q)?],
        (TketOp::T, [q]) => vec![b.build_t(*q)?],
        (TketOp::Tdg, [q]) => vec![b.build_tdg(*q)?],
        (TketOp::Measure, [q]) => b.build_measure_flip(*q)?.into(),
        (TketOp::QAlloc, []) => vec![b.build_qalloc()?],
        (TketOp::CX, [c, t]) => b.build_cx(*c, *t)?.into(),
        (TketOp::CY, [c, t]) => b.build_cy(*c, *t)?.into(),
        (TketOp::CZ, [c, t]) => b.build_cz(*c, *t)?.into(),
        (TketOp::Rx, [q, angle]) => {
            let float = build_to_radians(&mut b, *angle)?;
            vec![b.build_rx(*q, float)?]
        }
        (TketOp::Ry, [q, angle]) => {
            let float = build_to_radians(&mut b, *angle)?;
            vec![b.build_ry(*q, float)?]
        }
        (TketOp::Rz, [q, angle]) => {
            let float = build_to_radians(&mut b, *angle)?;
            vec![b.add_rz(*q, float)?]
        }
        (TketOp::CRz, [c, t, angle]) => {
            let float = build_to_radians(&mut b, *angle)?;
            b.build_crz(*c, *t, float)?.into()
        }
        (TketOp::Toffoli, [a, b_, c]) => b.build_toffoli(*a, *b_, *c)?.into(),
        _ => return Err(LowerTk2Error::UnknownOp(op, inputs.len())), // non-exhaustive
    };
    Ok(b.finish_hugr_with_outputs(outputs)?)
}

fn build_to_radians(b: &mut impl Dataflow, rotation: Wire) -> Result<Wire, BuildError> {
    let turns = b.add_to_halfturns(rotation)?;
    let pi = pi_mul_f64(b, 1.0);
    let float = b.add_dataflow_op(FloatOps::fmul, [turns, pi])?.out_wire(0);
    Ok(float)
}

fn direct_map(op: TketOp) -> Option<QSystemOp> {
    Some(match op {
        TketOp::TryQAlloc => QSystemOp::TryQAlloc,
        TketOp::QFree => QSystemOp::QFree,
        TketOp::Reset => QSystemOp::Reset,
        TketOp::MeasureFree => QSystemOp::Measure,
        _ => return None,
    })
}

/// Check there are no "tket.quantum" ops left in the HUGR.
///
/// # Errors
/// Returns vector of nodes that are not lowered.
pub fn check_lowered<H: HugrView>(hugr: &H) -> Result<(), Vec<H::Node>> {
    let unlowered: Vec<H::Node> = hugr
        .nodes()
        .filter_map(|node| {
            let optype = hugr.get_optype(node);
            optype.as_extension_op().and_then(|ext| {
                (ext.def().extension_id() == &tket::extension::TKET_EXTENSION_ID).then_some(node)
            })
        })
        .collect();

    if unlowered.is_empty() {
        Ok(())
    } else {
        Err(unlowered)
    }
}

/// A `Hugr -> Hugr` pass that replaces [tket::TketOp] nodes to
/// equivalent graphs made of [QSystemOp]s.
///
/// Invokes [lower_tk2_op]. If validation is enabled the resulting HUGR is
/// checked with [check_lowered].
#[derive(Default, Debug, Clone)]
pub struct LowerTketToQSystemPass;

impl<H: HugrMut<Node = Node>> ComposablePass<H> for LowerTketToQSystemPass {
    type Error = LowerTk2Error;
    type Result = ();

    fn run(&self, hugr: &mut H) -> Result<(), LowerTk2Error> {
        lower_tk2_op(hugr)?;
        #[cfg(test)]
        check_lowered(hugr).map_err(|missing_ops| LowerTk2Error::Unlowered { missing_ops })?;
        Ok(())
    }
}

#[cfg(test)]
mod test {
    use hugr::{
        HugrView,
        builder::{DFGBuilder, FunctionBuilder},
        extension::prelude::{UnwrapBuilder as _, bool_t, option_type, qb_t},
        type_row,
    };
    use tket::{Circuit, extension::rotation::rotation_type};

    use super::*;
    use rstest::rstest;

    #[test]
    fn test_lower_direct() {
        let mut b = FunctionBuilder::new("circuit", Signature::new_endo(type_row![])).unwrap();
        let [maybe_q] = b
            .add_dataflow_op(TketOp::TryQAlloc, [])
            .unwrap()
            .outputs_arr();
        let [q] = b.build_unwrap_sum(1, option_type(qb_t()), maybe_q).unwrap();
        let [q] = b.add_dataflow_op(TketOp::Reset, [q]).unwrap().outputs_arr();
        b.add_dataflow_op(TketOp::QFree, [q]).unwrap();
        let [maybe_q] = b
            .add_dataflow_op(TketOp::TryQAlloc, [])
            .unwrap()
            .outputs_arr();
        let [q] = b.build_unwrap_sum(1, option_type(qb_t()), maybe_q).unwrap();

        let [_] = b
            .add_dataflow_op(TketOp::MeasureFree, [q])
            .unwrap()
            .outputs_arr();
        let mut h = b
            .finish_hugr_with_outputs([])
            .unwrap_or_else(|e| panic!("{}", e));

        let lowered = lower_tk2_op(&mut h).unwrap();
        assert_eq!(lowered.len(), 5);
        let circ = Circuit::new(&h);
        let ops: Vec<QSystemOp> = circ
            .commands()
            .filter_map(|com| com.optype().cast())
            .collect();
        assert_eq!(
            ops,
            vec![
                QSystemOp::TryQAlloc,
                QSystemOp::Measure,
                QSystemOp::TryQAlloc,
                QSystemOp::Reset,
                QSystemOp::QFree,
            ]
        );
        assert_eq!(check_lowered(&h), Ok(()));
    }

    #[rstest]
    #[case(TketOp::H, Some(vec![QSystemOp::PhasedX, QSystemOp::Rz]))]
    #[case(TketOp::X, Some(vec![QSystemOp::PhasedX]))]
    #[case(TketOp::Y, Some(vec![QSystemOp::PhasedX]))]
    #[case(TketOp::Z, Some(vec![QSystemOp::Rz]))]
    #[case(TketOp::S, Some(vec![QSystemOp::Rz]))]
    #[case(TketOp::Sdg, Some(vec![QSystemOp::Rz]))]
    #[case(TketOp::V, Some(vec![QSystemOp::PhasedX]))]
    #[case(TketOp::Vdg, Some(vec![QSystemOp::PhasedX]))]
    #[case(TketOp::T, Some(vec![QSystemOp::Rz]))]
    #[case(TketOp::Tdg, Some(vec![QSystemOp::Rz]))]
    #[case(TketOp::Rx, Some(vec![QSystemOp::PhasedX]))]
    #[case(TketOp::Ry, Some(vec![QSystemOp::PhasedX]))]
    #[case(TketOp::Rz, Some(vec![QSystemOp::Rz]))]
    // multi qubit ordering is not deterministic
    #[case(TketOp::CX, None)]
    #[case(TketOp::CY, None)]
    #[case(TketOp::CZ, None)]
    #[case(TketOp::CRz, None)]
    #[case(TketOp::Toffoli, None)]
    // conditional doesn't fit in to commands
    #[case(TketOp::Measure, None)]
    #[case(TketOp::QAlloc, None)]
    fn test_lower(#[case] t2op: TketOp, #[case] qsystem_ops: Option<Vec<QSystemOp>>) {
        // build dfg with just the op

        let h = build_func(t2op).unwrap();
        let circ = Circuit::new(&h);
        let ops: Vec<QSystemOp> = circ
            .commands()
            .filter_map(|com| com.optype().cast())
            .collect();
        if let Some(qsystem_ops) = qsystem_ops {
            assert_eq!(ops, qsystem_ops);
        }

        assert_eq!(check_lowered(&h), Ok(()));
    }

    #[test]
    fn test_mixed() {
        let mut b = DFGBuilder::new(Signature::new(rotation_type(), bool_t())).unwrap();
        let [angle] = b.input_wires_arr();
        let qalloc = b.add_dataflow_op(TketOp::QAlloc, []).unwrap();
        let [q] = qalloc.outputs_arr();
        let [q] = b.add_dataflow_op(TketOp::H, [q]).unwrap().outputs_arr();
        let rx = b.add_dataflow_op(TketOp::Rx, [q, angle]).unwrap();
        let [q] = rx.outputs_arr();
        let q = b.add_barrier([q]).unwrap().out_wire(0);
        let [q, bool] = b
            .add_dataflow_op(TketOp::Measure, [q])
            .unwrap()
            .outputs_arr();
        let qfree = b.add_dataflow_op(TketOp::QFree, [q]).unwrap();
        b.set_order(&qalloc, &rx);
        b.set_order(&rx, &qfree);
        let mut h = b.finish_hugr_with_outputs([bool]).unwrap();

        let lowered = lower_tk2_op(&mut h).unwrap();
        assert_eq!(lowered.len(), 6);
        // dfg, input, output, alloc + (10 for unwrap), phasedx, rz, toturns, fmul, phasedx, free +
        // 5x(float + load), measure_reset, conditional, case(input, output) * 2, flip
        // (phasedx + 2*(float + load)), tket.read
        // + 10 for the barrier array wrapping, unwrapping and option unwrapping
        assert_eq!(h.descendants(h.module_root()).count(), 72);
        assert_eq!(check_lowered(&h), Ok(()));
        if let Err(e) = h.validate() {
            panic!("{}", e);
        }
    }
}
