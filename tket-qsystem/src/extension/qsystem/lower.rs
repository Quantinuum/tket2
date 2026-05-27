use derive_more::{Display, Error, From};
use hugr::builder::{Container, HugrBuilder};
use hugr::core::Visibility;
use hugr::extension::prelude::Barrier;
use hugr::extension::simple_op::MakeExtensionOp;
use hugr::hugr::linking::NameLinkingPolicy;
use hugr::hugr::linking::OnMultiDefn;
use hugr::hugr::patch::insert_cut::InsertCutError;
use hugr::ops::handle::{FuncID, NodeHandle};
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
use tket::passes::composable::WithScope;
use tket::passes::replace_types::{NodeTemplate, ReplaceTypesError};
use tket::passes::{ComposablePass, PassScope, ReplaceTypes};
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
    CircuitReplacement(tket::passes::lower::LowerError),
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
/// Single op replacements are done directly, while multi-op replacements are
/// done by lazily defining and calling functions that implement the
/// decomposition. Returns the nodes that were replaced.
///
/// The operation is parameterized by a `scope`. For non-[`PassScope::Global`]
/// passes multi-op replacement will not be performed, as they require adding
/// functions at the global module definition. See [`PassScope`] for more details.
///
/// # Arguments
///
/// * `hugr` - The HUGR to lower.
/// * `scope` - The scope across which to lower in the HUGR
///
/// # Errors
///
/// Returns an error if the replacement fails.
pub fn lower_tk2_ops(
    hugr: &mut impl HugrMut<Node = Node>,
    scope: impl Into<PassScope>,
) -> Result<Vec<Node>, LowerTk2Error> {
    let scope = scope.into();
    let mut funcs: BTreeMap<TketOp, NodeTemplate> = BTreeMap::new();
    let mut lowerer = ReplaceTypes::new_empty().with_scope(scope.clone());
    let mut barrier_funcs = BarrierInserter::new();

    let replacements: Vec<_> = scope
        .regions(hugr)
        .flat_map(|region| hugr.children(region))
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
        match op {
            ReplaceOps::Tk2(tket_op) => {
                // Handle TketOp replacements
                if let Some(direct) = direct_map(tket_op) {
                    lowerer.set_replace_op(
                        &tket_op.into_extension_op(),
                        NodeTemplate::SingleOp(direct.into()),
                    );

                    replaced_nodes.push(node);
                } else if matches!(scope, PassScope::Global(_)) {
                    // Only perform multi-op replacement for global passes, as we
                    // cannot define new functions for local entrypoint scopes.
                    let template = match funcs.entry(tket_op) {
                        Entry::Occupied(e) => e.get().clone(),
                        Entry::Vacant(e) => {
                            let template = func_as_node_template(build_func(tket_op)?);
                            e.insert(template).clone()
                        }
                    };
                    lowerer.set_replace_op(&tket_op.into_extension_op(), template);

                    replaced_nodes.push(node);
                }
            }
            ReplaceOps::Barrier(barrier) => {
                // Handle barrier replacements
                //
                // Only perform the replacement for global passes, as we
                // cannot define the barrier function for local entrypoint scopes.
                if let PassScope::Global(_) = scope {
                    barrier_funcs.insert_runtime_barrier(hugr, node, barrier)?;
                    replaced_nodes.push(node);
                }
            }
        }
    }

    barrier_funcs.register_operation_replacements(hugr, &mut lowerer);

    // Replace the operations.
    lowerer.with_scope(scope.clone()).run(hugr)?;

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

/// Given a hugr with a function definition as entrypoint, constructs a
/// [`NodeTemplate::LinkedHugr`] that produces a call to the function.
//
// TODO: Use [`NodeTemplate::call_to_function`] once it gets released in `hugr 0.25.6`.
fn func_as_node_template(func_def: Hugr) -> NodeTemplate {
    // Create a replacement hugr for the op nodes: Add a `call` node in the `func_def` hugr and set it as entrypoint.
    let func_signature = func_def.inner_function_type().unwrap().into_owned();

    // Build a new hugr and insert the function definition into it
    let mut b = FunctionBuilder::new_vis("", func_signature, Visibility::Private).unwrap();
    let func_id = FuncID::<true>::from(
        b.module_root_builder()
            .add_hugr(func_def)
            .inserted_entrypoint,
    );

    // Build a call to the function in the new separate function.
    let call = b.call(&func_id, &[], b.input_wires()).unwrap();
    let mut call_hugr = b.finish_hugr_with_outputs(call.outputs()).unwrap();
    call_hugr.set_entrypoint(call.node());

    NodeTemplate::LinkedHugr(
        Box::new(call_hugr),
        NameLinkingPolicy::default().on_multiple_defn(OnMultiDefn::UseTarget),
    )
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

/// Check there are no "tket.quantum" ops left in the HUGR that should have been
/// lowered by [lower_tk2_ops] with the given scope.
///
/// To check that there isn't any unlowered operations, use
/// [`PassScope::Global`] as the scope.
///
/// See [`LowerTketToQSystemPass`] for details on which operations are affected
/// depending on the scope.
///
/// # Errors
///
/// Returns vector of nodes that are not lowered.
pub fn check_lowered<H: HugrView>(
    hugr: &H,
    scope: impl Into<PassScope>,
) -> Result<(), Vec<H::Node>> {
    let scope = scope.into();
    let unlowered: Vec<H::Node> = scope
        .regions(hugr)
        .flat_map(|region| hugr.children(region))
        .filter_map(|node| {
            let tket_op = hugr.get_optype(node).cast::<TketOp>()?;

            if !matches!(scope, PassScope::Global(_)) && direct_map(tket_op).is_none() {
                // Local entrypoint scopes do not perform multi-op replacements,
                // as those need to add functions at the global module level.
                return None;
            }

            Some(node)
        })
        .collect();

    if unlowered.is_empty() {
        Ok(())
    } else {
        Err(unlowered)
    }
}

/// A `Hugr -> Hugr` pass that replaces [tket::TketOp] nodes to equivalent
/// graphs made of [QSystemOp]s.
///
/// Invokes [lower_tk2_ops]. If validation is enabled the resulting HUGR is
/// checked with [check_lowered].
///
/// The pass scope may be controlled via [`WithScope::with_scope`]. For
/// non-[`PassScope::Global`] scopes, multi-op replacement will not be
/// performed, as they require adding functions at the global module level. See
/// [`PassScope`] for more details.
#[derive(Default, Debug, Clone)]
pub struct LowerTketToQSystemPass {
    /// Where to apply the pass.
    ///
    /// Configurable via [`WithScope::with_scope`].
    scope: PassScope,
}

impl WithScope for LowerTketToQSystemPass {
    fn with_scope(mut self, scope: impl Into<PassScope>) -> Self {
        self.scope = scope.into();
        self
    }
}

impl<H: HugrMut<Node = Node>> ComposablePass<H> for LowerTketToQSystemPass {
    type Error = LowerTk2Error;
    type Result = ();

    fn run(&self, hugr: &mut H) -> Result<(), LowerTk2Error> {
        lower_tk2_ops(hugr, self.scope.clone())?;
        #[cfg(test)]
        check_lowered(hugr, self.scope.clone())
            .map_err(|missing_ops| LowerTk2Error::Unlowered { missing_ops })?;
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
    use tket::passes::composable::Preserve;
    use tket::{Circuit, extension::rotation::rotation_type};

    use super::*;
    use rstest::rstest;

    #[rstest]
    #[case::global(PassScope::Global(Preserve::Public))]
    #[case::entrypoint_flat(PassScope::EntrypointFlat)]
    #[case::entrypoint_recursive(PassScope::EntrypointRecursive)]

    fn test_lower_direct(#[case] scope: PassScope) {
        let mut b = FunctionBuilder::new("circuit", Signature::new_endo(type_row![])).unwrap();
        let [maybe_q] = b
            .add_dataflow_op(TketOp::TryQAlloc, [])
            .unwrap()
            .outputs_arr();
        let [q] = b
            .build_unwrap_sum(1, option_type(vec![qb_t()]), maybe_q)
            .unwrap();
        let [q] = b.add_dataflow_op(TketOp::Reset, [q]).unwrap().outputs_arr();
        b.add_dataflow_op(TketOp::QFree, [q]).unwrap();
        let [maybe_q] = b
            .add_dataflow_op(TketOp::TryQAlloc, [])
            .unwrap()
            .outputs_arr();
        let [q] = b
            .build_unwrap_sum(1, option_type(vec![qb_t()]), maybe_q)
            .unwrap();

        let [_] = b
            .add_dataflow_op(TketOp::MeasureFree, [q])
            .unwrap()
            .outputs_arr();
        let mut h = b
            .finish_hugr_with_outputs([])
            .unwrap_or_else(|e| panic!("{}", e));

        let lowered = lower_tk2_ops(&mut h, scope.clone()).unwrap();
        assert_eq!(lowered.len(), 5);
        let circ = Circuit::new(&h);
        let ops: Vec<QSystemOp> = circ
            .toposorted_children(circ.parent())
            .expect("circuit entrypoint should be dataflow region")
            .filter_map(|n| circ.hugr().get_optype(n).cast())
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
        assert_eq!(check_lowered(&h, scope), Ok(()));
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
            .toposorted_children(circ.parent())
            .expect("circuit entrypoint should be dataflow region")
            .filter_map(|n| circ.hugr().get_optype(n).cast())
            .collect();
        if let Some(qsystem_ops) = qsystem_ops {
            assert_eq!(ops, qsystem_ops);
        }

        assert_eq!(check_lowered(&h, Preserve::Public), Ok(()));
    }

    #[rstest]
    #[case::global(PassScope::Global(Preserve::Public))]
    #[case::entrypoint_flat(PassScope::EntrypointFlat)]
    #[case::entrypoint_recursive(PassScope::EntrypointRecursive)]
    fn test_mixed(#[case] scope: PassScope) {
        let mut b = DFGBuilder::new(Signature::new([rotation_type()], [bool_t()])).unwrap();
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

        let original_node_count = h.nodes().count();

        let lowered = lower_tk2_ops(&mut h, scope.clone()).unwrap();

        let expected_lower_count = match scope {
            PassScope::EntrypointFlat => 1,
            PassScope::EntrypointRecursive => 1,
            PassScope::Global(_) => 6,
            _ => unreachable!(),
        };
        assert_eq!(lowered.len(), expected_lower_count);

        let final_node_count = h.nodes().count();
        let expected_node_count = match scope {
            PassScope::EntrypointFlat => original_node_count,
            PassScope::EntrypointRecursive => original_node_count,
            PassScope::Global(_) => original_node_count + 59,
            _ => unreachable!(),
        };
        assert_eq!(final_node_count, expected_node_count);

        assert_eq!(check_lowered(&h, scope), Ok(()));
        if let Err(e) = h.validate() {
            panic!("{}", e);
        }
    }
}
