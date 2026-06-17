use derive_more::{Display, Error, From};
use hugr::builder::{Container, HugrBuilder};
use hugr::core::Visibility;
use hugr::extension::prelude::{Barrier, Noop, bool_t};
use hugr::extension::simple_op::{MakeExtensionOp, MakeRegisteredOp};
use hugr::hugr::linking::NameLinkingPolicy;
use hugr::hugr::linking::OnMultiDefn;
use hugr::hugr::patch::insert_cut::InsertCutError;
use hugr::ops::handle::{FuncID, NodeHandle};
use hugr::{
    Hugr, HugrView, Node, Wire,
    builder::{BuildError, Dataflow, DataflowHugr, FunctionBuilder},
    extension::{ExtensionId, ExtensionRegistry, ExtensionSet},
    hugr::{HugrError, hugrmut::HugrMut},
    ops::{self, DataflowOpTrait},
    std_extensions::arithmetic::{float_ops::FloatOps, float_types::ConstF64},
    types::Signature,
};
use lazy_static::lazy_static;
use std::collections::BTreeMap;
use std::collections::btree_map::Entry;
use tket::extension::measurement::{MeasurementOp, measurement_custom_type};
use tket::passes::composable::WithScope;
use tket::passes::replace_types::{NodeTemplate, ReplaceTypesError};
use tket::passes::{ComposablePass, PassScope, ReplaceTypes};
use tket::{TketOp, extension::rotation::RotationOpBuilder};

use crate::extension::futures::{FutureOp, FutureOpDef, future_type};
use crate::extension::qsystem::{self, QSystemPlatform};
use crate::helpers::{
    lowerer_with_future_linearization, replace_array_ops_requiring_copyable_bounds,
};

use super::barrier::BarrierInserter;
use super::common::SharedOp;
use super::helios::{self, HeliosOp, HeliosSynthesizer, SynthesizeHeliosOp};
use super::sol::{self, SolOp, SolSynthesizer, SynthesizeSolOp};
use super::synth_tket_op::SynthesizeTketOp;
use strum::IntoEnumIterator as _;

lazy_static! {
    /// Extension registry including [crate::extension::qsystem::REGISTRY] and
    /// [tket::extension::rotation::ROTATION_EXTENSION].
    pub static ref REGISTRY: ExtensionRegistry = {
        let mut registry = qsystem::REGISTRY.to_owned();
        registry.register(tket::extension::rotation::ROTATION_EXTENSION.to_owned());
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

    /// Legacy `tket.qsystem` ops that are Helios-specific (i.e. have no shared
    /// qsystem equivalent) could not previously be lowered to Sol via direct
    /// remapping.
    ///
    /// Deprecated: Helios-specific ops are now handled via the cross-platform
    /// lowering path; `lower_tk2_ops` will no longer return this error.
    #[deprecated(
        since = "0.26.0",
        note = "Helios-specific ops are now handled by the cross-platform lowering path; \
                    this error variant will no longer be returned by lower_tk2_ops."
    )]
    #[display(
        "Helios-specific legacy tket.qsystem ops cannot be lowered to Sol via direct remapping; \
         use cross-platform lowering instead."
    )]
    LegacyQSystemToSolUnsupported,
}

/// Ops detected for replacement, classified by type.
enum ReplaceOps {
    /// A `tket.quantum` op that must be decomposed into platform-native primitives.
    Tk2(TketOp),
    /// A runtime barrier that must be lowered to a platform-specific barrier call.
    Barrier(Barrier),
    /// A `tket.qsystem.helios` op encountered while targeting another platform.
    CrossPlatformHelios(HeliosOp),
    /// A `tket.qsystem.sol` op encountered while targeting another platform.
    CrossPlatformSol(SolOp),
}

/// Register replacements for the deprecated `tket.qsystem` ops onto `lowerer`.
///
/// `tket.qsystem` was the original name for `tket.qsystem.helios`.
/// The two extensions share identical op names, so each legacy op maps 1:1 to
/// its platform-appropriate counterpart.
///
/// For [`QSystemPlatform::Helios`] every legacy op maps to the corresponding
/// [`HeliosOp`]. For [`QSystemPlatform::Sol`], shared ops map directly to the
/// corresponding [`SolOp`]; Helios-specific ops (e.g. `ZZPhase`) are skipped
/// here and handled lazily via the cross-platform path in the processing loop.
fn register_legacy_qsystem_replacements(lowerer: &mut ReplaceTypes, platform: QSystemPlatform) {
    for helios_op in HeliosOp::iter() {
        let op_name = <&'static str>::from(helios_op);
        let legacy_op = qsystem::EXTENSION
            .instantiate_extension_op(op_name, &[])
            .expect("tket.qsystem and tket.qsystem.helios share op names");
        let replacement = match platform {
            QSystemPlatform::Helios => NodeTemplate::SingleOp(helios_op.into()),
            QSystemPlatform::Sol => match SharedOp::try_from(helios_op) {
                Ok(shared) => NodeTemplate::SingleOp(SolOp::from(shared).into()),
                Err(_) => {
                    // Helios-specific legacy ops are detected in the filter_map
                    // and handled lazily via CrossPlatformHelios in the loop.
                    continue;
                }
            },
        };
        lowerer.set_replace_op(&legacy_op, replacement);
    }
}

// ── Node classification ─────────────────────────────────────────────────────

/// Classify a single HUGR node for the lowering pass.
///
/// Returns `Some(Ok((node, op)))` when the node needs processing,
/// `Some(Err(_))` if classification itself fails, or `None` to skip the node.
fn classify_node(
    n: Node,
    optype: &ops::OpType,
    platform: QSystemPlatform,
) -> Option<Result<(Node, ReplaceOps), LowerTk2Error>> {
    if let Some(op) = optype.cast::<TketOp>() {
        return Some(Ok((n, ReplaceOps::Tk2(op))));
    }
    if let Some(op) = optype.cast::<Barrier>() {
        return Some(Ok((n, ReplaceOps::Barrier(op))));
    }
    let ext_id = optype.as_extension_op()?.def().extension_id();
    match platform {
        QSystemPlatform::Sol if ext_id == &helios::EXTENSION_ID => optype
            .cast::<HeliosOp>()
            .map(|h_op| Ok((n, ReplaceOps::CrossPlatformHelios(h_op)))),
        QSystemPlatform::Helios if ext_id == &sol::EXTENSION_ID => optype
            .cast::<SolOp>()
            .map(|s_op| Ok((n, ReplaceOps::CrossPlatformSol(s_op)))),
        // ZZPhase is the only Helios-specific legacy op; all others are shared
        // and already registered by register_legacy_qsystem_replacements.
        QSystemPlatform::Sol
            if ext_id == &qsystem::EXTENSION_ID
                && optype.as_extension_op()?.def().name().as_str()
                    == <&'static str>::from(HeliosOp::ZZPhase) =>
        {
            Some(Ok((n, ReplaceOps::CrossPlatformHelios(HeliosOp::ZZPhase))))
        }
        // Note: `helios::RuntimeBarrierDef` is not a `HeliosOp` variant, so a
        // pre-existing `helios::RuntimeBarrier` op in the HUGR would be silently
        // skipped here (cast returns None). This is fine for the current use
        // case: `RuntimeBarrier` is introduced by this lowering pass when it
        // lowers generic tket `Barrier` ops, while other qsystem platform ops
        // may already be present in the input HUGR and are handled above.
        // Relowering a fresh HUGR with generic barriers and platform ops is
        // supported, but relowering an already-platform-lowered HUGR that
        // contains runtime barriers to a different platform is not. If that
        // becomes a requirement, cross-platform `RuntimeBarrier` remapping will
        // need to be added here.
        _ => None,
    }
}

// ─────────────────────────────────────────────────────────────────────────────

/// Register any replacements related to the `Measurement` type.
fn register_measurement_replacements(lowerer: &mut ReplaceTypes) {
    // As the measurement type acts like an alias for `Future<Bool>`, most replacements
    // are straightforward.
    lowerer.set_replace_type(measurement_custom_type(), future_type(bool_t()));

    let noop = NodeTemplate::SingleOp(
        Noop::new(future_type(bool_t()))
            .to_extension_op()
            .unwrap()
            .into(),
    );
    lowerer.set_replace_op(
        &HeliosOp::FutureToMeasurement.to_extension_op().unwrap(),
        noop.clone(),
    );
    lowerer.set_replace_op(&SolOp::FutureToMeasurement.to_extension_op().unwrap(), noop);

    let future_bool_op = FutureOp {
        op: FutureOpDef::Read,
        typ: bool_t(),
    }
    .to_extension_op()
    .unwrap();
    lowerer.set_replace_op(
        &MeasurementOp::Read.to_extension_op().unwrap(),
        NodeTemplate::SingleOp(future_bool_op.into()),
    );

    // This is required as copyable `Measurements` are replaced by linear
    // `Futures`. Note we don't need to deal with static arrays as you cannot
    // create static arrays of `Measurement`` values in Guppy.
    replace_array_ops_requiring_copyable_bounds(lowerer);
}

/// Lower [`TketOp`] operations to target QSystem operations.
///
/// Single op replacements are done directly, while multi-op replacements are
/// done by lazily defining and calling functions that implement the
/// decomposition. Returns the nodes that were replaced.
///
/// This pass also replaces `tket.measurement` with `future(bool_t)` and
/// [HeliosOp::FutureToMeasurement] / [SolOp::FutureToMeasurement] becomes a no-op.
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
    platform: QSystemPlatform,
) -> Result<Vec<Node>, LowerTk2Error> {
    let scope = scope.into();
    let mut funcs: BTreeMap<TketOp, NodeTemplate> = BTreeMap::new();
    let mut lowerer = lowerer_with_future_linearization().with_scope(scope.clone());
    register_measurement_replacements(&mut lowerer);
    let mut barrier_funcs = BarrierInserter::new(platform);

    // Cross-platform decomposition templates are built lazily in the processing
    // loop, only when a cross-platform op is actually encountered.
    let mut helios_cross_funcs: BTreeMap<HeliosOp, NodeTemplate> = BTreeMap::new();
    let mut sol_cross_funcs: BTreeMap<SolOp, NodeTemplate> = BTreeMap::new();
    register_legacy_qsystem_replacements(&mut lowerer, platform);

    let replacements: Vec<_> = scope
        .regions(hugr)
        .flat_map(|region| hugr.children(region))
        .filter_map(|n| classify_node(n, hugr.get_optype(n), platform))
        .collect::<Result<Vec<_>, LowerTk2Error>>()?;
    let mut replaced_nodes = Vec::with_capacity(replacements.len());
    for (node, op) in replacements {
        match op {
            ReplaceOps::Tk2(tket_op) => match (direct_map(tket_op, platform), &scope) {
                (Some(direct), _) => {
                    lowerer.set_replace_op(
                        &tket_op.into_extension_op(),
                        NodeTemplate::SingleOp(direct),
                    );
                    replaced_nodes.push(node);
                }
                (None, PassScope::Global(_)) => {
                    let template = match funcs.entry(tket_op) {
                        Entry::Occupied(e) => e.get().clone(),
                        Entry::Vacant(e) => {
                            let t = func_as_node_template(build_func(platform, tket_op)?);
                            e.insert(t).clone()
                        }
                    };
                    lowerer.set_replace_op(&tket_op.into_extension_op(), template);
                    replaced_nodes.push(node);
                }
                (None, _) => {} // non-global multi-op: leave unchanged
            },
            ReplaceOps::Barrier(barrier) => {
                // Handle barrier replacements
                //
                // Only perform the replacement for global passes, as we
                // cannot define the barrier function for local entrypoint scopes.
                if let PassScope::Global(_) = &scope {
                    barrier_funcs.insert_runtime_barrier(hugr, node, barrier)?;
                    replaced_nodes.push(node);
                }
            }
            ReplaceOps::CrossPlatformHelios(h_op) => apply_cross_platform_helios(
                node,
                h_op,
                hugr.get_optype(node),
                &scope,
                &mut lowerer,
                &mut helios_cross_funcs,
                &mut replaced_nodes,
            )?,
            ReplaceOps::CrossPlatformSol(s_op) => apply_cross_platform_sol(
                node,
                s_op,
                &scope,
                &mut lowerer,
                &mut sol_cross_funcs,
                &mut replaced_nodes,
            )?,
        }
    }

    barrier_funcs.register_operation_replacements(hugr, &mut lowerer);

    // Replace the operations.
    lowerer.with_scope(scope.clone()).run(hugr)?;

    Ok(replaced_nodes)
}

fn platform_str(platform: QSystemPlatform) -> &'static str {
    match platform {
        QSystemPlatform::Helios => "helios",
        QSystemPlatform::Sol => "sol",
    }
}

fn build_func(platform: QSystemPlatform, op: TketOp) -> Result<Hugr, LowerTk2Error> {
    let sig = op.into_extension_op().signature().into_owned();
    let sig = Signature::new(sig.input, sig.output); // ignore extension delta
    // TODO check generated names are namespaced enough
    let f_name = format!(
        "__tk2_{}_{}",
        platform_str(platform),
        op.op_id().to_lowercase()
    );
    let mut f_build = FunctionBuilder::new(f_name, sig)?;
    let outputs = build_func_outputs(platform, &mut f_build, op)?;
    Ok(f_build.finish_hugr_with_outputs(outputs)?)
}

fn build_func_outputs(
    platform: QSystemPlatform,
    f_build: &mut FunctionBuilder<Hugr>,
    op: TketOp,
) -> Result<Vec<Wire>, LowerTk2Error> {
    match platform {
        QSystemPlatform::Helios => {
            build_func_with_builder(&mut HeliosSynthesizer::new(f_build), op)
        }
        QSystemPlatform::Sol => build_func_with_builder(&mut SolSynthesizer::new(f_build), op),
    }
}

fn build_func_with_builder<B>(b: &mut B, op: TketOp) -> Result<Vec<Wire>, LowerTk2Error>
where
    B: SynthesizeTketOp + Dataflow,
{
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
            let float = build_to_radians(b, *angle)?;
            vec![b.build_rx(*q, float)?]
        }
        (TketOp::Ry, [q, angle]) => {
            let float = build_to_radians(b, *angle)?;
            vec![b.build_ry(*q, float)?]
        }
        (TketOp::Rz, [q, angle]) => {
            let float = build_to_radians(b, *angle)?;
            vec![b.build_rz(*q, float)?]
        }
        (TketOp::CRz, [c, t, angle]) => {
            let float = build_to_radians(b, *angle)?;
            b.build_crz(*c, *t, float)?.into()
        }
        (TketOp::Toffoli, [a, b_, c]) => b.build_toffoli(*a, *b_, *c)?.into(),
        _ => return Err(LowerTk2Error::UnknownOp(op, inputs.len())), // non-exhaustive
    };
    Ok(outputs)
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

/// Map a [`TketOp`] to the [`SharedOp`] it directly corresponds to, if any.
///
/// These are the ops that have a platform-independent 1:1 replacement and
/// don't require a multi-op decomposition function.
fn tket_to_shared_op(op: TketOp) -> Option<SharedOp> {
    Some(match op {
        TketOp::TryQAlloc => SharedOp::TryQAlloc,
        TketOp::QFree => SharedOp::QFree,
        TketOp::Reset => SharedOp::Reset,
        TketOp::MeasureFree => SharedOp::LazyMeasure,
        _ => return None,
    })
}

fn direct_map(op: TketOp, platform: QSystemPlatform) -> Option<hugr::ops::OpType> {
    Some(platform.op_from_shared(tket_to_shared_op(op)?))
}

/// Returns true if `op` has a direct single-op replacement (regardless of platform).
fn has_direct_map(op: TketOp) -> bool {
    tket_to_shared_op(op).is_some()
}

impl QSystemPlatform {
    /// Convert a [`SharedOp`] to this platform's native [`hugr::ops::OpType`].
    fn op_from_shared(self, op: SharedOp) -> hugr::ops::OpType {
        match self {
            QSystemPlatform::Helios => HeliosOp::from(op).into(),
            QSystemPlatform::Sol => SolOp::from(op).into(),
        }
    }
}

/// Check that no ops belonging to any extension in `forbidden_extensions` are
/// present in the HUGR within `scope`.
///
/// For `TketOp`s specifically, non-[`PassScope::Global`] scopes only flag the
/// subset of ops that would have been lowered (i.e. those in `direct_map`),
/// because multi-op replacements require adding functions at the global module
/// level and are therefore skipped for local-entrypoint scopes.
///
/// # Errors
///
/// Returns the nodes whose ops are still present.
pub fn check_lowered<H: HugrView>(
    hugr: &H,
    scope: impl Into<PassScope>,
    forbidden_extensions: &ExtensionSet,
) -> Result<(), Vec<H::Node>> {
    let scope = scope.into();
    let unlowered: Vec<H::Node> = scope
        .regions(hugr)
        .flat_map(|region| hugr.children(region))
        .filter_map(|node| {
            let optype = hugr.get_optype(node);
            let ext_id: &ExtensionId = optype.as_extension_op()?.def().extension_id();
            if !forbidden_extensions.contains(ext_id) {
                return None;
            }
            // For TketOps in non-global scopes, ops that require multi-op
            // replacement are expected to remain.
            if let Some(tket_op) = optype.cast::<TketOp>()
                && !matches!(scope, PassScope::Global(_))
                && !has_direct_map(tket_op)
            {
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

// ── Cross-platform decomposition helpers ────────────────────────────────────

/// Register a cross-platform Helios→Sol replacement in `lowerer`.
///
/// For shared ops, registers a direct 1:1 remap. For Helios-specific ops
/// (e.g. `ZZPhase`), lazily builds the Sol decomposition template on first
/// encounter (Global scope only; non-Global scopes leave the op unchanged).
/// `optype` is read from the node so that both native `tket.qsystem.helios`
/// and legacy `tket.qsystem` ops are handled with the correct [`ExtensionOp`]
/// key.
fn apply_cross_platform_helios(
    node: Node,
    h_op: HeliosOp,
    optype: &ops::OpType,
    scope: &PassScope,
    lowerer: &mut ReplaceTypes,
    cache: &mut BTreeMap<HeliosOp, NodeTemplate>,
    replaced_nodes: &mut Vec<Node>,
) -> Result<(), LowerTk2Error> {
    match SharedOp::try_from(h_op) {
        Ok(shared) => {
            lowerer.set_replace_op(
                &h_op.to_extension_op().expect("valid registered HeliosOp"),
                NodeTemplate::SingleOp(SolOp::from(shared).into()),
            );
            replaced_nodes.push(node);
        }
        Err(_) if matches!(scope, PassScope::Global(_)) => {
            debug_assert_eq!(h_op, HeliosOp::ZZPhase, "unexpected Helios-specific op");
            let ext_op = optype
                .as_extension_op()
                .expect("CrossPlatformHelios node must be an extension op")
                .clone();
            let template = match cache.entry(h_op) {
                Entry::Occupied(e) => e.get().clone(),
                Entry::Vacant(e) => {
                    let t = func_as_node_template(build_helios_op_for_sol()?);
                    e.insert(t).clone()
                }
            };
            lowerer.set_replace_op(&ext_op, template);
            replaced_nodes.push(node);
        }
        Err(_) => {} // non-Global scope: leave unchanged
    }
    Ok(())
}

/// Register a cross-platform Sol→Helios replacement in `lowerer`.
///
/// For shared ops, registers a direct 1:1 remap. For Sol-specific ops
/// (e.g. `PhasedXX`), lazily builds the Helios decomposition template on first
/// encounter (Global scope only; non-Global scopes leave the op unchanged).
fn apply_cross_platform_sol(
    node: Node,
    s_op: SolOp,
    scope: &PassScope,
    lowerer: &mut ReplaceTypes,
    cache: &mut BTreeMap<SolOp, NodeTemplate>,
    replaced_nodes: &mut Vec<Node>,
) -> Result<(), LowerTk2Error> {
    match SharedOp::try_from(s_op) {
        Ok(shared) => {
            lowerer.set_replace_op(
                &s_op.to_extension_op().expect("valid registered SolOp"),
                NodeTemplate::SingleOp(HeliosOp::from(shared).into()),
            );
            replaced_nodes.push(node);
        }
        Err(_) if matches!(scope, PassScope::Global(_)) => {
            debug_assert_eq!(s_op, SolOp::PhasedXX, "unexpected Sol-specific op");
            let template = match cache.entry(s_op) {
                Entry::Occupied(e) => e.get().clone(),
                Entry::Vacant(e) => {
                    let t = func_as_node_template(build_sol_op_for_helios()?);
                    e.insert(t).clone()
                }
            };
            lowerer.set_replace_op(
                &s_op.to_extension_op().expect("valid registered SolOp"),
                template,
            );
            replaced_nodes.push(node);
        }
        Err(_) => {} // non-Global scope: leave unchanged
    }
    Ok(())
}

/// Build a [`Hugr`] function that decomposes [`HeliosOp::ZZPhase`] into Sol ops
/// via [`SynthesizeHeliosOp`] implemented on [`SolSynthesizer`].
fn build_helios_op_for_sol() -> Result<Hugr, LowerTk2Error> {
    let sig = HeliosOp::ZZPhase
        .to_extension_op()
        .expect("valid registered HeliosOp")
        .signature()
        .into_owned();
    let mut f_build = FunctionBuilder::new(
        "__tk2_helios_to_sol_zzphase",
        Signature::new(sig.input, sig.output),
    )?;
    let [qb1, qb2, angle] = f_build.input_wires_arr();
    let mut synth = SolSynthesizer::new(&mut f_build);
    let outputs: Vec<Wire> =
        SynthesizeHeliosOp::build_zz_phase(&mut synth, qb1, qb2, angle)?.into();
    Ok(f_build.finish_hugr_with_outputs(outputs)?)
}

/// Build a [`Hugr`] function that decomposes [`SolOp::PhasedXX`] into Helios ops
/// via [`SynthesizeSolOp`] implemented on [`HeliosSynthesizer`].
fn build_sol_op_for_helios() -> Result<Hugr, LowerTk2Error> {
    let sig = SolOp::PhasedXX
        .to_extension_op()
        .expect("valid registered SolOp")
        .signature()
        .into_owned();
    let mut f_build = FunctionBuilder::new(
        "__tk2_sol_to_helios_phasedxx",
        Signature::new(sig.input, sig.output),
    )?;
    let [qb1, qb2, angle1, angle2] = f_build.input_wires_arr();
    let mut synth = HeliosSynthesizer::new(&mut f_build);
    let outputs: Vec<Wire> =
        SynthesizeSolOp::build_phased_xx(&mut synth, qb1, qb2, angle1, angle2)?.into();
    Ok(f_build.finish_hugr_with_outputs(outputs)?)
}

/// A `Hugr -> Hugr` pass that replaces [`tket::TketOp`] nodes with equivalent
/// graphs made of target QSystem operations.
///
/// Invokes [lower_tk2_ops]. If validation is enabled the resulting HUGR is
/// checked with [check_lowered].
///
/// The pass scope may be controlled via [`WithScope::with_scope`]. For
/// non-[`PassScope::Global`] scopes, multi-op replacement will not be
/// performed, as they require adding functions at the global module level. See
/// [`PassScope`] for more details.
#[derive(Debug, Clone)]
pub struct LowerTketToQSystemPass {
    /// Where to apply the pass.
    ///
    /// Configurable via [`WithScope::with_scope`].
    scope: PassScope,
    /// Platform to lower for, which may affect the generated graph for some
    /// operations.
    ///
    /// Configurable via new
    platform: QSystemPlatform,
}

impl LowerTketToQSystemPass {
    /// Creates a new pass with the given scope and platform.
    pub fn new(platform: QSystemPlatform) -> Self {
        Self {
            scope: Default::default(),
            platform,
        }
    }
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
        lower_tk2_ops(hugr, self.scope.clone(), self.platform)?;
        #[cfg(test)]
        {
            let forbidden = ExtensionSet::from_iter([tket::extension::TKET_EXTENSION_ID]);
            check_lowered(hugr, self.scope.clone(), &forbidden)
                .map_err(|missing_ops| LowerTk2Error::Unlowered { missing_ops })?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod test {
    use hugr::{
        HugrView,
        builder::{DFGBuilder, FunctionBuilder, inout_sig},
        extension::{
            prelude::{UnwrapBuilder as _, bool_t, option_type, qb_t, usize_t},
            simple_op::{HasDef, MakeOpDef},
        },
        ops::OpType,
        std_extensions::collections::{
            array::{Array, ArrayKind, op_builder::GenericArrayOpBuilder},
            borrow_array::{BArrayOpBuilder, BorrowArray},
        },
        type_row,
        types::{Type, TypeRow},
    };
    use tket::{Circuit, extension::rotation::rotation_type};
    use tket::{
        extension::measurement::{MeasurementOp, measurement_type},
        passes::composable::Preserve,
    };

    use crate::extension::qsystem::{helios::HeliosOp, sol::SolOp};

    use super::*;
    use rstest::rstest;

    #[derive(Debug, PartialEq, Eq)]
    enum ExpectedOp {
        Helios(HeliosOp),
        Sol(SolOp),
    }

    impl ExpectedOp {
        fn cast(optype: &hugr::ops::OpType, platform: QSystemPlatform) -> Option<Self> {
            match platform {
                QSystemPlatform::Helios => optype.cast().map(Self::Helios),
                QSystemPlatform::Sol => optype.cast().map(Self::Sol),
            }
        }

        fn from_shared(shared: SharedOp, platform: QSystemPlatform) -> Self {
            match platform {
                QSystemPlatform::Helios => Self::Helios(HeliosOp::from(shared)),
                QSystemPlatform::Sol => Self::Sol(SolOp::from(shared)),
            }
        }
    }

    /// Returns an [`ExtensionSet`] of extensions whose ops must not appear in
    /// the HUGR after lowering to `platform`.
    ///
    /// This includes `tket.quantum` (all `TketOp`s should be gone) and the
    /// extension belonging to the *other* platform (cross-contamination).
    fn forbidden_extensions_for(platform: QSystemPlatform) -> ExtensionSet {
        use crate::extension::qsystem::{helios, sol};
        ExtensionSet::from_iter([
            tket::extension::TKET_EXTENSION_ID,
            match platform {
                QSystemPlatform::Helios => sol::EXTENSION_ID,
                QSystemPlatform::Sol => helios::EXTENSION_ID,
            },
        ])
    }

    fn legacy_op(name: &str) -> hugr::ops::OpType {
        use crate::extension::qsystem as qs;
        qs::EXTENSION
            .instantiate_extension_op(name, &[])
            .unwrap()
            .into()
    }

    fn toposorted_circuit_nodes<H: HugrView<Node = Node>>(
        circ: &Circuit<H>,
    ) -> impl Iterator<Item = Node> + '_ {
        circ.toposorted_children(circ.parent())
            .expect("circuit entrypoint should be dataflow region")
    }

    #[rstest]
    #[case::global_helios(PassScope::Global(Preserve::Public), QSystemPlatform::Helios)]
    #[case::entrypoint_flat_helios(PassScope::EntrypointFlat, QSystemPlatform::Helios)]
    #[case::entrypoint_recursive_helios(PassScope::EntrypointRecursive, QSystemPlatform::Helios)]
    #[case::global_sol(PassScope::Global(Preserve::Public), QSystemPlatform::Sol)]
    #[case::entrypoint_flat_sol(PassScope::EntrypointFlat, QSystemPlatform::Sol)]
    #[case::entrypoint_recursive_sol(PassScope::EntrypointRecursive, QSystemPlatform::Sol)]
    fn test_lower_direct(#[case] scope: PassScope, #[case] platform: QSystemPlatform) {
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

        let [r] = b
            .add_dataflow_op(TketOp::MeasureFree, [q])
            .unwrap()
            .outputs_arr();
        let [_] = b
            .add_dataflow_op(MeasurementOp::Read, [r])
            .unwrap()
            .outputs_arr();
        let mut h = b
            .finish_hugr_with_outputs([])
            .unwrap_or_else(|e| panic!("{}", e));

        let lowered = lower_tk2_ops(&mut h, scope.clone(), platform).unwrap();
        h.validate().unwrap();
        assert_eq!(lowered.len(), 5);
        let circ = Circuit::new(&h);
        let ops: Vec<ExpectedOp> = circ
            .toposorted_children(circ.parent())
            .expect("circuit entrypoint should be dataflow region")
            .filter_map(|n| ExpectedOp::cast(circ.hugr().get_optype(n), platform))
            .collect();
        assert_eq!(
            ops,
            [
                SharedOp::TryQAlloc,
                SharedOp::LazyMeasure,
                SharedOp::TryQAlloc,
                SharedOp::Reset,
                SharedOp::QFree
            ]
            .into_iter()
            .map(|s| ExpectedOp::from_shared(s, platform))
            .collect::<Vec<_>>()
        );
        assert_eq!(
            check_lowered(&h, scope, &forbidden_extensions_for(platform)),
            Ok(())
        );
    }

    #[rstest]
    #[case(TketOp::H, QSystemPlatform::Helios, Some(vec![ExpectedOp::Helios(HeliosOp::PhasedX), ExpectedOp::Helios(HeliosOp::Rz)]))]
    #[case(TketOp::X, QSystemPlatform::Helios, Some(vec![ExpectedOp::Helios(HeliosOp::PhasedX)]))]
    #[case(TketOp::Y, QSystemPlatform::Helios, Some(vec![ExpectedOp::Helios(HeliosOp::PhasedX)]))]
    #[case(TketOp::Z, QSystemPlatform::Helios, Some(vec![ExpectedOp::Helios(HeliosOp::Rz)]))]
    #[case(TketOp::S, QSystemPlatform::Helios, Some(vec![ExpectedOp::Helios(HeliosOp::Rz)]))]
    #[case(TketOp::Sdg, QSystemPlatform::Helios, Some(vec![ExpectedOp::Helios(HeliosOp::Rz)]))]
    #[case(TketOp::V, QSystemPlatform::Helios, Some(vec![ExpectedOp::Helios(HeliosOp::PhasedX)]))]
    #[case(TketOp::Vdg, QSystemPlatform::Helios, Some(vec![ExpectedOp::Helios(HeliosOp::PhasedX)]))]
    #[case(TketOp::T, QSystemPlatform::Helios, Some(vec![ExpectedOp::Helios(HeliosOp::Rz)]))]
    #[case(TketOp::Tdg, QSystemPlatform::Helios, Some(vec![ExpectedOp::Helios(HeliosOp::Rz)]))]
    #[case(TketOp::Rx, QSystemPlatform::Helios, Some(vec![ExpectedOp::Helios(HeliosOp::PhasedX)]))]
    #[case(TketOp::Ry, QSystemPlatform::Helios, Some(vec![ExpectedOp::Helios(HeliosOp::PhasedX)]))]
    #[case(TketOp::Rz, QSystemPlatform::Helios, Some(vec![ExpectedOp::Helios(HeliosOp::Rz)]))]
    #[case(TketOp::H, QSystemPlatform::Sol, Some(vec![ExpectedOp::Sol(SolOp::PhasedX), ExpectedOp::Sol(SolOp::Rz)]))]
    #[case(TketOp::X, QSystemPlatform::Sol, Some(vec![ExpectedOp::Sol(SolOp::PhasedX)]))]
    #[case(TketOp::Y, QSystemPlatform::Sol, Some(vec![ExpectedOp::Sol(SolOp::PhasedX)]))]
    #[case(TketOp::Z, QSystemPlatform::Sol, Some(vec![ExpectedOp::Sol(SolOp::Rz)]))]
    #[case(TketOp::S, QSystemPlatform::Sol, Some(vec![ExpectedOp::Sol(SolOp::Rz)]))]
    #[case(TketOp::Sdg, QSystemPlatform::Sol, Some(vec![ExpectedOp::Sol(SolOp::Rz)]))]
    #[case(TketOp::V, QSystemPlatform::Sol, Some(vec![ExpectedOp::Sol(SolOp::PhasedX)]))]
    #[case(TketOp::Vdg, QSystemPlatform::Sol, Some(vec![ExpectedOp::Sol(SolOp::PhasedX)]))]
    #[case(TketOp::T, QSystemPlatform::Sol, Some(vec![ExpectedOp::Sol(SolOp::Rz)]))]
    #[case(TketOp::Tdg, QSystemPlatform::Sol, Some(vec![ExpectedOp::Sol(SolOp::Rz)]))]
    #[case(TketOp::Rx, QSystemPlatform::Sol, Some(vec![ExpectedOp::Sol(SolOp::PhasedX)]))]
    #[case(TketOp::Ry, QSystemPlatform::Sol, Some(vec![ExpectedOp::Sol(SolOp::PhasedX)]))]
    #[case(TketOp::Rz, QSystemPlatform::Sol, Some(vec![ExpectedOp::Sol(SolOp::Rz)]))]
    // multi qubit ordering is not deterministic
    #[case(TketOp::CX, QSystemPlatform::Helios, None)]
    #[case(TketOp::CY, QSystemPlatform::Helios, None)]
    #[case(TketOp::CZ, QSystemPlatform::Helios, None)]
    #[case(TketOp::CRz, QSystemPlatform::Helios, None)]
    #[case(TketOp::Toffoli, QSystemPlatform::Helios, None)]
    // Uncomment when rebasing is added
    //#[case(TketOp::CX, QSystemPlatform::Helios, None)]
    //#[case(TketOp::CY, QSystemPlatform::Helios, None)]
    //#[case(TketOp::CZ, QSystemPlatform::Helios, None)]
    //#[case(TketOp::CRz, QSystemPlatform::Helios, None)]
    //#[case(TketOp::Toffoli, QSystemPlatform::Helios, None)]

    // conditional doesn't fit in to commands
    #[case(TketOp::Measure, QSystemPlatform::Helios, None)]
    #[case(TketOp::QAlloc, QSystemPlatform::Helios, None)]
    #[case(TketOp::Measure, QSystemPlatform::Sol, None)]
    #[case(TketOp::QAlloc, QSystemPlatform::Sol, None)]
    fn test_lower(
        #[case] t2op: TketOp,
        #[case] platform: QSystemPlatform,
        #[case] qsystem_ops: Option<Vec<ExpectedOp>>,
    ) {
        // build dfg with just the op

        let h = build_func(platform, t2op).unwrap();
        let circ = Circuit::new(&h);
        let nodes = toposorted_circuit_nodes(&circ);
        let ops: Vec<ExpectedOp> = match platform {
            QSystemPlatform::Helios => nodes
                .filter_map(|node| circ.hugr().get_optype(node).cast().map(ExpectedOp::Helios))
                .collect(),
            QSystemPlatform::Sol => nodes
                .filter_map(|node| circ.hugr().get_optype(node).cast().map(ExpectedOp::Sol))
                .collect(),
        };
        if let Some(qsystem_ops) = qsystem_ops {
            assert_eq!(ops, qsystem_ops);
        }

        assert_eq!(
            check_lowered(&h, Preserve::Public, &forbidden_extensions_for(platform)),
            Ok(())
        );
    }

    #[test]
    fn test_build_func_uses_platform_specific_ops() {
        let helios = build_func(QSystemPlatform::Helios, TketOp::CX).unwrap();
        let helios_circuit = Circuit::new(&helios);
        assert!(
            toposorted_circuit_nodes(&helios_circuit).any(|node| matches!(
                helios_circuit.hugr().get_optype(node).cast(),
                Some(HeliosOp::ZZPhase)
            ))
        );
        assert!(
            !toposorted_circuit_nodes(&helios_circuit).any(|node| matches!(
                helios_circuit.hugr().get_optype(node).cast(),
                Some(SolOp::PhasedXX)
            ))
        );

        let sol = build_func(QSystemPlatform::Sol, TketOp::CX).unwrap();
        let sol_circuit = Circuit::new(&sol);
        assert!(toposorted_circuit_nodes(&sol_circuit).any(|node| matches!(
            sol_circuit.hugr().get_optype(node).cast(),
            Some(SolOp::PhasedXX)
        )));
        assert!(!toposorted_circuit_nodes(&sol_circuit).any(|node| matches!(
            sol_circuit.hugr().get_optype(node).cast(),
            Some(HeliosOp::ZZPhase)
        )));
    }

    #[rstest]
    #[case::global_helios(PassScope::Global(Preserve::Public), QSystemPlatform::Helios)]
    #[case::entrypoint_flat_helios(PassScope::EntrypointFlat, QSystemPlatform::Helios)]
    #[case::entrypoint_recursive_helios(PassScope::EntrypointRecursive, QSystemPlatform::Helios)]
    #[case::global_sol(PassScope::Global(Preserve::Public), QSystemPlatform::Sol)]
    #[case::entrypoint_flat_sol(PassScope::EntrypointFlat, QSystemPlatform::Sol)]
    #[case::entrypoint_recursive_sol(PassScope::EntrypointRecursive, QSystemPlatform::Sol)]
    fn test_mixed(#[case] scope: PassScope, #[case] platform: QSystemPlatform) {
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

        let lowered = lower_tk2_ops(&mut h, scope.clone(), platform).unwrap();

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

        assert_eq!(
            check_lowered(&h, scope, &forbidden_extensions_for(platform)),
            Ok(())
        );
        if let Err(e) = h.validate() {
            panic!("{}", e);
        }
    }

    fn legacy_qsystem_hugr() -> hugr::Hugr {
        let mut b = FunctionBuilder::new("f", Signature::new_endo(type_row![])).unwrap();
        let [maybe_q] = b
            .add_dataflow_op(legacy_op("TryQAlloc"), [])
            .unwrap()
            .outputs_arr();
        let [q] = b
            .build_unwrap_sum(1, option_type(vec![qb_t()]), maybe_q)
            .unwrap();
        let [q] = b
            .add_dataflow_op(legacy_op("Reset"), [q])
            .unwrap()
            .outputs_arr();
        let [maybe_q2] = b
            .add_dataflow_op(legacy_op("TryQAlloc"), [])
            .unwrap()
            .outputs_arr();
        let [q2] = b
            .build_unwrap_sum(1, option_type(vec![qb_t()]), maybe_q2)
            .unwrap();
        let angle = const_f64(&mut b, 1.0);
        let [q, q2] = b
            .add_dataflow_op(legacy_op("ZZPhase"), [q, q2, angle])
            .unwrap()
            .outputs_arr();
        b.add_dataflow_op(legacy_op("QFree"), [q]).unwrap();
        b.add_dataflow_op(legacy_op("QFree"), [q2]).unwrap();
        b.finish_hugr_with_outputs([]).unwrap()
    }

    /// Build a HUGR containing legacy `tket.qsystem` ops (the old combined
    /// extension) and verify they are migrated to `tket.qsystem.helios` ops
    /// by [`lower_tk2_ops`].
    #[test]
    fn test_migrate_legacy_qsystem_ops() {
        let mut h = legacy_qsystem_hugr();

        // Sanity-check: legacy ops are present before lowering.
        let legacy_exts = ExtensionSet::from_iter([qsystem::EXTENSION_ID]);
        assert!(check_lowered(&h, Preserve::Public, &legacy_exts).is_err());

        lower_tk2_ops(&mut h, Preserve::Public, QSystemPlatform::Helios).unwrap();

        // No tket.qsystem ops should remain after lowering.
        assert_eq!(check_lowered(&h, Preserve::Public, &legacy_exts), Ok(()));

        // The migrated ops should be tket.qsystem.helios variants.
        let circ = Circuit::new(&h);
        let helios_ops: Vec<HeliosOp> = toposorted_circuit_nodes(&circ)
            .filter_map(|node| circ.hugr().get_optype(node).cast())
            .collect();
        assert_eq!(
            helios_ops,
            vec![
                HeliosOp::TryQAlloc,
                HeliosOp::TryQAlloc,
                HeliosOp::Reset,
                HeliosOp::ZZPhase,
                HeliosOp::QFree,
                HeliosOp::QFree,
            ],
        );
    }

    /// Legacy `tket.qsystem::ZZPhase` ops targeting Sol are lowered via the
    /// cross-platform decomposition path (issue #1620).
    #[test]
    fn test_legacy_qsystem_zz_phase_lowers_via_cross_platform_to_sol() {
        let mut h = legacy_qsystem_hugr();
        lower_tk2_ops(&mut h, Preserve::Public, QSystemPlatform::Sol).unwrap();
        assert_eq!(
            check_lowered(
                &h,
                Preserve::Public,
                &forbidden_extensions_for(QSystemPlatform::Sol)
            ),
            Ok(())
        );
    }

    /// A `tket.qsystem.helios::ZZPhase` op targeting Sol is lowered via the
    /// cross-platform decomposition path (issue #1620).
    #[test]
    fn test_helios_zz_phase_lowers_to_sol() {
        let mut b = FunctionBuilder::new("f", Signature::new_endo(type_row![])).unwrap();
        let [maybe_q1] = b
            .add_dataflow_op(HeliosOp::TryQAlloc, [])
            .unwrap()
            .outputs_arr();
        let [q1] = b
            .build_unwrap_sum(1, option_type(vec![qb_t()]), maybe_q1)
            .unwrap();
        let [maybe_q2] = b
            .add_dataflow_op(HeliosOp::TryQAlloc, [])
            .unwrap()
            .outputs_arr();
        let [q2] = b
            .build_unwrap_sum(1, option_type(vec![qb_t()]), maybe_q2)
            .unwrap();
        let angle = const_f64(&mut b, 1.0);
        let [q1, q2] = b
            .add_dataflow_op(HeliosOp::ZZPhase, [q1, q2, angle])
            .unwrap()
            .outputs_arr();
        b.add_dataflow_op(HeliosOp::QFree, [q1]).unwrap();
        b.add_dataflow_op(HeliosOp::QFree, [q2]).unwrap();
        let mut h = b.finish_hugr_with_outputs([]).unwrap();

        lower_tk2_ops(&mut h, Preserve::Public, QSystemPlatform::Sol).unwrap();
        assert_eq!(
            check_lowered(
                &h,
                Preserve::Public,
                &forbidden_extensions_for(QSystemPlatform::Sol)
            ),
            Ok(())
        );
        // TryQAlloc and QFree are shared ops; assert they were remapped to their
        // Sol equivalents (exercises the Ok(shared) arm of apply_cross_platform_helios).
        let circ = Circuit::new(&h);
        let sol_ops: Vec<SolOp> = toposorted_circuit_nodes(&circ)
            .filter_map(|node| circ.hugr().get_optype(node).cast())
            .collect();
        assert!(sol_ops.contains(&SolOp::TryQAlloc));
        assert!(sol_ops.contains(&SolOp::QFree));
        // ZZPhase must have been decomposed into PhasedXX (inside the replacement function).
        // ZZPhase decomposes into a called function, so scan all hugr nodes.
        assert!(
            h.nodes()
                .any(|n| h.get_optype(n).cast() == Some(SolOp::PhasedXX))
        );
    }

    /// A `tket.qsystem.sol::PhasedXX` op targeting Helios is lowered via the
    /// cross-platform decomposition path.
    #[test]
    fn test_sol_phased_xx_lowers_to_helios() {
        let mut b = FunctionBuilder::new("f", Signature::new_endo(type_row![])).unwrap();
        let [maybe_q1] = b
            .add_dataflow_op(SolOp::TryQAlloc, [])
            .unwrap()
            .outputs_arr();
        let [q1] = b
            .build_unwrap_sum(1, option_type(vec![qb_t()]), maybe_q1)
            .unwrap();
        let [maybe_q2] = b
            .add_dataflow_op(SolOp::TryQAlloc, [])
            .unwrap()
            .outputs_arr();
        let [q2] = b
            .build_unwrap_sum(1, option_type(vec![qb_t()]), maybe_q2)
            .unwrap();
        let angle1 = const_f64(&mut b, 0.5);
        let angle2 = const_f64(&mut b, 0.25);
        let [q1, q2] = b
            .add_dataflow_op(SolOp::PhasedXX, [q1, q2, angle1, angle2])
            .unwrap()
            .outputs_arr();
        b.add_dataflow_op(SolOp::QFree, [q1]).unwrap();
        b.add_dataflow_op(SolOp::QFree, [q2]).unwrap();
        let mut h = b.finish_hugr_with_outputs([]).unwrap();

        lower_tk2_ops(&mut h, Preserve::Public, QSystemPlatform::Helios).unwrap();
        assert_eq!(
            check_lowered(
                &h,
                Preserve::Public,
                &forbidden_extensions_for(QSystemPlatform::Helios)
            ),
            Ok(())
        );
        // TryQAlloc and QFree are shared ops; assert they were remapped to their
        // Helios equivalents (exercises the Ok(shared) arm of apply_cross_platform_sol).
        let circ = Circuit::new(&h);
        let helios_ops: Vec<HeliosOp> = toposorted_circuit_nodes(&circ)
            .filter_map(|node| circ.hugr().get_optype(node).cast())
            .collect();
        assert!(helios_ops.contains(&HeliosOp::TryQAlloc));
        assert!(helios_ops.contains(&HeliosOp::QFree));
        // PhasedXX must have been decomposed into ZZPhase (inside the replacement function).
        // PhasedXX decomposes into a called function, so scan all hugr nodes.
        assert!(
            h.nodes()
                .any(|n| h.get_optype(n).cast() == Some(HeliosOp::ZZPhase))
        );
    }

    /// Legacy `tket.qsystem` ops that correspond to [`SharedOp`] variants
    /// (e.g. `Reset`, `TryQAlloc`) can be lowered to Sol directly. Helios-specific
    /// ops are remapped via the cross-platform decomposition path.
    #[test]
    fn test_legacy_shared_qsystem_ops_lower_to_sol() {
        // Build a HUGR with only shared legacy ops (no ZZPhase).
        let mut b = FunctionBuilder::new("f", Signature::new_endo(type_row![])).unwrap();
        let [maybe_q] = b
            .add_dataflow_op(legacy_op("TryQAlloc"), [])
            .unwrap()
            .outputs_arr();
        let [q] = b
            .build_unwrap_sum(1, option_type(vec![qb_t()]), maybe_q)
            .unwrap();
        let [q] = b
            .add_dataflow_op(legacy_op("Reset"), [q])
            .unwrap()
            .outputs_arr();
        b.add_dataflow_op(legacy_op("QFree"), [q]).unwrap();
        let mut h = b.finish_hugr_with_outputs([]).unwrap();

        lower_tk2_ops(&mut h, Preserve::Public, QSystemPlatform::Sol).unwrap();

        // Legacy ops should have been replaced with Sol equivalents.
        let legacy_exts = ExtensionSet::from_iter([qsystem::EXTENSION_ID]);
        assert_eq!(check_lowered(&h, Preserve::Public, &legacy_exts), Ok(()));

        let circ = Circuit::new(&h);
        let sol_ops: Vec<SolOp> = toposorted_circuit_nodes(&circ)
            .filter_map(|node| circ.hugr().get_optype(node).cast())
            .collect();
        assert_eq!(sol_ops, vec![SolOp::TryQAlloc, SolOp::Reset, SolOp::QFree],);
    }

    /// Build a HUGR containing measurement ops (both from `tket.quantum` and
    /// `tket.qsystem`) and verify that it no longer contains any measurement types
    /// after the pass.
    #[rstest]
    #[case::helios(QSystemPlatform::Helios)]
    #[case::helios(QSystemPlatform::Sol)]
    fn test_measurements_removed(#[case] platform: QSystemPlatform) {
        let mut circuit = DFGBuilder::new(inout_sig(vec![qb_t(); 2], vec![bool_t()])).unwrap();
        let [q1, q2] = circuit.input_wires_arr();

        // MeasureFree
        let m1 = circuit
            .add_dataflow_op(TketOp::MeasureFree, [q1])
            .unwrap()
            .out_wire(0);

        // LazyMeasure
        let lazy_measure: OpType = match platform {
            QSystemPlatform::Helios => HeliosOp::LazyMeasure.into(),
            QSystemPlatform::Sol => SolOp::LazyMeasure.into(),
        };
        let f2 = circuit
            .add_dataflow_op(lazy_measure, [q2])
            .unwrap()
            .out_wire(0);

        // FutureToMeasurement
        let future_to_msmt: OpType = match platform {
            QSystemPlatform::Helios => HeliosOp::FutureToMeasurement.into(),
            QSystemPlatform::Sol => SolOp::FutureToMeasurement.into(),
        };
        let m2 = circuit
            .add_dataflow_op(future_to_msmt, [f2])
            .unwrap()
            .out_wire(0);

        // Read both measurements
        let b1 = circuit
            .add_dataflow_op(MeasurementOp::Read, [m1])
            .unwrap()
            .out_wire(0);

        let _b2 = circuit
            .add_dataflow_op(MeasurementOp::Read, [m2])
            .unwrap()
            .out_wire(0);

        let mut h = circuit.finish_hugr_with_outputs([b1]).unwrap();
        h.validate().unwrap();

        lower_tk2_ops(&mut h, PassScope::Global(Preserve::Public), platform).unwrap();
        h.validate().unwrap();

        // Check no measurement types remain
        let sig = h.signature(h.entrypoint()).unwrap();
        assert!(!sig.input().iter().any(|t| t == &measurement_type()));
        assert!(!sig.output().iter().any(|t| t == &measurement_type()));

        assert!(
            !h.nodes()
                .filter_map(|n| h.get_optype(n).as_extension_op())
                .any(|op| MeasurementOp::from_op(op).is_ok())
        );
    }

    #[rstest]
    #[case(Array, QSystemPlatform::Helios)]
    #[case(BorrowArray, QSystemPlatform::Helios)]
    #[case(Array, QSystemPlatform::Sol)]
    #[case(BorrowArray, QSystemPlatform::Sol)]
    fn test_array_clone_discard_measurement<AK: ArrayKind>(
        #[case] _ak: AK,
        #[case] platform: QSystemPlatform,
    ) {
        let elem_ty = measurement_type();
        let size = 4;
        let arr_ty = AK::ty(size, elem_ty.clone());
        let mut dfb =
            DFGBuilder::new(inout_sig(vec![arr_ty.clone()], vec![arr_ty.clone()])).unwrap();
        let [arr_in] = dfb.input_wires_arr();
        let (arr1, arr2) = dfb
            .add_generic_array_clone::<AK>(elem_ty.clone(), size, arr_in)
            .unwrap();
        dfb.add_generic_array_discard::<AK>(elem_ty, size, arr2)
            .unwrap();
        let mut h = dfb.finish_hugr_with_outputs([arr1]).unwrap();

        h.validate().unwrap();
        lower_tk2_ops(&mut h, PassScope::Global(Preserve::Public), platform).unwrap();
        h.validate().unwrap();

        let sig = h.signature(h.entrypoint()).unwrap();
        let future_arr_ty = &TypeRow::from(vec![AK::ty(size, future_type(bool_t()))]);
        assert_eq!(sig.input(), future_arr_ty);
        assert_eq!(sig.output(), future_arr_ty);
    }

    #[rstest]
    #[case(Type::new_tuple(vec![measurement_type(), usize_t()]), Type::new_tuple(vec![future_type(bool_t()), usize_t()]), true, QSystemPlatform::Helios)]
    #[case(
        measurement_type(),
        future_type(bool_t()),
        true,
        QSystemPlatform::Helios
    )]
    #[case(usize_t(), usize_t(), false, QSystemPlatform::Helios)]
    #[case(Type::new_tuple(vec![measurement_type(), usize_t()]), Type::new_tuple(vec![future_type(bool_t()), usize_t()]), true, QSystemPlatform::Sol)]
    #[case(measurement_type(), future_type(bool_t()), true, QSystemPlatform::Sol)]
    #[case(usize_t(), usize_t(), false, QSystemPlatform::Sol)]
    fn test_barray_get_measurement(
        #[case] src_ty: Type,
        #[case] dest_ty: Type,
        #[case] expect_dup: bool,
        #[case] platform: QSystemPlatform,
    ) {
        use hugr::std_extensions::collections::borrow_array::borrow_array_type;

        let arr_ty = borrow_array_type(4, src_ty.clone());
        let mut dfb = DFGBuilder::new(inout_sig(
            vec![arr_ty.clone(), usize_t()],
            vec![arr_ty, src_ty.clone()],
        ))
        .unwrap();
        let [arr_in, idx] = dfb.input_wires_arr();
        let (opt_elem, arr) = dfb
            .add_borrow_array_get(src_ty.clone(), 4, arr_in, idx)
            .unwrap();
        let [elem] = dfb
            .build_unwrap_sum(1, option_type(vec![src_ty.clone()]), opt_elem)
            .unwrap();
        let mut h = dfb.finish_hugr_with_outputs([arr, elem]).unwrap();

        h.validate().unwrap();
        lower_tk2_ops(&mut h, PassScope::Global(Preserve::Public), platform).unwrap();
        h.validate().unwrap();

        let sig = h.signature(h.entrypoint()).unwrap();
        let dest_arr_ty = borrow_array_type(4, dest_ty.clone());
        assert_eq!(
            sig.as_ref(),
            &inout_sig(
                vec![dest_arr_ty.clone(), usize_t()],
                vec![dest_arr_ty, dest_ty]
            )
        );
        let contains_dup = h.nodes().any(|n| {
            h.get_optype(n).as_extension_op().is_some_and(|eop| {
                FutureOp::from_op(eop).is_ok_and(|fop| fop.op == FutureOpDef::Dup)
            })
        });
        assert_eq!(contains_dup, expect_dup);
    }
}
