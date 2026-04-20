//! This module defines the Hugr extension used to represent H-series
//! quantum operations.
//!
//! In the case of lazy operations,
//! laziness is represented by returning `tket.futures.Future` classical
//! values. Qubits are never lazy.
use std::{str::FromStr, sync::Arc};

use delegate::delegate;
use hugr::{
    Extension, Hugr, Node, Wire,
    builder::{BuildError, Container, Dataflow, DataflowHugr, HugrBuilder},
    extension::{
        ExtensionId, OpDef, SignatureFunc, Version,
        prelude::qb_t,
        simple_op::{MakeOpDef, MakeRegisteredOp, try_from_name},
    },
    ops::Value,
    std_extensions::arithmetic::{
        float_ops::FloatOps,
        float_types::{ConstF64, float64_type},
    },
    types::{Signature, TypeRow},
};

use super::common::{self, CommonOp, CommonOpBuilder, SharedOp};
use super::lower::pi_mul_f64;
use super::synth_tket_op::SynthesizeTketOp;
use derive_more::Display;
use lazy_static::lazy_static;
use strum::{EnumIter, EnumString, IntoStaticStr};

/// The "tket.qsystem.helios" extension id.
pub const EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.qsystem.helios");
/// The "tket.qsystem.helios" extension version.
pub const EXTENSION_VERSION: Version = Version::new(0, 5, 1);

lazy_static! {
    /// The "tket.qsystem.helios" extension.
    pub static ref EXTENSION: Arc<Extension> = {
         Extension::new_arc(EXTENSION_ID, EXTENSION_VERSION, |ext, ext_ref| {
            HeliosOp::load_all_ops( ext, ext_ref).unwrap();
            RuntimeBarrierDef.add_to_extension(ext, ext_ref).unwrap();
        })
    };

}

/// Quantum operations for Quantinuum Helios quantum computer.
#[derive(
    Clone,
    Copy,
    Debug,
    serde::Serialize,
    serde::Deserialize,
    Hash,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    EnumIter,
    IntoStaticStr,
    EnumString,
    Display,
)]
#[non_exhaustive]
pub enum HeliosOp {
    /// Measure a qubit and lose it.
    Measure,
    /// Lazily measure a qubit and lose it.
    LazyMeasure,
    /// Lazily measure a qubit and reset it to the Z |0> eigenstate.
    LazyMeasureReset,
    /// Rotate a qubit around the Z axis, not physical (alias 'rz')
    Rz,
    /// PhasedX gate (aliases 'rxy', 'rp').
    PhasedX,
    /// ZZ gate with an angle (alias 'rzz').
    ZZPhase,
    /// Allocate a qubit in the Z |0> eigenstate.
    TryQAlloc,
    /// Free a qubit (lose track of it).
    QFree,
    /// Reset a qubit to the Z |0> eigenstate.
    Reset,
    /// Measure a qubit and reset it to the Z |0> eigenstate.
    MeasureReset,
    /// Measure a qubit (return 0 or 1) or detect leakage (return 2).
    LazyMeasureLeaked,
}

impl MakeOpDef for HeliosOp {
    fn opdef_id(&self) -> hugr::ops::OpName {
        <&'static str>::from(self).into()
    }

    fn init_signature(&self, _extension_ref: &std::sync::Weak<Extension>) -> SignatureFunc {
        if let Ok(shared_op) = SharedOp::try_from(*self) {
            shared_op.signature()
        } else {
            // For Helios-specific ops, provide custom signatures.
            match self {
                HeliosOp::ZZPhase => Signature::new(
                    vec![qb_t(), qb_t(), float64_type()],
                    TypeRow::from(vec![qb_t(), qb_t()]),
                )
                .into(),
                _ => unreachable!("All other HeliosOps should have been convertible to SharedOps."),
            }
        }
    }

    fn from_def(op_def: &OpDef) -> Result<Self, hugr::extension::simple_op::OpLoadError> {
        try_from_name(op_def.name(), op_def.extension_id())
    }

    fn extension(&self) -> ExtensionId {
        EXTENSION_ID
    }

    fn extension_ref(&self) -> std::sync::Weak<Extension> {
        Arc::downgrade(&EXTENSION)
    }

    fn description(&self) -> String {
        if let Ok(shared_op) = SharedOp::try_from(*self) {
            shared_op.description()
        } else {
            // For Helios-specific ops, provide custom descriptions.
            match self {
                HeliosOp::ZZPhase => "ZZ gate with an angle, specific to the Helios platform.",
                _ => unreachable!("All other HeliosOps should have been convertible to SharedOps."),
            }
        }
        .to_string()
    }
}

impl MakeRegisteredOp for HeliosOp {
    fn extension_id(&self) -> ExtensionId {
        EXTENSION_ID
    }

    fn extension_ref(&self) -> Arc<Extension> {
        EXTENSION.clone()
    }
}

impl TryFrom<HeliosOp> for SharedOp {
    type Error = &'static str;

    fn try_from(helios_op: HeliosOp) -> Result<Self, Self::Error> {
        use HeliosOp::*;
        match helios_op {
            Measure => Ok(SharedOp::Measure),
            LazyMeasure => Ok(SharedOp::LazyMeasure),
            Rz => Ok(SharedOp::Rz),
            PhasedX => Ok(SharedOp::PhasedX),
            TryQAlloc => Ok(SharedOp::TryQAlloc),
            QFree => Ok(SharedOp::QFree),
            Reset => Ok(SharedOp::Reset),
            MeasureReset => Ok(SharedOp::MeasureReset),
            LazyMeasureLeaked => Ok(SharedOp::LazyMeasureLeaked),
            LazyMeasureReset => Ok(SharedOp::LazyMeasureReset),
            ZZPhase => Err("Helios-specific ops don't have a corresponding SharedOp."),
        }
    }
}

impl From<SharedOp> for HeliosOp {
    fn from(shared_op: SharedOp) -> Self {
        use SharedOp::*;
        match shared_op {
            Measure => HeliosOp::Measure,
            LazyMeasure => HeliosOp::LazyMeasure,
            Rz => HeliosOp::Rz,
            PhasedX => HeliosOp::PhasedX,
            TryQAlloc => HeliosOp::TryQAlloc,
            QFree => HeliosOp::QFree,
            Reset => HeliosOp::Reset,
            MeasureReset => HeliosOp::MeasureReset,
            LazyMeasureLeaked => HeliosOp::LazyMeasureLeaked,
            LazyMeasureReset => HeliosOp::LazyMeasureReset,
        }
    }
}
impl CommonOp for HeliosOp {}
/// The name of the "tket.qsystem.RuntimeBarrier" operation.
pub const RUNTIME_BARRIER_NAME: hugr::ops::OpName = common::RUNTIME_BARRIER_NAME;

/// Helper struct for the "tket.qsystem.RuntimeBarrier" operation definition.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RuntimeBarrierDef;

impl FromStr for RuntimeBarrierDef {
    type Err = ();

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        common::runtime_barrier_from_str(s).map(|()| Self)
    }
}

impl MakeOpDef for RuntimeBarrierDef {
    fn from_def(op_def: &OpDef) -> Result<Self, hugr::extension::simple_op::OpLoadError>
    where
        Self: Sized,
    {
        try_from_name(op_def.name(), op_def.extension_id())
    }

    fn extension(&self) -> ExtensionId {
        EXTENSION_ID
    }

    fn extension_ref(&self) -> std::sync::Weak<Extension> {
        Arc::downgrade(&EXTENSION)
    }

    fn init_signature(
        &self,
        _extension_ref: &std::sync::Weak<Extension>,
    ) -> hugr::extension::SignatureFunc {
        common::runtime_barrier_signature()
    }

    fn description(&self) -> String {
        common::runtime_barrier_description()
    }

    fn opdef_id(&self) -> hugr::ops::OpName {
        RUNTIME_BARRIER_NAME
    }
}

/// An extension trait for [Dataflow] providing methods to add
/// "tket.qsystem.helios" operations.
pub trait HeliosOpBuilder: CommonOpBuilder {
    /// Add a "tket.qsystem.helios.LazyMeasure" op.
    #[deprecated(since = "0.25.0", note = "Use the SynthesizeHeliosOp trait instead.")]
    fn add_lazy_measure(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::add_lazy_measure_with::<HeliosOp>(self, qb)
    }

    /// Add a "tket.qsystem.helios.LazyMeasureLeaked" op.
    #[deprecated(since = "0.25.0", note = "Use the SynthesizeHeliosOp trait instead.")]
    fn add_lazy_measure_leaked(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::add_lazy_measure_leaked_with::<HeliosOp>(self, qb)
    }

    /// Add a "tket.qsystem.helios.LazyMeasureReset" op.
    #[deprecated(since = "0.25.0", note = "Use the SynthesizeHeliosOp trait instead.")]
    fn add_lazy_measure_reset(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError> {
        CommonOpBuilder::add_lazy_measure_reset_with::<HeliosOp>(self, qb)
    }

    /// Add a "tket.qsystem.helios.Measure" op.
    #[deprecated(since = "0.25.0", note = "Use the SynthesizeHeliosOp trait instead.")]
    fn add_measure(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::add_measure_with::<HeliosOp>(self, qb)
    }

    /// Add a "tket.qsystem.helios.Reset" op.
    #[deprecated(since = "0.25.0", note = "Use the SynthesizeHeliosOp trait instead.")]
    fn add_reset(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::add_reset_with::<HeliosOp>(self, qb)
    }

    /// Add a maximally entangling "tket.qsystem.ZZPhase(pi/2)" op.
    #[expect(deprecated)]
    fn build_zz_max(&mut self, qb1: Wire, qb2: Wire) -> Result<[Wire; 2], BuildError> {
        let pi_2 = pi_mul_f64(self, 0.5);
        self.add_zz_phase(qb1, qb2, pi_2)
    }

    /// Add a "tket.qsystem.helios.ZZPhase" op.
    #[deprecated(since = "0.25.0", note = "Use the SynthesizeHeliosOp trait instead.")]
    fn add_zz_phase(&mut self, qb1: Wire, qb2: Wire, angle: Wire) -> Result<[Wire; 2], BuildError> {
        Ok(self
            .add_dataflow_op(HeliosOp::ZZPhase, [qb1, qb2, angle])?
            .outputs_arr())
    }

    /// Add a "tket.qsystem.helios.PhasedX" op.
    #[deprecated(since = "0.25.0", note = "Use the SynthesizeHeliosOp trait instead.")]
    fn add_phased_x(&mut self, qb: Wire, angle1: Wire, angle2: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::add_phased_x_with::<HeliosOp>(self, qb, angle1, angle2)
    }

    /// Add a "tket.qsystem.helios.Rz" op.
    #[deprecated(since = "0.25.0", note = "Use the SynthesizeHeliosOp trait instead.")]
    fn add_rz(&mut self, qb: Wire, angle: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::add_rz_with::<HeliosOp>(self, qb, angle)
    }

    /// Add a "tket.qsystem.helios.TryQAlloc" op.
    #[deprecated(since = "0.25.0", note = "Use the SynthesizeHeliosOp trait instead.")]
    fn add_try_alloc(&mut self) -> Result<Wire, BuildError> {
        CommonOpBuilder::add_try_alloc_with::<HeliosOp>(self)
    }

    /// Add a "tket.qsystem.helios.QFree" op.
    #[deprecated(since = "0.25.0", note = "Use the SynthesizeHeliosOp trait instead.")]
    fn add_qfree(&mut self, qb: Wire) -> Result<(), BuildError> {
        CommonOpBuilder::add_qfree_with::<HeliosOp>(self, qb)
    }

    /// Add a "tket.qsystem.helios.MeasureReset" op.
    /// This operation is equivalent to a `Measure` followed by a `Reset`.
    #[deprecated(since = "0.25.0", note = "Use the SynthesizeHeliosOp trait instead.")]
    fn add_measure_reset(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError> {
        CommonOpBuilder::add_measure_reset_with::<HeliosOp>(self, qb)
    }

    /// Add a "tket.qsystem.helios.RuntimeBarrier" op.
    #[deprecated(since = "0.25.0", note = "Use the SynthesizeHeliosOp trait instead.")]
    fn add_runtime_barrier(&mut self, qbs: Wire, array_size: u64) -> Result<Wire, BuildError> {
        CommonOpBuilder::add_runtime_barrier_with(self, &EXTENSION, qbs, array_size)
    }

    /// Build a hadamard gate in terms of QSystem primitives.
    #[deprecated(
        since = "0.25.0",
        note = "Use lowering through SynthesizeTketOp instead."
    )]
    fn build_h(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_h_with::<HeliosOp>(self, qb)
    }

    /// Build an X gate in terms of QSystem primitives.
    #[deprecated(
        since = "0.25.0",
        note = "Use lowering through SynthesizeTketOp instead."
    )]
    fn build_x(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_x_with::<HeliosOp>(self, qb)
    }

    /// Build a Y gate in terms of QSystem primitives.
    #[deprecated(
        since = "0.25.0",
        note = "Use lowering through SynthesizeTketOp instead."
    )]
    fn build_y(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_y_with::<HeliosOp>(self, qb)
    }

    /// Build a Z gate in terms of QSystem primitives.
    #[deprecated(
        since = "0.25.0",
        note = "Use lowering through SynthesizeTketOp instead."
    )]
    fn build_z(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_z_with::<HeliosOp>(self, qb)
    }

    /// Build an S gate in terms of QSystem primitives.
    #[deprecated(
        since = "0.25.0",
        note = "Use lowering through SynthesizeTketOp instead."
    )]
    fn build_s(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_s_with::<HeliosOp>(self, qb)
    }

    /// Build an Sdg gate in terms of QSystem primitives.
    #[deprecated(
        since = "0.25.0",
        note = "Use lowering through SynthesizeTketOp instead."
    )]
    fn build_sdg(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_sdg_with::<HeliosOp>(self, qb)
    }

    /// Build a V gate in terms of QSystem primitives.
    #[deprecated(
        since = "0.25.0",
        note = "Use lowering through SynthesizeTketOp instead."
    )]
    fn build_v(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_v_with::<HeliosOp>(self, qb)
    }

    /// Build a Vdg gate in terms of QSystem primitives.
    #[deprecated(
        since = "0.25.0",
        note = "Use lowering through SynthesizeTketOp instead."
    )]
    fn build_vdg(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_vdg_with::<HeliosOp>(self, qb)
    }

    /// Build a T gate in terms of QSystem primitives.
    #[deprecated(
        since = "0.25.0",
        note = "Use lowering through SynthesizeTketOp instead."
    )]
    fn build_t(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_t_with::<HeliosOp>(self, qb)
    }

    /// Build a Tdg gate in terms of QSystem primitives.
    #[deprecated(
        since = "0.25.0",
        note = "Use lowering through SynthesizeTketOp instead."
    )]
    fn build_tdg(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_tdg_with::<HeliosOp>(self, qb)
    }

    /// Build a CNOT gate in terms of QSystem primitives.
    #[expect(deprecated)]
    #[deprecated(
        since = "0.25.0",
        note = "Use lowering through SynthesizeTketOp instead."
    )]
    fn build_cx(&mut self, c: Wire, t: Wire) -> Result<[Wire; 2], BuildError> {
        let pi = pi_mul_f64(self, 1.0);
        let pi_2 = pi_mul_f64(self, 0.5);
        let pi_minus_2 = pi_mul_f64(self, -0.5);

        let t = self.add_phased_x(t, pi_minus_2, pi_2)?;
        let [c, t] = self.build_zz_max(c, t)?;
        let c = self.add_rz(c, pi_minus_2)?;
        let t = self.add_phased_x(t, pi_2, pi)?;
        let t = self.add_rz(t, pi_minus_2)?;
        Ok([c, t])
    }

    /// Build a CY gate in terms of QSystem primitives.
    #[expect(deprecated)]
    #[deprecated(
        since = "0.25.0",
        note = "Use lowering through SynthesizeTketOp instead."
    )]
    fn build_cy(&mut self, a: Wire, b: Wire) -> Result<[Wire; 2], BuildError> {
        let pi = pi_mul_f64(self, 1.0);
        let pi_2 = pi_mul_f64(self, 0.5);
        let pi_minus_2 = pi_mul_f64(self, -0.5);

        let a = self.add_phased_x(a, pi, pi)?;
        let b = self.add_phased_x(b, pi_minus_2, pi)?;
        let [a, b] = self.build_zz_max(a, b)?;
        let a = self.add_phased_x(a, pi, pi_2)?;
        let b = self.add_phased_x(b, pi_minus_2, pi_minus_2)?;
        let a = self.add_rz(a, pi_minus_2)?;
        let b = self.add_rz(b, pi_2)?;
        Ok([a, b])
    }

    /// Build a CZ gate in terms of QSystem primitives.
    #[expect(deprecated)]
    #[deprecated(
        since = "0.25.0",
        note = "Use lowering through SynthesizeTketOp instead."
    )]
    fn build_cz(&mut self, a: Wire, b: Wire) -> Result<[Wire; 2], BuildError> {
        let pi_minus_2 = pi_mul_f64(self, -0.5);
        let [a, b] = self.build_zz_max(a, b)?;
        let b = self.add_rz(b, pi_minus_2)?;
        let a = self.add_rz(a, pi_minus_2)?;
        Ok([a, b])
    }

    /// Build a RX gate in terms of QSystem primitives.
    #[deprecated(
        since = "0.25.0",
        note = "Use lowering through SynthesizeTketOp instead."
    )]
    fn build_rx(&mut self, qb: Wire, theta: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_rx_with::<HeliosOp>(self, qb, theta)
    }

    /// Build a RY gate in terms of QSystem primitives.
    #[deprecated(
        since = "0.25.0",
        note = "Use lowering through SynthesizeTketOp instead."
    )]
    fn build_ry(&mut self, qb: Wire, theta: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_ry_with::<HeliosOp>(self, qb, theta)
    }

    /// Build a CRZ gate in terms of QSystem primitives.
    #[deprecated(
        since = "0.25.0",
        note = "Use lowering through SynthesizeTketOp instead."
    )]
    fn build_crz(&mut self, a: Wire, b: Wire, lambda: Wire) -> Result<[Wire; 2], BuildError> {
        let two = self.add_load_const(Value::from(ConstF64::new(2.0)));
        let lambda_2 = self
            .add_dataflow_op(FloatOps::fdiv, [lambda, two])?
            .out_wire(0);
        let lambda_minus_2 = self
            .add_dataflow_op(FloatOps::fneg, [lambda_2])?
            .out_wire(0);
        #[expect(deprecated)]
        let [a, b] = self.add_zz_phase(a, b, lambda_minus_2)?;
        #[expect(deprecated)]
        let b = self.add_rz(b, lambda_2)?;
        Ok([a, b])
    }

    /// Build a Toffoli (CCX) gate in terms of QSystem primitives.
    #[expect(deprecated)]
    #[deprecated(
        since = "0.25.0",
        note = "Use lowering through SynthesizeTketOp instead."
    )]
    fn build_toffoli(&mut self, a: Wire, b: Wire, c: Wire) -> Result<[Wire; 3], BuildError> {
        let pi = pi_mul_f64(self, 1.0);
        let pi_2 = pi_mul_f64(self, 0.5);
        let pi_minus_2 = pi_mul_f64(self, -0.5);
        let pi_4 = pi_mul_f64(self, 0.25);
        let pi_minus_4 = pi_mul_f64(self, -0.25);
        let pi_minus_3_4 = pi_mul_f64(self, -0.75);
        let zero = pi_mul_f64(self, 0.0);

        let c = self.add_phased_x(c, pi, pi_minus_2)?;
        let [b, c] = self.build_zz_max(b, c)?;
        let c = self.add_phased_x(c, pi_4, pi_2)?;
        let [a, c] = self.build_zz_max(a, c)?;
        let c = self.add_phased_x(c, pi_4, zero)?;
        let [b, c] = self.build_zz_max(b, c)?;
        let c = self.add_phased_x(c, pi_4, pi_minus_2)?;
        let [a, c] = self.build_zz_max(a, c)?;
        let a = self.add_phased_x(a, pi, pi_4)?;
        let c = self.add_phased_x(c, pi_minus_3_4, pi)?;
        let [a, b] = self.add_zz_phase(a, b, pi_4)?;
        let c = self.add_rz(c, pi)?;
        let a = self.add_phased_x(a, pi, pi_minus_4)?;
        let b = self.add_rz(b, pi_minus_3_4)?;
        let a = self.add_rz(a, pi_4)?;

        Ok([a, b, c])
    }

    /// Build a projective measurement with a conditional flip.
    #[deprecated(
        since = "0.25.0",
        note = "Use lowering through SynthesizeTketOp instead."
    )]
    fn build_measure_flip(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError> {
        CommonOpBuilder::build_measure_flip_with::<HeliosOp>(self, qb)
    }

    /// Build a qalloc operation that panics on failure.
    #[deprecated(
        since = "0.25.0",
        note = "Use lowering through SynthesizeTketOp instead."
    )]
    fn build_qalloc(&mut self) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_qalloc_with::<HeliosOp>(self)
    }

    /// Build an array from qubit wires, apply a barrier, and unwrap the array afterwards.
    fn build_wrapped_barrier(
        &mut self,
        qbs: impl IntoIterator<Item = Wire>,
    ) -> Result<Vec<Wire>, BuildError>
    where
        Self: Sized,
    {
        CommonOpBuilder::build_wrapped_barrier_with(self, &EXTENSION, qbs)
    }

    /// Build a "tket.qsystem.sol.PhasedXX" op in terms of Helios primitives.
    fn build_phased_xx(
        &mut self,
        _qb1: Wire,
        _qb2: Wire,
        _angle1: Wire,
        _angle2: Wire,
    ) -> Result<[Wire; 2], BuildError> {
        unimplemented!("PhasedXX lowering for Helios is not yet implemented")
    }

    /// Build a "tket.qsystem.sol.Tk2" op in terms of Helios primitives.
    fn build_tk2(
        &mut self,
        _qb1: Wire,
        _qb2: Wire,
        _angle1: Wire,
        _angle2: Wire,
        _angle3: Wire,
    ) -> Result<[Wire; 2], BuildError> {
        unimplemented!("Tk2 lowering for Helios is not yet implemented")
    }
}

impl<D: Dataflow> HeliosOpBuilder for D {}

#[derive(Debug)]
/// Implmements traits for lowering operations in terms of Helios primitives.
pub(super) struct HeliosBuilder<D> {
    inner: D,
}

impl<D> HeliosBuilder<D> {
    pub(super) fn new(inner: D) -> Self {
        Self { inner }
    }
}

impl<D> Container for HeliosBuilder<D>
where
    D: Container,
{
    delegate! {
        to self.inner {
            fn container_node(&self) -> Node;
            fn hugr_mut(&mut self) -> &mut Hugr;
            fn hugr(&self) -> &Hugr;
        }
    }
}

impl<D> HugrBuilder for HeliosBuilder<D>
where
    D: HugrBuilder,
{
    delegate! {
        to self.inner {
            fn finish_hugr(self) -> Result<Hugr, hugr::hugr::validate::ValidationError<Node>>;
        }
    }
}

impl<D> Dataflow for HeliosBuilder<D>
where
    D: Dataflow,
{
    delegate! {
        to self.inner {
            fn num_inputs(&self) -> usize;
        }
    }
}

#[expect(deprecated)]
impl<D> SynthesizeTketOp for HeliosBuilder<D>
where
    D: DataflowHugr + HeliosOpBuilder,
{
    delegate! {
        to self.inner {
            fn build_h(&mut self, qb: Wire) -> Result<Wire, BuildError>;
            fn build_x(&mut self, qb: Wire) -> Result<Wire, BuildError>;
            fn build_y(&mut self, qb: Wire) -> Result<Wire, BuildError>;
            fn build_z(&mut self, qb: Wire) -> Result<Wire, BuildError>;
            fn build_s(&mut self, qb: Wire) -> Result<Wire, BuildError>;
            fn build_sdg(&mut self, qb: Wire) -> Result<Wire, BuildError>;
            fn build_v(&mut self, qb: Wire) -> Result<Wire, BuildError>;
            fn build_vdg(&mut self, qb: Wire) -> Result<Wire, BuildError>;
            fn build_t(&mut self, qb: Wire) -> Result<Wire, BuildError>;
            fn build_tdg(&mut self, qb: Wire) -> Result<Wire, BuildError>;
            fn build_measure_flip(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError>;
            fn build_qalloc(&mut self) -> Result<Wire, BuildError>;
            fn build_cx(&mut self, c: Wire, t: Wire) -> Result<[Wire; 2], BuildError>;
            fn build_cy(&mut self, c: Wire, t: Wire) -> Result<[Wire; 2], BuildError>;
            fn build_cz(&mut self, c: Wire, t: Wire) -> Result<[Wire; 2], BuildError>;
            fn build_rx(&mut self, qb: Wire, theta: Wire) -> Result<Wire, BuildError>;
            fn build_ry(&mut self, qb: Wire, theta: Wire) -> Result<Wire, BuildError>;
            #[call(add_rz)]
            fn build_rz(&mut self, qb: Wire, theta: Wire) -> Result<Wire, BuildError>;
            fn build_crz(&mut self, c: Wire, t: Wire, theta: Wire) -> Result<[Wire; 2], BuildError>;
            fn build_toffoli(&mut self, a: Wire, b: Wire, c: Wire) -> Result<[Wire; 3], BuildError>;
        }
    }
}

/// Builder trait for lowering `HeliosOp`s into a target operation set.
pub(crate) trait SynthesizeHeliosOp: Dataflow {
    /// Build a "tket.qsystem.helios.LazyMeasure" op.
    fn build_lazy_measure(&mut self, qb: Wire) -> Result<Wire, BuildError>;

    /// Build a "tket.qsystem.helios.LazyMeasureLeaked" op.
    fn build_lazy_measure_leaked(&mut self, qb: Wire) -> Result<Wire, BuildError>;

    /// Build a "tket.qsystem.helios.LazyMeasureReset" op.
    fn build_lazy_measure_reset(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError>;

    /// Build a "tket.qsystem.helios.Measure" op.
    fn build_measure(&mut self, qb: Wire) -> Result<Wire, BuildError>;

    /// Build a "tket.qsystem.helios.Reset" op.
    fn build_reset(&mut self, qb: Wire) -> Result<Wire, BuildError>;

    /// Build a "tket.qsystem.helios.ZZPhase" op.
    fn build_zz_phase(
        &mut self,
        qb1: Wire,
        qb2: Wire,
        angle: Wire,
    ) -> Result<[Wire; 2], BuildError>;

    fn build_zz_max(&mut self, qb1: Wire, qb2: Wire) -> Result<[Wire; 2], BuildError> {
        let pi_2 = pi_mul_f64(self, 0.5);
        self.build_zz_phase(qb1, qb2, pi_2)
    }

    /// Build a "tket.qsystem.helios.PhasedX" op.
    fn build_phased_x(&mut self, qb: Wire, angle1: Wire, angle2: Wire) -> Result<Wire, BuildError>;

    /// Build a "tket.qsystem.helios.Rz" op.
    fn build_rz(&mut self, qb: Wire, angle: Wire) -> Result<Wire, BuildError>;
    /// Build a "tket.qsystem.helios.TryQAlloc" op.
    fn build_try_alloc(&mut self) -> Result<Wire, BuildError>;
    /// Build a "tket.qsystem.helios.QFree" op.
    fn build_qfree(&mut self, qb: Wire) -> Result<(), BuildError>;
    /// Build a "tket.qsystem.helios.MeasureReset" op.
    /// This operation is equivalent to a `Measure` followed by a `Reset`.
    fn build_measure_reset(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError>;

    /// Build a "tket.qsystem.helios.RuntimeBarrier" op.
    fn build_runtime_barrier(&mut self, qbs: Wire, array_size: u64) -> Result<Wire, BuildError>;
}

#[expect(deprecated)]
impl<D> SynthesizeHeliosOp for HeliosBuilder<D>
where
    D: DataflowHugr + HeliosOpBuilder,
{
    delegate! {
        to self.inner {
            #[call(add_lazy_measure)]
            fn build_lazy_measure(&mut self, qb: Wire) -> Result<Wire, BuildError>;
            #[call(add_lazy_measure_leaked)]
            fn build_lazy_measure_leaked(&mut self, qb: Wire) -> Result<Wire, BuildError>;
            #[call(add_lazy_measure_reset)]
            fn build_lazy_measure_reset(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError>;
            #[call(add_measure)]
            fn build_measure(&mut self, qb: Wire) -> Result<Wire, BuildError>;
            #[call(add_reset)]
            fn build_reset(&mut self, qb: Wire) -> Result<Wire, BuildError>;
            #[call(add_zz_phase)]
            fn build_zz_phase(
                &mut self,
                qb1: Wire,
                qb2: Wire,
                angle: Wire,
            ) -> Result<[Wire; 2], BuildError>;
            #[call(add_phased_x)]
            fn build_phased_x(&mut self, qb: Wire, angle1: Wire, angle2: Wire) -> Result<Wire, BuildError>;
            #[call(add_rz)]
            fn build_rz(&mut self, qb: Wire, angle: Wire) -> Result<Wire, BuildError>;
            #[call(add_try_alloc)]
            fn build_try_alloc(&mut self) -> Result<Wire, BuildError>;
            #[call(add_qfree)]
            fn build_qfree(&mut self, qb: Wire) -> Result<(), BuildError>;
            #[call(add_measure_reset)]
            fn build_measure_reset(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError>;
            #[call(add_runtime_barrier)]
            fn build_runtime_barrier(&mut self, qbs: Wire, array_size: u64) -> Result<Wire, BuildError>;
        }
    }
}
#[cfg(test)]
mod test {
    use crate::extension::qsystem::common::test_utils;

    use hugr::HugrView;
    use hugr::builder::{DataflowHugr, FunctionBuilder};
    use tket::extension::bool::bool_type;

    use super::*;

    #[test]
    fn create_extension() {
        test_utils::assert_extension_roundtrip::<HeliosOp>(&EXTENSION, &EXTENSION_ID);
    }

    #[test]
    fn lazy_circuit() {
        test_utils::assert_lazy_circuit(|builder, qb| builder.add_lazy_measure_reset(qb));
    }

    #[test]
    fn leaked() {
        test_utils::assert_leaked_measurement(|builder, qb| builder.add_lazy_measure_leaked(qb));
    }

    #[test]
    fn all_ops() {
        let hugr = {
            let mut func_builder = FunctionBuilder::new(
                "all_ops",
                Signature::new(vec![qb_t(), float64_type()], vec![bool_type()]),
            )
            .unwrap();
            let [q0, angle] = func_builder.input_wires_arr();
            let q1 = CommonOpBuilder::build_qalloc_with::<HeliosOp>(&mut func_builder).unwrap();
            let q0 = func_builder.add_reset(q0).unwrap();
            let q1 = func_builder.add_phased_x(q1, angle, angle).unwrap();
            let [q0, q1] = func_builder.build_zz_max(q0, q1).unwrap();
            let [q0, q1] = func_builder.add_zz_phase(q0, q1, angle).unwrap();

            let [q0, q1] = func_builder
                .build_wrapped_barrier([q0, q1])
                .unwrap()
                .try_into()
                .unwrap();

            let q0 = func_builder.add_rz(q0, angle).unwrap();
            let [q0, _b] = func_builder.add_measure_reset(q0).unwrap();
            let b = func_builder.add_measure(q0).unwrap();
            func_builder.add_qfree(q1).unwrap();

            func_builder.finish_hugr_with_outputs([b]).unwrap()
        };
        hugr.validate().unwrap()
    }

    #[test]
    fn test_cast() {
        test_utils::assert_cast_roundtrip::<HeliosOp>();
    }
}
