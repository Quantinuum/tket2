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
    builder::{BuildError, Container, Dataflow},
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

use super::common::{self, CommonOp, CommonOpBuilder, PhasedXRzSynth, SharedOp};
use super::lower::pi_mul_f64;
use super::sol::SynthesizeSolOp;
use derive_more::Display;
use lazy_static::lazy_static;
use strum::{EnumIter, EnumString, IntoStaticStr};

/// The "tket.qsystem.helios" extension id.
pub const EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.qsystem.helios");
/// The "tket.qsystem.helios" extension version.
pub const EXTENSION_VERSION: Version = Version::new(0, 6, 0);

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
    /// Lazily measure a qubit and lose it.
    LazyMeasure,
    /// Lazily measure a qubit and reset it to the Z |0> eigenstate.
    LazyMeasureReset,
    /// Rotate a qubit around the Z axis, not physical (alias 'rz').
    Rz,
    /// PhasedX gate (alias 'rxy').
    PhasedX,
    /// ZZPhase gate (alias 'rzz').
    ZZPhase,
    /// Allocate a qubit in the Z |0> eigenstate.
    TryQAlloc,
    /// Free a qubit (lose track of it).
    QFree,
    /// Reset a qubit to the Z |0> eigenstate.
    Reset,
    /// Measure a qubit (return 0 or 1) or detect leakage (return 2).
    LazyMeasureLeaked,
    /// Convert a `Future<Bool>` to a `Measurement` (for compatibility with the TKET
    /// quantum extension).
    FutureToMeasurement,
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
            LazyMeasure => Ok(SharedOp::LazyMeasure),
            Rz => Ok(SharedOp::Rz),
            PhasedX => Ok(SharedOp::PhasedX),
            TryQAlloc => Ok(SharedOp::TryQAlloc),
            QFree => Ok(SharedOp::QFree),
            Reset => Ok(SharedOp::Reset),
            LazyMeasureLeaked => Ok(SharedOp::LazyMeasureLeaked),
            LazyMeasureReset => Ok(SharedOp::LazyMeasureReset),
            FutureToMeasurement => Ok(SharedOp::FutureToMeasurement),
            ZZPhase => Err("Helios-specific ops don't have a corresponding SharedOp."),
        }
    }
}

impl From<SharedOp> for HeliosOp {
    fn from(shared_op: SharedOp) -> Self {
        use SharedOp::*;
        match shared_op {
            LazyMeasure => HeliosOp::LazyMeasure,
            Rz => HeliosOp::Rz,
            PhasedX => HeliosOp::PhasedX,
            TryQAlloc => HeliosOp::TryQAlloc,
            QFree => HeliosOp::QFree,
            Reset => HeliosOp::Reset,
            LazyMeasureLeaked => HeliosOp::LazyMeasureLeaked,
            LazyMeasureReset => HeliosOp::LazyMeasureReset,
            FutureToMeasurement => HeliosOp::FutureToMeasurement,
        }
    }
}
impl CommonOp for HeliosOp {
    fn platform_extension() -> Arc<Extension> {
        EXTENSION.clone()
    }
}
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

#[derive(Debug)]
/// Implmements traits for lowering operations in terms of Helios primitives.
pub(super) struct HeliosSynthesizer<'a, D> {
    inner: &'a mut D,
}

impl<'a, D> HeliosSynthesizer<'a, D> {
    pub(super) fn new(inner: &'a mut D) -> Self {
        Self { inner }
    }
}

impl<D> Container for HeliosSynthesizer<'_, D>
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

impl<D> Dataflow for HeliosSynthesizer<'_, D>
where
    D: Dataflow,
{
    delegate! {
        to self.inner {
            fn num_inputs(&self) -> usize;
        }
    }
}

impl<D> PhasedXRzSynth for HeliosSynthesizer<'_, D>
where
    D: CommonOpBuilder<HeliosOp>,
{
    type Op = HeliosOp;
    type Nested<'a, D2: CommonOpBuilder<HeliosOp> + 'a> = HeliosSynthesizer<'a, D2>;

    fn synthesizer_for<'a, D2: CommonOpBuilder<HeliosOp>>(
        inner: &'a mut D2,
    ) -> HeliosSynthesizer<'a, D2> {
        HeliosSynthesizer::new(inner)
    }

    fn synth_phased_x(&mut self, qb: Wire, angle1: Wire, angle2: Wire) -> Result<Wire, BuildError> {
        SynthesizeHeliosOp::build_phased_x(self, qb, angle1, angle2)
    }

    fn synth_rz(&mut self, qb: Wire, angle: Wire) -> Result<Wire, BuildError> {
        SynthesizeHeliosOp::build_rz(self, qb, angle)
    }

    fn synth_try_alloc(&mut self) -> Result<Wire, BuildError> {
        SynthesizeHeliosOp::build_try_alloc(self)
    }

    fn synth_lazy_measure_reset(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError> {
        SynthesizeHeliosOp::build_lazy_measure_reset(self, qb)
    }

    fn build_cx(&mut self, c: Wire, t: Wire) -> Result<[Wire; 2], BuildError> {
        let pi = pi_mul_f64(self, 1.0);
        let pi_2 = pi_mul_f64(self, 0.5);
        let pi_minus_2 = pi_mul_f64(self, -0.5);

        let t = self.synth_phased_x(t, pi_minus_2, pi_2)?;
        let [c, t] = self.build_zz_max(c, t)?;
        let c = self.synth_rz(c, pi_minus_2)?;
        let t = self.synth_phased_x(t, pi_2, pi)?;
        let t = self.synth_rz(t, pi_minus_2)?;
        Ok([c, t])
    }

    fn build_cy(&mut self, a: Wire, b: Wire) -> Result<[Wire; 2], BuildError> {
        let pi = pi_mul_f64(self, 1.0);
        let pi_2 = pi_mul_f64(self, 0.5);
        let pi_minus_2 = pi_mul_f64(self, -0.5);

        let a = self.synth_phased_x(a, pi, pi)?;
        let b = self.synth_phased_x(b, pi_minus_2, pi)?;
        let [a, b] = self.build_zz_max(a, b)?;
        let a = self.synth_phased_x(a, pi, pi_2)?;
        let b = self.synth_phased_x(b, pi_minus_2, pi_minus_2)?;
        let a = self.synth_rz(a, pi_minus_2)?;
        let b = self.synth_rz(b, pi_2)?;
        Ok([a, b])
    }

    fn build_cz(&mut self, a: Wire, b: Wire) -> Result<[Wire; 2], BuildError> {
        let pi_minus_2 = pi_mul_f64(self, -0.5);
        let [a, b] = self.build_zz_max(a, b)?;
        let b = self.synth_rz(b, pi_minus_2)?;
        let a = self.synth_rz(a, pi_minus_2)?;
        Ok([a, b])
    }

    fn build_crz(&mut self, a: Wire, b: Wire, lambda: Wire) -> Result<[Wire; 2], BuildError> {
        let two = self.add_load_const(Value::from(ConstF64::new(2.0)));
        let lambda_2 = self
            .add_dataflow_op(FloatOps::fdiv, [lambda, two])?
            .out_wire(0);
        let lambda_minus_2 = self
            .add_dataflow_op(FloatOps::fneg, [lambda_2])?
            .out_wire(0);

        let [a, b] = self.build_zz_phase(a, b, lambda_minus_2)?;
        let b = self.synth_rz(b, lambda_2)?;
        Ok([a, b])
    }

    fn build_toffoli(&mut self, a: Wire, b: Wire, c: Wire) -> Result<[Wire; 3], BuildError> {
        let pi = pi_mul_f64(self, 1.0);
        let pi_2 = pi_mul_f64(self, 0.5);
        let pi_minus_2 = pi_mul_f64(self, -0.5);
        let pi_4 = pi_mul_f64(self, 0.25);
        let pi_minus_4 = pi_mul_f64(self, -0.25);
        let pi_minus_3_4 = pi_mul_f64(self, -0.75);
        let zero = pi_mul_f64(self, 0.0);

        let c = self.synth_phased_x(c, pi, pi_minus_2)?;
        let [b, c] = self.build_zz_max(b, c)?;
        let c = self.synth_phased_x(c, pi_4, pi_2)?;
        let [a, c] = self.build_zz_max(a, c)?;
        let c = self.synth_phased_x(c, pi_4, zero)?;
        let [b, c] = self.build_zz_max(b, c)?;
        let c = self.synth_phased_x(c, pi_4, pi_minus_2)?;
        let [a, c] = self.build_zz_max(a, c)?;
        let a = self.synth_phased_x(a, pi, pi_4)?;
        let c = self.synth_phased_x(c, pi_minus_3_4, pi)?;
        let [a, b] = self.build_zz_phase(a, b, pi_4)?;
        let c = self.synth_rz(c, pi)?;
        let a = self.synth_phased_x(a, pi, pi_minus_4)?;
        let b = self.synth_rz(b, pi_minus_3_4)?;
        let a = self.synth_rz(a, pi_4)?;
        Ok([a, b, c])
    }
}

/// Builder trait for lowering `HeliosOp`s into a target operation set.
pub trait SynthesizeHeliosOp: Dataflow {
    /// Build a "tket.qsystem.helios.LazyMeasure" op.
    fn build_lazy_measure(&mut self, qb: Wire) -> Result<Wire, BuildError>;

    /// Build a "tket.qsystem.helios.LazyMeasureLeaked" op.
    fn build_lazy_measure_leaked(&mut self, qb: Wire) -> Result<Wire, BuildError>;

    /// Build a "tket.qsystem.helios.LazyMeasureReset" op.
    fn build_lazy_measure_reset(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError>;

    /// Build a "tket.qsystem.helios.Reset" op.
    fn build_reset(&mut self, qb: Wire) -> Result<Wire, BuildError>;

    /// Build a "tket.qsystem.helios.ZZPhase" op.
    fn build_zz_phase(
        &mut self,
        qb1: Wire,
        qb2: Wire,
        angle: Wire,
    ) -> Result<[Wire; 2], BuildError>;

    /// Build a "tket.qsystem.helios.ZZPhase" op with the maximum angle of pi/2.
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

    /// Build a "tket.qsystem.helios.RuntimeBarrier" op.
    fn build_runtime_barrier(&mut self, qbs: Wire, array_size: u64) -> Result<Wire, BuildError>;
}

impl<D> SynthesizeHeliosOp for HeliosSynthesizer<'_, D>
where
    D: CommonOpBuilder<HeliosOp>,
{
    fn build_lazy_measure(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        self.inner.add_lazy_measure(qb)
    }

    fn build_lazy_measure_leaked(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        self.inner.add_lazy_measure_leaked(qb)
    }

    fn build_lazy_measure_reset(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError> {
        self.inner.add_lazy_measure_reset(qb)
    }

    fn build_reset(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        self.inner.add_reset(qb)
    }

    fn build_zz_phase(
        &mut self,
        qb1: Wire,
        qb2: Wire,
        angle: Wire,
    ) -> Result<[Wire; 2], BuildError> {
        Ok(self
            .inner
            .add_dataflow_op(HeliosOp::ZZPhase, [qb1, qb2, angle])?
            .outputs_arr())
    }

    fn build_phased_x(&mut self, qb: Wire, angle1: Wire, angle2: Wire) -> Result<Wire, BuildError> {
        self.inner.add_phased_x(qb, angle1, angle2)
    }

    fn build_rz(&mut self, qb: Wire, angle: Wire) -> Result<Wire, BuildError> {
        self.inner.add_rz(qb, angle)
    }

    fn build_try_alloc(&mut self) -> Result<Wire, BuildError> {
        self.inner.add_try_alloc()
    }

    fn build_qfree(&mut self, qb: Wire) -> Result<(), BuildError> {
        self.inner.add_qfree(qb)
    }

    fn build_runtime_barrier(&mut self, qbs: Wire, array_size: u64) -> Result<Wire, BuildError> {
        self.inner.add_runtime_barrier(qbs, array_size)
    }
}

/// Implement [`SynthesizeSolOp`] for [`HeliosSynthesizer`] so that a Sol
/// operation can be expressed in terms of Helios primitives.
///
/// All shared ops delegate to the corresponding [`SynthesizeHeliosOp`] method.
impl<D> SynthesizeSolOp for HeliosSynthesizer<'_, D>
where
    D: CommonOpBuilder<HeliosOp>,
{
    fn build_lazy_measure(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        SynthesizeHeliosOp::build_lazy_measure(self, qb)
    }

    fn build_lazy_measure_leaked(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        SynthesizeHeliosOp::build_lazy_measure_leaked(self, qb)
    }

    fn build_lazy_measure_reset(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError> {
        SynthesizeHeliosOp::build_lazy_measure_reset(self, qb)
    }

    fn build_reset(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        SynthesizeHeliosOp::build_reset(self, qb)
    }

    fn build_phased_xx(
        &mut self,
        qb1: Wire,
        qb2: Wire,
        angle1: Wire,
        angle2: Wire,
    ) -> Result<[Wire; 2], BuildError> {
        let pi_2 = pi_mul_f64(self, 0.5);
        let pi_minus_2 = pi_mul_f64(self, -0.5);
        let [added] = self
            .inner
            .add_dataflow_op(FloatOps::fadd, [pi_minus_2, angle1])?
            .outputs_arr();

        let qb1 = SynthesizeHeliosOp::build_phased_x(self, qb1, pi_2, added)?;
        let qb2 = SynthesizeHeliosOp::build_phased_x(self, qb2, pi_2, added)?;
        let [qb1, qb2] = SynthesizeHeliosOp::build_zz_phase(self, qb1, qb2, angle2)?;
        let qb1 = SynthesizeHeliosOp::build_phased_x(self, qb1, pi_minus_2, added)?;
        let qb2 = SynthesizeHeliosOp::build_phased_x(self, qb2, pi_minus_2, added)?;
        Ok([qb1, qb2])
    }

    fn build_phased_x(&mut self, qb: Wire, angle1: Wire, angle2: Wire) -> Result<Wire, BuildError> {
        SynthesizeHeliosOp::build_phased_x(self, qb, angle1, angle2)
    }

    fn build_rz(&mut self, qb: Wire, angle: Wire) -> Result<Wire, BuildError> {
        SynthesizeHeliosOp::build_rz(self, qb, angle)
    }

    fn build_try_alloc(&mut self) -> Result<Wire, BuildError> {
        SynthesizeHeliosOp::build_try_alloc(self)
    }

    fn build_qfree(&mut self, qb: Wire) -> Result<(), BuildError> {
        SynthesizeHeliosOp::build_qfree(self, qb)
    }

    fn build_runtime_barrier(&mut self, qbs: Wire, array_size: u64) -> Result<Wire, BuildError> {
        SynthesizeHeliosOp::build_runtime_barrier(self, qbs, array_size)
    }
}

#[cfg(test)]
mod test {
    use crate::extension::futures::FutureOpBuilder;
    use crate::extension::qsystem::common::test_utils;

    use hugr::HugrView;
    use hugr::builder::{Dataflow, DataflowHugr, FunctionBuilder};
    use hugr::extension::prelude::{UnwrapBuilder, bool_t};
    use hugr::std_extensions::arithmetic::int_types::int_type;
    use hugr::std_extensions::collections::array::ArrayOpBuilder;

    use super::{EXTENSION, EXTENSION_ID, HeliosOp, HeliosSynthesizer, SynthesizeHeliosOp};
    use hugr::extension::prelude::qb_t;
    use hugr::std_extensions::arithmetic::float_types::float64_type;
    use hugr::types::Signature;

    #[test]
    fn create_extension() {
        test_utils::assert_extension_roundtrip::<HeliosOp>(&EXTENSION, &EXTENSION_ID);
    }

    #[test]
    fn lazy_circuit() {
        let hugr = {
            let mut func_builder =
                FunctionBuilder::new("circuit", Signature::new([qb_t()], [qb_t(), bool_t()]))
                    .unwrap();
            let [qb] = func_builder.input_wires_arr();
            let [qb, lazy_b] = {
                let mut builder = HeliosSynthesizer::new(&mut func_builder);
                builder.build_lazy_measure_reset(qb).unwrap()
            };
            let [b] = func_builder.add_read(lazy_b, bool_t()).unwrap();
            func_builder.finish_hugr_with_outputs([qb, b]).unwrap()
        };
        hugr.validate().unwrap();
    }

    #[test]
    fn leaked() {
        let hugr = {
            let mut func_builder =
                FunctionBuilder::new("leaked", Signature::new([qb_t()], [int_type(6)])).unwrap();
            let [qb] = func_builder.input_wires_arr();
            let lazy_i = {
                let mut builder = HeliosSynthesizer::new(&mut func_builder);
                builder.build_lazy_measure_leaked(qb).unwrap()
            };
            let [i] = func_builder.add_read(lazy_i, int_type(6)).unwrap();
            func_builder.finish_hugr_with_outputs([i]).unwrap()
        };
        hugr.validate().unwrap();
    }

    #[test]
    fn all_ops() {
        let hugr = {
            let mut func_builder = FunctionBuilder::new(
                "all_ops",
                Signature::new([qb_t(), float64_type()], [bool_t()]),
            )
            .unwrap();
            let [q0, angle] = func_builder.input_wires_arr();
            let [q0, q1] = {
                let mut builder = HeliosSynthesizer::new(&mut func_builder);
                let try_q1 = builder.build_try_alloc().unwrap();
                let [q1] = builder
                    .build_expect_sum(
                        1,
                        hugr::extension::prelude::option_type(vec![qb_t()]),
                        try_q1,
                        |_| "No more qubits available to allocate.".to_string(),
                    )
                    .unwrap();
                let q0 = builder.build_reset(q0).unwrap();
                let q1 = builder.build_phased_x(q1, angle, angle).unwrap();
                let [q0, q1] = builder.build_zz_max(q0, q1).unwrap();
                builder.build_zz_phase(q0, q1, angle).unwrap()
            };

            let q_arr = func_builder.add_new_array(qb_t(), [q0, q1]).unwrap();
            let q_arr = {
                let mut builder = HeliosSynthesizer::new(&mut func_builder);
                builder.build_runtime_barrier(q_arr, 2).unwrap()
            };
            let [q0, q1] = func_builder
                .add_array_unpack(qb_t(), 2, q_arr)
                .unwrap()
                .try_into()
                .unwrap();

            let b = {
                let mut builder = HeliosSynthesizer::new(&mut func_builder);
                let q0 = SynthesizeHeliosOp::build_rz(&mut builder, q0, angle).unwrap();
                let [q0, f1] = builder.build_lazy_measure_reset(q0).unwrap();
                let f2 = builder.build_lazy_measure(q0).unwrap();
                let [_b] = builder.add_read(f1, bool_t()).unwrap();
                let [b] = builder.add_read(f2, bool_t()).unwrap();
                builder.build_qfree(q1).unwrap();
                b
            };

            func_builder.finish_hugr_with_outputs([b]).unwrap()
        };
        hugr.validate().unwrap()
    }

    #[test]
    fn test_cast() {
        crate::extension::qsystem::common::test_utils::assert_cast_roundtrip::<HeliosOp>();
    }
}
