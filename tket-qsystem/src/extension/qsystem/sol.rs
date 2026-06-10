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
use derive_more::Display;
use lazy_static::lazy_static;
use strum::{EnumIter, EnumString, IntoStaticStr};

/// The "tket.qsystem.sol" extension id.
pub const EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.qsystem.sol");
/// The "tket.qsystem.sol" extension version.
pub const EXTENSION_VERSION: Version = Version::new(0, 6, 0);

lazy_static! {
    /// The "tket.qsystem.sol" extension.
    pub static ref EXTENSION: Arc<Extension> = {
         Extension::new_arc(EXTENSION_ID, EXTENSION_VERSION, |ext, ext_ref| {
            SolOp::load_all_ops( ext, ext_ref).unwrap();
            RuntimeBarrierDef.add_to_extension(ext, ext_ref).unwrap();
        })
    };

}

/// Quantum operations for Quantinuum H-series quantum computers.
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
pub enum SolOp {
    /// Lazily measure a qubit and lose it.
    LazyMeasure,
    /// Lazily measure a qubit and reset it to the Z |0> eigenstate.
    LazyMeasureReset,
    /// Rotate a qubit around the Z axis, not physical (alias 'rz').
    Rz,
    /// PhasedX gate (alias 'rp').
    PhasedX,
    /// Allocate a qubit in the Z |0> eigenstate.
    TryQAlloc,
    /// Free a qubit (lose track of it).
    QFree,
    /// Reset a qubit to the Z |0> eigenstate.
    Reset,
    /// Measure a qubit (return 0 or 1) or detect leakage (return 2).
    LazyMeasureLeaked,
    /// PhasedXX gate (alias 'rpp')
    PhasedXX,
    /// Convert a `Future<Bool>` to a `Measurement` (for compatibility with the TKET
    /// quantum extension).
    FutureToMeasurement,
}

impl MakeOpDef for SolOp {
    fn opdef_id(&self) -> hugr::ops::OpName {
        <&'static str>::from(self).into()
    }

    fn init_signature(&self, _extension_ref: &std::sync::Weak<Extension>) -> SignatureFunc {
        if let Ok(shared_op) = SharedOp::try_from(*self) {
            shared_op.signature()
        } else {
            match self {
                SolOp::PhasedXX => Signature::new(
                    vec![qb_t(), qb_t(), float64_type(), float64_type()],
                    TypeRow::from(vec![qb_t(), qb_t()]),
                )
                .into(),
                _ => unreachable!("All other SolOps should have been convertible to SharedOps."),
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
            match self {
                SolOp::PhasedXX => "PhasedXX gate, specific to the Sol platform.",
                _ => unreachable!("All other SolOps should have been convertible to SharedOps."),
            }
        }
        .to_string()
    }
}

impl MakeRegisteredOp for SolOp {
    fn extension_id(&self) -> ExtensionId {
        EXTENSION_ID
    }

    fn extension_ref(&self) -> Arc<Extension> {
        EXTENSION.clone()
    }
}

impl TryFrom<SolOp> for SharedOp {
    type Error = &'static str;

    fn try_from(sol_op: SolOp) -> Result<Self, Self::Error> {
        use SolOp::*;
        match sol_op {
            LazyMeasure => Ok(SharedOp::LazyMeasure),
            Reset => Ok(SharedOp::Reset),
            Rz => Ok(SharedOp::Rz),
            PhasedX => Ok(SharedOp::PhasedX),
            TryQAlloc => Ok(SharedOp::TryQAlloc),
            QFree => Ok(SharedOp::QFree),
            LazyMeasureLeaked => Ok(SharedOp::LazyMeasureLeaked),
            LazyMeasureReset => Ok(SharedOp::LazyMeasureReset),
            FutureToMeasurement => Ok(SharedOp::FutureToMeasurement),
            _ => Err("Sol-specific ops don't have a corresponding SharedOp."),
        }
    }
}

impl From<SharedOp> for SolOp {
    fn from(shared_op: SharedOp) -> Self {
        use SharedOp::*;
        match shared_op {
            LazyMeasure => SolOp::LazyMeasure,
            Reset => SolOp::Reset,
            Rz => SolOp::Rz,
            PhasedX => SolOp::PhasedX,
            TryQAlloc => SolOp::TryQAlloc,
            QFree => SolOp::QFree,
            LazyMeasureLeaked => SolOp::LazyMeasureLeaked,
            LazyMeasureReset => SolOp::LazyMeasureReset,
            FutureToMeasurement => SolOp::FutureToMeasurement,
        }
    }
}
impl CommonOp for SolOp {
    fn platform_extension() -> Arc<Extension> {
        EXTENSION.clone()
    }
}

/// The name of the "tket.qsystem.sol.RuntimeBarrier" operation.
pub const RUNTIME_BARRIER_NAME: hugr::ops::OpName = common::RUNTIME_BARRIER_NAME;

/// Helper struct for the "tket.qsystem.sol.RuntimeBarrier" operation definition.
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
/// Implmements traits for lowering operations in terms of Sol primitives.
pub(super) struct SolSynthesizer<'a, D> {
    inner: &'a mut D,
}

impl<'a, D> SolSynthesizer<'a, D> {
    pub(super) fn new(inner: &'a mut D) -> Self {
        Self { inner }
    }
}

impl<D> Container for SolSynthesizer<'_, D>
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

impl<D> Dataflow for SolSynthesizer<'_, D>
where
    D: Dataflow,
{
    delegate! {
        to self.inner {
            fn num_inputs(&self) -> usize;
        }
    }
}

impl<D> PhasedXRzSynth for SolSynthesizer<'_, D>
where
    D: CommonOpBuilder<SolOp>,
{
    type Op = SolOp;
    type Nested<'a, D2: CommonOpBuilder<SolOp> + 'a> = SolSynthesizer<'a, D2>;

    fn synthesizer_for<'a, D2: CommonOpBuilder<SolOp>>(
        inner: &'a mut D2,
    ) -> SolSynthesizer<'a, D2> {
        SolSynthesizer::new(inner)
    }

    fn synth_phased_x(&mut self, qb: Wire, angle1: Wire, angle2: Wire) -> Result<Wire, BuildError> {
        SynthesizeSolOp::build_phased_x(self, qb, angle1, angle2)
    }

    fn synth_rz(&mut self, qb: Wire, angle: Wire) -> Result<Wire, BuildError> {
        SynthesizeSolOp::build_rz(self, qb, angle)
    }

    fn synth_try_alloc(&mut self) -> Result<Wire, BuildError> {
        SynthesizeSolOp::build_try_alloc(self)
    }

    fn synth_lazy_measure_reset(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError> {
        SynthesizeSolOp::build_lazy_measure_reset(self, qb)
    }

    fn build_cx(&mut self, c: Wire, t: Wire) -> Result<[Wire; 2], BuildError> {
        let pi_2 = pi_mul_f64(self, 0.5);
        let pi_minus_2 = pi_mul_f64(self, -0.5);
        let zero = pi_mul_f64(self, 0.0);

        let c = self.synth_phased_x(c, pi_2, pi_2)?;
        let [c, t] = SynthesizeSolOp::build_phased_xx(self, c, t, pi_2, zero)?;
        let c = self.synth_phased_x(c, pi_minus_2, pi_2)?;
        let c = self.synth_rz(c, pi_minus_2)?;
        let t = self.synth_phased_x(t, pi_minus_2, zero)?;
        Ok([c, t])
    }

    fn build_cy(&mut self, a: Wire, b: Wire) -> Result<[Wire; 2], BuildError> {
        let pi_2 = pi_mul_f64(self, 0.5);
        let pi_minus_2 = pi_mul_f64(self, -0.5);
        let zero = pi_mul_f64(self, 0.0);

        let a = self.synth_phased_x(a, pi_2, pi_2)?;
        let b = self.synth_rz(b, pi_minus_2)?;
        let [a, b] = SynthesizeSolOp::build_phased_xx(self, a, b, pi_2, zero)?;
        let a = self.synth_phased_x(a, pi_minus_2, pi_2)?;
        let a = self.synth_rz(a, pi_minus_2)?;
        let b = self.synth_phased_x(b, pi_minus_2, zero)?;
        let b = self.synth_rz(b, pi_2)?;
        Ok([a, b])
    }

    fn build_cz(&mut self, a: Wire, b: Wire) -> Result<[Wire; 2], BuildError> {
        let pi_minus = pi_mul_f64(self, -1.0);
        let pi_2 = pi_mul_f64(self, 0.5);
        let pi_minus_2 = pi_mul_f64(self, -0.5);

        let a = self.synth_phased_x(a, pi_2, pi_minus_2)?;
        let b = self.synth_phased_x(b, pi_2, pi_minus_2)?;
        let [a, b] = SynthesizeSolOp::build_phased_xx(self, a, b, pi_2, pi_minus)?;
        let a = self.synth_phased_x(a, pi_2, pi_2)?;
        let b = self.synth_phased_x(b, pi_2, pi_2)?;
        let a = self.synth_rz(a, pi_minus_2)?;
        let b = self.synth_rz(b, pi_minus_2)?;
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

        let pi_minus = pi_mul_f64(self, -1.0);
        let pi_2 = pi_mul_f64(self, 0.5);
        let pi_minus_2 = pi_mul_f64(self, -0.5);

        let a = self.synth_phased_x(a, pi_2, pi_minus_2)?;
        let b = self.synth_phased_x(b, pi_2, pi_minus_2)?;
        let [a, b] = SynthesizeSolOp::build_phased_xx(self, a, b, lambda_minus_2, pi_minus)?;
        let a = self.synth_phased_x(a, pi_2, pi_2)?;
        let b = self.synth_phased_x(b, pi_2, pi_2)?;
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

        let a = self.synth_phased_x(a, pi_2, pi_minus_3_4)?;
        let b = self.synth_phased_x(b, pi_2, pi_minus_3_4)?;
        let a = self.synth_rz(a, pi_minus_3_4)?;
        let b = self.synth_rz(b, pi_minus_3_4)?;
        let c = self.synth_phased_x(c, pi_2, pi_minus_2)?;
        let c = self.synth_rz(c, pi_minus_3_4)?;

        let [a, c] = SynthesizeSolOp::build_phased_xx(self, a, c, pi_2, zero)?;

        let a = self.synth_phased_x(a, pi_minus_2, zero)?;
        let c = self.synth_phased_x(c, pi_4, pi_minus_2)?;

        let [a, b] = SynthesizeSolOp::build_phased_xx(self, a, b, pi_minus_4, zero)?;
        let c = self.synth_rz(c, pi_2)?;

        let [b, c] = SynthesizeSolOp::build_phased_xx(self, b, c, pi_4, zero)?;
        let c = self.synth_phased_x(c, pi_2, pi_minus_2)?;
        let c = self.synth_rz(c, pi)?;

        let [a, c] = SynthesizeSolOp::build_phased_xx(self, a, c, pi_2, zero)?;
        let a = self.synth_phased_x(a, pi_2, pi_minus_2)?;
        let c = self.synth_phased_x(c, pi_2, pi_minus_2)?;
        let a = self.synth_rz(a, pi_2)?;
        let c = self.synth_rz(c, pi_2)?;

        let [b, c] = SynthesizeSolOp::build_phased_xx(self, b, c, pi_minus_4, zero)?;
        let b = self.synth_phased_x(b, pi_2, pi_minus_2)?;
        let b = self.synth_rz(b, pi)?;

        Ok([a, b, c])
    }
}

/// Builder trait for lowering `SolOp`s into a target operation set.
pub trait SynthesizeSolOp: Dataflow {
    /// Build a "tket.qsystem.sol.LazyMeasure" op.
    fn build_lazy_measure(&mut self, qb: Wire) -> Result<Wire, BuildError>;

    /// Build a "tket.qsystem.sol.LazyMeasureLeaked" op.
    fn build_lazy_measure_leaked(&mut self, qb: Wire) -> Result<Wire, BuildError>;

    /// Build a "tket.qsystem.sol.LazyMeasureReset" op.
    fn build_lazy_measure_reset(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError>;

    /// Build a "tket.qsystem.sol.Reset" op.
    fn build_reset(&mut self, qb: Wire) -> Result<Wire, BuildError>;

    /// Build a "tket.qsystem.sol.PhasedXX" op.
    fn build_phased_xx(
        &mut self,
        qb1: Wire,
        qb2: Wire,
        angle1: Wire,
        angle2: Wire,
    ) -> Result<[Wire; 2], BuildError>;

    /// Build a "tket.qsystem.sol.PhasedX" op.
    fn build_phased_x(&mut self, qb: Wire, angle1: Wire, angle2: Wire) -> Result<Wire, BuildError>;

    /// Build a "tket.qsystem.sol.Rz" op.
    fn build_rz(&mut self, qb: Wire, angle: Wire) -> Result<Wire, BuildError>;

    /// Build a "tket.qsystem.sol.TryQAlloc" op.
    fn build_try_alloc(&mut self) -> Result<Wire, BuildError>;

    /// Build a "tket.qsystem.sol.QFree" op.
    fn build_qfree(&mut self, qb: Wire) -> Result<(), BuildError>;

    /// Build a "tket.qsystem.sol.RuntimeBarrier" op.
    fn build_runtime_barrier(&mut self, qbs: Wire, array_size: u64) -> Result<Wire, BuildError>;
}

impl<D> SynthesizeSolOp for SolSynthesizer<'_, D>
where
    D: CommonOpBuilder<SolOp>,
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

    fn build_phased_xx(
        &mut self,
        qb1: Wire,
        qb2: Wire,
        angle1: Wire,
        angle2: Wire,
    ) -> Result<[Wire; 2], BuildError> {
        Ok(self
            .inner
            .add_dataflow_op(SolOp::PhasedXX, [qb1, qb2, angle1, angle2])?
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

#[cfg(test)]
mod test {
    use crate::extension::futures::FutureOpBuilder;
    use crate::extension::qsystem::common::test_utils;
    use crate::extension::qsystem::synth_tket_op::SynthesizeTketOp as _;

    use hugr::HugrView;
    use hugr::builder::{DataflowHugr, FunctionBuilder};
    use hugr::extension::prelude::bool_t;
    use hugr::std_extensions::collections::array::ArrayOpBuilder;

    use super::*;

    #[test]
    fn create_extension() {
        test_utils::assert_extension_roundtrip::<SolOp>(&EXTENSION, &EXTENSION_ID);
    }

    #[test]
    fn lazy_circuit() {
        test_utils::assert_lazy_circuit(|builder, qb| {
            let mut synthesizer = SolSynthesizer::new(builder);
            synthesizer.build_lazy_measure_reset(qb)
        });
    }

    #[test]
    fn leaked() {
        test_utils::assert_leaked_measurement(|builder, qb| {
            let mut synthesizer = SolSynthesizer::new(builder);
            synthesizer.build_lazy_measure_leaked(qb)
        });
    }

    #[test]
    fn all_ops() {
        let hugr = {
            let mut func_builder = FunctionBuilder::new(
                "all_ops",
                Signature::new(vec![qb_t(), float64_type()], vec![bool_t()]),
            )
            .unwrap();
            let [q0, angle] = func_builder.input_wires_arr();
            let [q0, q1] = {
                let mut synthesizer = SolSynthesizer::new(&mut func_builder);
                let q1 = synthesizer.build_qalloc().unwrap();
                let q0 = synthesizer.build_reset(q0).unwrap();
                let q1 = synthesizer.build_phased_x(q1, angle, angle).unwrap();
                let zero = pi_mul_f64(&mut synthesizer, 0.0);
                synthesizer.build_phased_xx(q0, q1, angle, zero).unwrap()
            };
            let q_arr = func_builder.add_new_array(qb_t(), [q0, q1]).unwrap();
            let q_arr = {
                let mut synthesizer = SolSynthesizer::new(&mut func_builder);
                synthesizer.build_runtime_barrier(q_arr, 2).unwrap()
            };
            let [q0, q1] = func_builder
                .add_array_unpack(qb_t(), 2, q_arr)
                .unwrap()
                .try_into()
                .unwrap();

            let b = {
                let mut synthesizer = SolSynthesizer::new(&mut func_builder);
                let q0 = SynthesizeSolOp::build_rz(&mut synthesizer, q0, angle).unwrap();
                let [q0, f1] = synthesizer.build_lazy_measure_reset(q0).unwrap();
                let f2 = synthesizer.build_lazy_measure(q0).unwrap();
                let [_b] = synthesizer.add_read(f1, bool_t()).unwrap();
                let [b] = synthesizer.add_read(f2, bool_t()).unwrap();
                synthesizer.build_qfree(q1).unwrap();
                b
            };

            func_builder.finish_hugr_with_outputs([b]).unwrap()
        };
        hugr.validate().unwrap()
    }

    #[test]
    fn test_cast() {
        test_utils::assert_cast_roundtrip::<SolOp>();
    }
}
