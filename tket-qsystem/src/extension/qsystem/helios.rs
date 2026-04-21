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

impl<D> SynthesizeTketOp for HeliosBuilder<D>
where
    D: DataflowHugr + CommonOpBuilder,
{
    fn build_h(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_h_with::<HeliosOp>(&mut self.inner, qb)
    }

    fn build_x(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_x_with::<HeliosOp>(&mut self.inner, qb)
    }

    fn build_y(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_y_with::<HeliosOp>(&mut self.inner, qb)
    }

    fn build_z(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_z_with::<HeliosOp>(&mut self.inner, qb)
    }

    fn build_s(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_s_with::<HeliosOp>(&mut self.inner, qb)
    }

    fn build_sdg(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_sdg_with::<HeliosOp>(&mut self.inner, qb)
    }

    fn build_v(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_v_with::<HeliosOp>(&mut self.inner, qb)
    }

    fn build_vdg(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_vdg_with::<HeliosOp>(&mut self.inner, qb)
    }

    fn build_t(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_t_with::<HeliosOp>(&mut self.inner, qb)
    }

    fn build_tdg(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_tdg_with::<HeliosOp>(&mut self.inner, qb)
    }

    fn build_measure_flip(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError> {
        CommonOpBuilder::build_measure_flip_with::<HeliosOp>(&mut self.inner, qb)
    }

    fn build_qalloc(&mut self) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_qalloc_with::<HeliosOp>(&mut self.inner)
    }

    fn build_cx(&mut self, c: Wire, t: Wire) -> Result<[Wire; 2], BuildError> {
        let pi = pi_mul_f64(self, 1.0);
        let pi_2 = pi_mul_f64(self, 0.5);
        let pi_minus_2 = pi_mul_f64(self, -0.5);

        let t = self.build_phased_x(t, pi_minus_2, pi_2)?;
        let [c, t] = self.build_zz_max(c, t)?;
        let c = SynthesizeHeliosOp::build_rz(self, c, pi_minus_2)?;
        let t = self.build_phased_x(t, pi_2, pi)?;
        let t = SynthesizeHeliosOp::build_rz(self, t, pi_minus_2)?;
        Ok([c, t])
    }

    fn build_cy(&mut self, a: Wire, b: Wire) -> Result<[Wire; 2], BuildError> {
        let pi = pi_mul_f64(self, 1.0);
        let pi_2 = pi_mul_f64(self, 0.5);
        let pi_minus_2 = pi_mul_f64(self, -0.5);

        let a = self.build_phased_x(a, pi, pi)?;
        let b = self.build_phased_x(b, pi_minus_2, pi)?;
        let [a, b] = self.build_zz_max(a, b)?;
        let a = self.build_phased_x(a, pi, pi_2)?;
        let b = self.build_phased_x(b, pi_minus_2, pi_minus_2)?;
        let a = SynthesizeHeliosOp::build_rz(self, a, pi_minus_2)?;
        let b = SynthesizeHeliosOp::build_rz(self, b, pi_2)?;
        Ok([a, b])
    }

    fn build_cz(&mut self, a: Wire, b: Wire) -> Result<[Wire; 2], BuildError> {
        let pi_minus_2 = pi_mul_f64(self, -0.5);
        let [a, b] = self.build_zz_max(a, b)?;
        let b = SynthesizeHeliosOp::build_rz(self, b, pi_minus_2)?;
        let a = SynthesizeHeliosOp::build_rz(self, a, pi_minus_2)?;
        Ok([a, b])
    }

    fn build_rx(&mut self, qb: Wire, theta: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_rx_with::<HeliosOp>(&mut self.inner, qb, theta)
    }

    fn build_ry(&mut self, qb: Wire, theta: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_ry_with::<HeliosOp>(&mut self.inner, qb, theta)
    }

    fn build_rz(&mut self, qb: Wire, theta: Wire) -> Result<Wire, BuildError> {
        SynthesizeHeliosOp::build_rz(self, qb, theta)
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
        let b = SynthesizeHeliosOp::build_rz(self, b, lambda_2)?;
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

        let c = self.build_phased_x(c, pi, pi_minus_2)?;
        let [b, c] = self.build_zz_max(b, c)?;
        let c = self.build_phased_x(c, pi_4, pi_2)?;
        let [a, c] = self.build_zz_max(a, c)?;
        let c = self.build_phased_x(c, pi_4, zero)?;
        let [b, c] = self.build_zz_max(b, c)?;
        let c = self.build_phased_x(c, pi_4, pi_minus_2)?;
        let [a, c] = self.build_zz_max(a, c)?;
        let a = self.build_phased_x(a, pi, pi_4)?;
        let c = self.build_phased_x(c, pi_minus_3_4, pi)?;
        let [a, b] = self.build_zz_phase(a, b, pi_4)?;
        let c = SynthesizeHeliosOp::build_rz(self, c, pi)?;
        let a = self.build_phased_x(a, pi, pi_minus_4)?;
        let b = SynthesizeHeliosOp::build_rz(self, b, pi_minus_3_4)?;
        let a = SynthesizeHeliosOp::build_rz(self, a, pi_4)?;
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
    /// Build a "tket.qsystem.helios.MeasureReset" op.
    /// This operation is equivalent to a `Measure` followed by a `Reset`.
    fn build_measure_reset(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError>;

    /// Build a "tket.qsystem.helios.RuntimeBarrier" op.
    fn build_runtime_barrier(&mut self, qbs: Wire, array_size: u64) -> Result<Wire, BuildError>;
}

impl<D> SynthesizeHeliosOp for HeliosBuilder<D>
where
    D: DataflowHugr + CommonOpBuilder,
{
    fn build_lazy_measure(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::add_lazy_measure_with::<HeliosOp>(&mut self.inner, qb)
    }

    fn build_lazy_measure_leaked(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::add_lazy_measure_leaked_with::<HeliosOp>(&mut self.inner, qb)
    }

    fn build_lazy_measure_reset(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError> {
        CommonOpBuilder::add_lazy_measure_reset_with::<HeliosOp>(&mut self.inner, qb)
    }

    fn build_measure(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::add_measure_with::<HeliosOp>(&mut self.inner, qb)
    }

    fn build_reset(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::add_reset_with::<HeliosOp>(&mut self.inner, qb)
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
        CommonOpBuilder::add_phased_x_with::<HeliosOp>(&mut self.inner, qb, angle1, angle2)
    }

    fn build_rz(&mut self, qb: Wire, angle: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::add_rz_with::<HeliosOp>(&mut self.inner, qb, angle)
    }

    fn build_try_alloc(&mut self) -> Result<Wire, BuildError> {
        CommonOpBuilder::add_try_alloc_with::<HeliosOp>(&mut self.inner)
    }

    fn build_qfree(&mut self, qb: Wire) -> Result<(), BuildError> {
        CommonOpBuilder::add_qfree_with::<HeliosOp>(&mut self.inner, qb)
    }

    fn build_measure_reset(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError> {
        CommonOpBuilder::add_measure_reset_with::<HeliosOp>(&mut self.inner, qb)
    }

    fn build_runtime_barrier(&mut self, qbs: Wire, array_size: u64) -> Result<Wire, BuildError> {
        CommonOpBuilder::add_runtime_barrier_with(&mut self.inner, &EXTENSION, qbs, array_size)
    }
}
#[cfg(test)]
mod test {
    use crate::extension::futures::FutureOpBuilder;
    use crate::extension::qsystem::common::test_utils;

    use hugr::HugrView;
    use hugr::builder::{DataflowHugr, FunctionBuilder};
    use hugr::extension::prelude::{UnwrapBuilder, bool_t};
    use hugr::std_extensions::arithmetic::int_types::int_type;
    use hugr::std_extensions::collections::array::ArrayOpBuilder;
    use tket::extension::bool::bool_type;

    use super::*;

    #[test]
    fn create_extension() {
        test_utils::assert_extension_roundtrip::<HeliosOp>(&EXTENSION, &EXTENSION_ID);
    }

    #[test]
    fn lazy_circuit() {
        let hugr = {
            let mut builder = HeliosBuilder::new(
                FunctionBuilder::new(
                    "circuit",
                    Signature::new(vec![qb_t()], vec![qb_t(), bool_t()]),
                )
                .unwrap(),
            );
            let [qb] = builder.input_wires_arr();
            let [qb, lazy_b] = builder.build_lazy_measure_reset(qb).unwrap();
            let [b] = builder.add_read(lazy_b, bool_t()).unwrap();
            builder.finish_hugr_with_outputs([qb, b]).unwrap()
        };
        hugr.validate().unwrap();
    }

    #[test]
    fn leaked() {
        let hugr = {
            let mut builder = HeliosBuilder::new(
                FunctionBuilder::new("leaked", Signature::new(vec![qb_t()], vec![int_type(6)]))
                    .unwrap(),
            );
            let [qb] = builder.input_wires_arr();
            let lazy_i = builder.build_lazy_measure_leaked(qb).unwrap();
            let [i] = builder.add_read(lazy_i, int_type(6)).unwrap();
            builder.finish_hugr_with_outputs([i]).unwrap()
        };
        hugr.validate().unwrap();
    }

    #[test]
    fn all_ops() {
        let hugr = {
            let mut builder = HeliosBuilder::new(
                FunctionBuilder::new(
                    "all_ops",
                    Signature::new(vec![qb_t(), float64_type()], vec![bool_type()]),
                )
                .unwrap(),
            );
            let [q0, angle] = builder.input_wires_arr();
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
            let [q0, q1] = builder.build_zz_phase(q0, q1, angle).unwrap();

            let q_arr = builder.inner.add_new_array(qb_t(), [q0, q1]).unwrap();
            let q_arr = builder.build_runtime_barrier(q_arr, 2).unwrap();
            let [q0, q1] = builder
                .inner
                .add_array_unpack(qb_t(), 2, q_arr)
                .unwrap()
                .try_into()
                .unwrap();

            let q0 = SynthesizeHeliosOp::build_rz(&mut builder, q0, angle).unwrap();
            let [q0, _b] = builder.build_measure_reset(q0).unwrap();
            let b = builder.build_measure(q0).unwrap();
            builder.build_qfree(q1).unwrap();

            builder.finish_hugr_with_outputs([b]).unwrap()
        };
        hugr.validate().unwrap()
    }

    #[test]
    fn test_cast() {
        crate::extension::qsystem::common::test_utils::assert_cast_roundtrip::<HeliosOp>();
    }
}
