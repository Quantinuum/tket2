//! This module defines the Hugr extension used to represent H-series
//! quantum operations.
//!
//! In the case of lazy operations,
//! laziness is represented by returning `tket.futures.Future` classical
//! values. Qubits are never lazy.
use std::{str::FromStr, sync::Arc};

use hugr::{
    Extension, Wire,
    builder::{BuildError, Dataflow},
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
use derive_more::Display;
use lazy_static::lazy_static;
use strum::{EnumIter, EnumString, IntoStaticStr};

/// The "tket.qsystem.sol" extension id.
pub const EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.qsystem.sol");
/// The "tket.qsystem.sol" extension version.
pub const EXTENSION_VERSION: Version = Version::new(0, 5, 1);

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
    /// PhasedXX gate (alias 'rpp')
    PhasedXX,
    /// Tk2 gate (alias 'rxxyyzz')
    Tk2,
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
                SolOp::Tk2 => Signature::new(
                    vec![
                        qb_t(),
                        qb_t(),
                        float64_type(),
                        float64_type(),
                        float64_type(),
                    ],
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
                SolOp::PhasedXX => "PhasedXX gate, a.k.a. rpp, specific to the Sol platform.",
                SolOp::Tk2 => "Tk2 gate, a.k.a. rxxyyzz. Specific to the Sol platform.",
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
            Measure => Ok(SharedOp::Measure),
            LazyMeasure => Ok(SharedOp::LazyMeasure),
            Reset => Ok(SharedOp::Reset),
            Rz => Ok(SharedOp::Rz),
            PhasedX => Ok(SharedOp::PhasedX),
            TryQAlloc => Ok(SharedOp::TryQAlloc),
            QFree => Ok(SharedOp::QFree),
            MeasureReset => Ok(SharedOp::MeasureReset),
            LazyMeasureLeaked => Ok(SharedOp::LazyMeasureLeaked),
            LazyMeasureReset => Ok(SharedOp::LazyMeasureReset),
            _ => Err("Sol-specific ops don't have a corresponding SharedOp."),
        }
    }
}

impl From<SharedOp> for SolOp {
    fn from(shared_op: SharedOp) -> Self {
        use SharedOp::*;
        match shared_op {
            Measure => SolOp::Measure,
            LazyMeasure => SolOp::LazyMeasure,
            Reset => SolOp::Reset,
            Rz => SolOp::Rz,
            PhasedX => SolOp::PhasedX,
            TryQAlloc => SolOp::TryQAlloc,
            QFree => SolOp::QFree,
            MeasureReset => SolOp::MeasureReset,
            LazyMeasureLeaked => SolOp::LazyMeasureLeaked,
            LazyMeasureReset => SolOp::LazyMeasureReset,
        }
    }
}
impl CommonOp for SolOp {}

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

/// An extension trait for [Dataflow] providing methods to add
/// "tket.qsystem.sol" operations.
pub trait SolOpBuilder: CommonOpBuilder {
    /// Add a "tket.qsystem.sol.LazyMeasure" op.
    fn add_lazy_measure(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::add_lazy_measure_with::<SolOp>(self, qb)
    }
    /// Add a "tket.qsystem.sol.PhasedXX" op.
    fn add_phased_xx(
        &mut self,
        qb1: Wire,
        qb2: Wire,
        angle1: Wire,
        angle2: Wire,
    ) -> Result<[Wire; 2], BuildError> {
        Ok(self
            .add_dataflow_op(SolOp::PhasedXX, [qb1, qb2, angle1, angle2])?
            .outputs_arr())
    }

    /// Add a "tket.qsystem.sol.Tk2" op.
    fn add_tk2(
        &mut self,
        qb1: Wire,
        qb2: Wire,
        angle1: Wire,
        angle2: Wire,
        angle3: Wire,
    ) -> Result<[Wire; 2], BuildError> {
        Ok(self
            .add_dataflow_op(SolOp::Tk2, [qb1, qb2, angle1, angle2, angle3])?
            .outputs_arr())
    }

    /// Add a "tket.qsystem.sol.LazyMeasureLeaked" op.
    fn add_lazy_measure_leaked(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::add_lazy_measure_leaked_with::<SolOp>(self, qb)
    }

    /// Add a "tket.qsystem.sol.LazyMeasureReset" op.
    fn add_lazy_measure_reset(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError> {
        CommonOpBuilder::add_lazy_measure_reset_with::<SolOp>(self, qb)
    }

    /// Add a "tket.qsystem.sol.Measure" op.
    fn add_measure(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::add_measure_with::<SolOp>(self, qb)
    }

    /// Add a "tket.qsystem.sol.Reset" op.
    fn add_reset(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::add_reset_with::<SolOp>(self, qb)
    }

    /// Add a maximally entangling "tket.qsystem.sol.ZZPhase(pi/2)" op.
    fn build_zz_max(&mut self, qb1: Wire, qb2: Wire) -> Result<[Wire; 2], BuildError> {
        let pi_2 = pi_mul_f64(self, 0.5);

        self.build_zz_phase(qb1, qb2, pi_2)
    }

    /// Build a "tket.qsystem.helios.ZZPhase" op in terms of Sol primitives.
    fn build_zz_phase(
        &mut self,
        qb1: Wire,
        qb2: Wire,
        angle: Wire,
    ) -> Result<[Wire; 2], BuildError> {
        let pi_minus = pi_mul_f64(self, -1.0);
        let pi_2 = pi_mul_f64(self, 0.5);
        let pi_minus_2 = pi_mul_f64(self, -0.5);

        let qb1 = self.add_phased_x(qb1, pi_2, pi_minus_2)?;
        let qb2 = self.add_phased_x(qb2, pi_2, pi_minus_2)?;
        let [qb1, qb2] = self.add_phased_xx(qb1, qb2, angle, pi_minus)?;
        let qb1 = self.add_phased_x(qb1, pi_2, pi_2)?;
        let qb2 = self.add_phased_x(qb2, pi_2, pi_2)?;
        Ok([qb1, qb2])
    }

    /// Add a "tket.qsystem.sol.PhasedX" op.
    fn add_phased_x(&mut self, qb: Wire, angle1: Wire, angle2: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::add_phased_x_with::<SolOp>(self, qb, angle1, angle2)
    }

    /// Add a "tket.qsystem.sol.Rz" op.
    fn add_rz(&mut self, qb: Wire, angle: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::add_rz_with::<SolOp>(self, qb, angle)
    }

    /// Add a "tket.qsystem.sol.TryQAlloc" op.
    fn add_try_alloc(&mut self) -> Result<Wire, BuildError> {
        CommonOpBuilder::add_try_alloc_with::<SolOp>(self)
    }

    /// Add a "tket.qsystem.sol.QFree" op.
    fn add_qfree(&mut self, qb: Wire) -> Result<(), BuildError> {
        CommonOpBuilder::add_qfree_with::<SolOp>(self, qb)
    }

    /// Add a "tket.qsystem.sol.MeasureReset" op.
    /// This operation is equivalent to a `Measure` followed by a `Reset`.
    fn add_measure_reset(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError> {
        CommonOpBuilder::add_measure_reset_with::<SolOp>(self, qb)
    }

    /// Add a "tket.qsystem.sol.RuntimeBarrier" op.
    fn add_runtime_barrier(&mut self, qbs: Wire, array_size: u64) -> Result<Wire, BuildError> {
        CommonOpBuilder::add_runtime_barrier_with(self, &EXTENSION, qbs, array_size)
    }

    /// Build a hadamard gate in terms of QSystem primitives.
    fn build_h(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_h_with::<SolOp>(self, qb)
    }

    /// Build an X gate in terms of QSystem primitives.
    fn build_x(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_x_with::<SolOp>(self, qb)
    }

    /// Build a Y gate in terms of QSystem primitives.
    fn build_y(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_y_with::<SolOp>(self, qb)
    }

    /// Build a Z gate in terms of QSystem primitives.
    fn build_z(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_z_with::<SolOp>(self, qb)
    }

    /// Build an S gate in terms of QSystem primitives.
    fn build_s(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_s_with::<SolOp>(self, qb)
    }

    /// Build an Sdg gate in terms of QSystem primitives.
    fn build_sdg(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_sdg_with::<SolOp>(self, qb)
    }

    /// Build a V gate in terms of QSystem primitives.
    fn build_v(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_v_with::<SolOp>(self, qb)
    }

    /// Build a Vdg gate in terms of QSystem primitives.
    fn build_vdg(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_vdg_with::<SolOp>(self, qb)
    }

    /// Build a T gate in terms of QSystem primitives.
    fn build_t(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_t_with::<SolOp>(self, qb)
    }

    /// Build a Tdg gate in terms of QSystem primitives.
    fn build_tdg(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_tdg_with::<SolOp>(self, qb)
    }

    /// Build a CNOT gate in terms of QSystem primitives.
    fn build_cx(&mut self, c: Wire, t: Wire) -> Result<[Wire; 2], BuildError> {
        let pi_2 = pi_mul_f64(self, 0.5);
        let pi_minus_2 = pi_mul_f64(self, -0.5);
        let zero = pi_mul_f64(self, 0.0);

        let c = self.add_phased_x(c, pi_2, pi_2)?;
        let [c, t] = self.add_phased_xx(c, t, pi_2, zero)?;
        let c = self.add_phased_x(c, pi_minus_2, pi_2)?;
        let c = self.add_rz(c, pi_minus_2)?;
        let t = self.add_phased_x(t, pi_minus_2, zero)?;
        Ok([c, t])
    }

    /// Build a CY gate in terms of QSystem primitives.
    fn build_cy(&mut self, a: Wire, b: Wire) -> Result<[Wire; 2], BuildError> {
        let pi_2 = pi_mul_f64(self, 0.5);
        let pi_minus_2 = pi_mul_f64(self, -0.5);
        let zero = pi_mul_f64(self, 0.0);

        let a = self.add_phased_x(a, pi_2, pi_2)?;
        let b = self.add_rz(b, pi_minus_2)?;
        let [a, b] = self.add_phased_xx(a, b, pi_2, zero)?;
        let a = self.add_phased_x(a, pi_minus_2, pi_2)?;
        let a = self.add_rz(a, pi_minus_2)?;
        let b = self.add_phased_x(b, pi_minus_2, zero)?;
        let b = self.add_rz(b, pi_2)?;
        Ok([a, b])
    }

    /// Build a CZ gate in terms of QSystem primitives.
    fn build_cz(&mut self, a: Wire, b: Wire) -> Result<[Wire; 2], BuildError> {
        let pi_minus = pi_mul_f64(self, -1.0);
        let pi_2 = pi_mul_f64(self, 0.5);
        let pi_minus_2 = pi_mul_f64(self, -0.5);

        let a = self.add_phased_x(a, pi_2, pi_minus_2)?;
        let b = self.add_phased_x(b, pi_2, pi_minus_2)?;
        let [a, b] = self.add_phased_xx(a, b, pi_2, pi_minus)?;
        let a = self.add_phased_x(a, pi_2, pi_2)?;
        let b = self.add_phased_x(b, pi_2, pi_2)?;
        let a = self.add_rz(a, pi_minus_2)?;
        let b = self.add_rz(b, pi_minus_2)?;
        Ok([a, b])
    }

    /// Build a RX gate in terms of QSystem primitives.
    fn build_rx(&mut self, qb: Wire, theta: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_rx_with::<SolOp>(self, qb, theta)
    }

    /// Build a RY gate in terms of QSystem primitives.
    fn build_ry(&mut self, qb: Wire, theta: Wire) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_ry_with::<SolOp>(self, qb, theta)
    }

    /// Build a CRZ gate in terms of QSystem primitives.
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

        let a = self.add_phased_x(a, pi_2, pi_minus_2)?;
        let b = self.add_phased_x(b, pi_2, pi_minus_2)?;
        let [a, b] = self.add_phased_xx(a, b, lambda_minus_2, pi_minus)?;
        let a = self.add_phased_x(a, pi_2, pi_2)?;
        let b = self.add_phased_x(b, pi_2, pi_2)?;
        let b = self.add_rz(b, lambda_2)?;
        Ok([a, b])
    }

    /// Build a Toffoli (CCX) gate in terms of QSystem primitives.
    fn build_toffoli(&mut self, a: Wire, b: Wire, c: Wire) -> Result<[Wire; 3], BuildError> {
        let pi = pi_mul_f64(self, 1.0);
        let pi_2 = pi_mul_f64(self, 0.5);
        let pi_minus_2 = pi_mul_f64(self, -0.5);
        let pi_4 = pi_mul_f64(self, 0.25);
        let pi_minus_4 = pi_mul_f64(self, -0.25);
        let pi_minus_3_4 = pi_mul_f64(self, -0.75);
        let zero = pi_mul_f64(self, 0.0);

        let a = self.add_phased_x(a, pi_2, pi_minus_3_4)?;
        let b = self.add_phased_x(b, pi_2, pi_minus_3_4)?;
        let a = self.add_rz(a, pi_minus_3_4)?;
        let b = self.add_rz(b, pi_minus_3_4)?;
        let c = self.add_phased_x(c, pi_2, pi_minus_2)?;
        let c = self.add_rz(c, pi_minus_3_4)?;

        let [a, c] = self.add_phased_xx(a, c, pi_2, zero)?;

        let a = self.add_phased_x(a, pi_minus_2, zero)?;
        let c = self.add_phased_x(c, pi_4, pi_minus_2)?;

        let [a, b] = self.add_phased_xx(a, b, pi_minus_4, zero)?;
        let c = self.add_rz(c, pi_2)?;

        let [b, c] = self.add_phased_xx(b, c, pi_4, zero)?;
        let c = self.add_phased_x(c, pi_2, pi_minus_2)?;
        let c = self.add_rz(c, pi)?;

        let [a, c] = self.add_phased_xx(a, c, pi_2, zero)?;
        let a = self.add_phased_x(a, pi_2, pi_minus_2)?;
        let c = self.add_phased_x(c, pi_2, pi_minus_2)?;
        let a = self.add_rz(a, pi_2)?;
        let c = self.add_rz(c, pi_2)?;

        let [b, c] = self.add_phased_xx(b, c, pi_minus_4, zero)?;
        let b = self.add_phased_x(b, pi_2, pi_minus_2)?;
        let b = self.add_rz(b, pi)?;

        Ok([a, b, c])
    }

    /// Build a projective measurement with a conditional flip.
    fn build_measure_flip(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError> {
        CommonOpBuilder::build_measure_flip_with::<SolOp>(self, qb)
    }

    /// Build a qalloc operation that panics on failure.
    fn build_qalloc(&mut self) -> Result<Wire, BuildError> {
        CommonOpBuilder::build_qalloc_with::<SolOp>(self)
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
}

impl<D: Dataflow> SolOpBuilder for D {}

#[cfg(test)]
mod test {
    use crate::extension::qsystem::common::test_utils;

    use hugr::HugrView;
    use hugr::builder::{DataflowHugr, FunctionBuilder};
    use tket::extension::bool::bool_type;

    use super::*;

    #[test]
    fn create_extension() {
        test_utils::assert_extension_roundtrip::<SolOp>(&EXTENSION, &EXTENSION_ID);
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
            // TODO use sol native ops rather than helios
            let mut func_builder = FunctionBuilder::new(
                "all_ops",
                Signature::new(vec![qb_t(), float64_type()], vec![bool_type()]),
            )
            .unwrap();
            let [q0, angle] = func_builder.input_wires_arr();
            let q1 = func_builder.build_qalloc().unwrap();
            let q0 = func_builder.add_reset(q0).unwrap();
            let q1 = func_builder.add_phased_x(q1, angle, angle).unwrap();
            let [q0, q1] = func_builder.build_zz_max(q0, q1).unwrap();
            let [q0, q1] = func_builder.build_zz_phase(q0, q1, angle).unwrap();

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
        test_utils::assert_cast_roundtrip::<SolOp>();
    }
}
