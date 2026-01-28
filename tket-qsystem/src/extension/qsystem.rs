//! This module defines the Hugr extension used to represent H-series
//! quantum operations.
//!
//! In the case of lazy operations,
//! laziness is represented by returning `tket.futures.Future` classical
//! values. Qubits are never lazy.
use std::{
    str::FromStr,
    sync::{Arc, Weak},
};

use hugr::{
    Extension, Wire,
    builder::{BuildError, Dataflow, DataflowSubContainer, SubContainer},
    extension::{
        ExtensionId, ExtensionRegistry, OpDef, PRELUDE, SignatureFunc, Version,
        prelude::{UnwrapBuilder, bool_t, option_type, qb_t},
        simple_op::{MakeOpDef, MakeRegisteredOp, try_from_name},
    },
    ops::{ExtensionOp, OpName, Value},
    std_extensions::{
        arithmetic::{
            float_ops::FloatOps,
            float_types::{ConstF64, EXTENSION as FLOAT_TYPES, float64_type},
            int_types::int_type,
        },
        collections::array::{ArrayOpBuilder, array_type_parametric},
    },
    type_row,
    types::{PolyFuncType, Signature, Type, TypeArg, TypeRow, type_param::TypeParam},
};

use crate::extension::futures;
use derive_more::Display;
use lazy_static::lazy_static;
use strum::{EnumIter, EnumString, IntoStaticStr};
use tket::extension::bool::{BoolOp, bool_type};

use super::futures::future_type;

mod barrier;
mod lower;
use lower::pi_mul_f64;
pub use lower::{LowerTk2Error, LowerTketToQSystemPass, check_lowered, lower_tk2_op};

/// The "tket.qsystem" extension id.
pub const EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.qsystem");
/// The "tket.qsystem" extension version.
pub const EXTENSION_VERSION: Version = Version::new(0, 5, 0);

lazy_static! {
    /// The "tket.qsystem" extension.
    pub static ref EXTENSION: Arc<Extension> = {
         Extension::new_arc(EXTENSION_ID, EXTENSION_VERSION, |ext, ext_ref| {
            QSystemOp::load_all_ops( ext, ext_ref).unwrap();
            RuntimeBarrierDef.add_to_extension(ext, ext_ref).unwrap();
        })
    };

    /// Extension registry including the "tket.qsystem" extension and
    /// dependencies.
    pub static ref REGISTRY: ExtensionRegistry = ExtensionRegistry::new([
        EXTENSION.to_owned(),
        futures::EXTENSION.to_owned(),
        PRELUDE.to_owned(),
        FLOAT_TYPES.to_owned(),
    ]);
}

/// Target platform for QSystem operations. This can determine supported operations,
/// the native gateset, and steer optimisation choices.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QSystemPlatform {
    /// Quantinuum Helios, supporting rxy, rzz, rz
    Helios,
    /// Quantinuum Sol, supporting rp, rpp, rpg, tk2
    Sol,
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
pub enum QSystemOp {
    /// Measure a qubit and lose it.
    Measure,
    /// Lazily measure a qubit and lose it.
    LazyMeasure,
    /// Lazily measure a qubit and reset it to the Z |0> eigenstate.
    LazyMeasureReset,
    /// Rotate a qubit around the Z axis. Not physical.
    Rz,
    /// PhasedX gate.
    PhasedX,
    /// ZZ gate with an angle.
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
    /// PhasedXX gate, sol-only (rpp)
    PhasedXX,
    /// NPhasedX with N=2, sol-only (rpg)
    TwinPhasedX,
    /// Tk2 gate, sol-only (rxxyyzz)
    Tk2,
}

impl MakeOpDef for QSystemOp {
    fn opdef_id(&self) -> hugr::ops::OpName {
        <&'static str>::from(self).into()
    }

    fn init_signature(&self, _extension_ref: &std::sync::Weak<Extension>) -> SignatureFunc {
        use QSystemOp::*;
        let one_qb_row = TypeRow::from(vec![qb_t()]);
        let two_qb_row = TypeRow::from(vec![qb_t(), qb_t()]);
        match self {
            LazyMeasure => Signature::new(qb_t(), future_type(bool_t())),
            LazyMeasureLeaked => Signature::new(qb_t(), future_type(int_type(6))),
            LazyMeasureReset => Signature::new(qb_t(), vec![qb_t(), future_type(bool_t())]),
            Reset => Signature::new(one_qb_row.clone(), one_qb_row),
            ZZPhase => Signature::new(vec![qb_t(), qb_t(), float64_type()], two_qb_row),
            Measure => Signature::new(one_qb_row, bool_type()),
            Rz => Signature::new(vec![qb_t(), float64_type()], one_qb_row),
            PhasedX => Signature::new(vec![qb_t(), float64_type(), float64_type()], one_qb_row),
            TryQAlloc => Signature::new(type_row![], Type::from(option_type(one_qb_row))),
            QFree => Signature::new(one_qb_row, type_row![]),
            MeasureReset => Signature::new(one_qb_row.clone(), vec![qb_t(), bool_type()]),
            PhasedXX => Signature::new(
                vec![qb_t(), qb_t(), float64_type(), float64_type()],
                two_qb_row,
            ),
            TwinPhasedX => Signature::new(
                vec![qb_t(), qb_t(), float64_type(), float64_type()],
                two_qb_row,
            ),
            Tk2 => Signature::new(
                vec![
                    qb_t(),
                    qb_t(),
                    float64_type(),
                    float64_type(),
                    float64_type(),
                ],
                two_qb_row,
            ),
        }
        .into()
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
        match self {
            QSystemOp::Measure => "Measure a qubit and lose it.",
            QSystemOp::LazyMeasure => "Lazily measure a qubit and lose it.",
            QSystemOp::Rz => {
                "Rotate a qubit around the Z axis. Not physical on Helios or Sol platforms."
            }
            QSystemOp::PhasedX => "PhasedX gate.",
            QSystemOp::ZZPhase => "ZZ gate with an angle, specific to the Helios platform.",
            QSystemOp::TryQAlloc => "Allocate a qubit in the Z |0> eigenstate.",
            QSystemOp::QFree => "Free a qubit (lose track of it).",
            QSystemOp::Reset => "Reset a qubit to the Z |0> eigenstate.",
            QSystemOp::MeasureReset => "Measure a qubit and reset it to the Z |0> eigenstate.",
            QSystemOp::LazyMeasureLeaked => {
                "Measure a qubit (return 0 or 1) or detect leakage (return 2)."
            }
            QSystemOp::LazyMeasureReset => {
                "Lazily measure a qubit and reset it to the Z |0> eigenstate."
            }
            QSystemOp::PhasedXX => "PhasedXX gate, a.k.a. rpp, specific to the Sol platform.",
            QSystemOp::TwinPhasedX => {
                "NPhasedX with N=2, a.k.a. rpg. Specific to the Sol platform."
            }
            QSystemOp::Tk2 => "Tk2 gate, a.k.a. rxxyyzz. Specific to the Sol platform.",
        }
        .to_string()
    }
}

impl MakeRegisteredOp for QSystemOp {
    fn extension_id(&self) -> ExtensionId {
        EXTENSION_ID
    }

    fn extension_ref(&self) -> Arc<Extension> {
        EXTENSION.clone()
    }
}

/// The name of the "tket.qsystem.RuntimeBarrier" operation.
pub const RUNTIME_BARRIER_NAME: OpName = OpName::new_inline("RuntimeBarrier");

/// Helper struct for the "tket.qsystem.RuntimeBarrier" operation definition.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RuntimeBarrierDef;

impl FromStr for RuntimeBarrierDef {
    type Err = ();

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        if s == RuntimeBarrierDef.opdef_id().as_str() {
            Ok(Self)
        } else {
            Err(())
        }
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

    fn init_signature(&self, _extension_ref: &Weak<Extension>) -> SignatureFunc {
        PolyFuncType::new(
            [TypeParam::max_nat_type()],
            Signature::new_endo(
                array_type_parametric(TypeArg::new_var_use(0, TypeParam::max_nat_type()), qb_t())
                    .unwrap(),
            ),
        )
        .into()
    }

    fn description(&self) -> String {
        "Acts as a runtime barrier between operations on argument qubits.".to_string()
    }

    fn opdef_id(&self) -> OpName {
        RUNTIME_BARRIER_NAME
    }
}

/// An extension trait for [Dataflow] providing methods to add
/// "tket.qsystem" operations.
pub trait QSystemOpBuilder: Dataflow + UnwrapBuilder + ArrayOpBuilder {
    /// Add a "tket.qsystem.LazyMeasure" op.
    fn add_lazy_measure(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        Ok(self
            .add_dataflow_op(QSystemOp::LazyMeasure, [qb])?
            .out_wire(0))
    }
    /// Add a "tket.qsystem.PhasedXX" op.
    fn add_phased_xx(
        &mut self,
        platform: QSystemPlatform,
        qb1: Wire,
        qb2: Wire,
        angle1: Wire,
        angle2: Wire,
    ) -> Result<[Wire; 2], BuildError> {
        match platform {
            QSystemPlatform::Helios => {
                unimplemented!("PhasedXX lowering for Helios is not yet implemented")
            }
            QSystemPlatform::Sol => Ok(self
                .add_dataflow_op(QSystemOp::PhasedXX, [qb1, qb2, angle1, angle2])?
                .outputs_arr()),
        }
    }

    /// Add a "tket.qsystem.TwinPhasedX" op.
    fn add_twin_phased_x(
        &mut self,
        platform: QSystemPlatform,
        qb1: Wire,
        qb2: Wire,
        angle1: Wire,
        angle2: Wire,
    ) -> Result<[Wire; 2], BuildError> {
        match platform {
            QSystemPlatform::Helios => {
                unimplemented!("TwinPhasedX lowering for Helios is not yet implemented")
            }
            QSystemPlatform::Sol => Ok(self
                .add_dataflow_op(QSystemOp::TwinPhasedX, [qb1, qb2, angle1, angle2])?
                .outputs_arr()),
        }
    }

    /// Add a "tket.qsystem.Tk2" op.
    fn add_tk2(
        &mut self,
        platform: QSystemPlatform,
        qb1: Wire,
        qb2: Wire,
        angle1: Wire,
        angle2: Wire,
        angle3: Wire,
    ) -> Result<[Wire; 2], BuildError> {
        match platform {
            QSystemPlatform::Helios => {
                unimplemented!("Tk2 lowering for Helios is not yet implemented")
            }
            QSystemPlatform::Sol => Ok(self
                .add_dataflow_op(QSystemOp::Tk2, [qb1, qb2, angle1, angle2, angle3])?
                .outputs_arr()),
        }
    }

    /// Add a "tket.qsystem.LazyMeasureLeaked" op.
    fn add_lazy_measure_leaked(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        Ok(self
            .add_dataflow_op(QSystemOp::LazyMeasureLeaked, [qb])?
            .out_wire(0))
    }

    /// Add a "tket.qsystem.LazyMeasureReset" op.
    fn add_lazy_measure_reset(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError> {
        Ok(self
            .add_dataflow_op(QSystemOp::LazyMeasureReset, [qb])?
            .outputs_arr())
    }

    /// Add a "tket.qsystem.Measure" op.
    fn add_measure(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        Ok(self.add_dataflow_op(QSystemOp::Measure, [qb])?.out_wire(0))
    }

    /// Add a "tket.qsystem.Reset" op.
    fn add_reset(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        Ok(self.add_dataflow_op(QSystemOp::Reset, [qb])?.out_wire(0))
    }

    /// Add a maximally entangling "tket.qsystem.ZZPhase(pi/2)" op.
    fn build_zz_max(
        &mut self,
        platform: QSystemPlatform,
        qb1: Wire,
        qb2: Wire,
    ) -> Result<[Wire; 2], BuildError> {
        match platform {
            QSystemPlatform::Helios => {
                let pi_2 = pi_mul_f64(self, 0.5);
                self.add_zz_phase(platform, qb1, qb2, pi_2)
            }
            QSystemPlatform::Sol => {
                let pi = pi_mul_f64(self, 1.0);
                let pi_2 = pi_mul_f64(self, 0.5);
                let pi_minus_2 = pi_mul_f64(self, -0.5);
                let zero = pi_mul_f64(self, 0.0);

                let qb1 = self.add_rz(platform, qb1, pi)?;
                let qb2 = self.add_rz(platform, qb2, pi)?;
                let [qb1, qb2] = self.add_twin_phased_x(platform, qb1, qb2, pi_2, pi_2)?;
                let [qb1, qb2] = self.add_phased_xx(platform, qb1, qb2, pi_2, zero)?;
                let [qb1, qb2] = self.add_twin_phased_x(platform, qb1, qb2, pi_2, pi_minus_2)?;
                let qb1 = self.add_rz(platform, qb1, pi)?;
                let qb2 = self.add_rz(platform, qb2, pi)?;
                Ok([qb1, qb2])
            }
        }
    }

    /// Add a "tket.qsystem.ZZPhase" op.
    fn add_zz_phase(
        &mut self,
        platform: QSystemPlatform,
        qb1: Wire,
        qb2: Wire,
        angle: Wire,
    ) -> Result<[Wire; 2], BuildError> {
        Ok(match platform {
            QSystemPlatform::Helios => self
                .add_dataflow_op(QSystemOp::ZZPhase, [qb1, qb2, angle])?
                .outputs_arr(),
            QSystemPlatform::Sol => {
                let pi = pi_mul_f64(self, 1.0);
                let pi_2 = pi_mul_f64(self, 0.5);
                let pi_minus_2 = pi_mul_f64(self, -0.5);
                let zero = pi_mul_f64(self, 0.0);

                let qb1 = self.add_rz(platform, qb1, pi)?;
                let qb2 = self.add_rz(platform, qb2, pi)?;
                let [qb1, qb2] = self.add_twin_phased_x(platform, qb1, qb2, pi_2, pi_2)?;
                let [qb1, qb2] = self.add_phased_xx(platform, qb1, qb2, angle, zero)?;
                let [qb1, qb2] = self.add_twin_phased_x(platform, qb1, qb2, pi_2, pi_minus_2)?;
                let qb1 = self.add_rz(platform, qb1, pi)?;
                let qb2 = self.add_rz(platform, qb2, pi)?;

                [qb1, qb2]
                // unimplemented!("ZZPhase lowering for Sol is not yet implemented")
            }
        })
    }

    /// Add a "tket.qsystem.PhasedX" op.
    fn add_phased_x(
        &mut self,
        _platform: QSystemPlatform,
        qb: Wire,
        angle1: Wire,
        angle2: Wire,
    ) -> Result<Wire, BuildError> {
        Ok(self
            .add_dataflow_op(QSystemOp::PhasedX, [qb, angle1, angle2])?
            .out_wire(0))
    }

    /// Add a "tket.qsystem.Rz" op.
    fn add_rz(
        &mut self,
        _platform: QSystemPlatform,
        qb: Wire,
        angle: Wire,
    ) -> Result<Wire, BuildError> {
        Ok(self
            .add_dataflow_op(QSystemOp::Rz, [qb, angle])?
            .out_wire(0))
    }

    /// Add a "tket.qsystem.TryQAlloc" op.
    fn add_try_alloc(&mut self) -> Result<Wire, BuildError> {
        Ok(self.add_dataflow_op(QSystemOp::TryQAlloc, [])?.out_wire(0))
    }

    /// Add a "tket.qsystem.QFree" op.
    fn add_qfree(&mut self, qb: Wire) -> Result<(), BuildError> {
        self.add_dataflow_op(QSystemOp::QFree, [qb])?;
        Ok(())
    }

    /// Add a "tket.qsystem.MeasureReset" op.
    /// This operation is equivalent to a `Measure` followed by a `Reset`.
    fn add_measure_reset(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError> {
        Ok(self
            .add_dataflow_op(QSystemOp::MeasureReset, [qb])?
            .outputs_arr())
    }

    /// Add a "tket.qsystem.RuntimeBarrier" op.
    fn add_runtime_barrier(&mut self, qbs: Wire, array_size: u64) -> Result<Wire, BuildError> {
        let op = runtime_barrier_ext_op(array_size)?;
        Ok(self.add_dataflow_op(op, [qbs])?.out_wire(0))
    }

    /// Build a hadamard gate in terms of QSystem primitives.
    fn build_h(&mut self, platform: QSystemPlatform, qb: Wire) -> Result<Wire, BuildError> {
        let pi = pi_mul_f64(self, 1.0);
        let pi_2 = pi_mul_f64(self, 0.5);
        let pi_minus_2 = pi_mul_f64(self, -0.5);

        let q = self.add_phased_x(platform, qb, pi_2, pi_minus_2)?;
        self.add_rz(platform, q, pi)
    }

    /// Build an X gate in terms of QSystem primitives.
    fn build_x(&mut self, platform: QSystemPlatform, qb: Wire) -> Result<Wire, BuildError> {
        let pi = pi_mul_f64(self, 1.0);
        let zero = pi_mul_f64(self, 0.0);
        self.add_phased_x(platform, qb, pi, zero)
    }

    /// Build a Y gate in terms of QSystem primitives.
    fn build_y(&mut self, platform: QSystemPlatform, qb: Wire) -> Result<Wire, BuildError> {
        let pi = pi_mul_f64(self, 1.0);
        let pi_2 = pi_mul_f64(self, 0.5);
        self.add_phased_x(platform, qb, pi, pi_2)
    }

    /// Build a Z gate in terms of QSystem primitives.
    fn build_z(&mut self, platform: QSystemPlatform, qb: Wire) -> Result<Wire, BuildError> {
        let pi = pi_mul_f64(self, 1.0);
        self.add_rz(platform, qb, pi)
    }

    /// Build an S gate in terms of QSystem primitives.
    fn build_s(&mut self, platform: QSystemPlatform, qb: Wire) -> Result<Wire, BuildError> {
        let pi_2 = pi_mul_f64(self, 0.5);
        self.add_rz(platform, qb, pi_2)
    }

    /// Build an Sdg gate in terms of QSystem primitives.
    fn build_sdg(&mut self, platform: QSystemPlatform, qb: Wire) -> Result<Wire, BuildError> {
        let pi_minus_2 = pi_mul_f64(self, -0.5);
        self.add_rz(platform, qb, pi_minus_2)
    }

    /// Build a V gate in terms of QSystem primitives.
    fn build_v(&mut self, platform: QSystemPlatform, qb: Wire) -> Result<Wire, BuildError> {
        let pi_2 = pi_mul_f64(self, 0.5);
        let zero = pi_mul_f64(self, 0.0);
        self.add_phased_x(platform, qb, pi_2, zero)
    }

    /// Build a Vdg gate in terms of QSystem primitives.
    fn build_vdg(&mut self, platform: QSystemPlatform, qb: Wire) -> Result<Wire, BuildError> {
        let pi_minus_2 = pi_mul_f64(self, -0.5);
        let zero = pi_mul_f64(self, 0.0);
        self.add_phased_x(platform, qb, pi_minus_2, zero)
    }

    /// Build a T gate in terms of QSystem primitives.
    fn build_t(&mut self, platform: QSystemPlatform, qb: Wire) -> Result<Wire, BuildError> {
        let pi_4 = pi_mul_f64(self, 0.25);
        self.add_rz(platform, qb, pi_4)
    }

    /// Build a Tdg gate in terms of QSystem primitives.
    fn build_tdg(&mut self, platform: QSystemPlatform, qb: Wire) -> Result<Wire, BuildError> {
        let pi_minus_4 = pi_mul_f64(self, -0.25);
        self.add_rz(platform, qb, pi_minus_4)
    }

    /// Build a CNOT gate in terms of QSystem primitives.
    fn build_cx(
        &mut self,
        platform: QSystemPlatform,
        c: Wire,
        t: Wire,
    ) -> Result<[Wire; 2], BuildError> {
        Ok(match platform {
            QSystemPlatform::Helios => {
                let pi = pi_mul_f64(self, 1.0);
                let pi_2 = pi_mul_f64(self, 0.5);
                let pi_minus_2 = pi_mul_f64(self, -0.5);

                let t = self.add_phased_x(platform, t, pi_minus_2, pi_2)?;
                let [c, t] = self.build_zz_max(platform, c, t)?;
                let c = self.add_rz(platform, c, pi_minus_2)?;
                let t = self.add_phased_x(platform, t, pi_2, pi)?;
                let t = self.add_rz(platform, t, pi_minus_2)?;
                [c, t]
            }
            QSystemPlatform::Sol => {
                let pi_2 = pi_mul_f64(self, 0.5);
                let pi_minus_2 = pi_mul_f64(self, -0.5);
                let zero = pi_mul_f64(self, 0.0);

                let c = self.add_phased_x(platform, c, pi_2, pi_2)?;
                let [c, t] = self.add_phased_xx(platform, c, t, pi_2, zero)?;
                let c = self.add_phased_x(platform, c, pi_minus_2, pi_2)?;
                let c = self.add_rz(platform, c, pi_minus_2)?;
                let t = self.add_phased_x(platform, t, pi_minus_2, zero)?;
                [c, t]
                // unimplemented!("CX lowering for Sol is not yet implemented")
            }
        })
    }

    /// Build a CY gate in terms of QSystem primitives.
    fn build_cy(
        &mut self,
        platform: QSystemPlatform,
        a: Wire,
        b: Wire,
    ) -> Result<[Wire; 2], BuildError> {
        Ok(match platform {
            QSystemPlatform::Helios => {
                let pi = pi_mul_f64(self, 1.0);
                let pi_2 = pi_mul_f64(self, 0.5);
                let pi_minus_2 = pi_mul_f64(self, -0.5);

                let a = self.add_phased_x(platform, a, pi, pi)?;
                let b = self.add_phased_x(platform, b, pi_minus_2, pi)?;
                let [a, b] = self.build_zz_max(platform, a, b)?;
                let a = self.add_phased_x(platform, a, pi, pi_2)?;
                let b = self.add_phased_x(platform, b, pi_minus_2, pi_minus_2)?;
                let a = self.add_rz(platform, a, pi_minus_2)?;
                let b = self.add_rz(platform, b, pi_2)?;
                [a, b]
            }
            QSystemPlatform::Sol => {
                let pi_2 = pi_mul_f64(self, 0.5);
                let pi_minus_2 = pi_mul_f64(self, -0.5);
                let zero = pi_mul_f64(self, 0.0);

                let a = self.add_phased_x(platform, a, pi_2, pi_2)?;
                let b = self.add_rz(platform, b, pi_minus_2)?;
                let [a, b] = self.add_phased_xx(platform, a, b, pi_2, zero)?;
                let a = self.add_phased_x(platform, a, pi_minus_2, pi_2)?;
                let a = self.add_rz(platform, a, pi_minus_2)?;
                let b = self.add_phased_x(platform, b, pi_minus_2, zero)?;
                let b = self.add_rz(platform, b, pi_2)?;
                [a, b]
                // unimplemented!("CY lowering for Sol is not yet implemented")
            }
        })
    }

    /// Build a CZ gate in terms of QSystem primitives.
    fn build_cz(
        &mut self,
        platform: QSystemPlatform,
        a: Wire,
        b: Wire,
    ) -> Result<[Wire; 2], BuildError> {
        Ok(match platform {
            QSystemPlatform::Helios => {
                let pi_minus_2 = pi_mul_f64(self, -0.5);
                let [a, b] = self.build_zz_max(platform, a, b)?;
                let b = self.add_rz(platform, b, pi_minus_2)?;
                let a = self.add_rz(platform, a, pi_minus_2)?;
                [a, b]
            }
            QSystemPlatform::Sol => {
                let pi = pi_mul_f64(self, 1.0);
                let pi_2 = pi_mul_f64(self, 0.5);
                let pi_minus_2 = pi_mul_f64(self, -0.5);

                let [a, b] = self.add_twin_phased_x(platform, a, b, pi_2, pi_minus_2)?;
                let [a, b] = self.add_phased_xx(platform, a, b, pi_2, pi)?;
                let [a, b] = self.add_twin_phased_x(platform, a, b, pi_2, pi_2)?;
                let a = self.add_rz(platform, a, pi_minus_2)?;
                let b = self.add_rz(platform, b, pi_minus_2)?;
                [a, b]
                // unimplemented!("CZ lowering for Sol is not yet implemented")
            }
        })
    }

    /// Build a RX gate in terms of QSystem primitives.
    fn build_rx(
        &mut self,
        platform: QSystemPlatform,
        qb: Wire,
        theta: Wire,
    ) -> Result<Wire, BuildError> {
        let zero = pi_mul_f64(self, 0.0);
        self.add_phased_x(platform, qb, theta, zero)
    }

    /// Build a RY gate in terms of QSystem primitives.
    fn build_ry(
        &mut self,
        platform: QSystemPlatform,
        qb: Wire,
        theta: Wire,
    ) -> Result<Wire, BuildError> {
        let pi_2 = pi_mul_f64(self, 0.5);
        self.add_phased_x(platform, qb, theta, pi_2)
    }

    /// Build a CRZ gate in terms of QSystem primitives.
    fn build_crz(
        &mut self,
        platform: QSystemPlatform,
        a: Wire,
        b: Wire,
        lambda: Wire,
    ) -> Result<[Wire; 2], BuildError> {
        Ok(match platform {
            QSystemPlatform::Helios => {
                let two = self.add_load_const(Value::from(ConstF64::new(2.0)));
                let lambda_2 = self
                    .add_dataflow_op(FloatOps::fdiv, [lambda, two])?
                    .out_wire(0);
                let lambda_minus_2 = self
                    .add_dataflow_op(FloatOps::fneg, [lambda_2])?
                    .out_wire(0);

                let [a, b] = self.add_zz_phase(platform, a, b, lambda_minus_2)?;
                let b = self.add_rz(platform, b, lambda_2)?;
                [a, b]
            }
            QSystemPlatform::Sol => {
                let two = self.add_load_const(Value::from(ConstF64::new(2.0)));
                let lambda_2 = self
                    .add_dataflow_op(FloatOps::fdiv, [lambda, two])?
                    .out_wire(0);
                let lambda_minus_2 = self
                    .add_dataflow_op(FloatOps::fneg, [lambda_2])?
                    .out_wire(0);

                let pi = pi_mul_f64(self, 1.0);
                let pi_2 = pi_mul_f64(self, 0.5);
                let pi_minus_2 = pi_mul_f64(self, -0.5);

                let [a, b] = self.add_twin_phased_x(platform, a, b, pi_2, pi_minus_2)?;
                let [a, b] = self.add_phased_xx(platform, a, b, lambda_minus_2, pi)?;
                let [a, b] = self.add_twin_phased_x(platform, a, b, pi_2, pi_2)?;

                let b = self.add_rz(platform, b, lambda_2)?;
                [a, b]

                // unimplemented!("CRZ lowering for Sol is not yet implemented")
            }
        })
    }

    /// Build a Toffoli (CCX) gate in terms of QSystem primitives.
    fn build_toffoli(
        &mut self,
        platform: QSystemPlatform,
        a: Wire,
        b: Wire,
        c: Wire,
    ) -> Result<[Wire; 3], BuildError> {
        Ok(match platform {
            QSystemPlatform::Helios => {
                let pi = pi_mul_f64(self, 1.0);
                let pi_2 = pi_mul_f64(self, 0.5);
                let pi_minus_2 = pi_mul_f64(self, -0.5);
                let pi_4 = pi_mul_f64(self, 0.25);
                let pi_minus_4 = pi_mul_f64(self, -0.25);
                let pi_minus_3_4 = pi_mul_f64(self, -0.75);
                let zero = pi_mul_f64(self, 0.0);

                let c = self.add_phased_x(platform, c, pi, pi_minus_2)?;
                let [b, c] = self.build_zz_max(platform, b, c)?;
                let c = self.add_phased_x(platform, c, pi_4, pi_2)?;
                let [a, c] = self.build_zz_max(platform, a, c)?;
                let c = self.add_phased_x(platform, c, pi_4, zero)?;
                let [b, c] = self.build_zz_max(platform, b, c)?;
                let c = self.add_phased_x(platform, c, pi_4, pi_minus_2)?;
                let [a, c] = self.build_zz_max(platform, a, c)?;
                let a = self.add_phased_x(platform, a, pi, pi_4)?;
                let c = self.add_phased_x(platform, c, pi_minus_3_4, pi)?;
                let [a, b] = self.add_zz_phase(platform, a, b, pi_4)?;
                let c = self.add_rz(platform, c, pi)?;
                let a = self.add_phased_x(platform, a, pi, pi_minus_4)?;
                let b = self.add_rz(platform, b, pi_minus_3_4)?;
                let a = self.add_rz(platform, a, pi_4)?;

                [a, b, c]
            }
            QSystemPlatform::Sol => {
                let pi = pi_mul_f64(self, 1.0);
                let pi_2 = pi_mul_f64(self, 0.5);
                let pi_minus_2 = pi_mul_f64(self, -0.5);
                let pi_4 = pi_mul_f64(self, 0.25);
                let pi_minus_4 = pi_mul_f64(self, -0.25);
                let pi_minus_3_4 = pi_mul_f64(self, -0.75);
                let zero = pi_mul_f64(self, 0.0);

                let [a, b] = self.add_twin_phased_x(platform, a, b, pi_2, pi_minus_3_4)?;
                let a = self.add_rz(platform, a, pi_minus_3_4)?;
                let b = self.add_rz(platform, b, pi_minus_3_4)?;
                let c = self.add_phased_x(platform, c, pi_2, pi_minus_2)?;
                let c = self.add_rz(platform, c, pi_minus_3_4)?;

                let [a, c] = self.add_phased_xx(platform, a, c, pi_2, zero)?;

                let a = self.add_phased_x(platform, a, pi_minus_2, zero)?;
                let c = self.add_phased_x(platform, c, pi_4, pi_minus_2)?;

                let [a, b] = self.add_phased_xx(platform, a, b, pi_minus_4, zero)?;
                let c = self.add_rz(platform, c, pi_2)?;

                let [b, c] = self.add_phased_xx(platform, b, c, pi_4, zero)?;
                let c = self.add_phased_x(platform, c, pi_2, pi_minus_2)?;
                let c = self.add_rz(platform, c, pi)?;
                

                let [a, c] = self.add_phased_xx(platform, a, c, pi_2, zero)?;
                let [a, c] = self.add_twin_phased_x(platform, a, c, pi_2, pi_minus_2)?;
                let a = self.add_rz(platform, a, pi_2)?;
                let c = self.add_rz(platform, c, pi_2)?;

                let [b, c] = self.add_phased_xx(platform, b, c, pi_minus_4, zero)?;
                let b = self.add_phased_x(platform, b, pi_2, pi_minus_2)?;
                let b = self.add_rz(platform, b, pi)?;

                [a, b, c]
                // unimplemented!("Toffoli lowering for Sol is not yet implemented")
            }
        })
    }

    /// Build a projective measurement with a conditional flip.
    fn build_measure_flip(
        &mut self,
        platform: QSystemPlatform,
        qb: Wire,
    ) -> Result<[Wire; 2], BuildError> {
        let [qb, b] = self.add_measure_reset(qb)?;
        let sum_b = self.add_dataflow_op(BoolOp::read, [b])?.out_wire(0);
        let mut conditional = self.conditional_builder(
            ([type_row![], type_row![]], sum_b),
            [(qb_t(), qb)],
            vec![qb_t()].into(),
        )?;

        // case 0: 0 state measured, leave alone
        let case0 = conditional.case_builder(0)?;
        let [qb] = case0.input_wires_arr();
        case0.finish_with_outputs([qb])?;

        // case 1: 1 state measured, flip
        let mut case1 = conditional.case_builder(1)?;
        let [qb] = case1.input_wires_arr();
        let qb = case1.build_x(platform, qb)?;
        case1.finish_with_outputs([qb])?;

        let [qb] = conditional.finish_sub_container()?.outputs_arr();
        Ok([qb, sum_b])
    }

    /// Build a qalloc operation that panics on failure.
    fn build_qalloc(&mut self) -> Result<Wire, BuildError> {
        let maybe_qb = self.add_try_alloc()?;
        let [qb] = self.build_expect_sum(1, option_type(qb_t()), maybe_qb, |_| {
            "No more qubits available to allocate.".to_string()
        })?;
        Ok(qb)
    }

    /// Build an array from qubit wires, apply a barrier, and unwrap the array afterwards.
    fn build_wrapped_barrier(
        &mut self,
        qbs: impl IntoIterator<Item = Wire>,
    ) -> Result<Vec<Wire>, BuildError>
    where
        Self: Sized,
    {
        let qbs: Vec<_> = qbs.into_iter().collect();
        let size = qbs.len() as u64;
        let q_arr = self.add_new_array(qb_t(), qbs)?;
        let q_arr = self.add_runtime_barrier(q_arr, size)?;

        self.add_array_unpack(qb_t(), size, q_arr)
    }
}

/// Build a runtime barrier operation on an array of qubits given its size.
pub(crate) fn runtime_barrier_ext_op(
    array_size: u64,
) -> Result<ExtensionOp, hugr::extension::SignatureError> {
    ExtensionOp::new(
        EXTENSION.get_op(&RUNTIME_BARRIER_NAME).unwrap().clone(),
        [TypeArg::BoundedNat(array_size)],
    )
}

impl<D: Dataflow> QSystemOpBuilder for D {}

#[cfg(test)]
mod test {
    use std::sync::Arc;

    use cool_asserts::assert_matches;
    use futures::FutureOpBuilder as _;
    use hugr::HugrView;
    use hugr::builder::{DataflowHugr, FunctionBuilder};
    use hugr::extension::simple_op::MakeExtensionOp;
    use hugr::ops::OpType;
    use rstest::rstest;
    use strum::IntoEnumIterator as _;

    use super::*;

    fn get_opdef(op: QSystemOp) -> Option<&'static Arc<OpDef>> {
        EXTENSION.get_op(&op.op_id())
    }

    #[test]
    fn create_extension() {
        assert_eq!(EXTENSION.name(), &EXTENSION_ID);

        for o in QSystemOp::iter() {
            assert_eq!(QSystemOp::from_def(get_opdef(o).unwrap()), Ok(o));
        }
    }

    #[test]
    fn lazy_circuit() {
        let hugr = {
            let mut func_builder =
                FunctionBuilder::new("circuit", Signature::new(qb_t(), vec![qb_t(), bool_t()]))
                    .unwrap();
            let [qb] = func_builder.input_wires_arr();
            let [qb, lazy_b] = func_builder.add_lazy_measure_reset(qb).unwrap();
            let [b] = func_builder.add_read(lazy_b, bool_t()).unwrap();
            func_builder.finish_hugr_with_outputs([qb, b]).unwrap()
        };
        assert_matches!(hugr.validate(), Ok(_));
    }

    #[test]
    fn leaked() {
        let hugr = {
            let mut func_builder =
                FunctionBuilder::new("leaked", Signature::new(qb_t(), vec![int_type(6)])).unwrap();
            let [qb] = func_builder.input_wires_arr();
            let lazy_i = func_builder.add_lazy_measure_leaked(qb).unwrap();
            let [i] = func_builder.add_read(lazy_i, int_type(6)).unwrap();
            func_builder.finish_hugr_with_outputs([i]).unwrap()
        };
        assert_matches!(hugr.validate(), Ok(_));
    }

    #[rstest]
    #[case(QSystemPlatform::Helios)]
    //TODO: uncomment when 2q operation rebasing is supported for Sol
    //#[case(QSystemPlatform::Sol)]
    fn all_ops(#[case] platform: QSystemPlatform) {
        let hugr = {
            let mut func_builder = FunctionBuilder::new(
                "all_ops",
                Signature::new(vec![qb_t(), float64_type()], vec![bool_type()]),
            )
            .unwrap();
            let [q0, angle] = func_builder.input_wires_arr();
            let q1 = func_builder.build_qalloc().unwrap();
            let q0 = func_builder.add_reset(q0).unwrap();
            let q1 = func_builder
                .add_phased_x(platform, q1, angle, angle)
                .unwrap();
            let [q0, q1] = func_builder.build_zz_max(platform, q0, q1).unwrap();
            let [q0, q1] = func_builder.add_zz_phase(platform, q0, q1, angle).unwrap();

            let [q0, q1] = func_builder
                .build_wrapped_barrier([q0, q1])
                .unwrap()
                .try_into()
                .unwrap();

            let q0 = func_builder.add_rz(platform, q0, angle).unwrap();
            let [q0, _b] = func_builder.add_measure_reset(q0).unwrap();
            let b = func_builder.add_measure(q0).unwrap();
            func_builder.add_qfree(q1).unwrap();

            func_builder.finish_hugr_with_outputs([b]).unwrap()
        };
        hugr.validate().unwrap()
    }

    #[test]
    fn test_cast() {
        // test overlapping names don't cause cast errors
        for op in QSystemOp::iter() {
            let optype: OpType = op.into();
            let new_op: QSystemOp = optype.cast().unwrap();
            assert_eq!(op, new_op);
            assert_eq!(optype.cast::<tket::TketOp>(), None);
        }
    }
}
