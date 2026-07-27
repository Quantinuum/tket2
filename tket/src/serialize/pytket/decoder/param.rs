//! Definition of a loaded parameter (either floating point or a rotation type) attached to a HUGR wire.
pub(super) mod parser;

use std::sync::LazyLock;

use hugr::builder::{DFGBuilder, Dataflow};
use hugr::ops::Value;
use hugr::std_extensions::arithmetic::float_ops::FloatOps;
use hugr::std_extensions::arithmetic::float_types::{ConstF64, float64_type};
use hugr::types::Type;
use hugr::{Hugr, Wire};

use crate::extension::rotation::{RotationOp, rotation_type};

/// The type of a loaded parameter in the Hugr, including its unit.
#[derive(Debug, derive_more::Display, Clone, Copy, Hash, PartialEq, Eq)]
pub enum ParameterType {
    /// A float parameter in radians.
    FloatRadians,
    /// A float parameter in half-turns.
    FloatHalfTurns,
    /// A rotation parameter in half-turns.
    Rotation,
}

impl ParameterType {
    /// Returns the type of the parameter.
    pub fn to_type(&self) -> &'static Type {
        static FLOAT_TYPE: LazyLock<Type> = LazyLock::new(float64_type);
        static ROTATION_TYPE: LazyLock<Type> = LazyLock::new(rotation_type);
        match self {
            ParameterType::FloatRadians => &FLOAT_TYPE,
            ParameterType::FloatHalfTurns => &FLOAT_TYPE,
            ParameterType::Rotation => &ROTATION_TYPE,
        }
    }
}

/// A loaded parameter in the Hugr.
///
/// Tracking the type of the wire lets us delay conversion between the types
/// until they are actually needed.
#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq)]
pub struct LoadedParameter {
    /// The type of the parameter.
    typ: ParameterType,
    /// The wire where the parameter is loaded.
    wire: Wire,
}

impl LoadedParameter {
    /// Returns a `LoadedParameter` with the given type and unit.
    pub fn new(typ: ParameterType, wire: Wire) -> LoadedParameter {
        LoadedParameter { typ, wire }
    }

    /// Returns the type of the parameter.
    #[inline]
    pub fn typ(&self) -> ParameterType {
        self.typ
    }

    /// Returns the wire where the parameter is loaded.
    #[inline]
    pub fn wire(&self) -> Wire {
        self.wire
    }

    /// Returns a `LoadedParameter` for a float param in radians.
    #[inline]
    pub fn float_radians(wire: Wire) -> LoadedParameter {
        LoadedParameter {
            typ: ParameterType::FloatRadians,
            wire,
        }
    }

    /// Returns a `LoadedParameter` for a float param in half-turns.
    #[inline]
    pub fn float_half_turns(wire: Wire) -> LoadedParameter {
        LoadedParameter {
            typ: ParameterType::FloatHalfTurns,
            wire,
        }
    }

    /// Returns a `LoadedParameter` for a rotation param in half-turns.
    #[inline]
    pub fn rotation(wire: Wire) -> LoadedParameter {
        LoadedParameter {
            typ: ParameterType::Rotation,
            wire,
        }
    }

    /// Returns the hugr type for the parameter.
    #[inline]
    pub fn wire_type(&self) -> &'static Type {
        self.typ.to_type()
    }

    /// Convert the parameter into a given type, if necessary.
    ///
    /// Adds the necessary operations to the Hugr and returns a new wire.
    ///
    /// See [`LoadedParameter::as_rotation`],
    /// [`LoadedParameter::as_float_radians`] and
    /// [`LoadedParameter::as_float_half_turns`] for more convenient methods.
    #[inline]
    pub fn with_type<H: AsRef<Hugr> + AsMut<Hugr>>(
        &self,
        typ: ParameterType,
        hugr: &mut DFGBuilder<H>,
    ) -> LoadedParameter {
        match typ {
            ParameterType::FloatRadians => self.as_float_radians(hugr),
            ParameterType::FloatHalfTurns => self.as_float_half_turns(hugr),
            ParameterType::Rotation => self.as_rotation(hugr),
        }
    }

    /// Convert the parameter into a float in radians.
    ///
    /// Adds the necessary operations to the Hugr and returns a new wire.
    pub fn as_float_radians<H: AsRef<Hugr> + AsMut<Hugr>>(
        &self,
        hugr: &mut DFGBuilder<H>,
    ) -> LoadedParameter {
        match self.typ {
            ParameterType::FloatRadians => *self,
            ParameterType::FloatHalfTurns => {
                let pi = hugr.add_load_const(Value::from(ConstF64::new(std::f64::consts::PI)));
                let float_radians = hugr
                    .add_dataflow_op(FloatOps::fmul, [self.wire(), pi])
                    .expect("Error converting float to rotation")
                    .out_wire(0);
                LoadedParameter::float_radians(float_radians)
            }
            ParameterType::Rotation => self.as_float_half_turns(hugr).as_float_radians(hugr),
        }
    }

    /// Convert the parameter into a float in half-turns.
    ///
    /// Adds the necessary operations to the Hugr and returns a new wire.
    pub fn as_float_half_turns<H: AsRef<Hugr> + AsMut<Hugr>>(
        &self,
        hugr: &mut DFGBuilder<H>,
    ) -> LoadedParameter {
        match self.typ {
            ParameterType::FloatHalfTurns => *self,
            ParameterType::FloatRadians => {
                let pi = hugr.add_load_const(Value::from(ConstF64::new(std::f64::consts::PI)));
                let float_halfturns = hugr
                    .add_dataflow_op(FloatOps::fdiv, [self.wire, pi])
                    .expect("Error converting float to rotation")
                    .out_wire(0);
                LoadedParameter::float_half_turns(float_halfturns)
            }
            ParameterType::Rotation => {
                let wire = hugr
                    .add_dataflow_op(RotationOp::to_halfturns, [self.wire()])
                    .unwrap()
                    .out_wire(0);
                LoadedParameter::float_half_turns(wire)
            }
        }
    }

    /// Convert the parameter into a rotation in half-turns.
    ///
    /// Adds the necessary operations to the Hugr and returns a new wire.
    pub fn as_rotation<H: AsRef<Hugr> + AsMut<Hugr>>(
        &self,
        hugr: &mut DFGBuilder<H>,
    ) -> LoadedParameter {
        match self.typ {
            ParameterType::Rotation => *self,
            ParameterType::FloatHalfTurns => {
                let wire = hugr
                    .add_dataflow_op(RotationOp::from_halfturns_unchecked, [self.wire()])
                    .unwrap()
                    .out_wire(0);
                LoadedParameter::rotation(wire)
            }
            ParameterType::FloatRadians => self.as_float_half_turns(hugr).as_rotation(hugr),
        }
    }
}
