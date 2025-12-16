use hugr::Wire;
use hugr::builder::{BuildError, Dataflow};
use hugr::extension::simple_op::{MakeOpDef, MakeRegisteredOp};
use hugr::extension::{ExtensionId, Version, prelude::option_type};
use hugr::ops::constant::{CustomConst, TryHash, downcast_equal_consts};
use hugr::std_extensions::arithmetic::float_types::float64_type;
use hugr::{
    Extension,
    types::{ConstTypeError, CustomType, Signature, Type, TypeBound},
};
use smol_str::SmolStr;
use std::f64::consts::PI;
use std::sync::{Arc, Weak};
use strum::{EnumIter, EnumString, IntoStaticStr};

use lazy_static::lazy_static;

/// Name of tket rotation extension.
pub const ROTATION_EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.rotation");

/// Current version of the TKET rotation extension
pub const ROTATION_EXTENSION_VERSION: Version = Version::new(0, 2, 0);

lazy_static! {
    /// The extension definition for TKET rotation type and ops.
    pub static ref ROTATION_EXTENSION: Arc<Extension> =  {
            Extension::new_arc(ROTATION_EXTENSION_ID, ROTATION_EXTENSION_VERSION, |e, extension_ref| {
                add_to_extension(e, extension_ref);
            }
    )};
}

/// Identifier for the rotation type.
pub const ROTATION_TYPE_ID: SmolStr = SmolStr::new_inline("rotation");
/// Rotation type (as [CustomType])
pub fn rotation_custom_type(extension_ref: &Weak<Extension>) -> CustomType {
    CustomType::new(
        ROTATION_TYPE_ID,
        [],
        ROTATION_EXTENSION_ID,
        TypeBound::Copyable,
        extension_ref,
    )
}

/// Type representing a rotation that is a number of half turns (as [Type])
pub fn rotation_type() -> Type {
    rotation_custom_type(&Arc::downgrade(&ROTATION_EXTENSION)).into()
}

/// A rotation
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ConstRotation {
    half_turns: f64,
}

impl ConstRotation {
    /// The constant π
    pub const PI: Self = Self::new_unchecked(1.0);
    /// The constant 2π
    pub const TAU: Self = Self::new_unchecked(2.0);
    /// The constant π/2
    pub const PI_2: Self = Self::new_unchecked(0.5);
    /// The constant π/4
    pub const PI_4: Self = Self::new_unchecked(0.25);

    const fn new_unchecked(half_turns: f64) -> Self {
        Self { half_turns }
    }
    /// Create a new [`ConstRotation`] from a number of half turns
    pub fn new(half_turns: f64) -> Result<Self, ConstTypeError> {
        // test for a valid float value
        if half_turns.is_nan() || half_turns.is_infinite() {
            return Err(ConstTypeError::CustomCheckFail(
                hugr::types::CustomCheckFailure::Message(format!(
                    "Invalid rotation value {half_turns}."
                )),
            ));
        }
        Ok(Self { half_turns })
    }

    /// Returns the value of the constant in radians
    pub fn to_radians(&self) -> f64 {
        self.half_turns * PI
    }

    /// Create a new [`ConstRotation`] from a floating-point value in radians,
    pub fn from_radians(theta: f64) -> Result<Self, ConstTypeError> {
        Self::new(theta / PI)
    }

    /// Returns the number of half turns in the rotation.
    pub fn half_turns(&self) -> f64 {
        self.half_turns
    }
}
impl TryHash for ConstRotation {}

#[typetag::serde]
impl CustomConst for ConstRotation {
    fn name(&self) -> SmolStr {
        format!("a(π*{})", self.half_turns).into()
    }

    fn get_type(&self) -> Type {
        rotation_type()
    }

    fn equal_consts(&self, other: &dyn CustomConst) -> bool {
        downcast_equal_consts(self, other)
    }
}

#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq, EnumIter, IntoStaticStr, EnumString)]
#[expect(non_camel_case_types)]
#[non_exhaustive]
/// Rotation operations
pub enum RotationOp {
    /// Construct rotation from a floating point number of half-turns (would be multiples of PI in radians).
    /// Returns an Option, failing when the input is NaN or infinite.
    from_halfturns,
    /// Construct rotation from a floating point number of half-turns (would be multiples of PI in radians).
    /// Panics if the input is NaN or infinite.
    from_halfturns_unchecked,
    /// Convert rotation to number of half-turns (would be multiples of PI in radians).
    to_halfturns,
    /// Add two angles together (experimental, may be removed, use float addition
    /// first instead if possible).
    radd,
}

impl MakeOpDef for RotationOp {
    fn opdef_id(&self) -> hugr::ops::OpName {
        <&'static str>::from(self).into()
    }

    fn from_def(
        op_def: &hugr::extension::OpDef,
    ) -> Result<Self, hugr::extension::simple_op::OpLoadError>
    where
        Self: Sized,
    {
        hugr::extension::simple_op::try_from_name(op_def.name(), op_def.extension_id())
    }

    fn init_signature(&self, extension_ref: &Weak<Extension>) -> hugr::extension::SignatureFunc {
        let rotation_type = Type::new_extension(rotation_custom_type(extension_ref));
        match self {
            RotationOp::from_halfturns => Signature::new(
                float64_type(),
                Type::from(option_type(rotation_type.clone())),
            ),
            RotationOp::from_halfturns_unchecked => {
                Signature::new(float64_type(), rotation_type.clone())
            }
            RotationOp::to_halfturns => Signature::new(rotation_type.clone(), float64_type()),
            RotationOp::radd => Signature::new(
                vec![rotation_type.clone(), rotation_type.clone()],
                rotation_type,
            ),
        }
        .into()
    }

    fn description(&self) -> String {
        match self {
            RotationOp::from_halfturns => {
                "Construct rotation from number of half-turns (would be multiples of PI in radians). Returns None if the float is non-finite."
            }
            RotationOp::from_halfturns_unchecked => {
                "Construct rotation from number of half-turns (would be multiples of PI in radians). Panics if the float is non-finite."
            }
            RotationOp::to_halfturns => {
                "Convert rotation to number of half-turns (would be multiples of PI in radians)."
            }
            RotationOp::radd => "Add two angles together (experimental).",
        }
        .to_owned()
    }

    fn extension(&self) -> hugr::extension::ExtensionId {
        ROTATION_EXTENSION_ID
    }

    fn extension_ref(&self) -> Weak<Extension> {
        Arc::downgrade(&ROTATION_EXTENSION)
    }

    // TODO constant folding
    // https://github.com/quantinuum/tket2/issues/405
}

impl MakeRegisteredOp for RotationOp {
    fn extension_id(&self) -> hugr::extension::ExtensionId {
        ROTATION_EXTENSION_ID
    }

    fn extension_ref(&self) -> Arc<Extension> {
        ROTATION_EXTENSION.clone()
    }
}

pub(super) fn add_to_extension(extension: &mut Extension, extension_ref: &Weak<Extension>) {
    extension
        .add_type(
            ROTATION_TYPE_ID,
            vec![],
            "rotation type expressed as number of half turns".to_owned(),
            TypeBound::Copyable.into(),
            extension_ref,
        )
        .unwrap();

    RotationOp::load_all_ops(extension, extension_ref).expect("add fail");
}

/// An extension trait for [Dataflow] providing methods to add
/// "tket.rotation" operations.
pub trait RotationOpBuilder: Dataflow {
    /// Add a "tket.rotation.from_halfturns" op.
    fn add_from_halfturns(&mut self, turns: Wire) -> Result<Wire, BuildError> {
        Ok(self
            .add_dataflow_op(RotationOp::from_halfturns, [turns])?
            .out_wire(0))
    }

    /// Add a "tket.rotation.from_halfturns_unchecked" op.
    fn add_from_halfturns_unchecked(&mut self, turns: Wire) -> Result<Wire, BuildError> {
        Ok(self
            .add_dataflow_op(RotationOp::from_halfturns_unchecked, [turns])?
            .out_wire(0))
    }

    /// Add a "tket.rotation.to_halfturns" op.
    fn add_to_halfturns(&mut self, rotation: Wire) -> Result<Wire, BuildError> {
        Ok(self
            .add_dataflow_op(RotationOp::to_halfturns, [rotation])?
            .out_wire(0))
    }
}

impl<D: Dataflow> RotationOpBuilder for D {}

#[cfg(test)]
mod test {
    use hugr::{
        builder::{DFGBuilder, DataflowHugr},
        ops::OpType,
    };
    use strum::IntoEnumIterator;

    use super::*;

    #[test]
    fn test_rotation_consts() {
        let const_57 = ConstRotation::new(5.7).unwrap();
        let const_01 = ConstRotation::new(0.1).unwrap();
        let const_256 = ConstRotation::new(256.0).unwrap();
        assert_ne!(const_57, const_01);
        assert_ne!(const_57, const_256);
        assert_eq!(const_57, ConstRotation::new(5.7).unwrap());

        assert_eq!(const_57.get_type(), rotation_type());
        assert!(matches!(
            ConstRotation::new(f64::INFINITY),
            Err(ConstTypeError::CustomCheckFail(_))
        ));
        assert!(matches!(
            ConstRotation::new(f64::NAN),
            Err(ConstTypeError::CustomCheckFail(_))
        ));
        let const_af1 = ConstRotation::from_radians(0.75 * PI).unwrap();
        assert_eq!(const_af1.half_turns(), 0.75);

        assert!(const_57.equal_consts(&ConstRotation::new(5.7).unwrap()));
        assert_ne!(const_57, const_01);

        assert_eq!(const_256.name(), "a(π*256)");
    }

    #[test]
    fn test_ops() {
        let ops = RotationOp::iter().collect::<Vec<_>>();
        for op in ops {
            let optype: OpType = op.into();
            assert_eq!(optype.cast(), Some(op));
        }
    }

    #[test]
    fn test_builder() {
        let mut builder = DFGBuilder::new(Signature::new(
            rotation_type(),
            vec![Type::from(option_type(rotation_type())), rotation_type()],
        ))
        .unwrap();

        let [rotation] = builder.input_wires_arr();
        let turns = builder.add_to_halfturns(rotation).unwrap();
        let mb_rotation = builder.add_from_halfturns(turns).unwrap();
        let unwrapped_rotation = builder.add_from_halfturns_unchecked(turns).unwrap();
        let _hugr = builder
            .finish_hugr_with_outputs([mb_rotation, unwrapped_rotation])
            .unwrap();
    }

    #[rstest::rstest]
    fn const_rotation_statics(
        #[values(
            ConstRotation::TAU,
            ConstRotation::PI,
            ConstRotation::PI_2,
            ConstRotation::PI_4
        )]
        konst: ConstRotation,
    ) {
        assert_eq!(ConstRotation::new(konst.half_turns()), Ok(konst));
    }
}
