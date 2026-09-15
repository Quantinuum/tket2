use hugr::extension::{Version, simple_op::MakeRegisteredOp};
use hugr::std_extensions::logic::LogicOp;

use crate::update_maps::{
    OpReplacementTemplate, OpUpdateMap, TypeMapping, TypeReplacementTemplate, VersionedElement,
};
use tket::{hugr::extension::prelude::bool_t, passes::replace_types::NodeTemplate};

pub fn get_measurement_migratation_op_map() -> OpUpdateMap {
    vec![
        (
            VersionedElement::new(
                "MeasureFree".to_string(),
                "tket.quantum".to_string(),
                Version::new(0, 2, 1),
            ),
            OpReplacementTemplate::VersionedElements(vec![
                VersionedElement::new(
                    "MeasureFree".to_string(),
                    "tket.quantum".to_string(),
                    Version::new(0, 3, 0),
                ),
                VersionedElement::new(
                    "Read".to_string(),
                    "tket.measurement".to_string(),
                    Version::new(0, 1, 0),
                ),
            ]),
        ),
        (
            VersionedElement::new(
                "read".to_string(),
                "tket.bool".to_string(),
                Version::new(0, 2, 0),
            ),
            OpReplacementTemplate::Empty,
        ),
        (
            VersionedElement::new(
                "and".to_string(),
                "tket.bool".to_string(),
                Version::new(0, 2, 0),
            ),
            OpReplacementTemplate::TemplateInstance(NodeTemplate::SingleOp(
                LogicOp::And.to_extension_op().unwrap().into(),
            )),
        ),
        (
            VersionedElement::new(
                "eq".to_string(),
                "tket.bool".to_string(),
                Version::new(0, 2, 0),
            ),
            OpReplacementTemplate::TemplateInstance(NodeTemplate::SingleOp(
                LogicOp::Eq.to_extension_op().unwrap().into(),
            )),
        ),
        (
            VersionedElement::new(
                "not".to_string(),
                "tket.bool".to_string(),
                Version::new(0, 2, 0),
            ),
            OpReplacementTemplate::TemplateInstance(NodeTemplate::SingleOp(
                LogicOp::Not.to_extension_op().unwrap().into(),
            )),
        ),
        (
            VersionedElement::new(
                "or".to_string(),
                "tket.bool".to_string(),
                Version::new(0, 2, 0),
            ),
            // NICOLA: TODO: Using VersionedElements for testing reason
            OpReplacementTemplate::VersionedElements(vec![VersionedElement::new(
                "Or".to_string(),
                "logic".to_string(),
                Version::new(0, 1, 0),
            )]),
        ),
        (
            VersionedElement::new(
                "xor".to_string(),
                "tket.bool".to_string(),
                Version::new(0, 2, 0),
            ),
            OpReplacementTemplate::VersionedElements(vec![VersionedElement::new(
                "Xor".to_string(),
                "logic".to_string(),
                Version::new(0, 1, 0),
            )]),
        ),
        (
            VersionedElement::new(
                "read".to_string(),
                "tket.bool".to_string(),
                Version::new(0, 2, 0),
            ),
            OpReplacementTemplate::Empty,
        ),
        (
            VersionedElement::new(
                "make_opaque".to_string(),
                "tket.bool".to_string(),
                Version::new(0, 2, 0),
            ),
            OpReplacementTemplate::Empty,
        ),
    ]
    .into()
}

pub fn get_measurement_migratation_type_map() -> TypeMapping {
    vec![(
        VersionedElement::new(
            "bool".to_string(),
            "tket.bool".to_string(),
            Version::new(0, 2, 0),
        ),
        TypeReplacementTemplate::Type(bool_t()),
    )]
    .into()
}
