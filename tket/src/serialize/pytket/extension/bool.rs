//! Encoder and decoder for the tket.bool extension

use super::PytketEmitter;
use crate::Circuit;
use crate::extension::bool::{BOOL_EXTENSION_ID, BOOL_TYPE_NAME, BoolOp, ConstBool};
use crate::serialize::pytket::config::TypeTranslatorSet;
use crate::serialize::pytket::decoder::{
    DecodeStatus, LoadedParameter, PytketDecoderContext, TrackedBit, TrackedQubit,
};
use crate::serialize::pytket::encoder::{
    EmitCommandOptions, EncodeStatus, PytketEncoderContext, TrackedValues,
    make_tk1_classical_expression, make_tk1_classical_operation,
};
use crate::serialize::pytket::extension::{PytketDecoder, PytketTypeTranslator, RegisterCount};
use crate::serialize::pytket::{PytketDecodeError, PytketEncodeError};
use hugr::HugrView;
use hugr::extension::ExtensionId;
use hugr::extension::simple_op::MakeExtensionOp;
use hugr::ops::ExtensionOp;
use hugr::ops::constant::OpaqueValue;
use itertools::Itertools;
use tket_json_rs::clexpr::op::ClOp;
use tket_json_rs::clexpr::operator::{ClArgument, ClOperator, ClTerminal, ClVariable};

/// Encoder for [prelude](hugr::extension::prelude) operations.
#[derive(Debug, Clone, Default)]
pub struct BoolEmitter;

impl<H: HugrView> PytketEmitter<H> for BoolEmitter {
    fn extensions(&self) -> Option<Vec<ExtensionId>> {
        Some(vec![BOOL_EXTENSION_ID])
    }

    fn op_to_pytket(
        &self,
        node: H::Node,
        op: &ExtensionOp,
        circ: &Circuit<H>,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<EncodeStatus, PytketEncodeError<H::Node>> {
        let Ok(rot_op) = BoolOp::from_extension_op(op) else {
            return Ok(EncodeStatus::Unsupported);
        };

        let (num_inputs, num_outputs, clop) = match rot_op {
            // Conversion ops between native bools and `tket.bool`.
            // Both are represented as a pytket bit, so this is a no-op.
            BoolOp::read | BoolOp::make_opaque => {
                encoder.emit_transparent_node(node, circ, |_| Vec::new())?;
                return Ok(EncodeStatus::Success);
            }
            BoolOp::eq => (2, 1, ClOp::BitEq),
            BoolOp::not => (1, 1, ClOp::BitNot),
            BoolOp::and => (2, 1, ClOp::BitAnd),
            BoolOp::or => (2, 1, ClOp::BitOr),
            BoolOp::xor => (2, 1, ClOp::BitXor),
        };

        // We assume here all operations are a single expression node, with only
        // variable inputs. If new [`BoolOp`]s are added that do not follow
        // this, the following code will need to be adjusted.
        let bit_count = (num_inputs + num_outputs) as usize;
        let output_bits = (0..num_outputs).collect_vec();
        let mut expression = ClOperator::default();
        expression.op = clop;
        expression.args = (0..num_inputs)
            .map(|i| ClArgument::Terminal(ClTerminal::Variable(ClVariable::Bit { index: i })))
            .collect_vec();

        let op = make_tk1_classical_expression(bit_count, &output_bits, &[], expression);
        encoder.emit_node_command(node, circ, EmitCommandOptions::new(), move |_| op)?;
        Ok(EncodeStatus::Success)
    }

    fn const_to_pytket(
        &self,
        value: &OpaqueValue,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<Option<TrackedValues>, PytketEncodeError<H::Node>> {
        let Some(const_b) = value.value().downcast_ref::<ConstBool>() else {
            return Ok(None);
        };

        let new_bit = encoder.values.new_bit();
        if const_b.value() {
            let op = set_bits_op(&[true]);
            encoder.emit_command(op, &[], &[new_bit], None);
        }

        Ok(Some(TrackedValues::new_bits([new_bit])))
    }
}

impl PytketTypeTranslator for BoolEmitter {
    fn extensions(&self) -> Vec<ExtensionId> {
        vec![BOOL_EXTENSION_ID]
    }

    fn type_to_pytket(
        &self,
        typ: &hugr::types::CustomType,
        _set: &TypeTranslatorSet,
    ) -> Option<RegisterCount> {
        if typ.name() == &*BOOL_TYPE_NAME {
            Some(RegisterCount::only_bits(1))
        } else {
            None
        }
    }
}

impl PytketDecoder for BoolEmitter {
    fn op_types(&self) -> Vec<tket_json_rs::OpType> {
        vec![tket_json_rs::OpType::ClExpr]
    }

    fn op_to_hugr<'h>(
        &self,
        op: &tket_json_rs::circuit_json::Operation,
        qubits: &[TrackedQubit],
        bits: &[TrackedBit],
        params: &[LoadedParameter],
        _opgroup: Option<&str>,
        decoder: &mut PytketDecoderContext<'h>,
    ) -> Result<DecodeStatus, PytketDecodeError> {
        let Some(clexpr) = &op.classical_expr else {
            return Ok(DecodeStatus::Unsupported);
        };

        // We only decode classical expressions if their operands are the input
        // bits in 0..n order, and the output comes immediately after.
        //
        // This can be easily relaxed if needed.
        for (i, reg) in clexpr.reg_posn.iter().enumerate() {
            if reg.index != i as u32 || reg.bits.0.len() != 1 {
                return Ok(DecodeStatus::Unsupported);
            }
        }
        for (i, arg) in clexpr.expr.args.iter().enumerate() {
            match arg {
                ClArgument::Terminal(ClTerminal::Variable(ClVariable::Bit { index }))
                    if *index != i as u32 =>
                {
                    return Ok(DecodeStatus::Unsupported);
                }
                ClArgument::Terminal(ClTerminal::Variable(ClVariable::Register { index }))
                    if *index != i as u32 =>
                {
                    return Ok(DecodeStatus::Unsupported);
                }
                _ => continue,
            }
        }

        let (op, num_inputs) = match clexpr.expr.op {
            ClOp::BitEq => (BoolOp::eq, 2),
            ClOp::BitNot => (BoolOp::not, 1),
            ClOp::BitAnd => (BoolOp::and, 2),
            ClOp::BitOr => (BoolOp::or, 2),
            ClOp::BitXor => (BoolOp::xor, 2),
            _ => return Ok(DecodeStatus::Unsupported),
        };

        if !params.is_empty() {
            return Ok(DecodeStatus::Unsupported);
        }

        decoder.add_node_with_wires(
            op,
            qubits,
            qubits,
            &bits[..num_inputs],
            &bits[num_inputs..],
            &[],
        )?;

        Ok(DecodeStatus::Success)
    }
}

/// Return a pytket operation setting the values of a list of bits.
pub(crate) fn set_bits_op(values: &[bool]) -> tket_json_rs::circuit_json::Operation {
    make_tk1_classical_operation(
        tket_json_rs::OpType::SetBits,
        values.len(),
        tket_json_rs::circuit_json::Classical::SetBits {
            values: values.to_vec(),
        },
    )
}
