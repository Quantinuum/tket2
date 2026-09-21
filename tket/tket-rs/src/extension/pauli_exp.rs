//! Pauli-string exponentials on individual qubit wires.
//!
//! `PauliExp<n, paulis>` has signature `(qubit × n, rotation) -> (qubit × n)`
//! and denotes `exp(-i π angle (P₀ ⊗ … ⊗ Pₙ₋₁) / 2)`, with the angle in
//! half-turns. Static Paulis are bounded naturals: I=0, X=1, Y=2, Z=3.
//! Every qubit is returned in its original position, including identity factors.
//! For n=0, the empty tensor product is 1, so this is a global phase.

use std::sync::{Arc, LazyLock};

use hugr::{
    Extension,
    extension::{ExtensionId, SignatureError, SignatureFromArgs, Version, prelude::qb_t},
    ops::{ExtensionOp, OpName},
    types::{PolyFuncType, PolyFuncTypeRV, Signature, TypeArg, type_param::TypeParam},
};

use super::rotation::rotation_type;

/// Identifier of the Pauli exponential extension.
pub const PAULI_EXP_EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.pauli_exp");
/// Version of the Pauli exponential extension.
pub const PAULI_EXP_VERSION: Version = Version::new(0, 1, 0);
/// Name of the Pauli exponential operation.
pub const PAULI_EXP_OP_ID: OpName = OpName::new_inline("PauliExp");

/// The Pauli exponential extension.
pub static PAULI_EXP_EXTENSION: LazyLock<Arc<Extension>> = LazyLock::new(|| {
    Extension::new_arc(PAULI_EXP_EXTENSION_ID, PAULI_EXP_VERSION, |ext, ext_ref| {
        ext.add_op(
            PAULI_EXP_OP_ID,
            "exp(-i*pi*angle*(P0 tensor ... tensor P(n-1))/2). Inputs: n qubits, then a rotation in half-turns; outputs: n qubits in the same order. Static arguments: n and n Paulis (I=0, X=1, Y=2, Z=3). The empty string gives a global phase.".into(),
            PauliExpSignature([
                TypeParam::max_nat_kind(),
                TypeParam::new_list_kind(TypeParam::bounded_nat_kind(4.try_into().unwrap())),
            ]),
            ext_ref,
        )
        .unwrap();
    })
});

struct PauliExpSignature([TypeParam; 2]);

impl SignatureFromArgs for PauliExpSignature {
    fn static_params(&self) -> &[TypeParam] {
        &self.0
    }

    fn compute_signature(&self, args: &[TypeArg]) -> Result<PolyFuncTypeRV, SignatureError> {
        let [TypeArg::BoundedNat(n), TypeArg::List(paulis)] = args else {
            return Err(SignatureError::InvalidTypeArgs);
        };
        if usize::try_from(*n).ok() != Some(paulis.len())
            || !paulis
                .iter()
                .all(|p| matches!(p, TypeArg::BoundedNat(0..=3)))
        {
            return Err(SignatureError::InvalidTypeArgs);
        }
        let outputs = vec![qb_t(); paulis.len()];
        let mut inputs = outputs.clone();
        inputs.push(rotation_type());
        Ok(PolyFuncType::from(Signature::new(inputs, outputs)).into())
    }
}

/// Instantiate a Pauli exponential with an angle supplied on its last input wire.
///
/// Returns an error if the Pauli count differs from `num_qubits` or a Pauli is
/// outside 0..=3. The first `num_qubits` inputs and all outputs are qubits.
pub fn pauli_exp(num_qubits: u64, paulis: &[u8]) -> Result<ExtensionOp, SignatureError> {
    PAULI_EXP_EXTENSION.instantiate_extension_op(
        &PAULI_EXP_OP_ID,
        vec![
            TypeArg::BoundedNat(num_qubits),
            TypeArg::new_list(paulis.iter().map(|p| TypeArg::BoundedNat(u64::from(*p)))),
        ],
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use hugr::{
        HugrView,
        builder::{DFGBuilder, Dataflow, DataflowHugr},
    };

    #[test]
    fn linear_qubit_signature() {
        for paulis in [vec![], vec![0], vec![1], vec![0, 1, 2, 3]] {
            let n = paulis.len();
            let op = pauli_exp(n as u64, &paulis).unwrap();
            let mut inputs = vec![qb_t(); n];
            inputs.push(rotation_type());
            let signature = Signature::new(inputs, vec![qb_t(); n]);
            let mut builder = DFGBuilder::new(signature).unwrap();
            let wires = builder.input_wires().collect::<Vec<_>>();
            let node = builder.add_dataflow_op(op, wires).unwrap();
            let outputs = node.outputs();
            let hugr = builder.finish_hugr_with_outputs(outputs).unwrap();
            hugr.validate().unwrap();
        }
    }

    #[test]
    fn rejects_invalid_static_arguments() {
        assert!(pauli_exp(2, &[1]).is_err());
        assert!(pauli_exp(0, &[0]).is_err());
        assert!(pauli_exp(1, &[4]).is_err());
        assert!(pauli_exp(1, &[255]).is_err());
        for args in [
            vec![],
            vec![TypeArg::BoundedNat(1), TypeArg::String("X".into())],
            vec![
                TypeArg::BoundedNat(1),
                TypeArg::new_list([TypeArg::String("X".into())]),
            ],
        ] {
            assert!(
                PAULI_EXP_EXTENSION
                    .instantiate_extension_op(&PAULI_EXP_OP_ID, args)
                    .is_err()
            );
        }
    }
}
