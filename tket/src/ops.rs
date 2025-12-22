use std::sync::{Arc, Weak};

use crate::extension::bool::bool_type;
use crate::extension::rotation::rotation_type;
use crate::extension::sympy::SympyOpDef;
use crate::extension::{TKET_EXTENSION, TKET_EXTENSION_ID as EXTENSION_ID};
use hugr::ops::custom::ExtensionOp;
use hugr::types::Type;
use hugr::{
    extension::{
        ExtensionId, OpDef, SignatureFunc,
        prelude::{bool_t, option_type, qb_t},
        simple_op::{MakeOpDef, MakeRegisteredOp, try_from_name},
    },
    ops::OpType,
    type_row,
    types::Signature,
};

use derive_more::Display;
use serde::{Deserialize, Serialize};
use smol_str::ToSmolStr;
use strum::{EnumIter, EnumString, IntoStaticStr};

/// Standar TKET quantum operations.
#[derive(
    Clone,
    Copy,
    Debug,
    Serialize,
    Deserialize,
    Hash,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    EnumIter,
    IntoStaticStr,
    EnumString,
)]
#[non_exhaustive]
/// Simple enum of tket quantum operations.
pub enum TketOp {
    /// Hadamard gate
    ///
    /// This is a single qubit gate.
    H,
    /// Controlled-X (CNOT).
    ///
    /// Inputs:
    /// - A control qubit
    /// - The target qubit
    ///
    /// Outputs:
    /// - The control qubit
    /// - The target qubit
    CX,
    /// Controlled-Y.
    ///
    /// Inputs:
    /// - A control qubit
    /// - The target qubit
    ///
    /// Outputs:
    /// - The control qubit
    /// - The target qubit
    CY,
    /// Controlled-Z.
    ///
    /// Inputs:
    /// - A control qubit
    /// - The target qubit
    ///
    /// Outputs:
    /// - The control qubit
    /// - The target qubit
    CZ,
    /// Controlled-Rz rotation.
    ///
    /// Inputs:
    /// - A control qubit
    /// - The target qubit
    /// - A [`rotation`](rotation_type) angle expressed in half-turns.
    ///
    /// Outputs:
    /// - The control qubit
    /// - The target qubit
    ///
    /// See [`TketOp::Rz`].
    CRz,
    /// T gate.
    ///
    /// Single qubit gate.
    T,
    /// T dagger gate.
    ///
    /// Dagger of [`TketOp::T`].
    ///
    /// This is a single qubit gate.
    Tdg,
    /// S gate.
    ///
    /// This is a single qubit gate.
    S,
    /// S dagger gate.
    ///
    /// Dagger of [`TketOp::S`].
    ///
    /// This is a single qubit gate.
    Sdg,
    /// Pauli X gate.
    ///
    /// This is a single qubit gate.
    X,
    /// Pauli Y gate.
    ///
    /// This is a single qubit gate.
    Y,
    /// Pauli Z gate.
    ///
    /// This is a single qubit gate.
    Z,
    /// Pauli X rotation.
    ///
    /// Inputs:
    /// - A qubit
    /// - A [`rotation`](rotation_type) angle expressed in half-turns.
    ///
    /// Outputs:
    /// - A qubit
    ///
    /// This is a single qubit gate.
    Rx,
    /// Pauli Y rotation.
    ///
    /// Inputs:
    /// - A qubit
    /// - A [`rotation`](rotation_type) angle expressed in half-turns.
    ///
    /// Outputs:
    /// - A qubit
    ///
    /// This is a single qubit gate.
    Ry,
    /// Pauli Z rotation.
    ///
    /// Inputs:
    /// - A qubit
    /// - A [`rotation`](rotation_type) angle expressed in half-turns.
    ///
    /// Outputs:
    /// - A qubit
    ///
    /// This is a single qubit gate.
    Rz,
    /// Toffoli gate.
    ///
    /// Inputs:
    /// - Control qubit 1
    /// - Control qubit 2
    /// - The target qubit
    ///
    /// Outputs:
    /// - Control qubit 1
    /// - Control qubit 2
    /// - The target qubit
    Toffoli,
    /// Measure a qubit and return it along with the measurement result.
    ///
    /// Inputs:
    /// - A qubit
    ///
    /// Outputs:
    /// - A qubit
    /// - A boolean indicating whether the qubit was measured as 1
    Measure,
    /// Measure a qubit and consume the qubit.
    ///
    /// Inputs:
    /// - A qubit
    ///
    /// Outputs:
    /// - A boolean indicating whether the qubit was measured as 1
    MeasureFree,
    /// Allocate a qubit.
    ///
    /// Inputs:
    /// - None
    ///
    /// Outputs:
    /// - A qubit
    QAlloc,
    /// Try to allocate a qubit, returning an option with the qubit or None on failure.
    ///
    /// Inputs:
    /// - None
    ///
    /// Outputs:
    /// - An [`option_type`] with a qubit or None on failure
    TryQAlloc,
    /// Free a qubit.
    ///
    /// Inputs:
    /// - A qubit
    ///
    /// Outputs:
    /// - None
    QFree,
    /// Reset a qubit to |0>.
    ///
    /// Inputs:
    /// - A qubit
    ///
    /// Outputs:
    /// - A qubit in the |0> state
    Reset,
    /// V gate.
    ///
    /// This is a single qubit gate.
    V,
    /// V dagger gate.
    ///
    /// Dagger of [`TketOp::V`].
    ///
    /// This is a single qubit gate.
    Vdg,
}

impl TketOp {
    /// Expose the operation names directly in TketOp
    pub fn exposed_name(&self) -> smol_str::SmolStr {
        <TketOp as Into<OpType>>::into(*self).to_smolstr()
    }

    /// Wraps the operation in an [`ExtensionOp`]
    pub fn into_extension_op(self) -> ExtensionOp {
        <Self as MakeRegisteredOp>::to_extension_op(self)
            .expect("Failed to convert to extension op.")
    }
}

/// Whether an op is a given TketOp.
pub fn op_matches(op: &OpType, tket_op: TketOp) -> bool {
    op.to_string() == tket_op.exposed_name()
}

/// Simple enum representation of Pauli matrices.
#[derive(
    Clone, Copy, Debug, Serialize, Deserialize, EnumIter, Display, PartialEq, PartialOrd, EnumString,
)]
pub enum Pauli {
    /// Pauli identity matrix.
    I,
    /// Pauli X matrix.
    X,
    /// Pauli Y matrix.
    Y,
    /// Pauli Z matrix.
    Z,
}

impl Pauli {
    /// Check if this pauli commutes with another.
    pub fn commutes_with(&self, other: Self) -> bool {
        *self == Pauli::I || other == Pauli::I || *self == other
    }
}
impl MakeOpDef for TketOp {
    fn opdef_id(&self) -> hugr::ops::OpName {
        <&'static str>::from(self).into()
    }

    fn init_signature(&self, _extension_ref: &std::sync::Weak<hugr::Extension>) -> SignatureFunc {
        use TketOp::*;
        match self {
            H | T | S | V | X | Y | Z | Tdg | Sdg | Vdg | Reset => Signature::new_endo(qb_t()),
            CX | CZ | CY => Signature::new_endo(vec![qb_t(); 2]),
            Toffoli => Signature::new_endo(vec![qb_t(); 3]),
            Measure => Signature::new(qb_t(), vec![qb_t(), bool_t()]),
            MeasureFree => Signature::new(qb_t(), bool_type()),
            Rz | Rx | Ry => Signature::new(vec![qb_t(), rotation_type()], qb_t()),
            CRz => Signature::new(vec![qb_t(), qb_t(), rotation_type()], vec![qb_t(); 2]),
            QAlloc => Signature::new(type_row![], qb_t()),
            TryQAlloc => Signature::new(type_row![], Type::from(option_type(qb_t()))),
            QFree => Signature::new(qb_t(), type_row![]),
        }
        .into()
    }

    fn extension(&self) -> ExtensionId {
        EXTENSION_ID.to_owned()
    }

    fn post_opdef(&self, def: &mut OpDef) {
        def.add_misc(
            "commutation",
            serde_json::to_value(self.qubit_commutation()).unwrap(),
        );
    }

    fn from_def(op_def: &OpDef) -> Result<Self, hugr::extension::simple_op::OpLoadError> {
        try_from_name(op_def.name(), op_def.extension_id())
    }

    fn extension_ref(&self) -> Weak<hugr::Extension> {
        Arc::downgrade(&TKET_EXTENSION)
    }
}

impl MakeRegisteredOp for TketOp {
    fn extension_id(&self) -> ExtensionId {
        EXTENSION_ID.to_owned()
    }

    fn extension_ref(&self) -> Arc<hugr::Extension> {
        TKET_EXTENSION.clone()
    }
}

impl TketOp {
    pub(crate) fn qubit_commutation(&self) -> Vec<(usize, Pauli)> {
        use TketOp::*;

        match self {
            X | V | Vdg | Rx => vec![(0, Pauli::X)],
            Y => vec![(0, Pauli::Y)],
            T | Z | S | Tdg | Sdg | Rz | Measure => vec![(0, Pauli::Z)],
            CX => vec![(0, Pauli::Z), (1, Pauli::X)],
            CZ => vec![(0, Pauli::Z), (1, Pauli::Z)],
            // by default, no commutation
            _ => vec![],
        }
    }

    /// Check if this op is a quantum op.
    pub fn is_quantum(&self) -> bool {
        use TketOp::*;
        match self {
            H | CX | T | S | V | X | Y | Z | Tdg | Sdg | Vdg | Rz | Rx | Toffoli | Ry | CZ | CY
            | CRz => true,
            Measure | MeasureFree | QAlloc | TryQAlloc | QFree | Reset => false,
        }
    }
}

/// Initialize a new custom symbolic expression constant op from a string.
pub fn symbolic_constant_op(arg: String) -> OpType {
    SympyOpDef.with_expr(arg).into()
}

#[cfg(test)]
pub(crate) mod test {

    use std::str::FromStr;
    use std::sync::Arc;

    use hugr::builder::{DFGBuilder, Dataflow, DataflowHugr};
    use hugr::extension::prelude::{option_type, qb_t};
    use hugr::extension::simple_op::{MakeExtensionOp, MakeOpDef};
    use hugr::extension::{OpDef, prelude::UnwrapBuilder as _};
    use hugr::types::Signature;
    use hugr::{CircuitUnit, HugrView, type_row};
    use itertools::Itertools;
    use rstest::{fixture, rstest};
    use strum::IntoEnumIterator;

    use super::TketOp;
    use crate::Pauli;
    use crate::circuit::Circuit;
    use crate::extension::bool::bool_type;
    use crate::extension::{TKET_EXTENSION as EXTENSION, TKET_EXTENSION_ID as EXTENSION_ID};
    use crate::utils::build_simple_circuit;
    fn get_opdef(op: TketOp) -> Option<&'static Arc<OpDef>> {
        EXTENSION.get_op(&op.op_id())
    }
    #[test]
    fn create_extension() {
        assert_eq!(EXTENSION.name(), &EXTENSION_ID);

        for o in TketOp::iter() {
            assert_eq!(TketOp::from_def(get_opdef(o).unwrap()), Ok(o));
        }
    }

    #[fixture]
    pub(crate) fn t2_bell_circuit() -> Circuit {
        let h = build_simple_circuit(2, |circ| {
            circ.append(TketOp::H, [0])?;
            circ.append(TketOp::CX, [0, 1])?;
            Ok(())
        });

        h.unwrap()
    }

    #[rstest]
    fn check_t2_bell(t2_bell_circuit: Circuit) {
        assert_eq!(t2_bell_circuit.commands().count(), 2);
    }

    #[test]
    fn ancilla_circ() {
        let h = build_simple_circuit(1, |circ| {
            let empty: [CircuitUnit; 0] = []; // requires type annotation
            let ancilla = circ.append_with_outputs(TketOp::QAlloc, empty)?[0];
            let ancilla = circ.append_with_outputs(TketOp::Reset, [ancilla])?[0];

            let ancilla = circ.append_with_outputs(
                TketOp::CX,
                [CircuitUnit::Linear(0), CircuitUnit::Wire(ancilla)],
            )?[0];
            let ancilla = circ.append_with_outputs(TketOp::Measure, [ancilla])?[0];
            circ.append_and_consume(TketOp::QFree, [ancilla])?;

            Ok(())
        })
        .unwrap();

        // 5 commands: alloc, reset, cx, measure, free
        assert_eq!(h.commands().count(), 5);
    }

    #[test]
    fn try_qalloc_measure_free() {
        let mut b = DFGBuilder::new(Signature::new(type_row![], bool_type())).unwrap();

        let try_q = b
            .add_dataflow_op(TketOp::TryQAlloc, [])
            .unwrap()
            .out_wire(0);
        let [q] = b.build_unwrap_sum(1, option_type(qb_t()), try_q).unwrap();
        let measured = b
            .add_dataflow_op(TketOp::MeasureFree, [q])
            .unwrap()
            .out_wire(0);
        let h = b.finish_hugr_with_outputs([measured]).unwrap();

        let top_ops = h
            .children(h.entrypoint())
            .map(|n| h.get_optype(n))
            .collect_vec();

        assert_eq!(top_ops.len(), 5);
        // first two are I/O
        assert_eq!(
            TketOp::from_op(top_ops[2].as_extension_op().unwrap()).unwrap(),
            TketOp::TryQAlloc
        );
        assert!(top_ops[3].is_conditional());
        assert_eq!(
            TketOp::from_op(top_ops[4].as_extension_op().unwrap()).unwrap(),
            TketOp::MeasureFree
        );
    }
    #[test]
    fn tket_op_properties() {
        for op in TketOp::iter() {
            // The exposed name should start with "tket.quantum."
            assert!(op.exposed_name().starts_with(&EXTENSION_ID.to_string()));

            let ext_op = op.into_extension_op();
            assert_eq!(ext_op.args(), &[]);
            assert_eq!(ext_op.def().extension_id(), &EXTENSION_ID);
            let name = ext_op.def().name();
            assert_eq!(TketOp::from_str(name), Ok(op));
        }

        // Other calls
        assert!(TketOp::H.is_quantum());
        assert!(!TketOp::Measure.is_quantum());

        for (op, pauli) in [
            (TketOp::X, Pauli::X),
            (TketOp::Y, Pauli::Y),
            (TketOp::Z, Pauli::Z),
        ]
        .iter()
        {
            assert_eq!(op.qubit_commutation(), &[(0, *pauli)]);
        }
    }
}
