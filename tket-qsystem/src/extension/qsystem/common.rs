use std::sync::Arc;

use hugr::{
    Extension, Wire,
    builder::{BuildError, Dataflow, DataflowSubContainer, SubContainer},
    extension::{
        SignatureFunc,
        prelude::{UnwrapBuilder, bool_t, option_type, qb_t},
        simple_op::MakeRegisteredOp,
    },
    ops::{ExtensionOp, OpName},
    std_extensions::collections::array::{ArrayOpBuilder, array_type_parametric},
    type_row,
    types::{PolyFuncType, Signature, Type, TypeArg, TypeRow, type_param::TypeParam},
};
use tket::extension::bool::BoolOp;
use tket::extension::bool::bool_type;

use super::lower::pi_mul_f64;

/// A trait for common operations that are shared between Quantinuum platforms.
pub trait CommonOp: MakeRegisteredOp + Copy + From<SharedOp> {}

#[derive(Clone, Copy)]
/// An enum representing operations that are shared between Quantinuum platforms.
pub enum SharedOp {
    Measure,
    LazyMeasure,
    LazyMeasureReset,
    Rz,
    PhasedX,
    TryQAlloc,
    QFree,
    Reset,
    MeasureReset,
    LazyMeasureLeaked,
}

impl SharedOp {
    pub(crate) fn description(&self) -> &'static str {
        match self {
            SharedOp::Measure => "Measure a qubit and lose it.",
            SharedOp::LazyMeasure => "Lazily measure a qubit and lose it.",
            SharedOp::LazyMeasureReset => {
                "Lazily measure a qubit and reset it to the Z |0> eigenstate."
            }
            SharedOp::Rz => {
                "Rotate a qubit around the Z axis. Not physical on Helios or Sol platforms."
            }
            SharedOp::PhasedX => "PhasedX gate.",
            SharedOp::TryQAlloc => "Allocate a qubit in the Z |0> eigenstate.",
            SharedOp::QFree => "Free a qubit (lose track of it).",
            SharedOp::Reset => "Reset a qubit to the Z |0> eigenstate.",
            SharedOp::MeasureReset => "Measure a qubit and reset it to the Z |0> eigenstate.",
            SharedOp::LazyMeasureLeaked => {
                "Measure a qubit (return 0 or 1) or detect leakage (return 2)."
            }
        }
    }

    pub(crate) fn signature(&self) -> SignatureFunc {
        let one_qb_row = TypeRow::from(vec![qb_t()]);
        match self {
            SharedOp::LazyMeasure => Signature::new(
                one_qb_row.clone(),
                vec![super::futures::future_type(bool_t())],
            ),
            SharedOp::LazyMeasureLeaked => Signature::new(
                one_qb_row.clone(),
                vec![super::futures::future_type(
                    hugr::std_extensions::arithmetic::int_types::int_type(6),
                )],
            ),
            SharedOp::LazyMeasureReset => Signature::new(
                one_qb_row.clone(),
                vec![qb_t(), super::futures::future_type(bool_t())],
            ),
            SharedOp::Reset => Signature::new(one_qb_row.clone(), one_qb_row.clone()),
            SharedOp::Measure => Signature::new(one_qb_row.clone(), vec![bool_type()]),
            SharedOp::Rz => Signature::new(
                vec![
                    qb_t(),
                    hugr::std_extensions::arithmetic::float_types::float64_type(),
                ],
                one_qb_row.clone(),
            ),
            SharedOp::PhasedX => Signature::new(
                vec![
                    qb_t(),
                    hugr::std_extensions::arithmetic::float_types::float64_type(),
                    hugr::std_extensions::arithmetic::float_types::float64_type(),
                ],
                one_qb_row.clone(),
            ),
            SharedOp::TryQAlloc => Signature::new(
                type_row![],
                vec![Type::from(option_type(one_qb_row.clone()))],
            ),
            SharedOp::QFree => Signature::new(one_qb_row.clone(), type_row![]),
            SharedOp::MeasureReset => Signature::new(one_qb_row, vec![qb_t(), bool_type()]),
        }
        .into()
    }
}

pub(crate) trait CommonOpBuilder: Dataflow + UnwrapBuilder + ArrayOpBuilder {
    fn add_lazy_measure_with<Op>(&mut self, qb: Wire) -> Result<Wire, BuildError>
    where
        Op: CommonOp,
    {
        Ok(self
            .add_dataflow_op(Op::from(SharedOp::LazyMeasure), [qb])?
            .out_wire(0))
    }

    fn add_lazy_measure_leaked_with<Op>(&mut self, qb: Wire) -> Result<Wire, BuildError>
    where
        Op: CommonOp,
    {
        Ok(self
            .add_dataflow_op(Op::from(SharedOp::LazyMeasureLeaked), [qb])?
            .out_wire(0))
    }

    fn add_lazy_measure_reset_with<Op>(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError>
    where
        Op: CommonOp,
    {
        Ok(self
            .add_dataflow_op(Op::from(SharedOp::LazyMeasureReset), [qb])?
            .outputs_arr())
    }

    fn add_measure_with<Op>(&mut self, qb: Wire) -> Result<Wire, BuildError>
    where
        Op: CommonOp,
    {
        Ok(self
            .add_dataflow_op(Op::from(SharedOp::Measure), [qb])?
            .out_wire(0))
    }

    fn add_reset_with<Op>(&mut self, qb: Wire) -> Result<Wire, BuildError>
    where
        Op: CommonOp,
    {
        Ok(self
            .add_dataflow_op(Op::from(SharedOp::Reset), [qb])?
            .out_wire(0))
    }

    fn add_phased_x_with<Op>(
        &mut self,
        qb: Wire,
        angle1: Wire,
        angle2: Wire,
    ) -> Result<Wire, BuildError>
    where
        Op: CommonOp,
    {
        Ok(self
            .add_dataflow_op(Op::from(SharedOp::PhasedX), [qb, angle1, angle2])?
            .out_wire(0))
    }

    fn add_rz_with<Op>(&mut self, qb: Wire, angle: Wire) -> Result<Wire, BuildError>
    where
        Op: CommonOp,
    {
        Ok(self
            .add_dataflow_op(Op::from(SharedOp::Rz), [qb, angle])?
            .out_wire(0))
    }

    fn add_try_alloc_with<Op>(&mut self) -> Result<Wire, BuildError>
    where
        Op: CommonOp,
    {
        Ok(self
            .add_dataflow_op(Op::from(SharedOp::TryQAlloc), [])?
            .out_wire(0))
    }

    fn add_qfree_with<Op>(&mut self, qb: Wire) -> Result<(), BuildError>
    where
        Op: CommonOp,
    {
        self.add_dataflow_op(Op::from(SharedOp::QFree), [qb])?;
        Ok(())
    }

    fn add_measure_reset_with<Op>(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError>
    where
        Op: CommonOp,
    {
        Ok(self
            .add_dataflow_op(Op::from(SharedOp::MeasureReset), [qb])?
            .outputs_arr())
    }

    fn add_runtime_barrier_with(
        &mut self,
        extension: &Arc<Extension>,
        qbs: Wire,
        array_size: u64,
    ) -> Result<Wire, BuildError> {
        let op = runtime_barrier_ext_op(extension, array_size)?;
        Ok(self.add_dataflow_op(op, [qbs])?.out_wire(0))
    }

    fn build_wrapped_barrier_with(
        &mut self,
        extension: &Arc<Extension>,
        qbs: impl IntoIterator<Item = Wire>,
    ) -> Result<Vec<Wire>, BuildError>
    where
        Self: Sized,
    {
        let qbs: Vec<_> = qbs.into_iter().collect();
        let size = qbs.len() as u64;
        let q_arr = self.add_new_array(qb_t(), qbs)?;
        let q_arr = self.add_runtime_barrier_with(extension, q_arr, size)?;

        self.add_array_unpack(qb_t(), size, q_arr)
    }
}

impl<D> CommonOpBuilder for D where D: Dataflow + UnwrapBuilder + ArrayOpBuilder {}

pub(crate) trait SharedTketOpSynthesis: CommonOpBuilder {
    type Op: CommonOp;

    fn synth_phased_x(&mut self, qb: Wire, angle1: Wire, angle2: Wire) -> Result<Wire, BuildError>;

    fn synth_rz(&mut self, qb: Wire, angle: Wire) -> Result<Wire, BuildError>;

    fn synth_try_alloc(&mut self) -> Result<Wire, BuildError>;

    fn synth_measure_reset(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError>;

    fn build_h_shared(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        let pi = pi_mul_f64(self, 1.0);
        let pi_2 = pi_mul_f64(self, 0.5);
        let pi_minus_2 = pi_mul_f64(self, -0.5);

        let q = self.synth_phased_x(qb, pi_2, pi_minus_2)?;
        self.synth_rz(q, pi)
    }

    fn build_x_shared(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        let pi = pi_mul_f64(self, 1.0);
        let zero = pi_mul_f64(self, 0.0);
        self.synth_phased_x(qb, pi, zero)
    }

    fn build_y_shared(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        let pi = pi_mul_f64(self, 1.0);
        let pi_2 = pi_mul_f64(self, 0.5);
        self.synth_phased_x(qb, pi, pi_2)
    }

    fn build_z_shared(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        let pi = pi_mul_f64(self, 1.0);
        self.synth_rz(qb, pi)
    }

    fn build_s_shared(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        let pi_2 = pi_mul_f64(self, 0.5);
        self.synth_rz(qb, pi_2)
    }

    fn build_sdg_shared(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        let pi_minus_2 = pi_mul_f64(self, -0.5);
        self.synth_rz(qb, pi_minus_2)
    }

    fn build_v_shared(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        let pi_2 = pi_mul_f64(self, 0.5);
        let zero = pi_mul_f64(self, 0.0);
        self.synth_phased_x(qb, pi_2, zero)
    }

    fn build_vdg_shared(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        let pi_minus_2 = pi_mul_f64(self, -0.5);
        let zero = pi_mul_f64(self, 0.0);
        self.synth_phased_x(qb, pi_minus_2, zero)
    }

    fn build_t_shared(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        let pi_4 = pi_mul_f64(self, 0.25);
        self.synth_rz(qb, pi_4)
    }

    fn build_tdg_shared(&mut self, qb: Wire) -> Result<Wire, BuildError> {
        let pi_minus_4 = pi_mul_f64(self, -0.25);
        self.synth_rz(qb, pi_minus_4)
    }

    fn build_rx_shared(&mut self, qb: Wire, theta: Wire) -> Result<Wire, BuildError> {
        let zero = pi_mul_f64(self, 0.0);
        self.synth_phased_x(qb, theta, zero)
    }

    fn build_ry_shared(&mut self, qb: Wire, theta: Wire) -> Result<Wire, BuildError> {
        let pi_2 = pi_mul_f64(self, 0.5);
        self.synth_phased_x(qb, theta, pi_2)
    }

    fn build_qalloc_shared(&mut self) -> Result<Wire, BuildError> {
        let maybe_qb = self.synth_try_alloc()?;
        let [qb] = self.build_expect_sum(1, option_type(vec![qb_t()]), maybe_qb, |_| {
            "No more qubits available to allocate.".to_string()
        })?;
        Ok(qb)
    }

    fn build_measure_flip_shared(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError> {
        let [qb, b] = self.synth_measure_reset(qb)?;
        let sum_b = self.add_dataflow_op(BoolOp::read, [b])?.out_wire(0);
        let mut conditional = self.conditional_builder(
            ([type_row![], type_row![]], sum_b),
            [(qb_t(), qb)],
            vec![qb_t()].into(),
        )?;

        let case0 = conditional.case_builder(0)?;
        let [qb] = case0.input_wires_arr();
        case0.finish_with_outputs([qb])?;

        let mut case1 = conditional.case_builder(1)?;
        let [qb] = case1.input_wires_arr();
        let pi = pi_mul_f64(&mut case1, 1.0);
        let zero = pi_mul_f64(&mut case1, 0.0);
        let qb = case1.add_phased_x_with::<Self::Op>(qb, pi, zero)?;
        case1.finish_with_outputs([qb])?;

        let [qb] = conditional.finish_sub_container()?.outputs_arr();
        Ok([qb, sum_b])
    }
}

pub(crate) const RUNTIME_BARRIER_NAME: OpName = OpName::new_inline("RuntimeBarrier");

pub(crate) fn runtime_barrier_from_str(s: &str) -> Result<(), ()> {
    if s == RUNTIME_BARRIER_NAME.as_str() {
        Ok(())
    } else {
        Err(())
    }
}

pub(crate) fn runtime_barrier_signature() -> SignatureFunc {
    PolyFuncType::new(
        [TypeParam::max_nat_type()],
        Signature::new_endo(vec![
            array_type_parametric(TypeArg::new_var_use(0, TypeParam::max_nat_type()), qb_t())
                .unwrap(),
        ]),
    )
    .into()
}

pub(crate) fn runtime_barrier_description() -> String {
    "Acts as a runtime barrier between operations on argument qubits.".to_string()
}

pub(crate) fn runtime_barrier_ext_op(
    extension: &Arc<Extension>,
    array_size: u64,
) -> Result<ExtensionOp, hugr::extension::SignatureError> {
    ExtensionOp::new(
        extension.get_op(&RUNTIME_BARRIER_NAME).unwrap().clone(),
        [TypeArg::BoundedNat(array_size)],
    )
}

#[cfg(test)]
pub(crate) mod test_utils {
    use std::{fmt::Debug, sync::Arc};

    use cool_asserts::assert_matches;
    use hugr::{
        Extension, Hugr, HugrView, Wire,
        builder::{BuildError, Dataflow, DataflowHugr, FunctionBuilder},
        extension::{
            ExtensionId, OpDef,
            prelude::{bool_t, qb_t},
            simple_op::MakeOpDef,
        },
        ops::OpType,
        std_extensions::arithmetic::int_types::int_type,
        types::Signature,
    };
    use strum::IntoEnumIterator;

    use crate::extension::futures::FutureOpBuilder;

    use super::CommonOp;

    pub(crate) fn assert_extension_roundtrip<Op>(
        extension: &Arc<Extension>,
        extension_id: &ExtensionId,
    ) where
        Op: CommonOp + MakeOpDef + IntoEnumIterator + Copy + PartialEq + Eq + Debug,
    {
        assert_eq!(extension.name(), extension_id);

        for op in Op::iter() {
            let op_def: &Arc<OpDef> = extension.get_op(&op.opdef_id()).unwrap();
            assert_eq!(Op::from_def(op_def), Ok(op));
        }
    }

    pub(crate) fn assert_lazy_circuit(
        build_lazy_measure_reset: impl FnOnce(
            &mut FunctionBuilder<Hugr>,
            Wire,
        ) -> Result<[Wire; 2], BuildError>,
    ) {
        let hugr = {
            let mut func_builder = FunctionBuilder::new(
                "circuit",
                Signature::new(vec![qb_t()], vec![qb_t(), bool_t()]),
            )
            .unwrap();
            let [qb] = func_builder.input_wires_arr();
            let [qb, lazy_b] = build_lazy_measure_reset(&mut func_builder, qb).unwrap();
            let [b] = func_builder.add_read(lazy_b, bool_t()).unwrap();
            func_builder.finish_hugr_with_outputs([qb, b]).unwrap()
        };
        assert_matches!(hugr.validate(), Ok(_));
    }

    pub(crate) fn assert_leaked_measurement(
        build_lazy_measure_leaked: impl FnOnce(
            &mut FunctionBuilder<Hugr>,
            Wire,
        ) -> Result<Wire, BuildError>,
    ) {
        let hugr = {
            let mut func_builder =
                FunctionBuilder::new("leaked", Signature::new(vec![qb_t()], vec![int_type(6)]))
                    .unwrap();
            let [qb] = func_builder.input_wires_arr();
            let lazy_i = build_lazy_measure_leaked(&mut func_builder, qb).unwrap();
            let [i] = func_builder.add_read(lazy_i, int_type(6)).unwrap();
            func_builder.finish_hugr_with_outputs([i]).unwrap()
        };
        assert_matches!(hugr.validate(), Ok(_));
    }

    pub(crate) fn assert_cast_roundtrip<Op>()
    where
        Op: CommonOp
            + MakeOpDef
            + IntoEnumIterator
            + Copy
            + PartialEq
            + Eq
            + Debug
            + Into<OpType>
            + 'static,
    {
        for op in Op::iter() {
            let optype: OpType = op.into();
            let new_op: Op = optype.cast().unwrap();
            assert_eq!(op, new_op);
            assert_eq!(optype.cast::<tket::TketOp>(), None);
        }
    }
}
