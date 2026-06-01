//! Provides a `ReplaceMeasurementPass` which replaces the tket.measurement type with
//! a `Future<Bool>` and rewrites any ops using it.

use derive_more::{Display, Error, From};
use hugr::extension::prelude::{Noop, bool_t};
use hugr::extension::simple_op::MakeRegisteredOp;
use hugr::{Node, hugr::hugrmut::HugrMut};
use tket::TketOp;
use tket::extension::measurement::{MeasurementOp, measurement_type};
use tket::passes::PassScope;
use tket::passes::composable::WithScope;
use tket::passes::non_local::LocalizeEdges;
use tket::passes::replace_types::{NodeTemplate, ReplaceTypesError};
use tket::passes::{ComposablePass, ReplaceTypes, non_local::FindNonLocalEdgesError};

use crate::extension::futures::{FutureOp, FutureOpDef, future_type};
use crate::extension::qsystem::QSystemOp;

#[derive(Error, Debug, Display, From)]
#[non_exhaustive]
/// An error reported from [ReplaceMeasurementPass].
pub enum ReplaceMeasurementPassError<N> {
    /// The HUGR was found to contain non-local edges.
    NonLocalEdgesError(FindNonLocalEdgesError<N>),
    /// There was an error while replacing the type/ops.
    ReplacementError(ReplaceTypesError),
}

/// A HUGR -> HUGR pass replacing `tket.measurement` with `future(bool_t)`.
///
/// [TketOp::MeasureFree] is replaced by [QSystemOp::LazyMeasure], and
/// [QSystemOp::FutureToMeasurement] becomes a no-op. 
/// 
/// The linearizer ops for measurement types ([MeasurementOp::Read], 
/// [MeasurementOp::Dup], and [MeasurementOp::Free]) are replaced by the corresponding 
/// linearizer ops for future types ([FutureOpDef::Read], [FutureOpDef::Dup], and 
/// [FutureOpDef::Free]).
#[derive(Default, Debug, Clone)]
pub struct ReplaceMeasurementPass {
    /// Where to apply the pass.
    ///
    /// Configurable via [`WithScope::with_scope`].
    scope: PassScope,
}

impl WithScope for ReplaceMeasurementPass {
    fn with_scope(mut self, scope: impl Into<PassScope>) -> Self {
        self.scope = scope.into();
        self
    }
}

impl<H: HugrMut<Node = Node>> ComposablePass<H> for ReplaceMeasurementPass {
    type Error = ReplaceMeasurementPassError<H::Node>;
    type Result = ();

    fn run(&self, hugr: &mut H) -> Result<(), Self::Error> {
        LocalizeEdges::default_with_scope(self.scope.clone()).check_no_nonlocal_edges(hugr)?;
        lowerer().with_scope(self.scope.clone()).run(hugr)?;
        Ok(())
    }
}

/// The configuration used for replacing measurement types and ops.
fn lowerer() -> ReplaceTypes {
    let mut lw = ReplaceTypes::default();

    // As the measurement type acts like an alias for `Future<Bool>`, all the
    // replacements are straightforward.
    lw.set_replace_type(
        measurement_type().as_extension().unwrap().clone(),
        future_type(bool_t()),
    );

    let future_bool_read = FutureOp {
        op: FutureOpDef::Read,
        typ: bool_t(),
    }
    .to_extension_op()
    .unwrap();
    let future_bool_dup = FutureOp {
        op: FutureOpDef::Dup,
        typ: bool_t(),
    }
    .to_extension_op()
    .unwrap();
    let future_bool_free = FutureOp {
        op: FutureOpDef::Free,
        typ: bool_t(),
    }
    .to_extension_op()
    .unwrap();
    lw.set_replace_op(
        &MeasurementOp::Read.to_extension_op().unwrap(),
        NodeTemplate::SingleOp(future_bool_read.into()),
    );
    lw.set_replace_op(
        &MeasurementOp::Dup.to_extension_op().unwrap(),
        NodeTemplate::SingleOp(future_bool_dup.into()),
    );
    lw.set_replace_op(
        &MeasurementOp::Free.to_extension_op().unwrap(),
        NodeTemplate::SingleOp(future_bool_free.into()),
    );

    lw.set_replace_op(
        &TketOp::MeasureFree.to_extension_op().unwrap(),
        NodeTemplate::SingleOp(QSystemOp::LazyMeasure.to_extension_op().unwrap().into()),
    );
    lw.set_replace_op(
        &QSystemOp::FutureToMeasurement.to_extension_op().unwrap(),
        NodeTemplate::SingleOp(
            Noop::new(future_type(bool_t()))
                .to_extension_op()
                .unwrap()
                .into(),
        ),
    );

    lw
}

#[cfg(test)]
mod test {
    use super::*;
    use hugr::HugrView;
    use hugr::builder::{DFGBuilder, Dataflow, DataflowHugr, inout_sig};
    use hugr::extension::prelude::qb_t;
    use hugr::extension::simple_op::MakeOpDef;

    #[test]
    fn test_replace_all_ops() {
        let mut circuit = DFGBuilder::new(inout_sig(vec![qb_t(); 2], vec![bool_t()])).unwrap();
        let [q1, q2] = circuit.input_wires_arr();

        // TketOp::MeasureFree
        let m1 = circuit
            .add_dataflow_op(TketOp::MeasureFree, [q1])
            .unwrap()
            .out_wire(0);

        // QSystemOp::LazyMeasure
        let f2 = circuit
            .add_dataflow_op(QSystemOp::LazyMeasure, [q2])
            .unwrap()
            .out_wire(0);

        // QSystemOp::FutureToMeasurement
        let m2 = circuit
            .add_dataflow_op(QSystemOp::FutureToMeasurement, [f2])
            .unwrap()
            .out_wire(0);

        // Duplicate one measurement
        let [m1_1, m1_2] = circuit
            .add_dataflow_op(MeasurementOp::Dup, [m1])
            .unwrap()
            .outputs_arr();

        // Read one
        let b1 = circuit
            .add_dataflow_op(MeasurementOp::Read, [m1_1])
            .unwrap()
            .out_wire(0);

        // Free the rest
        circuit
            .add_dataflow_op(MeasurementOp::Free, [m1_2])
            .unwrap();
        circuit.add_dataflow_op(MeasurementOp::Free, [m2]).unwrap();

        let mut h = circuit.finish_hugr_with_outputs([b1]).unwrap();
        h.validate().unwrap();

        ReplaceMeasurementPass::default().run(&mut h).unwrap();
        h.validate().unwrap();

        // Check no measurement types remain
        let sig = h.signature(h.entrypoint()).unwrap();
        assert!(!sig.input().iter().any(|t| t == &measurement_type()));
        assert!(!sig.output().iter().any(|t| t == &measurement_type()));

        assert!(
            !h.nodes()
                .filter_map(|n| h.get_optype(n).as_extension_op())
                .any(|op| MeasurementOp::from_op(op).is_ok())
        );
    }
}
