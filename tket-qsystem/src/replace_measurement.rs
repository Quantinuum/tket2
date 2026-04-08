//! Provides a `ReplaceMeasurementPass` which replaces the tket.measurement type with
//! a `Future<Bool>` and rewrites any ops using it.

use derive_more::{Display, Error, From};
use hugr::extension::prelude::bool_t;
use hugr::extension::simple_op::MakeRegisteredOp;
use hugr::{Node, hugr::hugrmut::HugrMut};
use hugr_passes::PassScope;
use hugr_passes::composable::WithScope;
use hugr_passes::non_local::LocalizeEdges;
use hugr_passes::replace_types::{NodeTemplate, ReplaceTypesError};
use hugr_passes::{ComposablePass, ReplaceTypes, non_local::FindNonLocalEdgesError};
use tket::{TketOp, extension::measurement_type};

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
/// [TketOp::MeasureFree] and [QSystemOp::Measure] ops are replaced by
/// [QSystemOp::LazyMeasure], while [QSystemOp::MeasureReset] ops are
/// replaced by [QSystemOp::LazyMeasureReset].
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
    lw.set_replace_op(
        &TketOp::Read.to_extension_op().unwrap(),
        NodeTemplate::SingleOp(future_bool_read.into()),
    );

    lw.set_replace_op(
        &TketOp::MeasureFree.to_extension_op().unwrap(),
        NodeTemplate::SingleOp(QSystemOp::LazyMeasure.to_extension_op().unwrap().into()),
    );
    lw.set_replace_op(
        &QSystemOp::Measure.to_extension_op().unwrap(),
        NodeTemplate::SingleOp(QSystemOp::LazyMeasure.to_extension_op().unwrap().into()),
    );
    lw.set_replace_op(
        &QSystemOp::MeasureReset.to_extension_op().unwrap(),
        NodeTemplate::SingleOp(
            QSystemOp::LazyMeasureReset
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
    use hugr::ops::OpType;
    use hugr::types::TypeRow;
    use rstest::rstest;

    use crate::extension::qsystem::QSystemOpBuilder;

    #[test]
    fn test_replace_measurement_type_and_read() {
        let mut dfb = DFGBuilder::new(inout_sig(vec![measurement_type()], vec![bool_t()])).unwrap();
        let [m] = dfb.input_wires_arr();
        let out = dfb.add_dataflow_op(TketOp::Read, [m]).unwrap();
        let mut h = dfb.finish_hugr_with_outputs(out.outputs()).unwrap();
        h.validate().unwrap();

        ReplaceMeasurementPass::default().run(&mut h).unwrap();
        h.validate().unwrap();

        // Check that type was replaced.
        let sig = h.signature(h.entrypoint()).unwrap();
        assert_eq!(sig.input(), &TypeRow::from(vec![future_type(bool_t())]));
        assert_eq!(sig.output(), &TypeRow::from(vec![bool_t()]));

        // Check that the op was replaced.
        assert!(
            h.nodes()
                .any(|n| FutureOpDef::try_from(h.get_optype(n)) == Ok(FutureOpDef::Read))
        );
        assert!(
            !h.nodes()
                .any(|n| h.get_optype(n).cast::<TketOp>() == Some(TketOp::Read))
        );
    }

    #[rstest]
    #[case(TketOp::MeasureFree, QSystemOp::LazyMeasure)]
    #[case(QSystemOp::Measure, QSystemOp::LazyMeasure)]
    fn test_replace_measurement_ops<T: Into<OpType>>(
        #[case] measure_op: T,
        #[case] expected_op: QSystemOp,
    ) {
        let mut dfb = DFGBuilder::new(inout_sig(
            vec![hugr::extension::prelude::qb_t()],
            vec![measurement_type()],
        ))
        .unwrap();
        let [q] = dfb.input_wires_arr();
        let out = dfb.add_dataflow_op(measure_op, [q]).unwrap();
        let mut h = dfb.finish_hugr_with_outputs(out.outputs()).unwrap();
        h.validate().unwrap();

        ReplaceMeasurementPass::default().run(&mut h).unwrap();
        h.validate().unwrap();

        let sig = h.signature(h.entrypoint()).unwrap();
        assert_eq!(sig.output(), &TypeRow::from(vec![future_type(bool_t())]));

        assert!(
            h.nodes()
                .any(|n| h.get_optype(n).cast::<QSystemOp>() == Some(expected_op))
        );
        assert!(
            !h.nodes()
                .any(|n| h.get_optype(n).cast::<TketOp>() == Some(TketOp::MeasureFree))
        );
    }

    #[test]
    fn test_replace_measure_reset_op() {
        let mut dfb = DFGBuilder::new(inout_sig(
            vec![hugr::extension::prelude::qb_t()],
            vec![hugr::extension::prelude::qb_t(), measurement_type()],
        ))
        .unwrap();
        let [q] = dfb.input_wires_arr();
        let out = dfb.add_measure_reset(q).unwrap();
        let mut h = dfb.finish_hugr_with_outputs(out).unwrap();
        h.validate().unwrap();

        ReplaceMeasurementPass::default().run(&mut h).unwrap();
        h.validate().unwrap();

        let sig = h.signature(h.entrypoint()).unwrap();
        assert_eq!(
            sig.output(),
            &TypeRow::from(vec![
                hugr::extension::prelude::qb_t(),
                future_type(bool_t()),
            ])
        );

        assert!(
            h.nodes()
                .any(|n| h.get_optype(n).cast::<QSystemOp>() == Some(QSystemOp::LazyMeasureReset))
        );
        assert!(
            !h.nodes()
                .any(|n| h.get_optype(n).cast::<QSystemOp>() == Some(QSystemOp::MeasureReset))
        );
    }
}
