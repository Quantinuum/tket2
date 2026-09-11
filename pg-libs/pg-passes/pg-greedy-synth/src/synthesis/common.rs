use crate::tqe::TQE;
use indicatif::{ProgressBar, ProgressState, ProgressStyle};
use pg_bitpacked::{XX_U8, XY_U8, XZ_U8, YX_U8, YY_U8, YZ_U8, ZX_U8, ZY_U8, ZZ_U8};
use pg_core::{GateData, GateType, Op};
use std::fmt::Write;

/// Updates the projected TQE depth after scheduling `tqe`.
pub(crate) fn update_depth(qubit_depth: &mut [u64], tqe: TQE) {
    let depth = qubit_depth[tqe.q0].max(qubit_depth[tqe.q1]) + 1;
    qubit_depth[tqe.q0] = depth;
    qubit_depth[tqe.q1] = depth;
}

/// Converts the TQE to a PG gate.
pub(crate) fn tqe_op(tqe: TQE) -> Op {
    let gate_type = match tqe.gate_type {
        XX_U8 => GateType::XX,
        XY_U8 => GateType::XY,
        XZ_U8 => GateType::XZ,
        YX_U8 => GateType::YX,
        YY_U8 => GateType::YY,
        YZ_U8 => GateType::YZ,
        ZX_U8 => GateType::ZX,
        ZY_U8 => GateType::ZY,
        ZZ_U8 => GateType::ZZ,
        _ => panic!("invalid TQE gate type: {}", tqe.gate_type),
    };
    Op::Gate {
        data: GateData::new(gate_type, vec![tqe.q0, tqe.q1]),
    }
}

/// Creates the progress display shared by graph and tableau reduction.
pub(crate) fn progress_bar(enabled: bool, length: usize) -> Option<ProgressBar> {
    if !enabled {
        return None;
    }

    let progress = ProgressBar::new(length as u64);
    progress.set_style(
        ProgressStyle::with_template(
            "{spinner:.green} [{elapsed_precise}] [{wide_bar:.cyan/blue}] {pos}/{len} ({eta})",
        )
        .expect("the progress-bar template is valid")
        .with_key("eta", |state: &ProgressState, writer: &mut dyn Write| {
            write!(writer, "{:.1}s", state.eta().as_secs_f64())
                .expect("writing to a progress-bar buffer cannot fail")
        })
        .progress_chars("#>-"),
    );
    Some(progress)
}
