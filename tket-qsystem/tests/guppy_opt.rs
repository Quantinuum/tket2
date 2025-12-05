//! Tests optimizing Guppy-generated programs.

use rayon::iter::ParallelIterator;
use smol_str::SmolStr;
use std::collections::HashMap;
use std::fs;
use std::io::BufReader;
use std::path::Path;
use tket::extension::{TKET1_EXTENSION_ID, TKET_EXTENSION_ID};

use hugr::algorithms::ComposablePass;
use hugr::{Hugr, HugrView};
use rstest::rstest;
use tket::passes::NormalizeGuppy;
use tket::serialize::pytket::{EncodeOptions, EncodedCircuit};
use tket::Circuit;

use tket1_passes::{Tket1Circuit, Tket1Pass};
use tket_qsystem::QSystemPass;

const GUPPY_EXAMPLES_DIR: &str = "../test_files/guppy_optimization";

/// JSON encoding of the clifford simp pytket pass.
const CLIFFORD_SIMP_STR: &str = r#"{"StandardPass": {"allow_swaps": true, "name": "CliffordSimp", "target_2qb_gate": "CX"}, "pass_class": "StandardPass"}"#;

enum HugrFileType {
    Original,
    Flat,
    Optimized,
}

fn load_guppy_circuit(name: &str, file_type: HugrFileType) -> std::io::Result<Hugr> {
    let suffix = match file_type {
        HugrFileType::Original => "",
        HugrFileType::Flat => ".flat",
        HugrFileType::Optimized => ".opt",
    };
    load_guppy_example(&format!("{name}/{name}{suffix}.hugr"))
}

fn load_guppy_example(path: &str) -> std::io::Result<Hugr> {
    let file = Path::new(GUPPY_EXAMPLES_DIR).join(path);
    let reader = fs::File::open(file)?;
    let reader = BufReader::new(reader);
    Ok(Hugr::load(reader, None).unwrap())
}

fn run_pytket(h: &mut Hugr) {
    let circ = Circuit::new(h);
    let mut encoded =
        EncodedCircuit::new(&circ, EncodeOptions::new().with_subcircuits(true)).unwrap();

    encoded
        .par_iter_mut()
        .for_each(|(_region, serial_circuit)| {
            let mut circuit_ptr = Tket1Circuit::from_serial_circuit(serial_circuit).unwrap();
            Tket1Pass::run_from_json(CLIFFORD_SIMP_STR, &mut circuit_ptr).unwrap();
            *serial_circuit = circuit_ptr.to_serial_circuit().unwrap();
        });

    encoded.reassemble_inplace(circ.into_hugr(), None).unwrap();
}

fn count_gates(h: &impl HugrView) -> HashMap<SmolStr, usize> {
    let mut counts = HashMap::new();
    for n in h.nodes() {
        if let Some(eop) = h.get_optype(n).as_extension_op() {
            if [TKET_EXTENSION_ID, TKET1_EXTENSION_ID].contains(eop.extension_id()) {
                *counts.entry(eop.qualified_id()).or_default() += 1;
            }
        }
    }
    counts
}

/// Run some simple optimization passes on the guppy-generated HUGRs and validate the result.
///
/// This test is intended to check the current status of the Guppy optimization passes.
///

#[rstest]
#[case::nested_array("nested_array", None)]
#[should_panic = "xfail"]
#[case::angles("angles", Some(vec![
    ("tket.quantum.Rz", 2), ("tket.quantum.MeasureFree", 1), ("tket.quantum.H", 2), ("tket.quantum.QAlloc", 1)
]))]
#[should_panic = "xfail"]
#[case::simple_cx("simple_cx", Some(vec![
    ("tket.quantum.QAlloc", 2), ("tket.quantum.MeasureFree", 2),
]))]
#[should_panic = "xfail"]
#[case::nested("nested", Some(vec![
    ("tket.quantum.CZ", 6), ("tket.quantum.QAlloc", 3), ("tket.quantum.MeasureFree", 3), ("tket.quantum.H", 6)
]))]
#[should_panic = "xfail"]
#[case::ranges("ranges", Some(vec![
    ("tket.quantum.H", 8), ("tket.quantum.MeasureFree", 4), ("tket.quantum.QAlloc", 4), ("tket.quantum.CX", 6)
]))]
#[should_panic = "xfail"]
#[case::false_branch("false_branch", Some(vec![
    ("TKET1.tk1op", 1), ("tket.quantum.H", 1), ("tket.quantum.QAlloc", 1), ("tket.quantum.MeasureFree", 1)
]))]
#[should_panic = "xfail"]
#[case::func_decls("func_decls", Some(vec![
    ("TKET1.tk1op", 1), ("tket.quantum.symbolic_angle", 1)
]))]
#[cfg_attr(miri, ignore)] // Opening files is not supported in (isolated) miri
fn optimize_flattened_guppy(#[case] name: &str, #[case] xfail: Option<Vec<(&str, usize)>>) {
    let mut hugr = load_guppy_circuit(name, HugrFileType::Flat)
        .unwrap_or_else(|_| load_guppy_circuit(name, HugrFileType::Original).unwrap());
    // We don't need NormalizeGuppy to "flatten" control flow here, but we still want
    // to get rid of other guppy artifacts.
    NormalizeGuppy::default().run(&mut hugr).unwrap();
    run_pytket(&mut hugr);
    let should_xfail = xfail.is_some();
    let expected_counts = match xfail {
        Some(counts) => counts.into_iter().map(|(k, v)| (k.into(), v)).collect(),
        None => count_gates(&load_guppy_circuit(name, HugrFileType::Optimized).unwrap()),
    };
    assert_eq!(count_gates(&hugr), expected_counts);
    if should_xfail {
        panic!("xfail");
    }
}

#[rstest]
#[cfg_attr(miri, ignore)] // Opening files is not supported in (isolated) miri
fn optimize_guppy_ranges_array() {
    // Demonstrates we can fully optimize the array operations in ranges
    // (after control flow is flattened) if we play around with the entrypoint.
    use hugr::algorithms::const_fold::ConstantFoldPass;
    use hugr::hugr::hugrmut::HugrMut;
    use tket::passes::BorrowSquashPass;
    let mut hugr = load_guppy_example("ranges/ranges.flat.array.hugr").unwrap();

    let f = hugr
        .children(hugr.module_root())
        .find(|n| {
            hugr.get_optype(*n)
                .as_func_defn()
                .is_some_and(|fd| fd.func_name() == "f")
        })
        .unwrap();
    hugr.set_entrypoint(f);
    ConstantFoldPass::default().run(&mut hugr).unwrap();
    BorrowSquashPass::default().run(&mut hugr).unwrap();
    run_pytket(&mut hugr);
    let expected_counts =
        count_gates(&load_guppy_circuit("ranges", HugrFileType::Optimized).unwrap());
    assert_eq!(count_gates(&hugr), expected_counts);
}

#[rstest]
#[case::angles("angles")]
#[should_panic]
#[case::nested("nested")]
#[should_panic]
#[case::ranges("ranges")]
#[cfg_attr(miri, ignore)] // Opening files is not supported in (isolated) miri
fn flatten_guppy(#[case] name: &str) {
    let mut hugr = load_guppy_circuit(name, HugrFileType::Original).unwrap();
    NormalizeGuppy::default().run(&mut hugr).unwrap();
    let target = load_guppy_circuit(name, HugrFileType::Flat).unwrap();
    assert_eq!(count_gates(&hugr), count_gates(&target));
}

/// Check that each example optimizes to the full extent given by the .opt .hugr files.
#[rstest]
#[case::nested_array("nested_array")]
#[should_panic]
#[case::angles("angles")]
#[should_panic]
#[case::false_branch("false_branch")]
#[should_panic]
#[case::simple_cx("simple_cx")]
#[should_panic]
#[case::nested("nested")]
#[should_panic]
#[case::ranges("ranges")]
fn optimize_guppy(#[case] name: &str) {
    let mut hugr = load_guppy_circuit(name, HugrFileType::Original).unwrap();
    let flat = count_gates(
        load_guppy_circuit(name, HugrFileType::Flat)
            .ok()
            .as_ref()
            .unwrap_or(&hugr),
    );
    let opt = count_gates(&load_guppy_circuit(name, HugrFileType::Optimized).unwrap());

    NormalizeGuppy::default().run(&mut hugr).unwrap();
    assert_eq!(count_gates(&hugr), flat);

    run_pytket(&mut hugr);

    assert_eq!(count_gates(&hugr), opt);

    // Lower to QSystem. This may blow up the HUGR size.
    QSystemPass::default().run(&mut hugr).unwrap();

    hugr.validate().unwrap_or_else(|e| panic!("{e}"));
}
