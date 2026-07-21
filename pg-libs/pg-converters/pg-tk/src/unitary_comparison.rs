use crate::converters::*;
use std::io::{BufRead, BufReader, Write};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};
use std::sync::{Mutex, OnceLock};

use pg_core::PauliGraph;
use serde_json::{Value, json};

/// The Python worker script, embedded at compile time
const WORKER_SRC: &str = include_str!("../scripts/compare_unitaries_worker.py");

/// Candidate Python interpreters to run the unitary comparison worker.
///
/// Resolution order:
/// 1. The `PG_TK_PYTHON` environment variable.
/// 2. `python`, then `python3`, as found on `PATH`.
fn python_candidates() -> Vec<String> {
    if let Ok(python) = std::env::var("PG_TK_PYTHON") {
        let python = python.trim();
        if !python.is_empty() {
            return vec![python.to_string()];
        }
    }
    vec!["python".to_string(), "python3".to_string()]
}

/// A long-lived Python worker to reduce the overhead of repeatedly starting Python.
struct TkCompareWorker {
    child: Child,
    stdin: ChildStdin,
    stdout: BufReader<ChildStdout>,
}

impl TkCompareWorker {
    fn new() -> Self {
        let candidates = python_candidates();
        // Probe each candidate interpreter by importing `pytket`.
        let mut failures = Vec::new();
        for candidate in &candidates {
            match Command::new(candidate)
                .args(["-c", "import pytket"])
                .output()
            {
                Ok(output) if output.status.success() => {
                    return Self::spawn_worker(candidate);
                }
                Ok(output) => failures.push(format!(
                    "`{candidate}`: `import pytket` failed ({}):\n{}",
                    output.status,
                    String::from_utf8_lossy(&output.stderr).trim()
                )),
                Err(err) => failures.push(format!("`{candidate}`: could not be spawned ({err})")),
            }
        }

        panic!(
            "No usable Python interpreter with `pytket` was found for the unitary comparison \
             worker. Tried {candidates:?}:\n{}\n\
             Install `pytket` in the environment, or set `PG_TK_PYTHON` to a Python interpreter \
             that has it.",
            failures.join("\n")
        );
    }

    /// Launch the worker
    fn spawn_worker(interpreter: &str) -> Self {
        let mut child = Command::new(interpreter)
            .args(["-c", WORKER_SRC])
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .expect("Failed to spawn python unitary comparison worker");

        let stdin = child
            .stdin
            .take()
            .expect("Failed to open stdin of python unitary comparison worker");
        let stdout = BufReader::new(
            child
                .stdout
                .take()
                .expect("Failed to open stdout of python unitary comparison worker"),
        );

        Self {
            child,
            stdin,
            stdout,
        }
    }

    fn compare(&mut self, input: &str) -> Value {
        self.stdin
            .write_all(input.as_bytes())
            .expect("Failed to write circuit JSON to python unitary comparison worker stdin");
        self.stdin
            .write_all(b"\n")
            .expect("Failed to write newline to python unitary comparison worker stdin");
        self.stdin
            .flush()
            .expect("Failed to flush python unitary comparison worker stdin");

        let mut stdout_line = String::new();
        let bytes_read = self
            .stdout
            .read_line(&mut stdout_line)
            .expect("Failed to read python unitary comparison worker stdout");
        if bytes_read == 0 {
            let status = self
                .child
                .try_wait()
                .expect("Failed to inspect python unitary comparison worker status");
            panic!(
                "Python unitary comparison worker closed stdout unexpectedly. Status: {status:?}"
            );
        }

        serde_json::from_str(stdout_line.trim()).unwrap_or_else(|err| {
            panic!("Failed to parse python worker output as JSON ({err}).\nstdout:\n{stdout_line}")
        })
    }
}

static TK_COMPARE_WORKER: OnceLock<Mutex<TkCompareWorker>> = OnceLock::new();

fn compare_unitaries_with_worker(input: &str) -> Value {
    let worker = TK_COMPARE_WORKER.get_or_init(|| Mutex::new(TkCompareWorker::new()));
    let mut worker = worker
        .lock()
        .expect("Python unitary comparison worker mutex was poisoned");
    worker.compare(input)
}

/// Compare the unitaries of two `PauliGraph`s by converting them to TKET JSON
/// and calling `pytket`'s `compare_unitaries` function.
///
/// # Arguments
///
/// - `first` (`&PauliGraph`) - The first `PauliGraph` to compare.
/// - `second` (`&PauliGraph`) - The second `PauliGraph` to compare.
///
/// # Returns
///
/// - `bool` - `true` if the unitaries of the two `PauliGraph`s are equivalent, `false` otherwise.
///
/// # Panics
///
/// This function will panic if:
/// - the conversion of either `PauliGraph` to TKET JSON fails, or
/// - no Python interpreter with `pytket` can be found.
///
/// The interpreter is resolved from the `PG_TK_PYTHON` environment variable if
/// set, otherwise `python`/`python3` on `PATH` — i.e. whatever Python
/// environment the caller has active.
pub fn compare_unitaries_via_tk(first: &PauliGraph, second: &PauliGraph) -> bool {
    let first_json = pg_to_tk_json(first)
        .unwrap_or_else(|err| {
            panic!("Failed to convert first PauliGraph to TKET JSON: {err}");
        })
        .to_string();
    let second_json = pg_to_tk_json(second)
        .unwrap_or_else(|err| {
            panic!("Failed to convert second PauliGraph to TKET JSON: {err}");
        })
        .to_string();
    let input = json!({ "first": first_json, "second": second_json }).to_string();

    let result = compare_unitaries_with_worker(&input);

    if let Some(trace) = result.get("error") {
        panic!(
            "Python error during unitary comparison:\n{}",
            trace.as_str().unwrap_or("<non-string error>")
        );
    }

    result["result"]
        .as_bool()
        .expect("Unexpected result format from python unitary comparison")
}
// add test module and tests
#[cfg(test)]
mod tests {
    use super::*;
    use pg_core::{GateData, GateType, Op, Pauli, PauliGraph, RotationData, TableauData};

    #[test]
    fn test_compare_identities_via_tk() {
        let pg1 = PauliGraph::new(2);
        let pg2 = PauliGraph::new(2);
        assert!(compare_unitaries_via_tk(&pg1, &pg2));
    }

    #[test]
    fn test_compare_unitaries_via_tk() {
        let mut pg1 = PauliGraph::new(2);
        pg1.add_op(Op::Gate {
            data: GateData::new(GateType::H, vec![0]),
        });
        let pg2 = PauliGraph::new(2);
        assert!(!compare_unitaries_via_tk(&pg1, &pg2));
    }

    #[test]
    fn test_compare_unitaries_via_tk_2() {
        let exp0 = Op::Rotation {
            data: RotationData::new(vec![Pauli::X], 1.0),
        };
        let exp1 = Op::Rotation {
            data: RotationData::new(vec![Pauli::Z], 1.0),
        };
        let cliff = Op::Gate {
            data: GateData::new(GateType::H, vec![0]),
        };
        let pg = PauliGraph::new(1).with_ops(vec![cliff.clone(), exp0]);
        let pg1 = PauliGraph::new(1).with_ops(vec![exp1, cliff]);
        assert!(compare_unitaries_via_tk(&pg, &pg1));
    }

    #[test]
    fn test_compare_tableaux() {
        let pg0 = PauliGraph::new(2).with_ops(vec![Op::Tableau {
            data: TableauData::new(
                vec![
                    (vec![Pauli::Z, Pauli::I], false),
                    (vec![Pauli::I, Pauli::Z], true),
                ],
                vec![
                    (vec![Pauli::X, Pauli::I], true),
                    (vec![Pauli::I, Pauli::X], true),
                ],
            ),
        }]);
        let pg1 = PauliGraph::new(2).with_ops(vec![Op::Gate {
            data: GateData::new(GateType::X, vec![0]),
        }]);
        assert!(compare_unitaries_via_tk(&pg0, &pg1));
    }
}
