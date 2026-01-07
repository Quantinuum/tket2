//! Rust interface for tket1-passes library
//!
//! This crate provides safe Rust bindings to the `libtket1-passes` C library
//! that exposes some of TKET1's passes as Rust functions.
#![warn(clippy::undocumented_unsafe_blocks)] // TODO: Fix and move to the workspace lints.

use std::ffi::{CStr, CString, c_char};
use std::ptr;
use thiserror::Error;
use tket_json_rs::{OpType, SerialCircuit};

// Include the auto-generated bindings
mod ffi {
    #![allow(
        non_camel_case_types,
        non_upper_case_globals,
        dead_code,
        non_snake_case,
        clippy::allow_attributes,
        reason = "generated code"
    )]
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}

/// Error types that can occur when using the tket1-passes library
#[derive(Error, Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum PassError {
    /// An error happened in TKET1
    #[error("TKET1 error: {0}")]
    Tket1Error(String),
    /// Expected TKET1-allocated data, got NULL pointer
    #[error("Expected TKET1-allocated data, got NULL pointer")]
    NullPointer,
    /// Failed to process JSON
    #[error("Failed to process JSON: {0}")]
    JsonError(String),
    /// Target gate type is not supported
    #[error("Invalid target gate: {0:?}. Supported gates are CX and TK2.")]
    InvalidTargetGate(OpType),
}

impl From<std::ffi::NulError> for PassError {
    fn from(_: std::ffi::NulError) -> Self {
        PassError::NullPointer
    }
}

impl From<serde_json::Error> for PassError {
    fn from(err: serde_json::Error) -> Self {
        PassError::JsonError(err.to_string())
    }
}

impl TryFrom<ffi::TketError> for PassError {
    type Error = &'static str;

    fn try_from(value: ffi::TketError) -> Result<Self, Self::Error> {
        if value == ffi::TketError_TKET_SUCCESS {
            return Err("No error occurred");
        }
        // SAFETY: tket_error_string returns a valid pointer to a null-terminated string.
        let c_str = unsafe { CStr::from_ptr(ffi::tket_error_string(value)) };
        let s = c_str.to_string_lossy().to_string();
        Ok(PassError::Tket1Error(s))
    }
}

/// A quantum circuit in TKET1's in-memory format
#[derive(Debug)]
pub struct Tket1Circuit {
    inner: *mut ffi::TketCircuit,
}

impl From<&SerialCircuit> for Tket1Circuit {
    fn from(serial_circuit: &SerialCircuit) -> Self {
        Tket1Circuit::from_serial_circuit(serial_circuit).unwrap()
    }
}

impl From<SerialCircuit> for Tket1Circuit {
    fn from(serial_circuit: SerialCircuit) -> Self {
        Tket1Circuit::from_serial_circuit(&serial_circuit).unwrap()
    }
}

impl From<Tket1Circuit> for SerialCircuit {
    fn from(circuit: Tket1Circuit) -> Self {
        circuit.to_serial_circuit().unwrap()
    }
}

impl Tket1Circuit {
    /// Create a new circuit from a JSON string
    pub fn from_serial_circuit(serial_circuit: &SerialCircuit) -> Result<Self, PassError> {
        let json = serde_json::to_vec(&serial_circuit)?;
        let c_json = CString::new(json)?;
        // SAFETY: c_json has been validated by CString::new.
        let circuit = unsafe { ffi::tket_circuit_from_json(c_json.as_ptr()) };

        if circuit.is_null() {
            return Err(PassError::JsonError(
                "Failed to create Circuit from JSON".to_string(),
            ));
        }

        Ok(Tket1Circuit { inner: circuit })
    }

    /// Convert the circuit to a JSON string
    pub fn to_serial_circuit(&self) -> Result<SerialCircuit, PassError> {
        let mut json_ptr: *mut c_char = ptr::null_mut();

        // SAFETY: json_ptr is a valid pointer to a null-terminated string.
        let error_code = unsafe { ffi::tket_circuit_to_json(self.inner, &mut json_ptr) };

        if let Ok(pass_error) = error_code.try_into() {
            return Err(pass_error);
        }

        if json_ptr.is_null() {
            return Err(PassError::NullPointer);
        }

        let serial_circuit = {
            // SAFETY: json_ptr is a valid pointer to a null-terminated string.
            let c_str = unsafe { CStr::from_ptr(json_ptr) };
            serde_json::from_str(c_str.to_string_lossy().as_ref())?
        };

        // SAFETY: json_ptr is a valid pointer to a null-terminated string.
        // The pointer is not shared with any other living object at this point, so it is safe to free
        unsafe { ffi::tket_free_string(json_ptr) };

        Ok(serial_circuit)
    }
}

impl Drop for Tket1Circuit {
    fn drop(&mut self) {
        if !self.inner.is_null() {
            // SAFETY: self.inner is a valid pointer to a TKET1 circuit.
            unsafe { ffi::tket_free_circuit(self.inner) };
        }
    }
}

/// A pytket pass in TKET1's in-memory format
#[derive(Debug)]
pub struct Tket1Pass {
    inner: *mut ffi::TketPass,
}

impl Tket1Pass {
    /// Load a json-encoded pass into memory.
    pub fn from_json(json: &str) -> Result<Self, PassError> {
        let c_json = CString::new(json)?;
        // SAFETY: c_json has been validated by CString::new.
        let pass = unsafe { ffi::tket_pass_from_json(c_json.as_ptr()) };
        if pass.is_null() {
            return Err(PassError::JsonError(
                "Failed to create pytket pass from JSON".to_string(),
            ));
        }
        Ok(Tket1Pass { inner: pass })
    }

    /// Apply the pass to a circuit
    pub fn run(&self, circuit: &mut Tket1Circuit) -> Result<(), PassError> {
        // SAFETY: circuit.inner and self.inner are valid pointers to TKET1 circuits and passes respectively, or NULL.
        let error_code: ffi::TketError = unsafe { ffi::tket_apply_pass(circuit.inner, self.inner) };
        if let Ok(pass_error) = error_code.try_into() {
            return Err(pass_error);
        }
        Ok(())
    }

    /// Load a json-encoded pass and run it on a circuit
    pub fn run_from_json(json: &str, circuit: &mut Tket1Circuit) -> Result<(), PassError> {
        let pass = Self::from_json(json)?;
        pass.run(circuit)
    }
}

impl Drop for Tket1Pass {
    fn drop(&mut self) {
        if !self.inner.is_null() {
            // SAFETY: self.inner is a valid pointer to a TKET1 pass.
            unsafe { ffi::tket_free_pass(self.inner) };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rstest::*;

    const CIRC_STR: &str = r#"{"bits": [], "commands": [{"args": [["q", [0]], ["q", [1]]], "op": {"type": "CX"}}, {"args": [["q", [0]], ["q", [1]]], "op": {"type": "CX"}}], "created_qubits": [], "discarded_qubits": [], "implicit_permutation": [[["q", [0]], ["q", [0]]], [["q", [1]], ["q", [1]]]], "phase": "0.0", "qubits": [["q", [0]], ["q", [1]]]}"#;
    const TWO_QUBIT_SQUASH_STR: &str = r#"{"StandardPass": {"allow_swaps": true, "fidelity": 1.0, "name": "KAKDecomposition", "target_2qb_gate": "CX"}, "pass_class": "StandardPass"}"#;
    const CLIFFORD_SIMP_STR: &str = r#"{"StandardPass": {"allow_swaps": true, "name": "CliffordSimp", "target_2qb_gate": "CX"}, "pass_class": "StandardPass"}"#;

    #[fixture]
    fn circuit() -> SerialCircuit {
        serde_json::from_str(CIRC_STR).unwrap()
    }

    #[fixture]
    fn two_qubit_squash_pass() -> Tket1Pass {
        Tket1Pass::from_json(TWO_QUBIT_SQUASH_STR).unwrap()
    }

    #[fixture]
    fn clifford_simp_pass() -> Tket1Pass {
        Tket1Pass::from_json(CLIFFORD_SIMP_STR).unwrap()
    }

    #[rstest]
    fn test_circuit_creation(circuit: SerialCircuit) {
        let circuit_ptr = Tket1Circuit::from_serial_circuit(&circuit).unwrap();
        let serial_circuit = circuit_ptr.to_serial_circuit().unwrap();
        assert_eq!(circuit, serial_circuit);
    }

    #[rstest]
    #[case::two_qubit_squash(two_qubit_squash_pass())]
    #[case::clifford_simp(clifford_simp_pass())]
    fn test_two_qubit_squash(circuit: SerialCircuit, #[case] pass: Tket1Pass) {
        assert_eq!(circuit.commands.len(), 2);
        let mut circuit_ptr = Tket1Circuit::from_serial_circuit(&circuit).unwrap();
        pass.run(&mut circuit_ptr).unwrap();
        let serial_circuit = circuit_ptr.to_serial_circuit().unwrap();
        assert_eq!(serial_circuit.commands.len(), 0);
    }

    #[test]
    fn test_error_null_circ() {
        let mut null_circ = Tket1Circuit {
            inner: ptr::null_mut(),
        };
        assert_eq!(
            clifford_simp_pass().run(&mut null_circ).unwrap_err(),
            PassError::Tket1Error("Invalid NULL pointer in arguments".to_string())
        );
    }

    #[test]
    fn test_error_handling_from_json() {
        // badly formatted JSON
        let circuit_json = r#"{"bits": [], "commands": [{"args": [["q", [0]], ["q", [1]]], "op": {"type": "CX"}}, {"args": [["q", [0]], ["q", [1]]], "op": {"type": "CX"}}], "created_qubits": [], "discarded_qubits": [], "implicit_permutation": [[["q", [0]], ["q", [0]]], [["q", [1]], ["q", [1]]]], "phase": "0.0", "qubits": [["q", [0]] ["q", [1]]]}"#;
        let c_str = CString::new(circuit_json).unwrap();
        // SAFETY: c_str has been validated by CString::new.
        let circ_ptr = unsafe { ffi::tket_circuit_from_json(c_str.as_ptr()) };
        assert!(circ_ptr.is_null());
    }
}
