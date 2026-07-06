use crate::Op;

/// Error type for PauliGraph operations and input validation.
#[derive(Debug, thiserror::Error)]
pub enum PauliGraphError {
    /// Invalid operation within a PauliGraph.
    #[error("Invalid PauliGraph Op: {op:?}, {message}")]
    InvalidOp {
        /// The invalid operation.
        op: Op,
        /// The error message.
        message: String,
    },

    /// Invalid input JSON during deserialization.
    #[error("Invalid input JSON: {0}")]
    InvalidInputJson(String),
}
