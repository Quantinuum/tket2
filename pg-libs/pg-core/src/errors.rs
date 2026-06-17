use crate::Op;
#[derive(Debug, thiserror::Error)]
pub enum PauliGraphError {
    #[error("Invalid PauliGraph Op: {op:?}, {message}")]
    InvalidOp { op: Op, message: String },

    #[error("Invalid input JSON: {message}")]
    InvalidInputJson { message: String },
    // Everything else: "invalid input, here's why"
    #[error("{message}")]
    Invalid { message: String },
}
