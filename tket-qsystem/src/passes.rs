//! Passes for optimizing and lowering HUGRs with native QSystem operations.

mod compat;
pub mod llvm;
pub mod rebase;

#[expect(deprecated)]
pub use compat::{QSystemPass, QSystemPassError};
pub use llvm::{QSystemLLVMPass, QSystemLLVMPassError};
pub use rebase::{QSystemRebasePass, QSystemRebasePassError};
