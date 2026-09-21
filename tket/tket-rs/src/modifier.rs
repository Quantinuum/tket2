//! Module for richer circuit representation and operations.
//! This module provides three extensions: modifiers, global phase, and safe drop.
//!
//! ## Modifiers
//! Modifiers are functions that takes circuits and return modified circuits
//! by applying modifiers: control, dagger, or power.
//!
//! ## Global Phase
//! Global phase is an operation that applies some global phase to a circuit.
//! It is implemented as a side-effect that takes a rotation angle as an input.

use hugr::{extension::simple_op::MakeExtensionOp, ops::ExtensionOp};

use crate::extension::modifier::Modifier;
pub mod control;
pub mod dagger;
pub mod modifier_resolver;
pub mod power;

/// An accumulated modifier that combines control, dagger, and power modifiers.
#[derive(Debug, Default, Clone, PartialEq, Eq, Hash)]
struct CombinedModifier {
    // Number of all control qubits
    control: usize,
    // Control arrays applied so far
    // The sum is supposed to be equal to `control`.
    accum_ctrl: Vec<usize>,
    /// Whether the dagger modifier has been applied.
    dagger: bool,
}

impl CombinedModifier {
    /// Add a modifier
    fn push<N>(
        &mut self,
        ext_op: &ExtensionOp,
        node: N,
    ) -> Result<(), modifier_resolver::ModifierResolverErrors<N>> {
        match Modifier::from_extension_op(ext_op) {
            Ok(Modifier::ControlModifier) => {
                let ctrl = ext_op.args()[0].as_nat().unwrap() as usize;
                self.control += ctrl;
                self.accum_ctrl.push(ctrl);
            }
            Ok(Modifier::DaggerModifier) => self.dagger = !self.dagger,
            Ok(Modifier::PowerModifier) => {
                return Err(
                    modifier_resolver::ModifierResolverErrors::PowerModifierNotSupported { node },
                );
            }
            Err(_) => {}
        }
        Ok(())
    }
}
