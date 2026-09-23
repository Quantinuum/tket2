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
use itertools::Itertools;

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

    /// Returns a compact string representation of the combined modifier environment.
    ///
    /// The string consists of `'C'` followed by the number of control arrays (`self.accum_ctrl`),
    /// joined by dots, and `'D'` if the dagger modifier has been applied.
    /// If no control qubits are present, `'C'` is omitted.
    fn compact_string(&self) -> String {
        let mut s = String::new();
        if self.control > 0 {
            s.push('C');
            s.push_str(&self.accum_ctrl.iter().map(|c| c.to_string()).join("."));
        }
        if self.dagger {
            s.push('D');
        }
        s
    }
}
