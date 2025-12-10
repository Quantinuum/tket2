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

use hugr::{
    core::HugrNode, extension::simple_op::MakeExtensionOp, hugr::hugrmut::HugrMut,
    ops::ExtensionOp, HugrView,
};

mod pass;
use crate::extension::modifier::Modifier;
pub mod control;
pub mod dagger;
pub mod modifier_resolver;
pub mod power;

pub use pass::ModifierResolverPass;

/// An accumulated modifier that combines control, dagger, and power modifiers.
#[derive(Debug, Default, Clone, PartialEq, Eq, Hash)]
pub struct CombinedModifier {
    // Number of all control qubits
    control: usize,
    // Control arrays applied so far
    // The sum is supposed to be equal to `control`.
    accum_ctrl: Vec<usize>,
    dagger: bool,
    #[allow(dead_code)]
    power: bool,
}

impl CombinedModifier {
    /// Add a modifier
    pub fn push(&mut self, ext_op: &ExtensionOp) {
        match Modifier::from_extension_op(ext_op) {
            Ok(Modifier::ControlModifier) => {
                let ctrl = ext_op.args()[0].as_nat().unwrap() as usize;
                self.control += ctrl;
                self.accum_ctrl.push(ctrl);
            }
            Ok(Modifier::DaggerModifier) => self.dagger = !self.dagger,
            Ok(Modifier::PowerModifier) => self.power = !self.power,
            Err(_) => {}
        }
    }
}

/// Flags for each modifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct ModifierFlags {
    control: bool,
    dagger: bool,
    power: bool,
}

impl ModifierFlags {
    fn from_metadata<N: HugrNode>(h: &impl HugrView<Node = N>, n: N) -> Option<Self> {
        h.get_metadata(n, "unitary")
            .and_then(serde_json::Value::as_u64)
            .map(|num| ModifierFlags {
                dagger: (num & 1) != 0,
                control: (num & 2) != 0,
                power: (num & 4) != 0,
            })
    }

    fn set_metadata<N: HugrNode>(&self, h: &mut impl HugrMut<Node = N>, n: N) {
        let mut num = 0;
        if self.dagger {
            num |= 1;
        }
        if self.control {
            num |= 2;
        }
        if self.power {
            num |= 4;
        }
        *h.get_metadata_mut(n, "unitary") = serde_json::Value::from(num);
    }

    fn satisfies(&self, combined: &CombinedModifier) -> bool {
        (combined.control == 0 || self.control)
            && (!combined.dagger || self.dagger)
            && (!combined.power || self.power)
    }

    fn from_combined(combined: &CombinedModifier) -> Self {
        ModifierFlags {
            control: combined.control > 0,
            dagger: combined.dagger,
            power: combined.power,
        }
    }

    fn or(self, other: &Option<Self>) -> Self {
        match other {
            None => self,
            Some(other) => ModifierFlags {
                control: self.control || other.control,
                dagger: self.dagger || other.dagger,
                power: self.power || other.power,
            },
        }
    }
}
