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
    HugrView, IncomingPort, Node, Wire, core::HugrNode, extension::simple_op::MakeExtensionOp,
    ops::ExtensionOp,
};
use itertools::Itertools;

use crate::extension::modifier::Modifier;
pub mod control;
pub mod dagger;
pub mod modifier_resolver;
pub mod power;

/// An accumulated modifier that combines control, dagger, and power modifiers.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
struct CombinedModifier<N: HugrNode = Node> {
    // Number of all control qubits
    control: usize,
    // Control arrays applied so far
    // The sum is supposed to be equal to `control`.
    accum_ctrl: Vec<usize>,
    /// Whether the dagger modifier has been applied.
    dagger: bool,
    /// Runtime exponent input ports collected from power modifiers.
    exponent_inputs: Vec<(N, IncomingPort)>,
}

impl<N: HugrNode> CombinedModifier<N> {
    /// Return the number of registered power exponents.
    fn num_exponents(&self) -> usize {
        self.exponent_inputs.len()
    }

    /// Resolve the registered exponent inputs to their source wires in `h`.
    fn exponent_wires(&self, h: &impl HugrView<Node = N>) -> Vec<Wire<N>> {
        self.exponent_inputs
            .iter()
            .map(|&(node, port)| Wire::from_connected_port(node, port, h))
            .collect()
    }

    /// Add a modifier
    fn push(&mut self, ext_op: &ExtensionOp, node: N) {
        match Modifier::from_extension_op(ext_op) {
            Ok(Modifier::ControlModifier) => {
                let ctrl = ext_op.args()[0].as_nat().unwrap() as usize;
                self.control += ctrl;
                self.accum_ctrl.push(ctrl);
            }
            Ok(Modifier::DaggerModifier) => self.dagger = !self.dagger,
            Ok(Modifier::PowerModifier) => {
                self.exponent_inputs.push((node, IncomingPort::from(1)));
            }
            Err(_) => {}
        }
    }

    /// Returns a compact string representation of the combined modifier environment.
    ///
    /// The string consists of `'C'` followed by the number of control arrays (`self.accum_ctrl`),
    /// joined by dots, and `'D'` if the dagger modifier has been applied.
    /// If no control qubits are present, `'C'` is omitted.
    /// Power modifiers are represented by `'P'` followed by the number of exponent inputs.
    fn compact_string(&self) -> String {
        let mut s = String::new();
        if self.control > 0 {
            s.push('C');
            s.push_str(&self.accum_ctrl.iter().map(|c| c.to_string()).join("."));
        }
        if self.dagger {
            s.push('D');
        }
        if !self.exponent_inputs.is_empty() {
            s.push('P');
            s.push_str(&self.num_exponents().to_string());
        }
        s
    }
}

impl<N: HugrNode> Default for CombinedModifier<N> {
    fn default() -> Self {
        Self {
            control: 0,
            accum_ctrl: Vec::new(),
            dagger: false,
            exponent_inputs: Vec::new(),
        }
    }
}
