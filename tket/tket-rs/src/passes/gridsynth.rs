//! A pass that applies the gridsynth algorithm to synthesise arbitrary rotations to Clifford+T.

use crate::TketOp;
use crate::extension::rotation::ConstRotation;
use crate::passes::{
    ComposablePass, InScope, InlineFunctionsPass, Normalize, PassScope, WithScope,
    inline_funcs::InlineFuncsError, normalize::NormalizeErrors,
};

use hugr::{
    HugrView, Node,
    hugr::{ValidationError, hugrmut::HugrMut},
};
use rsgridsynth::config::config_from_theta_epsilon;
use rsgridsynth::gridsynth::gridsynth_gates;
use std::collections::HashSet;
use std::sync::{LazyLock, Mutex};

/// rsgridsynth uses a global mutable precision counter PREC_BITS that isn't
/// thread safe, so serialise gridsynth string synthesis.
static GRIDSYNTH_LOCK: LazyLock<Mutex<()>> = LazyLock::new(|| Mutex::new(()));

/// Error raised by [GridSynthPass]
#[derive(derive_more::Error, Debug, derive_more::Display, derive_more::From)]
#[non_exhaustive]
pub enum GridSynthError {
    /// The approximation tolerance is outside the supported range.
    #[error(ignore)]
    #[display("Invalid gridsynth epsilon {_0}: expected a finite value strictly between 0 and 1")]
    InvalidEpsilon(f64),
    /// The resulting HUGR is invalid.
    InvalidHUGR(#[from] ValidationError<Node>),
    /// Error inlining functions.
    InlineError(#[from] InlineFuncsError),
    /// Error normalizing the HUGR.
    NormalizeError(#[from] NormalizeErrors),
    /// The angle of the Rz gate cannot be determined statically.
    #[error(ignore)]
    #[display("Could not determine angle of {_0} statically")]
    UndefinedAngleError(Node),
}

/// Applies the gridsynth algorithm to synthesise arbitrary rotations to Clifford+T.
///
/// Applies [InlineFunctionsPass] and [Normalize] to flatten the HUGR so angles can be known
/// statically. When this is not possible, we return a [GridSynthError::UndefinedAngleError].
#[derive(Debug, Clone)]
pub struct GridSynthPass {
    /// Where to apply the pass. See [PassScope] for details.
    scope: PassScope,
    /// Precision of the gridsynth approximation.
    epsilon: f64,
    /// Seed for the gridsynth algorithm.
    seed: u64,
}

impl Default for GridSynthPass {
    fn default() -> Self {
        Self {
            scope: PassScope::default(),
            epsilon: 1e-3,
            seed: 1234,
        }
    }
}

impl GridSynthPass {
    /// Sets the precision of the gridsynth approximation.
    ///
    /// Must be finite and strictly between 0 and 1. Invalid values are rejected
    /// when the pass runs, before modifying the HUGR.
    pub fn with_epsilon(mut self, epsilon: f64) -> Self {
        self.epsilon = epsilon;
        self
    }

    /// Sets the seed for the gridsynth algorithm.
    pub fn with_seed(mut self, seed: u64) -> Self {
        self.seed = seed;
        self
    }
}

impl<H: HugrMut<Node = Node> + 'static> ComposablePass<H> for GridSynthPass {
    type Error = GridSynthError;
    type Result = ();

    fn run(&self, hugr: &mut H) -> Result<(), Self::Error> {
        if !self.epsilon.is_finite() || self.epsilon <= 0.0 || self.epsilon >= 1.0 {
            return Err(GridSynthError::InvalidEpsilon(self.epsilon));
        }

        InlineFunctionsPass::default()
            .with_scope(self.scope.clone())
            .run(hugr)?;
        Normalize::default()
            .with_scope(self.scope.clone())
            .run(hugr)?;

        let rotations: Vec<_> = self
            .scope
            .regions(hugr)
            .flat_map(|parent| hugr.children(parent))
            .filter(|n| hugr.get_optype(*n).cast::<TketOp>() == Some(TketOp::Rz))
            .map(|rz_node| {
                let angle_port = hugr
                    .node_inputs(rz_node)
                    .nth(1)
                    .expect("Rz should have an angle input");

                let (source_node, _) = hugr
                    .single_linked_output(rz_node, angle_port)
                    .ok_or(GridSynthError::UndefinedAngleError(rz_node))?;

                if !hugr.get_optype(source_node).is_load_constant() {
                    return Err(GridSynthError::UndefinedAngleError(rz_node));
                }

                let const_node = hugr
                    .static_source(source_node)
                    .filter(|&n| hugr.get_optype(n).is_const())
                    .ok_or(GridSynthError::UndefinedAngleError(rz_node))?;

                let theta = find_angle(hugr, const_node);
                Ok((rz_node, theta, source_node, const_node))
            })
            .collect::<Result<_, GridSynthError>>()?;

        for &(rz_node, theta, _, _) in &rotations {
            let gates = gridsynth(theta, self.epsilon, self.seed);
            replace_rz_with_gates(hugr, rz_node, gates)?;
        }

        let loads: HashSet<Node> = rotations.iter().map(|&(_, _, load, _)| load).collect();
        let constants: HashSet<Node> = rotations
            .iter()
            .map(|&(_, _, _, constant)| constant)
            .collect();

        for node in loads.into_iter().chain(constants) {
            let unused = hugr
                .node_outputs(node)
                .all(|port| !hugr.is_linked(node, port));

            if unused && self.scope.in_scope(hugr, node) == InScope::Yes {
                hugr.remove_node(node);
            }
        }

        Ok(())
    }
}

/// Extracts the angle (in radians) from a rotation constant loaded directly into `Rz`.
fn find_angle<H: HugrView<Node = Node>>(hugr: &H, const_node: Node) -> f64 {
    let value = hugr
        .get_optype(const_node)
        .as_const()
        .expect("node is a Const")
        .value();

    value
        .get_custom_value::<ConstRotation>()
        .expect("constant loaded directly into Rz must be a ConstRotation")
        .to_radians()
}

/// Runs the gridsynth algorithm on `theta` (radians), and returns the gate string.
fn gridsynth(theta: f64, epsilon: f64, seed: u64) -> String {
    let _guard = GRIDSYNTH_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let verbose = false;
    let up_to_phase = false;
    let mut config = config_from_theta_epsilon(theta, epsilon, seed, verbose, up_to_phase);
    simplify(gridsynth_gates(&mut config).gates)
}

/// Compresses a gridsynth gate sequence into a shorter normal form.
fn simplify(gates: String) -> String {
    let mut pending = gates.into_bytes();
    pending.reverse();
    let mut simplified = Vec::with_capacity(pending.len());

    while let Some(gate) = pending.pop() {
        simplified.push(gate);
        let (consumed, replacement): (usize, &[u8]) = match simplified.as_slice() {
            // Cancellation rules
            [.., b'Z', b'Z']
            | [.., b'X', b'X']
            | [.., b'H', b'H']
            | [.., b'T', b'D']
            | [.., b'D', b'T'] => (2, b""),
            [.., b'S', b'S'] => (2, b"Z"),
            [.., b'T', b'T'] => (2, b"S"),
            [.., b'D', b'D'] => (2, b"SZ"),
            // Rules to push Paulis to the right
            [.., b'Z', b'S'] => (2, b"SZ"),
            [.., b'Z', b'T'] => (2, b"TZ"),
            [.., b'Z', b'D'] => (2, b"DZ"),
            [.., b'X', b'S'] => (2, b"SZX"),
            [.., b'X', b'T'] => (2, b"DX"),
            [.., b'X', b'D'] => (2, b"TX"),
            [.., b'Z', b'H'] => (2, b"HX"),
            [.., b'X', b'H'] => (2, b"HZ"),
            [.., b'X', b'Z'] => (2, b"ZX"),
            // Interaction of H and S (reduces number of H)
            [.., b'H', b'S', b'H'] => (3, b"SHSX"),
            // Interaction of S and T (reduces number of S)
            [.., b'D', b'S'] | [.., b'S', b'D'] => (2, b"T"),
            [.., b'T', b'S'] | [.., b'S', b'T'] => (2, b"DZ"),
            _ => continue,
        };
        simplified.truncate(simplified.len() - consumed);
        pending.extend(replacement.iter().rev().copied());
    }
    String::from_utf8(simplified).expect("gate rewrites preserve UTF-8")
}

/// Replace an `Rz` node with the Clifford+T gates in `gates`.
fn replace_rz_with_gates<H: HugrMut<Node = Node>>(
    hugr: &mut H,
    rz_node: Node,
    mut gates: String,
) -> Result<(), GridSynthError> {
    // W is a global phase factor so we can ignore it
    gates.retain(|c| c != 'W');

    let new_nodes: Vec<Node> = gates
        .chars()
        .filter_map(|gate| match gate {
            'H' => Some(TketOp::H),
            'S' => Some(TketOp::S),
            'T' => Some(TketOp::T),
            'D' => Some(TketOp::Tdg),
            'X' => Some(TketOp::X),
            'Z' => Some(TketOp::Z),
            'I' => None,
            _ => panic!("The gate {gate} is not supported"),
        })
        .map(|op| hugr.add_node_after(rz_node, op))
        .collect();

    let q_in_port = hugr.node_inputs(rz_node).next().expect("Rz qubit input");
    let (mut prev_node, mut prev_port) = hugr
        .single_linked_output(rz_node, q_in_port)
        .expect("Rz qubit input should be connected");

    let q_out_port = hugr.node_outputs(rz_node).next().expect("Rz qubit output");
    let (next_node, next_port) = hugr
        .single_linked_input(rz_node, q_out_port)
        .expect("Rz qubit output should be connected");

    hugr.remove_node(rz_node);

    for current_node in new_nodes {
        let in_port = hugr.node_inputs(current_node).next().unwrap();
        hugr.connect(prev_node, prev_port, current_node, in_port);
        prev_node = current_node;
        prev_port = hugr.node_outputs(current_node).next().unwrap();
    }
    hugr.connect(prev_node, prev_port, next_node, next_port);

    Ok(())
}

impl WithScope for GridSynthPass {
    fn with_scope(mut self, scope: impl Into<PassScope>) -> Self {
        self.scope = scope.into();
        self
    }
}

// Testing guppy-generated HUGRs from test_files/guppy_optimization/gridsynth/
#[cfg(test)]
mod tests {
    use super::*;
    use hugr::Hugr;
    use std::io::BufReader;

    fn load_guppy_hugr(name: &str) -> Hugr {
        let path = format!(
            "{}/../../test_files/guppy_optimization/gridsynth/{}.hugr",
            env!("CARGO_MANIFEST_DIR"),
            name
        );
        let bytes = std::fs::read(&path).unwrap_or_else(|e| {
            panic!("Failed to read {}: {}", path, e);
        });
        Hugr::load(BufReader::new(bytes.as_slice()), None).unwrap()
    }

    fn count_gate(hugr: &Hugr, gate: TketOp) -> usize {
        hugr.nodes()
            .filter(|n| hugr.get_optype(*n).cast::<TketOp>() == Some(gate))
            .count()
    }

    #[test]
    #[cfg_attr(miri, ignore)]
    fn gridsynth_single_rz_with_float() {
        let mut hugr = load_guppy_hugr("single_rz_with_float");
        assert_eq!(count_gate(&hugr, TketOp::Rz), 1);
        GridSynthPass::default().run(&mut hugr).unwrap();
        hugr.validate().unwrap();
        assert_eq!(count_gate(&hugr, TketOp::Rz), 0);
        assert_eq!(count_gate(&hugr, TketOp::S), 1);
    }

    #[test]
    #[cfg_attr(miri, ignore)]
    fn gridsynth_single_rz_with_pi() {
        let mut hugr = load_guppy_hugr("single_rz_with_pi");
        assert_eq!(count_gate(&hugr, TketOp::Rz), 1);
        GridSynthPass::default().run(&mut hugr).unwrap();
        hugr.validate().unwrap();
        assert_eq!(count_gate(&hugr, TketOp::Rz), 0);
        assert_eq!(count_gate(&hugr, TketOp::S), 1);
    }

    #[test]
    #[cfg_attr(miri, ignore)]
    fn gridsynth_single_rz_long_string() {
        let mut hugr = load_guppy_hugr("single_rz_long_string");
        assert_eq!(count_gate(&hugr, TketOp::Rz), 1);
        GridSynthPass::default().run(&mut hugr).unwrap();
        hugr.validate().unwrap();
        assert_eq!(count_gate(&hugr, TketOp::Rz), 0);
        assert!(count_gate(&hugr, TketOp::H) > 1);
        assert!(count_gate(&hugr, TketOp::T) > 1);
    }

    #[test]
    #[cfg_attr(miri, ignore)]
    fn gridsynth_rz_with_angle_from_variable() {
        let mut hugr = load_guppy_hugr("rz_with_angle_from_variable");
        assert_eq!(count_gate(&hugr, TketOp::Rz), 1);
        GridSynthPass::default().run(&mut hugr).unwrap();
        hugr.validate().unwrap();
        assert_eq!(count_gate(&hugr, TketOp::Rz), 0);
        assert_eq!(count_gate(&hugr, TketOp::S), 1);
    }

    #[test]
    #[cfg_attr(miri, ignore)]
    fn gridsynth_nested_function() {
        let mut hugr = load_guppy_hugr("nested_function");
        assert_eq!(count_gate(&hugr, TketOp::Rz), 1);
        GridSynthPass::default().run(&mut hugr).unwrap();
        hugr.validate().unwrap();
        assert_eq!(count_gate(&hugr, TketOp::Rz), 0);
        assert_eq!(count_gate(&hugr, TketOp::S), 1);
    }

    #[test]
    #[cfg_attr(miri, ignore)]
    fn gridsynth_test_epsilon_approximate() {
        let mut hugr = load_guppy_hugr("test_epsilon");
        assert_eq!(count_gate(&hugr, TketOp::Rz), 1);
        GridSynthPass::default()
            .with_epsilon(1e-2)
            .run(&mut hugr)
            .unwrap();
        hugr.validate().unwrap();
        assert_eq!(count_gate(&hugr, TketOp::Rz), 0);
        assert_eq!(count_gate(&hugr, TketOp::S), 1);
    }

    #[test]
    #[cfg_attr(miri, ignore)]
    fn gridsynth_test_epsilon_precise() {
        let mut hugr = load_guppy_hugr("test_epsilon");
        assert_eq!(count_gate(&hugr, TketOp::Rz), 1);
        GridSynthPass::default()
            .with_epsilon(1e-4)
            .run(&mut hugr)
            .unwrap();
        hugr.validate().unwrap();
        assert_eq!(count_gate(&hugr, TketOp::Rz), 0);
        assert!(count_gate(&hugr, TketOp::H) > 1);
        assert!(count_gate(&hugr, TketOp::T) > 1);
    }

    #[test]
    #[cfg_attr(miri, ignore)]
    fn gridsynth_undefined_angle_errors() {
        let mut hugr = load_guppy_hugr("undefined_angle");
        let result = GridSynthPass::default().run(&mut hugr);
        assert!(matches!(
            result.unwrap_err(),
            GridSynthError::UndefinedAngleError(_)
        ));
    }

    #[test]
    #[cfg_attr(miri, ignore)]
    fn gridsynth_measurement_based_phase_correction() {
        let mut hugr = load_guppy_hugr("measurement_based_phase_correction");
        assert_eq!(count_gate(&hugr, TketOp::Rz), 2);
        GridSynthPass::default().run(&mut hugr).unwrap();
        hugr.validate().unwrap();
        assert_eq!(count_gate(&hugr, TketOp::Rz), 0);
        assert_eq!(count_gate(&hugr, TketOp::S), 2);
        assert_eq!(count_gate(&hugr, TketOp::Z), 1);
    }
}
