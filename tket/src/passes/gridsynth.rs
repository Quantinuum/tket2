//! A pass that applies the gridsynth algorithm to synthesise arbitrary ratations to Clifford+T.

use crate::TketOp;
use crate::extension::rotation::ConstRotation;
use crate::passes::{
    ComposablePass, InlineFunctionsPass, Normalize, PassScope, WithScope,
    inline_funcs::InlineFuncsError, normalize::NormalizeErrors,
};

use hugr::{
    HugrView, Node,
    hugr::{ValidationError, hugrmut::HugrMut},
    std_extensions::arithmetic::float_types::ConstF64,
};
use rsgridsynth::config::config_from_theta_epsilon;
use rsgridsynth::gridsynth::gridsynth_gates;

/// Error raised by [GridSynthPass]
#[derive(derive_more::Error, Debug, derive_more::Display, derive_more::From)]
#[non_exhaustive]
pub enum GridSynthError {
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
}

impl Default for GridSynthPass {
    fn default() -> Self {
        Self {
            scope: PassScope::default(),
            epsilon: 1e-3,
        }
    }
}

impl GridSynthPass {
    /// Sets the precision of the gridsynth approximation.
    pub fn with_epsilon(mut self, epsilon: f64) -> Self {
        self.epsilon = epsilon;
        self
    }
}

impl<H: HugrMut<Node = Node> + 'static> ComposablePass<H> for GridSynthPass {
    type Error = GridSynthError;
    type Result = ();

    fn run(&self, hugr: &mut H) -> Result<(), Self::Error> {
        InlineFunctionsPass::default().run(hugr)?;
        Normalize::default().run(hugr)?;

        let rz_nodes: Vec<Node> = hugr
            .nodes()
            .filter(|n| hugr.get_optype(*n).cast::<TketOp>() == Some(TketOp::Rz))
            .collect();

        for rz_node in rz_nodes {
            let angle_port = hugr
                .node_inputs(rz_node)
                .nth(1)
                .expect("Rz should have an angle input");

            let (source_node, _) = match hugr.single_linked_output(rz_node, angle_port) {
                Some(link) => link,
                _ => return Err(GridSynthError::UndefinedAngleError(rz_node)),
            };

            // The Normalize pass should result in all statically known angles appearing
            // directly before the Rz gate. Therefore if the previous node is not
            // `LoadConstant`, the angle cannot be known statically.
            let const_node = if hugr.get_optype(source_node).is_load_constant() {
                match hugr.static_source(source_node) {
                    Some(c) if hugr.get_optype(c).is_const() => c,
                    _ => return Err(GridSynthError::UndefinedAngleError(rz_node)),
                }
            } else {
                return Err(GridSynthError::UndefinedAngleError(rz_node));
            };

            let theta = find_angle(hugr, const_node);
            hugr.remove_node(source_node);

            let const_out = hugr
                .node_outputs(const_node)
                .next()
                .expect("Const has a static output");

            if !hugr.is_linked(const_node, const_out) {
                hugr.remove_node(const_node);
            }

            let gates = gridsynth(theta, self.epsilon);
            replace_rz_with_gates(hugr, rz_node, &gates)?;
        }

        Ok(())
    }
}

/// Extracts the angle (in radians) held by a `Const` node.
fn find_angle<H: HugrView<Node = Node>>(hugr: &H, const_node: Node) -> f64 {
    let value = hugr
        .get_optype(const_node)
        .as_const()
        .expect("node is a Const")
        .value();

    if let Some(rot) = value.get_custom_value::<ConstRotation>() {
        rot.to_radians()
    } else if let Some(fl) = value.get_custom_value::<ConstF64>() {
        ConstRotation::new(fl.value())
            .unwrap_or_else(|_| {
                panic!(
                    "ConstF64 value {:?} for node {const_node} is not a valid rotation",
                    fl.value()
                )
            })
            .to_radians()
    } else {
        panic!("{const_node} has unexpected value type (expected ConstRotation or ConstF64)")
    }
}

/// Runs the gridsynth algorithm on `theta` (radians), and returns the gate string.
fn gridsynth(theta: f64, epsilon: f64) -> String {
    let seed = 1234;
    let verbose = false;
    let up_to_phase = false;
    let mut config = config_from_theta_epsilon(theta, epsilon, seed, verbose, up_to_phase);
    simplify(gridsynth_gates(&mut config).gates)
}

/// Compresses a gridsynth gate sequence into a shorter normal form.
fn simplify(gates: String) -> String {
    let mut gates = gates;
    let n = gates.len();
    let mut normal_form_reached = false;
    while !normal_form_reached {
        let new_gates = gates
            // Cancellation rules
            .replacen("ZZ", "", n)
            .replacen("XX", "", n)
            .replacen("HH", "", n)
            .replacen("SS", "Z", n)
            .replacen("TT", "S", n)
            .replacen("DD", "SZ", n)
            .replacen("TD", "", n)
            .replacen("DT", "", n)
            // Rules to push Paulis to the right
            .replacen("ZS", "SZ", n)
            .replacen("ZT", "TZ", n)
            .replacen("ZD", "DZ", n)
            .replacen("XS", "SZX", n)
            .replacen("XT", "DX", n)
            .replacen("XD", "TX", n)
            .replacen("ZH", "HX", n)
            .replacen("XH", "HZ", n)
            .replacen("XZ", "ZX", n)
            // Interaction of H and S (reduces number of H)
            .replacen("HSH", "SHSX", n)
            // Interaction of S and T (reduces number of S)
            .replacen("DS", "T", n)
            .replacen("SD", "T", n)
            .replacen("TS", "DZ", n)
            .replacen("ST", "DZ", n);
        // Stop when no more changes are possible
        normal_form_reached = new_gates == gates;
        gates = new_gates;
    }
    gates
}

/// Replace an `Rz` node with the Clifford+T gates in `gates`.
fn replace_rz_with_gates<H: HugrMut<Node = Node>>(
    hugr: &mut H,
    rz_node: Node,
    gates: &str,
) -> Result<(), GridSynthError> {
    // W is a global phase factor so we can ignore it
    let gates = gates.replacen('W', "", gates.len());

    let new_nodes: Vec<Node> = gates
        .chars()
        .map(|gate| match gate {
            'H' => TketOp::H,
            'S' => TketOp::S,
            'T' => TketOp::T,
            'D' => TketOp::Tdg,
            'X' => TketOp::X,
            'Z' => TketOp::Z,
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
            "{}/../test_files/guppy_optimization/gridsynth/{}.hugr",
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
