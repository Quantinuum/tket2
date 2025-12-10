//! Quantum circuit representation and operations.

pub mod command;
pub mod cost;
mod extract_dfg;
mod hash;
pub mod units;

use std::borrow::Cow;
use std::collections::HashSet;
use std::iter::Sum;

pub use command::{Command, CommandIterator};
pub use hash::CircuitHash;
use hugr::extension::prelude::{NoopDef, TupleOpDef};
use hugr::extension::simple_op::MakeOpDef;
use hugr::hugr::views::sibling_subgraph::InvalidSubgraph;
use hugr::hugr::views::{ExtractionResult, SiblingSubgraph};
use hugr::ops::handle::DataflowParentID;
use itertools::Either::{Left, Right};

use derive_more::{Display, Error, From};
use hugr::hugr::hugrmut::HugrMut;
use hugr::ops::dataflow::IOTrait;
use hugr::ops::{Input, OpName, OpParent, OpTag, OpTrait, Output};
use hugr::types::{PolyFuncType, Signature};
use hugr::{Hugr, PortIndex};
use hugr::{HugrView, OutgoingPort};
use itertools::Itertools;
use lazy_static::lazy_static;

pub use hugr::ops::OpType;
pub use hugr::types::{EdgeKind, Type, TypeRow};
pub use hugr::{Node, Port, Wire};
use smol_str::ToSmolStr;

use self::units::{filter, LinearUnit, Units};

/// A quantum circuit, represented as a function in a HUGR.
#[derive(Debug, Clone)]
pub struct Circuit<T: HugrView = Hugr> {
    /// The HUGR containing the circuit.
    hugr: T,
}

impl<T: Default + HugrView> Default for Circuit<T> {
    fn default() -> Self {
        let hugr = T::default();
        Self { hugr }
    }
}

impl<T: HugrView<Node = Node>> PartialEq for Circuit<T> {
    fn eq(&self, other: &Self) -> bool {
        match (
            self.circuit_hash(self.parent()),
            other.circuit_hash(other.parent()),
        ) {
            (Ok(hash1), Ok(hash2)) => hash1 == hash2,
            _ => false,
        }
    }
}

lazy_static! {
    /// Most [`Optype::ExtensionOp`]s are counted as operations in the circuit, except for
    /// some special ones like tuple pack/unpack and the Noop operation.
    ///
    /// We have to insert the extension id manually due to
    /// https://github.com/quantinuum/hugr/issues/1496
    static ref IGNORED_EXTENSION_OPS: HashSet<OpName> = {
        let mut set = HashSet::new();
        set.insert(format!("prelude.{}", NoopDef.opdef_id()).into());
        set.insert(format!("prelude.{}", TupleOpDef::MakeTuple.opdef_id()).into());
        set.insert(format!("prelude.{}", TupleOpDef::UnpackTuple.opdef_id()).into());
        set
    };
}
/// The [IGNORED_EXTENSION_OPS] definition depends on the buggy behaviour of [`NamedOp::name`], which returns bare names instead of scoped names on some cases.
/// Once this test starts failing it should be time to drop the `format!("prelude.{}", ...)`.
/// https://github.com/quantinuum/hugr/issues/1496
#[test]
fn issue_1496_remains() {
    assert_eq!("Noop", NoopDef.opdef_id())
}

impl<T: HugrView> Circuit<T> {
    /// Create a new circuit from a HUGR and its entrypoint.
    ///
    /// # Errors
    ///
    /// Returns an error if the HUGR entrypoint is not a DFG node.
    pub fn try_new(hugr: T) -> Result<Self, CircuitError<T::Node>> {
        check_hugr(&hugr)?;
        Ok(Self { hugr })
    }

    /// Create a new circuit from a HUGR and its entrypoint.
    ///
    /// See [`Circuit::try_new`] for a version that returns an error.
    ///
    /// # Panics
    ///
    /// Panics if the HUGR entrypoint is not a DFG node.
    pub fn new(hugr: T) -> Self {
        Self::try_new(hugr).unwrap_or_else(|e| panic!("{}", e))
    }

    /// Returns the node containing the circuit definition.
    pub fn parent(&self) -> T::Node {
        self.hugr.entrypoint()
    }

    /// Get a reference to the HUGR containing the circuit.
    pub fn hugr(&self) -> &T {
        &self.hugr
    }

    /// Unwrap the HUGR containing the circuit.
    pub fn into_hugr(self) -> T {
        self.hugr
    }

    /// Get a mutable reference to the HUGR containing the circuit.
    ///
    /// Mutation of the hugr MUST NOT invalidate the parent node,
    /// by changing the node's type to a non-DFG node or by removing it.
    pub fn hugr_mut(&mut self) -> &mut T {
        &mut self.hugr
    }

    /// Return the name of the circuit
    ///
    /// If the circuit is a function definition, returns the name of the
    /// function.
    ///
    /// If the name is empty or the circuit is not a function definition, returns
    /// `None`.
    #[inline]
    pub fn name(&self) -> Option<&str> {
        let op = self.hugr.get_optype(self.parent());
        let name = match op {
            OpType::FuncDefn(defn) => &defn.func_name(),
            _ => return None,
        };
        match name.as_str() {
            "" => None,
            name => Some(name),
        }
    }

    /// Returns the function type of the circuit.
    #[inline]
    pub fn circuit_signature(&self) -> Cow<'_, Signature> {
        let op = self.hugr.get_optype(self.parent());
        op.inner_function_type()
            .unwrap_or_else(|| panic!("{op} is an invalid circuit parent type."))
    }

    /// Returns the input node to the circuit.
    #[inline]
    pub fn input_node(&self) -> T::Node {
        self.hugr
            .get_io(self.parent())
            .expect("Circuit has no input node")[0]
    }

    /// Returns the output node to the circuit.
    #[inline]
    pub fn output_node(&self) -> T::Node {
        self.hugr
            .get_io(self.parent())
            .expect("Circuit has no output node")[1]
    }

    /// Returns the input and output nodes of the circuit.
    #[inline]
    pub fn io_nodes(&self) -> [T::Node; 2] {
        self.hugr
            .get_io(self.parent())
            .expect("Circuit has no I/O nodes")
    }

    /// The number of operations in the circuit.
    ///
    /// This includes [`TketOp`]s, pytket ops, and any other custom operations.
    ///
    /// Nested circuits are traversed to count their operations.
    ///
    ///   [`TketOp`]: crate::TketOp
    #[inline]
    pub fn num_operations(&self) -> usize
    where
        Self: Sized,
    {
        let mut count = 0;
        let mut roots = vec![self.hugr().module_root()];
        while let Some(node) = roots.pop() {
            for child in self.hugr().children(node) {
                let optype = self.hugr().get_optype(child);
                if matches!(optype, OpType::ExtensionOp(_) | OpType::OpaqueOp(_))
                    && !IGNORED_EXTENSION_OPS.contains(&optype.to_smolstr())
                {
                    count += 1;
                } else if self.hugr().first_child(node).is_some() {
                    roots.push(child);
                }
            }
        }
        count
    }

    /// Count the number of qubits in the circuit.
    #[inline]
    pub fn qubit_count(&self) -> usize
    where
        Self: Sized,
    {
        self.qubits().count()
    }

    /// Get the input units of the circuit and their types.
    #[inline]
    pub fn units(&self) -> Units<OutgoingPort, T::Node>
    where
        Self: Sized,
    {
        Units::new_circ_input(self)
    }

    /// Get the linear input units of the circuit and their types.
    #[inline]
    pub fn linear_units(&self) -> impl Iterator<Item = (LinearUnit, OutgoingPort, Type)> + '_
    where
        Self: Sized,
    {
        self.units().filter_map(filter::filter_linear)
    }

    /// Get the non-linear input units of the circuit and their types.
    #[inline]
    pub fn nonlinear_units(&self) -> impl Iterator<Item = (Wire<T::Node>, OutgoingPort, Type)> + '_
    where
        Self: Sized,
    {
        self.units().filter_map(filter::filter_non_linear)
    }

    /// Returns the units corresponding to qubits inputs to the circuit.
    #[inline]
    pub fn qubits(&self) -> impl Iterator<Item = (LinearUnit, OutgoingPort, Type)> + '_
    where
        Self: Sized,
    {
        self.units().filter_map(filter::filter_qubit)
    }

    /// Compute the cost of a group of nodes in a circuit based on a
    /// per-operation cost function.
    #[inline]
    pub fn nodes_cost<F, C>(&self, nodes: impl IntoIterator<Item = T::Node>, op_cost: F) -> C
    where
        C: Sum,
        F: Fn(&OpType) -> C,
    {
        nodes
            .into_iter()
            .map(|n| op_cost(self.hugr.get_optype(n)))
            .sum()
    }

    /// Return the graphviz representation of the underlying graph and hierarchy side by side.
    ///
    /// For a simpler representation, use the [`Circuit::mermaid_string`] format instead.
    pub fn dot_string(&self) -> String {
        // TODO: This will print the whole HUGR without identifying the circuit container.
        // Should we add some extra formatting for that?
        self.hugr.dot_string()
    }

    /// Return the mermaid representation of the underlying hierarchical graph.
    ///
    /// The hierarchy is represented using subgraphs. Edges are labelled with
    /// their source and target ports.
    ///
    /// For a more detailed representation, use the [`Circuit::dot_string`]
    /// format instead.
    pub fn mermaid_string(&self) -> String {
        // TODO: See comment in `dot_string`.
        self.hugr.mermaid_string()
    }
}

impl<T: HugrView> Circuit<T> {
    /// Ensures the circuit contains an owned HUGR.
    pub fn to_owned(&self) -> Circuit<Hugr> {
        let (mut hugr, map) = self.hugr.extract_hugr(self.hugr.module_root());
        let parent = map.extracted_node(self.parent());
        hugr.set_entrypoint(parent);
        Circuit { hugr }
    }
}

impl<T: HugrView<Node = Node>> Circuit<T> {
    /// Returns all the commands in the circuit, in some topological order.
    ///
    /// Ignores the Input and Output nodes.
    #[inline]
    pub fn commands(&self) -> CommandIterator<'_, T>
    where
        Self: Sized,
    {
        // Traverse the circuit in topological order.
        CommandIterator::new(self)
    }

    /// Returns the top-level operations in the circuit, in some topological
    /// order.
    ///
    /// This is a subset of the commands returned by [`Circuit::commands`], only
    /// including [`TketOp`]s, pytket ops, and any other custom operations.
    ///
    ///   [`TketOp`]: crate::TketOp
    #[inline]
    pub fn operations(&self) -> impl Iterator<Item = Command<'_, T>> + '_
    where
        Self: Sized,
    {
        // Traverse the circuit in topological order.
        self.commands().filter(|cmd| {
            cmd.optype().is_extension_op()
                && !IGNORED_EXTENSION_OPS.contains(&cmd.optype().to_smolstr())
        })
    }

    /// Extracts the circuit into a new owned HUGR containing the circuit at the root.
    /// Replaces the circuit container operation with an [`OpType::DFG`].
    ///
    /// Regions that are not descendants of the parent node are not included in the new HUGR.
    /// This may invalidate calls to functions defined elsewhere. Make sure to inline any
    /// external functions before calling this method.
    pub fn extract_dfg(&self) -> Result<Circuit<Hugr>, CircuitMutError> {
        let circ = self.to_owned();
        // TODO: Can we just ignore this now?
        //extract_dfg::rewrite_into_dfg(&mut circ)?;
        Ok(circ)
    }

    /// Construct the subgraph containing the entire circuit.
    pub fn subgraph(&self) -> Result<SiblingSubgraph, InvalidSubgraph>
    where
        T: Clone,
    {
        SiblingSubgraph::try_new_dataflow_subgraph::<_, DataflowParentID>(self.hugr())
    }

    /// Compute the cost of the circuit based on a per-operation cost function.
    #[inline]
    pub fn circuit_cost<F, C>(&self, op_cost: F) -> C
    where
        Self: Sized,
        C: Sum,
        F: Fn(&OpType) -> C,
    {
        self.commands().map(|cmd| op_cost(cmd.optype())).sum()
    }
}

impl<T: HugrView> From<T> for Circuit<T> {
    fn from(hugr: T) -> Self {
        Self::new(hugr)
    }
}

/// Checks if the passed hugr is a valid circuit,
/// and return [`CircuitError`] if not.
pub fn check_hugr<H: HugrView>(hugr: &H) -> Result<(), CircuitError<H::Node>> {
    let optype = hugr.entrypoint_optype();
    match optype {
        // Dataflow nodes are always valid.
        OpType::DFG(_) => Ok(()),
        // Function definitions are also valid, as long as they have a concrete signature.
        OpType::FuncDefn(defn) => match defn.signature().params().is_empty() {
            true => Ok(()),
            false => Err(CircuitError::ParametricSignature {
                parent: hugr.entrypoint(),
                optype: optype.clone(),
                signature: defn.signature().clone(),
            }),
        },
        OpType::DataflowBlock(_) => Ok(()),
        OpType::Case(_) => Ok(()),
        OpType::TailLoop(_) => Ok(()),
        _ => {
            debug_assert_eq!(None, optype.tag().partial_cmp(&OpTag::DataflowParent),);
            Err(CircuitError::InvalidParentOp {
                parent: hugr.entrypoint(),
                optype: optype.clone(),
            })
        }
    }
}

/// Remove an empty wire in a dataflow HUGR.
///
/// The wire to be removed is identified by the index of the outgoing port
/// at the circuit input node.
///
/// This will change the circuit signature and will shift all ports after
/// the removed wire by -1. If the wire is connected to the output node,
/// this will also change the signature output and shift the ports after
/// the removed wire by -1.
///
/// This will return an error if the wire is not empty or if a HugrError
/// occurs.
#[allow(dead_code)]
pub(crate) fn remove_empty_wire(
    circ: &mut Circuit<impl HugrMut<Node = Node>>,
    input_port: usize,
) -> Result<(), CircuitMutError> {
    let parent = circ.parent();
    let hugr = circ.hugr_mut();

    let [inp, out] = hugr.get_io(parent).expect("no IO nodes found at parent");
    if input_port >= hugr.num_outputs(inp) {
        return Err(CircuitMutError::InvalidPortOffset {
            input_port,
            dataflow_node: parent,
        });
    }
    let input_port = OutgoingPort::from(input_port);
    let link = hugr
        .linked_inputs(inp, input_port)
        .at_most_one()
        .map_err(|_| CircuitMutError::DeleteNonEmptyWire {
            input_port: input_port.index(),
            dataflow_node: parent,
        })?;
    if link.is_some() && link.unwrap().0 != out {
        return Err(CircuitMutError::DeleteNonEmptyWire {
            input_port: input_port.index(),
            dataflow_node: parent,
        });
    }
    if link.is_some() {
        hugr.disconnect(inp, input_port);
    }

    // Shift ports at input
    shift_ports(hugr, inp, input_port, hugr.num_outputs(inp))?;
    // Shift ports at output
    if let Some((out, output_port)) = link {
        shift_ports(hugr, out, output_port, hugr.num_inputs(out))?;
    }
    // Update input node, output node (if necessary) and parent signatures.
    update_signature(
        hugr,
        parent,
        input_port.index(),
        link.map(|(_, p)| p.index()),
    )?;
    // Resize ports at input/output node
    hugr.set_num_ports(inp, 0, hugr.num_outputs(inp) - 1);
    if let Some((out, _)) = link {
        hugr.set_num_ports(out, hugr.num_inputs(out) - 1, 0);
    }
    Ok(())
}

/// Errors that can occur when mutating a circuit.
#[derive(Display, Debug, Clone, Error, PartialEq)]
#[non_exhaustive]
pub enum CircuitError<N = Node> {
    /// The parent node for the circuit does not exist in the HUGR.
    #[display("{parent} cannot define a circuit as it is not present in the HUGR.")]
    MissingParentNode {
        /// The node that was used as the parent.
        parent: N,
    },
    /// Circuit parents must have a concrete signature.
    #[display(
        "{parent} with op {} cannot be used as a circuit parent. Circuits must have a concrete signature, but the node has signature '{}'.",
        optype,
        signature
    )]
    ParametricSignature {
        /// The node that was used as the parent.
        parent: N,
        /// The parent optype.
        optype: OpType,
        /// The parent signature.
        signature: PolyFuncType,
    },
    /// The parent node for the circuit has an invalid optype.
    #[display(
        "{parent} with op {} cannot be used as a circuit parent. Only 'DFG', 'DataflowBlock', or 'FuncDefn' operations are allowed.",
        optype
    )]
    InvalidParentOp {
        /// The node that was used as the parent.
        parent: N,
        /// The parent optype.
        optype: OpType,
    },
}

/// Errors that can occur when mutating a circuit.
#[derive(Display, Debug, Clone, Error, PartialEq, From)]
#[non_exhaustive]
pub enum CircuitMutError {
    /// A Hugr error occurred.
    #[from]
    HugrError(hugr::hugr::HugrError),
    /// A circuit validation error occurred.
    #[from]
    CircuitError(CircuitError),
    /// The wire to be deleted is not empty.
    #[display("Tried to delete non-empty input wire with offset {input_port} on dataflow node {dataflow_node}")]
    DeleteNonEmptyWire {
        /// The input port offset
        input_port: usize,
        /// The port's node
        dataflow_node: Node,
    },
    /// Invalid dataflow input offset
    #[display("Dataflow node {dataflow_node} does not have an input with offset {input_port}")]
    InvalidPortOffset {
        /// The input port offset
        input_port: usize,
        /// The port's node
        dataflow_node: Node,
    },
}

/// Shift ports in range (free_port + 1 .. max_ind) by -1.
fn shift_ports<C: HugrMut + ?Sized>(
    circ: &mut C,
    node: C::Node,
    free_port: impl Into<Port>,
    max_ind: usize,
) -> Result<Port, hugr::hugr::HugrError> {
    let mut free_port: Port = free_port.into();
    let dir = free_port.direction();
    let port_range = (free_port.index() + 1..max_ind).map(|p| Port::new(dir, p));
    for port in port_range {
        let links = circ.linked_ports(node, port).collect_vec();
        if !links.is_empty() {
            circ.disconnect(node, port);
        }
        for (other_n, other_p) in links {
            match other_p.as_directed() {
                Right(other_p) => {
                    let dst_port = free_port.as_incoming().unwrap();
                    circ.connect(other_n, other_p, node, dst_port)
                }
                Left(other_p) => {
                    let src_port = free_port.as_outgoing().unwrap();
                    circ.connect(node, src_port, other_n, other_p)
                }
            };
        }
        free_port = port;
    }
    Ok(free_port)
}

// Update the signature of circ when removing the in_index-th input wire and
// the out_index-th output wire.
fn update_signature(
    hugr: &mut impl HugrMut<Node = Node>,
    parent: Node,
    in_index: usize,
    out_index: Option<usize>,
) -> Result<(), CircuitMutError> {
    let inp = hugr
        .get_io(parent)
        .expect("no IO nodes found at circuit parent")[0];
    // Update input node
    let inp_types: TypeRow = {
        let OpType::Input(Input { types }) = hugr.get_optype(inp).clone() else {
            panic!("invalid circuit")
        };
        let mut types = types.into_owned();
        types.remove(in_index);
        types.into()
    };
    hugr.replace_op(inp, Input::new(inp_types.clone()));

    // Update output node if necessary.
    let out_types = out_index.map(|out_index| {
        let out = hugr.get_io(parent).unwrap()[1];
        let out_types: TypeRow = {
            let OpType::Output(Output { types }) = hugr.get_optype(out).clone() else {
                panic!("invalid circuit")
            };
            let mut types = types.into_owned();
            types.remove(out_index);
            types.into()
        };
        hugr.replace_op(out, Output::new(out_types.clone()));
        out_types
    });

    // Update the parent's signature
    let mut optype = hugr.get_optype(parent).clone();

    // Replace the parent node operation with the right operation type
    // This must be able to process all implementers of `DataflowParent`.
    match &mut optype {
        OpType::DFG(dfg) => {
            dfg.signature.input = inp_types;
            if let Some(out_types) = out_types {
                dfg.signature.output = out_types;
            }
        }
        OpType::FuncDefn(defn) => {
            let mut sig: Signature = defn.signature().clone().try_into().map_err(|_| {
                CircuitError::ParametricSignature {
                    parent,
                    optype: OpType::FuncDefn(defn.clone()),
                    signature: defn.signature().clone(),
                }
            })?;
            sig.input = inp_types;
            if let Some(out_types) = out_types {
                sig.output = out_types;
            }
            *defn.signature_mut() = sig.into();
        }
        OpType::DataflowBlock(block) => {
            block.inputs = inp_types;
            if out_types.is_some() {
                unimplemented!("DataflowBlock output signature update")
            }
        }
        OpType::Case(case) => {
            let out_types = out_types.unwrap_or_else(|| case.signature.output().clone());
            case.signature = Signature::new(inp_types, out_types)
        }
        OpType::TailLoop(_) => {
            unimplemented!("TailLoop signature update")
        }
        _ => Err(CircuitError::InvalidParentOp {
            parent,
            optype: optype.clone(),
        })?,
    }

    hugr.replace_op(parent, optype);

    Ok(())
}

#[cfg(test)]
mod tests {
    use cool_asserts::assert_matches;
    use hugr::CircuitUnit;
    use rstest::{fixture, rstest};

    use hugr::types::Signature;
    use hugr::{
        builder::{DFGBuilder, DataflowHugr},
        extension::prelude::bool_t,
    };

    use super::*;
    use crate::extension::rotation::ConstRotation;
    use crate::serialize::load_tk1_json_str;
    use crate::serialize::pytket::DecodeOptions;
    use crate::utils::build_simple_circuit;
    use crate::TketOp;

    #[fixture]
    fn tk1_circuit() -> Circuit {
        load_tk1_json_str(
            r#"{
            "name": "MyCirc",
            "phase": "0",
            "bits": [["c", [0]]],
            "qubits": [["q", [0]], ["q", [1]]],
            "commands": [
                {"args": [["q", [0]]], "op": {"type": "H"}},
                {"args": [["q", [0]], ["q", [1]]], "op": {"type": "CX"}},
                {"args": [["q", [1]]], "op": {"params": ["0.25"], "type": "Rz"}}
            ],
            "implicit_permutation": [[["q", [0]], ["q", [0]]], [["q", [1]], ["q", [1]]]]
        }"#,
            DecodeOptions::new(),
        )
        .unwrap()
    }

    /// 2-qubit circuit with a Hadamard, a CNOT, and an Rz gate.
    #[fixture]
    fn simple_circuit() -> Circuit {
        build_simple_circuit(2, |circ| {
            circ.append(TketOp::H, [0])?;
            circ.append(TketOp::CX, [0, 1])?;
            let angle = circ.add_constant(ConstRotation::PI_2);
            circ.append_and_consume(
                TketOp::Rz,
                [CircuitUnit::Linear(1), CircuitUnit::Wire(angle)],
            )?;
            Ok(())
        })
        .unwrap()
    }

    /// 2-qubit circuit with a Hadamard, a CNOT, and a X gate,
    /// defined inside a module.
    #[fixture]
    fn simple_module() -> Circuit {
        build_simple_circuit(2, |circ| {
            circ.append(TketOp::H, [0])?;
            circ.append(TketOp::CX, [0, 1])?;
            circ.append(TketOp::X, [1])?;
            Ok(())
        })
        .unwrap()
    }

    #[rstest]
    #[case::simple(simple_circuit(), 2, 0, Some("main"))]
    #[case::module(simple_module(), 2, 0, Some("main"))]
    #[case::tk1(tk1_circuit(), 2, 1, Some("MyCirc"))]
    fn test_circuit_properties(
        #[case] circ: Circuit,
        #[case] qubits: usize,
        #[case] bits: usize,
        #[case] name: Option<&str>,
    ) {
        assert_eq!(circ.name(), name);
        assert_eq!(circ.circuit_signature().input_count(), qubits + bits);
        assert_eq!(circ.circuit_signature().output_count(), qubits + bits);
        assert_eq!(circ.qubit_count(), qubits);

        assert_eq!(circ.units().count(), qubits + bits);
        assert_eq!(circ.nonlinear_units().count(), bits);
        assert_eq!(circ.linear_units().count(), qubits);
        assert_eq!(circ.qubits().count(), qubits);
    }

    #[test]
    fn remove_qubit() {
        let mut circ = build_simple_circuit(2, |circ| {
            circ.append(TketOp::X, [0])?;
            Ok(())
        })
        .unwrap();

        let orig_circ = circ.clone();
        assert_eq!(circ, orig_circ);

        assert_eq!(circ.qubit_count(), 2);
        assert!(remove_empty_wire(&mut circ, 1).is_ok());
        assert_eq!(circ.qubit_count(), 1);
        assert_ne!(circ, orig_circ);

        assert_eq!(
            remove_empty_wire(&mut circ, 0).unwrap_err(),
            CircuitMutError::DeleteNonEmptyWire {
                input_port: 0,
                dataflow_node: circ.parent()
            }
        );
    }

    #[test]
    fn test_invalid_parent() {
        let hugr = Hugr::default();

        assert_matches!(
            Circuit::try_new(hugr.clone()),
            Err(CircuitError::InvalidParentOp { .. }),
        );
    }

    #[test]
    fn remove_bit() {
        let h = DFGBuilder::new(Signature::new(vec![bool_t()], vec![])).unwrap();
        let mut circ: Circuit = h.finish_hugr_with_outputs([]).unwrap().into();

        assert_eq!(circ.units().count(), 1);
        assert!(remove_empty_wire(&mut circ, 0).is_ok());
        assert_eq!(circ.units().count(), 0);
        assert_eq!(
            remove_empty_wire(&mut circ, 2).unwrap_err(),
            CircuitMutError::InvalidPortOffset {
                input_port: 2,
                dataflow_node: circ.parent()
            }
        );
    }
}
