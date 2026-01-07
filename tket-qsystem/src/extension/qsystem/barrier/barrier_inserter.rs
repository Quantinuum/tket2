use hugr::builder::{Container, DataflowHugr};
use hugr::extension::prelude::qb_t;
use hugr::ops::OpTrait;
use hugr::std_extensions::collections::array::ArrayKind;
use hugr::types::{Signature, Type};
use hugr::{
    Hugr, HugrView, IncomingPort, Node, OutgoingPort, Wire,
    algorithms::replace_types::ReplaceTypes,
    builder::{DFGBuilder, Dataflow},
    extension::prelude::Barrier,
    hugr::{
        hugrmut::HugrMut,
        patch::{PatchHugrMut, insert_cut::InsertCut},
    },
};
use tket::passes::unpack_container::type_unpack::{TypeUnpacker, is_array_of};

use crate::extension::qsystem::{
    LowerTk2Error,
    barrier::wrapped_barrier::{WrappedBarrierBuilder, build_runtime_barrier_op},
};
use tket::passes::unpack_container::UnpackContainerBuilder;

type Target = (Node, IncomingPort);

/// Check if a type is specifically an array of qubits
fn is_qubit_array<AT: ArrayKind>(ty: &Type) -> Option<u64> {
    is_array_of::<AT>(ty, &qb_t())
}

/// Responsible for inserting runtime barriers into the HUGR
pub struct BarrierInserter {
    /// Factory for creating barrier operations
    barrier_builder: WrappedBarrierBuilder,
    /// Container operation factory for generic unpacking/repacking
    container_unpacker: UnpackContainerBuilder,
}

impl BarrierInserter {
    /// Create a new BarrierInserter instance
    pub fn new() -> Self {
        Self {
            barrier_builder: WrappedBarrierBuilder::new(),
            container_unpacker: UnpackContainerBuilder::new(TypeUnpacker::for_qubits()),
        }
    }

    /// Filter out types in the generic barrier that contain qubits
    fn filter_qubit_containers<H: HugrMut<Node = Node>>(
        &mut self,
        hugr: &H,
        barrier: &Barrier,
        node: Node,
    ) -> Vec<(Type, Target)> {
        barrier
            .type_row
            .iter()
            .enumerate()
            .filter(|(_, ty)| {
                self.container_unpacker
                    .type_analyzer()
                    .contains_element_type(ty)
            })
            .map(|(i, ty)| {
                let port = OutgoingPort::from(i);
                let target = hugr
                    .single_linked_input(node, port)
                    .expect("linearity violation.");
                (ty.clone(), target)
            })
            .collect()
    }

    /// Try to apply a simplified barrier for the special case of a single array of qubits
    fn try_array_barrier_shortcut(
        &mut self,
        hugr: &mut impl HugrMut<Node = Node>,
        parent: Node,
        ty: &Type,
        target: Target,
    ) -> Option<Result<(), LowerTk2Error>> {
        // Check if this is an array of qubits
        let size = is_qubit_array::<hugr::std_extensions::collections::array::Array>(ty)?;

        // TODO if other array type, convert

        // Build and insert the barrier
        Some(match build_runtime_barrier_op(size) {
            Ok(barr_hugr) => {
                let insert = InsertCut::new(parent, vec![target], barr_hugr);
                insert.apply_hugr_mut(hugr).map(|_| ()).map_err(Into::into)
            }
            Err(err) => Err(err.into()),
        })
    }

    /// Construct the endofunction HUGR to unpack types, apply the runtime barrier across qubits, and repack
    fn build_packing_hugr(&mut self, container_row: Vec<Type>) -> Result<Hugr, LowerTk2Error> {
        // Create a signature for an endofunction on the container row
        let mut dfg_b = DFGBuilder::new(Signature::new_endo(container_row.clone()))?;

        // Unpack the container row directly into wires
        let inputs = dfg_b.input_wires();
        let unpacked_wires =
            self.container_unpacker
                .unpack_row(&mut dfg_b, &container_row, inputs)?;

        // Tag the qubit wires
        let tagged_wires: Vec<(bool, Wire)> = unpacked_wires
            .into_iter()
            .map(|wire| {
                let node_sig = dfg_b
                    .hugr()
                    .get_optype(wire.node())
                    .dataflow_signature()
                    .unwrap();
                (node_sig.port_type(wire.source()) == Some(&qb_t()), wire)
            })
            .collect();

        // Extract just the qubit wires
        let qubit_wires: Vec<_> = tagged_wires
            .iter()
            .filter(|(is_qb, _)| *is_qb)
            .map(|(_, w)| *w)
            .collect();

        // Call the runtime barrier on all the qubit wires using centralized cache
        let mut barrier_outputs = self
            .barrier_builder
            .build_runtime_barrier(&mut dfg_b, qubit_wires)?;

        // Replace the qubit wires with the runtime barrier outputs
        let repack_wires = tagged_wires.into_iter().map(|(is_qb, w)| {
            if is_qb {
                barrier_outputs
                    .next()
                    .expect("Not enough runtime barrier outputs.")
            } else {
                w
            }
        });

        // Repack the wires directly into the container row
        let repacked_container_wires =
            self.container_unpacker
                .repack_row(&mut dfg_b, &container_row, repack_wires)?;

        let h = dfg_b.finish_hugr_with_outputs(repacked_container_wires)?;
        Ok(h)
    }

    /// Insert a runtime barrier after a Barrier in the Hugr
    pub fn insert_runtime_barrier(
        &mut self,
        hugr: &mut impl HugrMut<Node = Node>,
        barrier_node: Node,
        barrier: Barrier,
    ) -> Result<(), LowerTk2Error> {
        // Find all qubit containing types in the barrier
        let filtered_qbs = self.filter_qubit_containers(hugr, &barrier, barrier_node);

        if filtered_qbs.is_empty() {
            return Ok(());
        }

        let parent = hugr
            .get_parent(barrier_node)
            .expect("Barrier can't be root.");

        // Handle the special case of a single array of qubits
        if let [(ty, target)] = filtered_qbs.as_slice()
            && let Some(result) = self.try_array_barrier_shortcut(hugr, parent, ty, *target)
        {
            return result;
        }

        // For the general case, build unpacking
        let (types, targets) = filtered_qbs.into_iter().unzip::<_, _, Vec<_>, Vec<_>>();
        let insert_hugr = self.build_packing_hugr(types)?;

        // Insert the barrier into the HUGR
        let inserter = InsertCut::new(parent, targets, insert_hugr);
        inserter.apply_hugr_mut(hugr)?;
        Ok(())
    }

    /// Register function replacements for all temporary operations
    pub fn register_operation_replacements(
        self,
        hugr: &mut impl HugrMut<Node = Node>,
        lowerer: &mut ReplaceTypes,
    ) {
        self.barrier_builder
            .into_function_map()
            .register_operation_replacements(hugr, lowerer);
        self.container_unpacker
            .into_function_map()
            .register_operation_replacements(hugr, lowerer);
    }
}

impl Default for BarrierInserter {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use hugr::{
        builder::BuildError,
        extension::{prelude::bool_t, simple_op::MakeExtensionOp},
        ops::handle::NodeHandle,
        std_extensions::collections::array::array_type,
    };

    use super::*;

    fn create_test_hugr() -> Result<(Hugr, Node), BuildError> {
        // Create a dataflow graph with two qubits as input and output
        let mut builder = DFGBuilder::new(Signature::new_endo(vec![qb_t(), bool_t()]))?;
        let [qb1, qb2] = builder.input_wires_arr();

        // Create a barrier with two qubits
        let barrier_node =
            builder.add_dataflow_op(Barrier::new(vec![qb_t(), bool_t()]), [qb1, qb2])?;

        let outputs = barrier_node.outputs().collect::<Vec<_>>();
        let h = builder.finish_hugr_with_outputs(outputs)?;
        Ok((h, barrier_node.node()))
    }

    #[test]
    fn test_barrier_insertion() -> Result<(), LowerTk2Error> {
        let (mut hugr, barrier_node) = create_test_hugr().unwrap();
        let barrier = hugr.get_optype(barrier_node).as_extension_op().unwrap();

        let barrier = Barrier::from_extension_op(barrier).unwrap();

        let node_count_before = hugr.num_nodes();
        let mut inserter = BarrierInserter::new();
        inserter.insert_runtime_barrier(&mut hugr, barrier_node, barrier)?;

        // Verify the barrier was inserted
        let node_count_after = hugr.num_nodes();
        assert!(
            node_count_after > node_count_before,
            "Should have inserted barrier nodes"
        );

        Ok(())
    }

    #[test]
    fn test_qubit_array_barrier() -> Result<(), LowerTk2Error> {
        // Create a dataflow graph with a qubit array as input and output
        let array_size = 3;
        let array_ty = array_type(array_size, qb_t());
        let mut builder = DFGBuilder::new(Signature::new_endo(vec![array_ty.clone()]))?;

        let input = builder.input();
        let array = input.out_wire(0);

        // Create a barrier with qubit array
        let barrier_node = builder.add_dataflow_op(Barrier::new(vec![array_ty]), [array])?;

        let outputs = barrier_node.outputs().collect::<Vec<_>>();
        let mut hugr = builder.finish_hugr_with_outputs(outputs)?;
        let barrier = hugr
            .get_optype(barrier_node.node())
            .as_extension_op()
            .unwrap();

        let barrier = Barrier::from_extension_op(barrier).unwrap();

        let mut inserter = BarrierInserter::new();
        inserter.insert_runtime_barrier(&mut hugr, barrier_node.node(), barrier)?;

        // The array shortcut should have been used
        assert!(inserter.barrier_builder.into_function_map().is_empty());
        Ok(())
    }

    #[test]
    fn test_empty_barrier() -> Result<(), LowerTk2Error> {
        let mut builder = DFGBuilder::new(Signature::new_endo(vec![]))?;

        // Create a barrier with no qubits
        let barrier_node = builder.add_dataflow_op(Barrier::new(vec![]), [])?;

        let outputs = barrier_node.outputs().collect::<Vec<_>>();
        let mut hugr = builder.finish_hugr_with_outputs(outputs)?;
        let barrier = hugr
            .get_optype(barrier_node.node())
            .as_extension_op()
            .unwrap();

        let barrier = Barrier::from_extension_op(barrier).unwrap();
        let node_count_before = hugr.num_nodes();

        let mut inserter = BarrierInserter::new();
        inserter.insert_runtime_barrier(&mut hugr, barrier_node.node(), barrier)?;

        // Check that no nodes were added
        assert_eq!(
            hugr.num_nodes(),
            node_count_before,
            "No nodes should be added for empty barrier"
        );

        Ok(())
    }

    #[test]
    fn test_build_packing_hugr() -> Result<(), LowerTk2Error> {
        let mut inserter = BarrierInserter::new();

        // Test with mixed types: qubit, bool, qubit array
        let array_size = 2;
        let array_ty = array_type(array_size, qb_t());
        let container_row = vec![qb_t(), bool_t(), array_ty];

        let hugr = inserter.build_packing_hugr(container_row.clone())?;

        // Check the signature matches what we expect
        assert_eq!(
            hugr.entrypoint_optype().dataflow_signature().unwrap(),
            Signature::new_endo(container_row),
            "Packing HUGR should have matching signature"
        );

        // Check that the HUGR is valid
        assert!(hugr.validate().is_ok(), "Generated HUGR should be valid");

        let BarrierInserter {
            barrier_builder: op_factory,
            container_unpacker: container_factory,
        } = inserter;
        assert_eq!(
            op_factory.into_function_map().len() + container_factory.into_function_map().len(),
            3, // runtime barrier + array unpack + array repack
        );

        Ok(())
    }
}
