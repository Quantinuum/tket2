//! Resource flow logic for tracking how resources move through operations.
//!
//! This module defines the [`ResourceFlow`] trait that specifies how resources
//! flow through operations, along with a default implementation.

use crate::resource::types::ResourceId;
use derive_more::derive::{Display, Error};
use hugr::HugrView;
use hugr::ops::{OpTrait, OpType};
use hugr::types::Type;
use itertools::{EitherOrBoth, Itertools};

/// Error type for unsupported operations in ResourceFlow implementations.
#[derive(Debug, Display, Clone, PartialEq, Error)]
#[display("Unsupported operation: {_0}")]
pub struct UnsupportedOp(#[error(not(source))] OpType);

/// Trait for specifying how resources flow through operations.
///
/// This trait allows different implementations to define how linear resources
/// are mapped from inputs to outputs through various operation types.
///
/// This trait is dyn-compatible.
pub trait ResourceFlow<H: HugrView> {
    /// Map resource IDs from operation inputs to outputs.
    ///
    /// Given a `node`, the `hugr` in which it is defined and `inputs`, a list
    /// of the linear inputs to `node` given as resources, return the list
    /// of resources for the linear outputs of `node`.
    ///
    /// More specifically, `inputs` is a vector of the same length as the number
    /// of inputs in the signature of `node` and such that
    ///  - `inputs[i]` is Some(resource_id) if the i-th input port is a linear
    ///    type, and
    ///  - `inputs[i]` is `None` otherwise, i.e. if the i-th input port is a
    ///    copyable type.
    ///
    /// The returned vector `outputs` must satisfy
    ///  - if `outputs[i]` is `Some(resource_id)`, then the i-th output port is
    ///    a linear type and `resource_id` is an ID passed in `inputs`,
    ///  - if the i-th output port is a copyable type, then `outputs[i]` is
    ///    `None`.
    ///
    /// If the i-th output port is linear and `outputs[i]` is set to None, then
    /// a fresh resource ID will be created and assigned to that port.
    ///
    /// # Arguments
    /// * `node` - The operation node
    /// * `hugr` - The `hugr` in which `node` is defined
    /// * `inputs` - Resource IDs for each input port (None for non-linear
    ///   types)
    fn map_resources(
        &self,
        node: H::Node,
        hugr: &H,
        inputs: &[Option<ResourceId>],
    ) -> Result<Vec<Option<ResourceId>>, UnsupportedOp>;

    /// Convert this ResourceFlow into a boxed trait object.
    fn into_boxed<'a>(self) -> Box<dyn 'a + ResourceFlow<H>>
    where
        Self: 'a + Sized,
    {
        Box::new(self)
    }
}

impl<H: HugrView> ResourceFlow<H> for Box<dyn '_ + ResourceFlow<H>> {
    fn map_resources(
        &self,
        node: H::Node,
        hugr: &H,
        inputs: &[Option<ResourceId>],
    ) -> Result<Vec<Option<ResourceId>>, UnsupportedOp> {
        self.as_ref().map_resources(node, hugr, inputs)
    }
}

/// Default implementation of ResourceFlow.
///
/// This implementation considers that an operation is resource-preserving if
/// for all port indices i, either
///  - the i-th input and i-th output are both linear and are of the same type,
///  - or the i-th input and i-th output are both copyable,
///  - or one of the i-th input/output is copyable and the other does not exist.
///
/// For resource-preserving operations, linear inputs are then mapped to the
/// corresponding output. (All outputs with no corresponding input, must be
/// copyable.)
///
/// If on the other hand an operation is not resource preserving, all input
/// resources are discarded and all outputs will be given fresh resource IDs.
#[derive(Debug, Clone, Default)]
pub struct DefaultResourceFlow;

impl DefaultResourceFlow {
    /// Determine if an operation is resource-preserving based on input/output
    /// types.
    fn is_resource_preserving(input_types: &[Type], output_types: &[Type]) -> bool {
        // An operation is resource-preserving if for each i, if input[i] or
        // output[i] is linear, then type(input[i]) == type(output[i])

        for io_ty in input_types.iter().zip_longest(output_types.iter()) {
            let (input_ty, output_ty) = match io_ty {
                EitherOrBoth::Both(input_ty, output_ty) => (input_ty, output_ty),
                EitherOrBoth::Left(ty) | EitherOrBoth::Right(ty) => {
                    if !ty.copyable() {
                        // linear type on one side, nothing on the other
                        return false;
                    }
                    continue;
                }
            };

            if !input_ty.copyable() || !output_ty.copyable() {
                // If input/output is linear, both must be the same type
                if input_ty != output_ty {
                    return false;
                }
            }
        }

        true
    }
}

impl<H: HugrView> ResourceFlow<H> for DefaultResourceFlow {
    fn map_resources(
        &self,
        node: H::Node,
        hugr: &H,
        inputs: &[Option<ResourceId>],
    ) -> Result<Vec<Option<ResourceId>>, UnsupportedOp> {
        let op = hugr.get_optype(node);
        let signature = op.dataflow_signature().expect("dataflow op");
        let input_types = signature.input_types();
        let output_types = signature.output_types();

        debug_assert_eq!(
            inputs.len(),
            input_types.len(),
            "Input resource array length must match operation input count"
        );

        if Self::is_resource_preserving(input_types, output_types) {
            Ok(retain_linear_types(inputs.to_vec(), output_types))
        } else {
            // Not resource-preserving: all linear outputs are new resources (None)
            Ok(vec![None; output_types.len()])
        }
    }
}

fn retain_linear_types(
    mut resources: Vec<Option<ResourceId>>,
    types: &[Type],
) -> Vec<Option<ResourceId>> {
    resources.resize(types.len(), None);
    for (ty, resource) in types.iter().zip(resources.iter_mut()) {
        if ty.copyable() {
            *resource = None;
        }
    }
    resources
}
