//! Utilities for inserting element extraction and reconstruction functions for
//! container types like array and tuple.

pub mod op_function_map;
pub use op_function_map::OpFunctionMap;
pub mod type_unpack;
pub use type_unpack::TypeUnpacker;

use hugr::{
    Wire,
    builder::{BuildError, Dataflow},
    extension::{
        Extension,
        prelude::{UnpackTuple, UnwrapBuilder, option_type},
    },
    ops::{ExtensionOp, OpName},
    std_extensions::collections::{
        array::{Array, ArrayKind, op_builder::GenericArrayOpBuilder},
        borrow_array::BorrowArray,
    },
    types::{
        FuncValueType, PolyFuncTypeRV, SumType, Type, TypeArg, TypeBound, TypeRV,
        type_param::TypeParam,
    },
};
use std::sync::{Arc, LazyLock};

use type_unpack::{array_args, is_opt_of};

/// Invert the signature of a function type.
fn invert_sig(sig: &PolyFuncTypeRV) -> PolyFuncTypeRV {
    let body = FuncValueType::new(sig.body().output().clone(), sig.body().input().clone());
    PolyFuncTypeRV::new(sig.params(), body)
}

fn generic_array_unpack_sig<AK: ArrayKind>() -> PolyFuncTypeRV {
    PolyFuncTypeRV::new(
        vec![
            TypeParam::max_nat_type(),
            TypeParam::RuntimeType(TypeBound::Linear),
            TypeParam::new_list_type(TypeBound::Linear),
        ],
        FuncValueType::new(
            AK::ty_parametric(
                TypeArg::new_var_use(0, TypeParam::max_nat_type()),
                Type::new_var_use(1, TypeBound::Linear),
            )
            .unwrap(),
            TypeRV::new_row_var_use(2, TypeBound::Linear),
        ),
    )
}

/// Helper function to add array operations for any ArrayKind to the extension
fn add_array_ops<AK: ArrayKind>(
    ext: &mut Extension,
    ext_ref: &std::sync::Weak<Extension>,
    unpack_name: OpName,
    repack_name: OpName,
) -> Result<(), hugr::extension::ExtensionBuildError> {
    let array_unpack_sig = generic_array_unpack_sig::<AK>();
    // pack some wires into an array
    ext.add_op(
        repack_name,
        Default::default(),
        invert_sig(&array_unpack_sig),
        ext_ref,
    )?;
    // unpack an array into some wires
    ext.add_op(unpack_name, Default::default(), array_unpack_sig, ext_ref)?;
    Ok(())
}

/// Temporary extension name for barrier-specific operations.
pub const TEMP_UNPACK_EXT_NAME: hugr::hugr::IdentList =
    hugr::hugr::IdentList::new_static_unchecked("__tket.barrier.temp");

// Temporary operation names.
const UNPACK_OPT: OpName = OpName::new_static("option_unwrap");
const REPACK_OPT: OpName = OpName::new_static("option_tag");
const ARRAY_UNPACK: OpName = OpName::new_static("array_unpack");
const ARRAY_REPACK: OpName = OpName::new_static("array_repack");
const BARRAY_UNPACK: OpName = OpName::new_static("barray_unpack");
const BARRAY_REPACK: OpName = OpName::new_static("barray_repack");
const TUPLE_UNPACK: OpName = OpName::new_static("tuple_unpack");
const TUPLE_REPACK: OpName = OpName::new_static("tuple_repack");

static TEMP_UNPACK_EXT: LazyLock<Arc<Extension>> = LazyLock::new(|| {
    Extension::new_arc(
        TEMP_UNPACK_EXT_NAME,
        hugr::extension::Version::new(0, 0, 0),
        |ext, ext_ref| {
            // Generic option unwrap/tag operations
            let opt_unwrap_sig = PolyFuncTypeRV::new(
                vec![TypeParam::RuntimeType(TypeBound::Linear)],
                FuncValueType::new(
                    hugr::types::TypeRow::from(vec![Type::from(
                        hugr::extension::prelude::option_type(Type::new_var_use(
                            0,
                            TypeBound::Linear,
                        )),
                    )]),
                    hugr::types::TypeRow::from(vec![Type::new_var_use(0, TypeBound::Linear)]),
                ),
            );
            // produce option of element
            ext.add_op(
                REPACK_OPT,
                Default::default(),
                invert_sig(&opt_unwrap_sig),
                ext_ref,
            )
            .unwrap();
            // unwrap option of element
            ext.add_op(UNPACK_OPT, Default::default(), opt_unwrap_sig, ext_ref)
                .unwrap();

            // Add array operations for all ArrayKind types
            add_array_ops::<Array>(ext, ext_ref, ARRAY_UNPACK, ARRAY_REPACK).unwrap();
            add_array_ops::<BorrowArray>(ext, ext_ref, BARRAY_UNPACK, BARRAY_REPACK).unwrap();

            let tuple_unpack_sig = PolyFuncTypeRV::new(
                vec![
                    // incoming tuple row
                    TypeParam::new_list_type(TypeBound::Linear),
                    // unpacked row
                    TypeParam::new_list_type(TypeBound::Linear),
                ],
                FuncValueType::new(
                    Type::new_tuple(TypeRV::new_row_var_use(0, TypeBound::Linear)),
                    TypeRV::new_row_var_use(1, TypeBound::Linear),
                ),
            );
            // pack some wires into a tuple
            ext.add_op(
                TUPLE_REPACK,
                Default::default(),
                invert_sig(&tuple_unpack_sig),
                ext_ref,
            )
            .unwrap();
            // unpack a tuple into some wires
            ext.add_op(TUPLE_UNPACK, Default::default(), tuple_unpack_sig, ext_ref)
                .unwrap();
        },
    )
});

/// Factory for creating and caching container unpack/repack operations.
///
/// This factory provides a generic framework for unpacking and repacking container types
/// such as arrays, tuples, and option types. It uses lazy operation caching to avoid
/// regenerating the same function definitions multiple times.
#[derive(Clone)]
pub struct UnpackContainerBuilder {
    /// Function definitions for each instance of the operations.
    func_map: OpFunctionMap,
    /// Type analyzer for determining which types to unpack
    type_analyzer: TypeUnpacker,
}

impl UnpackContainerBuilder {
    /// Create a new instance with a custom type analyzer.
    pub fn new(type_analyzer: TypeUnpacker) -> Self {
        Self {
            func_map: OpFunctionMap::new(),
            type_analyzer,
        }
    }

    /// Consume and return the internal operation-to-function mapping.
    pub fn into_function_map(self) -> OpFunctionMap {
        self.func_map
    }

    /// Gets a reference to the internal type analyzer
    pub fn type_analyzer(&mut self) -> &mut TypeUnpacker {
        &mut self.type_analyzer
    }

    /// Get an operation from the extension.
    pub fn get_op(&self, name: &OpName, args: impl Into<Vec<TypeArg>>) -> Option<ExtensionOp> {
        ExtensionOp::new(TEMP_UNPACK_EXT.get_op(name)?.clone(), args).ok()
    }

    /// Insert an option unwrap operation for a given element type.
    pub fn unpack_option(
        &self,
        builder: &mut impl Dataflow,
        opt_wire: Wire,
        elem_ty: &Type,
    ) -> Result<Wire, BuildError> {
        let args = [elem_ty.clone().into()];
        let op = self.get_op(&UNPACK_OPT, args.clone()).expect("known op");
        self.func_map
            .insert_with(&op, &[elem_ty.clone().into()], |func_b| {
                let [in_wire] = func_b.input_wires_arr();
                let [out_wire] =
                    func_b.build_expect_sum(1, option_type(elem_ty.clone()), in_wire, |_| {
                        format!("Value of type Option<{elem_ty}> is None so cannot unpack.")
                    })?;
                Ok(vec![out_wire])
            })?;
        Ok(builder
            .add_dataflow_op(op, [opt_wire])?
            .outputs()
            .next()
            .expect("one output"))
    }

    /// Insert an option construction operation for a given element type.
    pub fn repack_option(
        &self,
        builder: &mut impl Dataflow,
        wire: Wire,
        elem_ty: &Type,
    ) -> Result<Wire, BuildError> {
        let args = [elem_ty.clone().into()];
        let op = self.get_op(&REPACK_OPT, args.clone()).expect("known op");
        self.func_map.insert_with(&op, &[], |func_b| {
            let [in_wire] = func_b.input_wires_arr();
            let out_wire = func_b.make_sum(
                1,
                vec![hugr::type_row![], vec![elem_ty.clone()].into()],
                [in_wire],
            )?;
            Ok(vec![out_wire])
        })?;
        Ok(builder
            .add_dataflow_op(op, [wire])?
            .outputs()
            .next()
            .expect("one output"))
    }

    /// Generic array unpacking using the ArrayKind trait
    fn unpack_array<AK: ArrayKind>(
        &self,
        builder: &mut impl Dataflow,
        array_wire: Wire,
        size: u64,
        elem_ty: &Type,
        op_name: &OpName,
    ) -> Result<Vec<Wire>, BuildError> {
        let args = match self.array_args::<AK>(size, elem_ty) {
            Some(args) => args,
            None => return Ok(vec![array_wire]), // Not a type we should unpack
        };

        let op = self.get_op(op_name, args.clone()).expect("known op");

        self.func_map.insert_with(&op, &args[..2], |func_b| {
            let w = func_b.input().out_wire(0);
            let elems = func_b.add_generic_array_unpack::<AK>(elem_ty.clone(), size, w)?;

            let result: Vec<_> = elems
                .into_iter()
                .map(|wire| self.unpack_container(func_b, elem_ty, wire))
                .collect::<Result<Vec<_>, _>>()?
                .concat();
            Ok(result)
        })?;

        Ok(builder
            .add_dataflow_op(op, [array_wire])?
            .outputs()
            .collect())
    }

    /// Helper function for array arguments
    fn array_args<AK: ArrayKind>(&self, size: u64, elem_ty: &Type) -> Option<[TypeArg; 3]> {
        let row = self
            .type_analyzer
            .unpack_type(&AK::ty(size, elem_ty.clone()))?;
        let args = [
            size.into(),
            elem_ty.clone().into(),
            TypeArg::List(row.into_iter().map(Into::into).collect()),
        ];
        Some(args)
    }

    /// Generic array repacking using the ArrayKind trait
    fn repack_array<AK: ArrayKind>(
        &self,
        builder: &mut impl Dataflow,
        elem_wires: impl IntoIterator<Item = Wire>,
        size: u64,
        elem_ty: &Type,
        op_name: &OpName,
    ) -> Result<Wire, BuildError> {
        let args = match self.array_args::<AK>(size, elem_ty) {
            Some(args) => args,
            None => {
                return Ok(elem_wires
                    .into_iter()
                    .next()
                    .expect("Non-unpackable container should only have one wire."));
            }
        };

        let inner_row_len = self.type_analyzer.num_unpacked_wires(elem_ty);
        let op = self.get_op(op_name, args.clone()).expect("known op");

        self.func_map.insert_with(&op, &args[..2], |func_b| {
            let input = func_b.input();
            // SAFETY: We guarantee no aliasing by only using this pointer in this closure.
            let elems: Result<Vec<_>, _> = input
                .outputs()
                .collect::<Vec<_>>()
                .chunks(inner_row_len)
                .map(|chunk| self.repack_container(func_b, elem_ty, chunk.to_vec()))
                .collect();

            let array_wire = func_b.add_new_generic_array::<AK>(elem_ty.clone(), elems?)?;
            Ok(vec![array_wire])
        })?;
        Ok(builder
            .add_dataflow_op(op, elem_wires)?
            .outputs()
            .next()
            .expect("one output"))
    }

    /// Generate tuple arguments
    fn tuple_args(&self, tuple_row: &[Type]) -> Option<[TypeArg; 2]> {
        let unpacked_row = self
            .type_analyzer
            .unpack_type(&Type::new_tuple(tuple_row.to_vec()))?;

        let args = [
            TypeArg::List(tuple_row.iter().cloned().map(Into::into).collect()),
            TypeArg::List(unpacked_row.into_iter().map(Into::into).collect()),
        ];

        Some(args)
    }

    /// Unpack a row of types into a flat list of wires containing all elements matching the analyzer
    pub fn unpack_row(
        &self,
        builder: &mut impl Dataflow,
        types: &[Type],
        wires: impl IntoIterator<Item = Wire>,
    ) -> Result<Vec<Wire>, BuildError> {
        // Process each type in the row with its corresponding wire
        let unpacked: Result<Vec<_>, _> = types
            .iter()
            .zip(wires)
            .map(|(ty, wire)| self.unpack_container(builder, ty, wire))
            .collect();

        // Flatten the nested vector of wires
        Ok(unpacked?.concat())
    }

    /// Repack a flat list of wires into a row of structured types
    pub fn repack_row(
        &self,
        builder: &mut impl Dataflow,
        types: &[Type],
        wires: impl IntoIterator<Item = Wire>,
    ) -> Result<Vec<Wire>, BuildError> {
        let mut wires = wires.into_iter();
        types
            .iter()
            .map(|ty| {
                let wire_count = self.type_analyzer.num_unpacked_wires(ty);
                let type_wires = wires.by_ref().take(wire_count).collect();
                self.repack_container(builder, ty, type_wires)
            })
            .collect()
    }

    /// Unpack a tuple into individual wires
    pub fn unpack_tuple(
        &self,
        builder: &mut impl Dataflow,
        tuple_wire: Wire,
        tuple_row: &[Type],
    ) -> Result<Vec<Wire>, BuildError> {
        let tuple_row = tuple_row.to_vec();
        let args = match self.tuple_args(&tuple_row) {
            Some(args) => args,
            None => return Ok(vec![tuple_wire]), // Not a tuple we should unpack
        };
        let op = self.get_op(&TUPLE_UNPACK, args.clone()).expect("known op");

        self.func_map.insert_with(&op, &args[..1], |func_b| {
            let w = func_b.input().out_wire(0);
            let unpacked_tuple_wires = func_b
                .add_dataflow_op(UnpackTuple::new(tuple_row.clone().into()), [w])?
                .outputs()
                .collect::<Vec<_>>();

            let unpacked = self.unpack_row(func_b, &tuple_row, unpacked_tuple_wires)?;
            Ok(unpacked)
        })?;

        Ok(builder
            .add_dataflow_op(op, [tuple_wire])?
            .outputs()
            .collect())
    }

    /// Repack wires into a tuple
    pub fn repack_tuple(
        &self,
        builder: &mut impl Dataflow,
        elem_wires: impl IntoIterator<Item = Wire>,
        tuple_row: &[Type],
    ) -> Result<Wire, BuildError> {
        let tuple_row = tuple_row.to_vec();
        let args = match self.tuple_args(&tuple_row) {
            Some(args) => args,
            None => {
                return Ok(elem_wires
                    .into_iter()
                    .next()
                    .expect("Non-unpackable container should only have one wire."));
            }
        };

        let op = self.get_op(&TUPLE_REPACK, args.clone()).expect("known op");

        self.func_map.insert_with(&op, &args[..1], |func_b| {
            let in_wires = func_b.input().outputs().collect::<Vec<_>>();

            let repacked_elem_wires = self.repack_row(func_b, &tuple_row, in_wires)?;
            let tuple_wire = func_b.make_tuple(repacked_elem_wires)?;

            Ok(vec![tuple_wire])
        })?;

        Ok(builder
            .add_dataflow_op(op, elem_wires)?
            .outputs()
            .next()
            .expect("one output"))
    }

    /// Unpack a container type to extract wires matching the analyzer criteria.
    pub fn unpack_container(
        &self,
        builder: &mut impl Dataflow,
        ty: &Type,
        container_wire: Wire,
    ) -> Result<Vec<Wire>, BuildError> {
        let elem_ty = self.type_analyzer.element_type();
        // If the type is a qubit, return it directly
        if ty == elem_ty {
            return Ok(vec![container_wire]);
        }

        // Check for option of qubit
        if is_opt_of(ty, &hugr::extension::prelude::qb_t()) {
            return Ok(vec![self.unpack_option(
                builder,
                container_wire,
                elem_ty,
            )?]);
        }

        macro_rules! handle_array_type {
            ($array_kind:ty, $unpack_op:expr) => {
                if let Some((n, elem_ty)) = ty.as_extension().and_then(array_args::<$array_kind>) {
                    return self.unpack_array::<$array_kind>(
                        builder,
                        container_wire,
                        n,
                        elem_ty,
                        &$unpack_op,
                    );
                }
            };
        }

        handle_array_type!(Array, ARRAY_UNPACK);
        handle_array_type!(BorrowArray, BARRAY_UNPACK);

        if let Some(row) = ty.as_sum().and_then(SumType::as_tuple) {
            let row: hugr::types::TypeRow =
                row.clone().try_into().expect("unexpected row variable.");
            return self.unpack_tuple(builder, container_wire, &row);
        }

        // No need to unpack if the type doesn't match our analyzer criteria
        Ok(vec![container_wire])
    }

    /// Repack a container type from its unpacked wires.
    pub fn repack_container(
        &self,
        builder: &mut impl Dataflow,
        ty: &Type,
        unpacked_wires: Vec<Wire>,
    ) -> Result<Wire, BuildError> {
        let elem_ty = self.type_analyzer.element_type();
        // If the type is a qubit, return the wire directly
        if ty == elem_ty {
            debug_assert!(unpacked_wires.len() == 1);
            return Ok(unpacked_wires[0]);
        }

        // Check for option of qubit
        if is_opt_of(ty, elem_ty) {
            debug_assert!(unpacked_wires.len() == 1);
            return self.repack_option(builder, unpacked_wires[0], elem_ty);
        }

        macro_rules! handle_array_type {
            ($array_kind:ty, $repack_op:expr) => {
                if let Some((n, elem_ty)) = ty.as_extension().and_then(array_args::<$array_kind>) {
                    return self.repack_array::<$array_kind>(
                        builder,
                        unpacked_wires,
                        n,
                        elem_ty,
                        &$repack_op,
                    );
                }
            };
        }

        handle_array_type!(Array, ARRAY_REPACK);
        handle_array_type!(BorrowArray, BARRAY_REPACK);

        if let Some(row) = ty.as_sum().and_then(SumType::as_tuple) {
            let row: hugr::types::TypeRow =
                row.clone().try_into().expect("unexpected row variable.");
            return self.repack_tuple(builder, unpacked_wires, &row);
        }

        debug_assert!(unpacked_wires.len() == 1);
        Ok(unpacked_wires[0])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hugr::{
        HugrView,
        builder::{DFGBuilder, DataflowHugr as _},
        extension::prelude::{bool_t, option_type, qb_t, usize_t},
        std_extensions::collections::array::array_type,
        types::Signature,
    };
    use rstest::rstest;

    #[test]
    fn test_container_factory_creation() {
        let analyzer = TypeUnpacker::for_qubits();
        let factory = UnpackContainerBuilder::new(analyzer);
        assert_eq!(factory.func_map.len(), 0);
    }

    #[test]
    fn test_option_unwrap_wrap() -> Result<(), BuildError> {
        let analyzer = TypeUnpacker::for_qubits();
        let factory = UnpackContainerBuilder::new(analyzer);
        let option_qb_type = Type::from(option_type(qb_t()));
        let mut builder = DFGBuilder::new(Signature::new_endo(vec![option_qb_type]))?;

        let input = builder.input().out_wire(0);
        let unwrapped = factory.unpack_option(&mut builder, input, &qb_t())?;
        let wrapped = factory.repack_option(&mut builder, unwrapped, &qb_t())?;

        let hugr = builder.finish_hugr_with_outputs([wrapped])?;
        assert!(hugr.validate().is_ok());
        Ok(())
    }

    #[rstest]
    #[case::array(Array, ARRAY_UNPACK, ARRAY_REPACK)]
    #[case::borrow_array(BorrowArray, BARRAY_UNPACK, BARRAY_REPACK)]
    fn test_array_unpack_repack<AK: ArrayKind>(
        #[case] _kind: AK,
        #[case] unpack_op: OpName,
        #[case] repack_op: OpName,
    ) -> Result<(), BuildError> {
        let analyzer = TypeUnpacker::for_qubits();
        let factory = UnpackContainerBuilder::new(analyzer);
        let array_size = 2;

        // Create the specific array type
        let array_type = AK::ty(array_size, qb_t());

        // Build a dataflow graph that unpacks and repacks the array
        let mut builder = DFGBuilder::new(Signature::new_endo(array_type))?;
        let input = builder.input().out_wire(0);

        // Unpack the array
        let unpacked =
            factory.unpack_array::<AK>(&mut builder, input, array_size, &qb_t(), &unpack_op)?;

        // Repack the array
        let repacked =
            factory.repack_array::<AK>(&mut builder, unpacked, array_size, &qb_t(), &repack_op)?;

        let hugr = builder.finish_hugr_with_outputs([repacked])?;
        assert!(hugr.validate().is_ok());

        Ok(())
    }

    #[test]
    fn test_tuple_unpack_repack() -> Result<(), BuildError> {
        let analyzer = TypeUnpacker::for_qubits();
        let factory = UnpackContainerBuilder::new(analyzer);
        let tuple_row = vec![qb_t(), bool_t()];
        let tuple_type = Type::new_tuple(tuple_row.clone());

        let mut builder = DFGBuilder::new(Signature::new_endo(tuple_type))?;

        let input = builder.input().out_wire(0);
        let unpacked = factory.unpack_tuple(&mut builder, input, &tuple_row)?;
        assert_eq!(unpacked.len(), tuple_row.len());

        let repacked = factory.repack_tuple(&mut builder, unpacked, &tuple_row)?;
        let hugr = builder.finish_hugr_with_outputs([repacked])?;
        assert!(hugr.validate().is_ok());
        Ok(())
    }

    #[test]
    fn test_unpack_repack_row() -> Result<(), BuildError> {
        let analyzer = TypeUnpacker::for_qubits();
        let factory = UnpackContainerBuilder::new(analyzer);
        let types = vec![qb_t(), bool_t(), array_type(2, qb_t())];
        let mut builder = DFGBuilder::new(hugr::types::Signature::new_endo(types.clone()))?;

        let inputs = builder.input().outputs().collect::<Vec<_>>();
        let unpacked = factory.unpack_row(&mut builder, &types, inputs)?;
        let repacked = factory.repack_row(&mut builder, &types, unpacked)?;

        let hugr = builder.finish_hugr_with_outputs(repacked)?;
        assert!(hugr.validate().is_ok());
        Ok(())
    }

    #[test]
    fn test_unpack_repack_row_non_qubit() -> Result<(), BuildError> {
        // Use a TypeUnpacker that targets bools, not qubits
        let analyzer = TypeUnpacker::new(bool_t());
        let factory = UnpackContainerBuilder::new(analyzer);
        let types = vec![bool_t(), usize_t(), Array::ty(2, bool_t())];
        let mut builder = DFGBuilder::new(hugr::types::Signature::new_endo(types.clone()))?;

        let inputs = builder.input().outputs().collect::<Vec<_>>();
        let unpacked = factory.unpack_row(&mut builder, &types, inputs)?;
        // Should unpack all bools and array of bools (array size 2)
        assert_eq!(unpacked.len(), 4, "Bool row should be fully unpacked");

        let repacked = factory.repack_row(&mut builder, &types, unpacked)?;
        assert_eq!(
            repacked.len(),
            3,
            "Repacked row should match original length"
        );

        let hugr = builder.finish_hugr_with_outputs(repacked)?;
        assert!(hugr.validate().is_ok());
        Ok(())
    }
}
