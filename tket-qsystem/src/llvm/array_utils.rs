//! Array codegen utilities.

// TODO move to hugr-llvm crate
// https://github.com/quantinuum/tket2/issues/899
use anyhow::Result;
use hugr::extension::prelude::usize_t;
use hugr::llvm::emit::EmitFuncContext;
use hugr::llvm::extension::collections::array;
use hugr::llvm::extension::collections::array::{
    build_array_fat_pointer, decompose_array_fat_pointer,
};
use hugr::llvm::inkwell::types::{BasicType, BasicTypeEnum};
use hugr::llvm::inkwell::values::BasicValueEnum;
use hugr::llvm::{CodegenExtension, inkwell};
use hugr::{HugrView, Node};
use inkwell::AddressSpace;
use inkwell::builder::{Builder, BuilderError};
use inkwell::context::Context;
use inkwell::types::{IntType, StructType};
use inkwell::values::{ArrayValue, IntValue, PointerValue, StructValue};

/// Specifies different array lowering strategies.
///
/// See [DEFAULT_HEAP_ARRAY_LOWERING] for the default array lowerings
/// implementing this trait.
pub trait ArrayLowering {
    /// The [CodegenExtension] specifying the array lowering.
    fn codegen_extension(&self) -> impl CodegenExtension;

    /// Turns an array value in the given lowering into a pointer to the first array element.
    fn array_to_ptr<'c>(
        &self,
        builder: &Builder<'c>,
        val: BasicValueEnum<'c>,
        elem_type: BasicTypeEnum<'c>,
        length: u32,
    ) -> Result<PointerValue<'c>>;

    /// Turns a pointer to the first array element into an array value in the given lowering.
    fn array_from_ptr<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &mut EmitFuncContext<'c, '_, H>,
        ptr: PointerValue<'c>,
        elem_type: BasicTypeEnum<'c>,
        length: u32,
    ) -> Result<BasicValueEnum<'c>>;
}

/// Array lowering via a heap as implemented in [mod@array].
#[derive(Clone)]
pub struct HeapArrayLowering<ACG: array::ArrayCodegen>(ACG);

/// The default heap array lowering strategy using [array::DefaultArrayCodegen].
pub const DEFAULT_HEAP_ARRAY_LOWERING: HeapArrayLowering<array::DefaultArrayCodegen> =
    HeapArrayLowering(array::DefaultArrayCodegen);

impl<ACG: array::ArrayCodegen> HeapArrayLowering<ACG> {
    /// Creates a new [HeapArrayLowering].
    pub const fn new(array_codegen: ACG) -> Self {
        Self(array_codegen)
    }
}

impl<ACG: array::ArrayCodegen + Clone> ArrayLowering for HeapArrayLowering<ACG> {
    fn codegen_extension(&self) -> impl CodegenExtension {
        array::ArrayCodegenExtension::new(self.0.clone())
    }

    fn array_to_ptr<'c>(
        &self,
        builder: &Builder<'c>,
        val: BasicValueEnum<'c>,
        elem_type: BasicTypeEnum<'c>,
        length: u32,
    ) -> Result<PointerValue<'c>> {
        let (array_ptr, offset) = decompose_array_fat_pointer(builder, val)?;
        let array_ty = elem_type.array_type(length);
        let elem_ptr = unsafe { builder.build_in_bounds_gep(array_ty, array_ptr, &[offset], "")? };
        Ok(elem_ptr)
    }

    fn array_from_ptr<'c, H: HugrView<Node = Node>>(
        &self,
        ctx: &mut EmitFuncContext<'c, '_, H>,
        ptr: PointerValue<'c>,
        _elem_type: BasicTypeEnum<'c>,
        _length: u32,
    ) -> Result<BasicValueEnum<'c>> {
        let usize_ty = ctx
            .typing_session()
            .llvm_type(&usize_t())
            .expect("Prelude codegen is registered")
            .into_int_type();
        let offset = usize_ty.const_zero();
        let array = build_array_fat_pointer(ctx, ptr, offset)?;
        Ok(array.into())
    }
}

/// Helper function to allocate an array on the stack.
///
/// Returns a pointer to the newly allocated array.
pub fn build_array_alloca<'c>(
    builder: &Builder<'c>,
    array: ArrayValue<'c>,
) -> Result<PointerValue<'c>, BuilderError> {
    let array_ty = array.get_type();
    let array_len: IntValue<'c> = {
        let ctx = builder.get_insert_block().unwrap().get_context();
        ctx.i32_type().const_int(u64::from(array_ty.len()), false)
    };
    let ptr = builder.build_array_alloca(array_ty.get_element_type(), array_len, "")?;
    builder.build_store(ptr, array)?;
    Result::Ok(ptr)
}

/// Helper function to load an array from a pointer.
pub fn build_int_array_load<'c>(
    builder: &Builder<'c>,
    array_ptr: PointerValue<'c>,
    elem_type: IntType<'c>,
    length: u32,
) -> Result<ArrayValue<'c>, BuilderError> {
    let array_ty = elem_type.array_type(length);
    let array = builder
        .build_load(array_ty, array_ptr, "")?
        .into_array_value();
    Result::Ok(array)
}

/// Enum representing the element types of a dense array.
pub enum ElemType {
    /// A signed integer element type.
    Int,
    /// An unsigned integer element type.
    Uint,
    /// A floating-point element type.
    Float,
    /// A boolean element type.
    Bool,
}

impl ElemType {
    /// Get the corresponding `inkwell::types::BasicTypeEnum`
    pub fn llvm_type<'a>(&self, ctx: &'a Context) -> BasicTypeEnum<'a> {
        match *self {
            ElemType::Int | ElemType::Uint => ctx.i64_type().into(),
            ElemType::Float => ctx.f64_type().into(),
            ElemType::Bool => ctx.bool_type().into(),
        }
    }
}

/// Helper function to create a dense array struct type.
///
/// The struct contains four fields:
///   (1) the size along the first data dimension
///   (2) the size along the second data dimension
///   (3) the pointer to the first element of the primary data in memory
///   (4) the pointer to the first element of the auxiliary sparsity flags
///       in memory
///
/// The fourth field points to an array of masking data of the same size as the
/// primary data in memory and contains boolean values to indicate the presence
/// of data in the primary array. Dense arrays have mask values of all zeros.
pub fn struct_1d_arr_t<'a>(ctx: &'a Context) -> StructType<'a> {
    let ptr_t = ctx.ptr_type(AddressSpace::default());
    ctx.struct_type(
        &[
            ctx.i32_type().into(), // x
            ctx.i32_type().into(), // y
            ptr_t.into(),          // pointer to first element
            ptr_t.into(),          // pointer to first mask element
        ],
        true,
    )
}

/// Helper function to allocate and initialize a dense array struct on the stack.
///
/// Returns a `PointerVal` to the struct and the `StructVal` itself. All of
/// the mask values are initialized to 0.
pub fn struct_1d_arr_alloc<'a>(
    ctx: &'a Context,
    builder: &Builder<'a>,
    length: u32,
    array_ptr: PointerValue<'a>,
) -> Result<(PointerValue<'a>, StructValue<'a>), BuilderError> {
    let out_arr_type = struct_1d_arr_t(ctx);
    let out_arr_ptr = builder.build_alloca(out_arr_type, "out_arr_alloca")?;

    let x_field = builder.build_struct_gep(out_arr_type, out_arr_ptr, 0, "x_ptr")?;
    let y_field = builder.build_struct_gep(out_arr_type, out_arr_ptr, 1, "y_ptr")?;
    let arr_field = builder.build_struct_gep(out_arr_type, out_arr_ptr, 2, "arr_ptr")?;
    let mask_field = builder.build_struct_gep(out_arr_type, out_arr_ptr, 3, "mask_ptr")?;

    let x_val = ctx.i32_type().const_int(length.into(), false);
    let y_val = ctx.i32_type().const_int(1, false);
    let mask_ptr = build_array_alloca(
        builder,
        ctx.bool_type().const_array(
            vec![ctx.bool_type().const_int(0, false); length.try_into().unwrap()].as_slice(),
        ),
    )?;

    builder.build_store(x_field, x_val)?;
    builder.build_store(y_field, y_val)?;
    builder.build_store(arr_field, array_ptr)?;
    builder.build_store(mask_field, mask_ptr)?;

    let out_arr = builder
        .build_load(out_arr_type, out_arr_ptr, "")?
        .into_struct_value();

    Result::Ok((out_arr_ptr, out_arr))
}

#[cfg(test)]
mod tests {
    use super::*;
    use hugr::llvm::{inkwell::context::Context, test::llvm_ctx};
    use rstest::rstest;

    /// Test that build_array_alloca properly allocates an array.
    #[test]
    fn test_build_array_alloca() {
        let context = Context::create();
        let module = context.create_module("test_module");
        let builder = context.create_builder();

        make_bb(&context, &module, &builder);

        let ptr = build_array(&context, &builder).expect("Array allocation should succeed");

        assert!(!ptr.is_null(), "Pointer should not be null");

        builder
            .build_return(None)
            .expect("Should be able to build return");

        // Verify the generated code is valid
        assert!(module.verify().is_ok(), "Module verification failed");
    }

    /// Helper function to create a basic block for testing.
    fn make_bb<'c>(
        context: &'c Context,
        module: &inkwell::module::Module<'c>,
        builder: &Builder<'c>,
    ) {
        let function_type = context.void_type().fn_type(&[], false);
        let function = module.add_function("test_function", function_type, None);
        let basic_block = context.append_basic_block(function, "entry");
        builder.position_at_end(basic_block);
    }

    fn build_array<'c>(
        context: &'c Context,
        builder: &Builder<'c>,
    ) -> Result<PointerValue<'c>, BuilderError> {
        // Create test array
        let i32_type = context.i32_type();
        let array =
            i32_type.const_array(&[i32_type.const_int(1, false), i32_type.const_int(2, false)]);

        build_array_alloca(builder, array)
    }

    /// Test that build_int_array_load properly loads an array.
    #[test]
    fn test_build_int_array_load() {
        let context = Context::create();
        let module = context.create_module("test_module");
        let builder = context.create_builder();

        make_bb(&context, &module, &builder);

        let array_ptr = build_array(&context, &builder).expect("Array allocation should succeed");
        let i32_type = context.i32_type();
        let array_length = 2;
        let loaded_array = build_int_array_load(&builder, array_ptr, i32_type, array_length)
            .expect("Array load should succeed");

        assert_eq!(loaded_array.get_type().len(), array_length,);

        builder.build_return(None).unwrap();

        // Verify the generated code is valid
        assert!(module.verify().is_ok(), "Module verification failed");
    }

    /// Test that struct_1d_arr_t creates the correct structure type.
    #[test]
    fn test_struct_1d_arr_t() {
        let context = Context::create();
        let struct_ty = struct_1d_arr_t(&context);

        // Fields should be (int, int, ptr, ptr)
        assert_eq!(struct_ty.get_field_types().len(), 4);
        assert!(struct_ty.get_field_types()[0].is_int_type());
        assert!(struct_ty.get_field_types()[1].is_int_type());
        assert!(struct_ty.get_field_types()[2].is_pointer_type());
        assert!(struct_ty.get_field_types()[3].is_pointer_type());
    }

    /// Test that struct_1d_arr_alloc properly allocates and initializes a dense array struct.
    #[test]
    fn test_struct_1d_arr_alloc() {
        let context = Context::create();
        let module = context.create_module("test_module");
        let builder = context.create_builder();

        make_bb(&context, &module, &builder);

        let array_ptr = build_array(&context, &builder).unwrap();
        let (struct_ptr, _) = struct_1d_arr_alloc(&context, &builder, 2, array_ptr).unwrap();
        assert!(!struct_ptr.is_null(), "Struct pointer should not be null");

        builder
            .build_return(None)
            .expect("Should be able to build return");

        // Verify the generated code is valid
        assert!(module.verify().is_ok(), "Module verification failed");
    }

    /// Tests that [ArrayLowering::array_to_ptr] and [ArrayLowering::array_from_ptr] are inverses.
    #[rstest]
    #[case(DEFAULT_HEAP_ARRAY_LOWERING)]
    fn test_array_ptr_conversion(#[case] array_lowering: impl ArrayLowering) {
        let mut llvm_ctx = llvm_ctx(-1);
        llvm_ctx.add_extensions(|cge| cge.add_default_prelude_extensions());

        let mod_ctx = llvm_ctx.get_emit_module_context();
        let function_type = mod_ctx.iw_context().void_type().fn_type(&[], false);
        let function = mod_ctx
            .module()
            .add_function("test_function", function_type, None);
        let mut emit_ctx = EmitFuncContext::new(mod_ctx, function).unwrap();

        let elem_ty = emit_ctx.iw_context().i32_type().into();
        let size = 2;

        let array_ptr = build_array(emit_ctx.iw_context(), emit_ctx.builder()).unwrap();
        let array = array_lowering
            .array_from_ptr(&mut emit_ctx, array_ptr, elem_ty, size)
            .unwrap();
        let new_array_ptr = array_lowering
            .array_to_ptr(emit_ctx.builder(), array, elem_ty, size)
            .unwrap();
        assert_eq!(array_ptr.get_type(), new_array_ptr.get_type());
        let new_array = array_lowering
            .array_from_ptr(&mut emit_ctx, new_array_ptr, elem_ty, size)
            .unwrap();
        assert_eq!(array.get_type(), new_array.get_type());
    }
}
