use anyhow::Result;
use hugr::llvm::inkwell;
use hugr::{HugrView, Node};
use inkwell::AddressSpace;
use inkwell::attributes::{Attribute, AttributeLoc};
use inkwell::values::FunctionValue;
use tket::hugr::llvm::emit::EmitFuncContext;
use tket::hugr::{self};

pub fn panic_str_fn<'c>(
    ctx: &EmitFuncContext<'c, '_, impl HugrView<Node = Node>>,
) -> Result<FunctionValue<'c>> {
    // The built-in QIS panic() is designed for global strings with known
    // lengths, bounded by 255 characters. This is to keep resources under
    // control for hardware implementations.
    //
    // In an emulation context, we are less resource-constrained, and some
    // user experience gains can be made by opting to use a more general panic
    // function exposed by selene's QIS implementation: `panic_str`. This takes
    // a standard char pointer, and is not bounded by static string allocation.
    let iwc = ctx.iw_context();
    let panic_str_type = iwc.void_type().fn_type(
        &[
            iwc.i32_type().into(),
            iwc.ptr_type(AddressSpace::default()).into(),
        ],
        false,
    );
    let f = ctx.get_extern_func("panic_str", panic_str_type)?;
    let noreturn = Attribute::get_named_enum_kind_id("noreturn");
    f.add_attribute(
        AttributeLoc::Function,
        iwc.create_enum_attribute(noreturn, 0),
    );
    Ok(f)
}
