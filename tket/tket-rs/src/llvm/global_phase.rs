//! LLVM lowering for the `tket.global_phase` extension.

use hugr::llvm::{CodegenExtension, CodegenExtsBuilder};
use hugr::{HugrView, Node};

use crate::extension::global_phase::{GLOBAL_PHASE_EXTENSION_ID, GLOBAL_PHASE_OP_ID};

/// Emit global phase operations as no-ops.
///
/// Global phases affects functions with a control modifier,
/// but on their own they do not have a direct observable effect in the runtime.
pub struct GlobalPhaseCodegenExtension;

impl CodegenExtension for GlobalPhaseCodegenExtension {
    fn add_extension<'a, H: HugrView<Node = Node> + 'a>(
        self,
        builder: CodegenExtsBuilder<'a, H>,
    ) -> CodegenExtsBuilder<'a, H>
    where
        Self: 'a,
    {
        builder.extension_op(
            GLOBAL_PHASE_EXTENSION_ID,
            GLOBAL_PHASE_OP_ID.clone(),
            |context, args| args.outputs.finish(context.builder(), []),
        )
    }
}

#[cfg(test)]
mod test {
    use hugr::builder::{Dataflow, DataflowHugr};
    use hugr::hugr::hugrmut::HugrMut;
    use hugr::llvm::check_emission;
    use hugr::llvm::emit::test::SimpleHugrConfig;
    use hugr::llvm::test::{TestContext, llvm_ctx};
    use hugr::ops::handle::NodeHandle;
    use rstest::rstest;

    use crate::extension::global_phase::GlobalPhase;
    use crate::extension::rotation::{ConstRotation, rotation_type};
    use crate::llvm::rotation::DEFAULT_ROTATION_EXTENSION;

    use super::*;

    #[rstest]
    #[case::constant(0, false)]
    #[case::dynamic(1, true)]
    fn emit_global_phase(
        #[case] _id: i32,
        #[with(_id)] mut llvm_ctx: TestContext,
        #[case] dynamic_angle: bool,
    ) {
        let inputs = if dynamic_angle {
            vec![rotation_type()]
        } else {
            vec![]
        };
        let mut hugr = SimpleHugrConfig::new()
            .with_ins(inputs)
            .finish(|mut builder| {
                let angle = if dynamic_angle {
                    builder.input_wires_arr::<1>()[0]
                } else {
                    builder.add_load_value(ConstRotation::PI)
                };
                let phase = builder
                    .add_dataflow_op(GlobalPhase, [angle])
                    .unwrap()
                    .node();
                let mut hugr = builder.finish_hugr_with_outputs([]).unwrap();
                // Preserve the ordering constraints emitted by Guppy.
                let [input, output] = hugr.get_io(hugr.entrypoint()).unwrap();
                hugr.add_other_edge(input, phase);
                hugr.add_other_edge(phase, output);
                hugr
            });
        llvm_ctx.add_extensions(|cge| {
            cge.add_default_prelude_extensions()
                .add_extension(DEFAULT_ROTATION_EXTENSION.to_owned())
                .add_extension(GlobalPhaseCodegenExtension)
        });
        check_emission!(hugr, llvm_ctx);
    }
}
