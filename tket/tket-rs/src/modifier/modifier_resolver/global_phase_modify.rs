//! Modifier resolution for global phase operations.
use hugr::{
    IncomingPort, Node, Wire,
    builder::Dataflow,
    core::HugrNode,
    hugr::hugrmut::HugrMut,
    std_extensions::arithmetic::{float_ops::FloatOps, float_types::ConstF64},
};

use crate::{
    TketOp,
    extension::{global_phase::GlobalPhase, rotation::RotationOp},
    modifier::modifier_resolver::{ModifierResolver, ModifierResolverErrors, connect},
};

impl<N: HugrNode> ModifierResolver<N> {
    /// Modify a global phase operation.
    /// This returns the incoming port for the rotation of the modified operation.
    pub(crate) fn modify_global_phase(
        &mut self,
        n: N,
        new_fn: &mut impl Dataflow,
        ancilla: &mut Vec<Wire<Node>>,
    ) -> Result<Vec<(Node, IncomingPort)>, ModifierResolverErrors<N>> {
        match (self.modifiers.dagger, self.control_num()) {
            (false, 0) => {
                let node = new_fn.add_child_node(GlobalPhase);
                let in_port = IncomingPort::from(0);
                Ok(vec![(node, in_port)])
            }
            (true, 0) => {
                let halfturn = new_fn.add_child_node(RotationOp::to_halfturns);
                let angle_float = Wire::new(halfturn, 0);
                let neg_angle_float = new_fn
                    .add_dataflow_op(FloatOps::fneg, vec![angle_float])
                    .map(|out| out.out_wire(0))?;
                let angle = new_fn
                    .add_dataflow_op(RotationOp::from_halfturns_unchecked, vec![neg_angle_float])
                    .map(|out| out.out_wire(0))?;
                new_fn.add_dataflow_op(GlobalPhase, vec![angle])?;
                Ok(vec![(halfturn, IncomingPort::from(0))])
            }
            // Cn+1Phase(cs, c, θ) = * CnPhase(cs, θ/2) * CnRz(cs, c, θ)
            (dagger, _) => {
                self.modifiers.dagger = false;

                let halfturn = new_fn.add_child_node(RotationOp::to_halfturns);
                let angle_float = Wire::new(halfturn, 0);

                let half = new_fn.add_load_value(ConstF64::new(if dagger { -0.5 } else { 0.5 }));
                let half_angle_float = new_fn
                    .add_dataflow_op(FloatOps::fmul, vec![angle_float, half])
                    .map(|out| out.out_wire(0))?;
                let angle_half = new_fn
                    .add_dataflow_op(RotationOp::from_halfturns_unchecked, vec![half_angle_float])
                    .map(|out| out.out_wire(0))?;

                let mut c = self.pop_control().unwrap();

                // C^nPhase(cs, θ/2)
                let c_phase = self.with_ancilla(&mut c, ancilla, |this, ancilla| {
                    this.modify_global_phase(n, new_fn, ancilla)
                })?;
                for (node, port) in c_phase {
                    new_fn
                        .hugr_mut()
                        .connect(angle_half.node(), angle_half.source(), node, port);
                }

                // C^nRz(cs, c, θ)
                let c_rz = self.modify_tket_op(n, TketOp::Rz, new_fn, ancilla)?;
                connect(new_fn, &c_rz.incoming[0], &c.into())?;
                c = c_rz.outgoing[0].try_into().unwrap();

                let mut result = vec![(halfturn, IncomingPort::from(0))];

                if dagger {
                    let neg_angle_float = new_fn
                        .add_dataflow_op(FloatOps::fneg, vec![angle_float])
                        .map(|out| out.out_wire(0))?;
                    let angle = new_fn
                        .add_dataflow_op(
                            RotationOp::from_halfturns_unchecked,
                            vec![neg_angle_float],
                        )
                        .map(|out| out.out_wire(0))?;
                    connect(new_fn, &c_rz.incoming[1], &angle.into())?;
                } else {
                    let in_wire = c_rz.incoming[1].try_into().unwrap();
                    result.push(in_wire)
                }

                self.push_control(c);
                self.modifiers.dagger = dagger;

                Ok(result)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use std::iter;

    use hugr::{
        Hugr, HugrView,
        builder::{DataflowHugr, DataflowSubContainer, FunctionBuilder, ModuleBuilder},
        extension::{prelude::qb_t, simple_op::MakeExtensionOp},
        ops::handle::{FuncID, NodeHandle},
        type_row,
        types::Signature,
    };

    use crate::{
        extension::rotation::ConstRotation,
        modifier::modifier_resolver::{
            resolve_modifier,
            tests::{SetUnitary, resolved_modifier_test_hugr},
        },
        passes::PassScope,
    };

    use super::*;

    fn foo(module: &mut ModuleBuilder<Hugr>, t_num: usize) -> FuncID<true> {
        let foo_sig = Signature::new_endo(iter::repeat_n(qb_t(), t_num).collect::<Vec<_>>());
        let mut func = module.define_function("foo", foo_sig.clone()).unwrap();
        func.set_unitary();
        let inputs: Vec<Wire> = func.input_wires().collect();
        let theta = func.add_load_value(ConstRotation::new(0.5).unwrap());
        func.add_dataflow_op(GlobalPhase, vec![theta]).unwrap();
        *func.finish_with_outputs(inputs).unwrap().handle()
    }

    #[test]
    fn global_phase_without_modifiers_is_preserved() {
        let mut function = FunctionBuilder::new("main", Signature::new_endo(type_row![])).unwrap();
        let angle = function.add_load_value(ConstRotation::new(0.5).unwrap());
        let phase = function
            .add_dataflow_op(GlobalPhase, [angle])
            .unwrap()
            .node();
        let mut h = function.finish_hugr_with_outputs([]).unwrap();

        resolve_modifier(&mut h, &PassScope::default()).unwrap();

        assert!(h.contains_node(phase));
        assert_eq!(
            h.single_linked_output(phase, IncomingPort::from(0)),
            Some((angle.node(), angle.source())),
        );
    }

    #[rstest::rstest]
    #[case(0, foo, true)]
    #[case(1, foo, false)]
    #[case(1, foo, true)]
    #[case(5, foo, false)]
    #[case(5, foo, true)]
    fn test_global_phase_modify(
        #[case] c_num: u64,
        #[case] foo: fn(&mut ModuleBuilder<Hugr>, usize) -> FuncID<true>,
        #[case] dagger: bool,
    ) {
        let h = resolved_modifier_test_hugr(0, c_num, foo, dagger);
        assert!(
            h.nodes()
                .any(|node| GlobalPhase::from_optype(h.get_optype(node)).is_some()),
            "modifier resolution discarded the global phase",
        );
    }
}
