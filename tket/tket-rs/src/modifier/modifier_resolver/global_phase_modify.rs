//! Modifier resolution for global phase operations.
use hugr::{
    IncomingPort, Node, Wire,
    builder::Dataflow,
    core::HugrNode,
    hugr::hugrmut::HugrMut,
    ops::{OpType, handle::NodeHandle},
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
    ///
    /// `n` is the original node being modified. It may be another op (e.g. a T gate) that
    /// decomposes into a global phase. Set `map_order_ports` only when `n` is itself a
    /// `GlobalPhase`: its state order ports are then mapped to those of every `GlobalPhase`
    /// emitted (there may be several).
    pub(crate) fn modify_global_phase(
        &mut self,
        n: N,
        map_order_ports: bool,
        new_fn: &mut impl Dataflow,
        ancilla: &mut Vec<Wire<Node>>,
    ) -> Result<Vec<(Node, IncomingPort)>, ModifierResolverErrors<N>> {
        match (self.modifiers.dagger, self.control_num()) {
            (false, 0) => {
                let node = new_fn.add_child_node(GlobalPhase);
                if map_order_ports {
                    self.map_phase_order_ports(n, node)?;
                }
                Ok(vec![(node, IncomingPort::from(0))])
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
                let phase = new_fn.add_dataflow_op(GlobalPhase, vec![angle])?.node();
                if map_order_ports {
                    self.map_phase_order_ports(n, phase)?;
                }
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
                    this.modify_global_phase(n, map_order_ports, new_fn, ancilla)
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

impl<N: HugrNode> ModifierResolver<N> {
    /// Map the StateOrder ports of `old_phase` to those of `new_phase`.
    fn map_phase_order_ports(
        &mut self,
        old_phase: N,
        new_phase: Node,
    ) -> Result<(), ModifierResolverErrors<N>> {
        let optype = OpType::from(GlobalPhase);
        if let Some(port) = optype.other_input_port() {
            self.map_insert((old_phase, port).into(), (new_phase, port).into())?;
        }
        if let Some(port) = optype.other_output_port() {
            self.map_insert((old_phase, port).into(), (new_phase, port).into())?;
        }
        Ok(())
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
        passes::{PassScope, force_order::force_order},
    };

    use super::*;
    use crate::modifier::modifier_resolver::tests::{modifier_test_hugr, test_resolve};

    fn foo(module: &mut ModuleBuilder<Hugr>, t_num: usize) -> FuncID<true> {
        let foo_sig = Signature::new_endo(iter::repeat_n(qb_t(), t_num).collect::<Vec<_>>());
        let mut func = module.define_function("foo", foo_sig.clone()).unwrap();
        func.set_unitary();
        let inputs: Vec<Wire> = func.input_wires().collect();
        let theta = func.add_load_value(ConstRotation::new(0.5).unwrap());
        func.add_dataflow_op(GlobalPhase, vec![theta]).unwrap();
        *func.finish_with_outputs(inputs).unwrap().handle()
    }

    /// Verify that global phases survive modifier resolution when no modifiers are applied.
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

    /// Global phases with order edges must keep them, including when the phase
    /// is split into several operations by control/dagger modifiers.
    #[rstest::rstest]
    #[case(0, false)]
    #[case(0, true)]
    #[case(1, false)]
    #[case(1, true)]
    #[case(3, false)]
    #[case(3, true)]
    fn test_global_phase_with_order_edges(#[case] c_num: u64, #[case] dagger: bool) {
        fn foo_ordered(module: &mut ModuleBuilder<Hugr>, _: usize) -> FuncID<true> {
            let mut func = module
                .define_function("foo", Signature::new_endo([qb_t()]))
                .unwrap();
            func.set_unitary();
            let [q] = func.input_wires_arr();
            let theta = func.add_load_value(ConstRotation::new(0.5).unwrap());
            let q = func.add_dataflow_op(TketOp::X, [q]).unwrap().out_wire(0);
            func.add_dataflow_op(GlobalPhase, vec![theta]).unwrap();
            let q = func.add_dataflow_op(TketOp::X, [q]).unwrap().out_wire(0);
            *func.finish_with_outputs([q]).unwrap().handle()
        }

        let (mut h, foo_node) = modifier_test_hugr(1, c_num, foo_ordered, dagger);
        // The `GlobalPhase` has no qubit wires, so nothing orders it relative to the X gates.
        // `force_order` chains the children of `foo` with StateOrder edges, ranked by node
        // index (i.e. creation order: load, X, phase, X). The phase therefore gets an order edge
        // on both its input and output, which the resolver must carry over.
        force_order(&mut h, foo_node, |_, n| hugr::NodeIndex::index(n) as i64).unwrap();
        let phase = h
            .descendants(foo_node)
            .find(|n| GlobalPhase::from_optype(h.get_optype(*n)).is_some())
            .unwrap();
        let order_in = h.get_optype(phase).other_input_port().unwrap();
        let order_out = h.get_optype(phase).other_output_port().unwrap();
        assert!(h.single_linked_output(phase, order_in).is_some());
        assert!(h.linked_inputs(phase, order_out).next().is_some());
        h.validate().unwrap();

        // After resolving, we should not have any missing correspondence errors.
        test_resolve(&mut h);
        assert!(
            h.nodes()
                .any(|node| GlobalPhase::from_optype(h.get_optype(node)).is_some()),
        );
    }
}
