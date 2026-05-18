//! Modify nodes related to function calls.
use hugr::{
    IncomingPort, Wire,
    builder::{BuildError, Dataflow},
    core::HugrNode,
    extension::simple_op::MakeExtensionOp,
    hugr::hugrmut::HugrMut,
    ops::{Call, CallIndirect, DataflowOpTrait, LoadFunction, OpType},
};

use super::{ModifierError, ModifierResolver, ModifierResolverErrors};
use crate::extension::modifier::Modifier;

impl<N: HugrNode> ModifierResolver<N> {
    pub(super) fn modify_call(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        call_node: N,
        optype: &OpType,
        new_dfg: &mut impl Dataflow,
    ) -> Result<(), ModifierResolverErrors<N>> {
        let OpType::Call(call) = optype else {
            return Err(ModifierResolverErrors::unreachable(
                "Not a Call".to_string(),
            ));
        };
        let offset = self.modifiers().accum_ctrl.len();
        let old_signature = (*call.signature()).clone();
        let callee = h
            .single_linked_output(call_node, call.called_function_port())
            .unwrap();

        // wire the callee
        let Some(new_callee) = self.modify_fn_if_needed(h, callee.0)? else {
            // If the function need not be modified, just copy the Call node as is.
            let new = self.add_node_no_modification(h, call_node, call.clone(), new_dfg)?;
            self.call_map_insert(callee.0, (new, call.called_function_port()));
            return Ok(());
        };

        let mut poly_sig = call.func_sig.clone();
        let type_args = call.type_args.clone();
        self.modify_signature(poly_sig.body_mut(), false);
        let new_call = Call::try_new(poly_sig, type_args).map_err(BuildError::from)?;
        let new_call_fn_port = new_call.called_function_port();
        let new_call_node = new_dfg.add_child_node(new_call);

        self.call_map_insert(new_callee, (new_call_node, new_call_fn_port));
        // wire the controls
        let mut controls = self.pack_controls(new_dfg)?;
        for (i, control) in controls.iter_mut().enumerate() {
            new_dfg
                .hugr_mut()
                .connect(control.node(), control.source(), new_call_node, i);
            *control = Wire::new(new_call_node, i);
        }
        let controls = self.unpack_controls(new_dfg, controls)?;
        *self.controls() = controls;
        // wire the inputs/outputs - we use the original signature here because the information about the controller is already in the offset
        self.wire_node_inout(
            call_node,
            new_call_node,
            (old_signature.input.iter(), old_signature.output.iter()),
            (0, 0, offset),
        )?;

        Ok(())
    }

    /// Apply the collected chain of modifiers to the function loaded by the `LoadFunction` node.
    /// Returns the new node that loads the modified function.
    /// This applies changes to the original graph `h`.
    pub(super) fn apply_modifier_chain_to_loaded_fn(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        modifier_node: N,
    ) -> Result<N, ModifierResolverErrors<N>> {
        // The final target of modifiers to apply.
        // Collection of modifiers to apply.
        let modifiers_and_targ = self.trace_modifiers_chain(h, modifier_node)?;

        let targ = modifiers_and_targ
            .last()
            .cloned()
            .ok_or(ModifierError::NoTarget(modifier_node))?;

        // The function to apply the modifier to. This is expected to be a LoadFunction node
        let (func, load) = Self::get_loaded_function(h, modifier_node, targ, h.get_optype(targ))?;

        // Only remove targ if it has exactly one consumer (the modifier chain).
        // If it has multiple consumers, leave it in place to preserve shared loads.
        let node_consumers = h.linked_inputs(targ, 0).count();
        if node_consumers == 1 {
            h.remove_node(targ);
        }

        // Modify the function
        let modified_fn = self.modify_fn(h, func)?;

        // Modify the function loader
        // Insert the new LoadFunction node to load the modified function
        let mut modified_sig = load.func_sig.clone();
        self.modify_signature(modified_sig.body_mut(), false);
        let load = LoadFunction::try_new(modified_sig, load.type_args).map_err(BuildError::from)?;
        let new_load = h.add_node_after(modifier_node, load);
        h.connect(modified_fn, 0, new_load, 0);

        Ok(new_load)
    }

    /// Trace the chain of modifiers starting from node `n`, collecting all modifier nodes until reaching
    /// a non-modifier target node. Returns the chain of nodes in order from the starting node to the target node.
    /// The return includes the starting node and the target node.
    pub(super) fn trace_modifiers_chain(
        &mut self,
        h: &impl HugrMut<Node = N>,
        n: N,
    ) -> Result<Vec<N>, ModifierError<N>> {
        // The final target of modifiers to apply.
        let mut current = n;
        // Collection of modifiers to apply.
        let modifiers = self.modifiers_mut();
        let mut chain: Vec<N> = Vec::new();
        loop {
            chain.push(current);
            let optype = h.get_optype(current);

            if Modifier::from_optype(optype).is_none() {
                break;
            }

            modifiers.push(optype.as_extension_op().unwrap());
            let next = h
                .single_linked_output(current, 0)
                .ok_or(ModifierError::NoTarget(n))?;
            current = next.0;
        }
        Ok(chain)
    }

    /// Given a target node `targ` which is expected to be a `LoadFunction`, retrieve the function node it loads.
    pub(super) fn get_loaded_function(
        h: &impl HugrMut<Node = N>,
        modifier_node: N,
        targ: N,
        optype: &OpType,
    ) -> Result<(N, LoadFunction), ModifierError<N>> {
        match optype {
            OpType::LoadFunction(load) => {
                let (fn_node, _) = h.single_linked_output(targ, 0).unwrap();
                let fn_optype = h.get_optype(fn_node);
                let OpType::FuncDefn(_) = fn_optype else {
                    return Err(ModifierError::ModifierNotApplicable(
                        modifier_node,
                        fn_optype.clone(),
                    ));
                };
                // TODO: We want some machinery to prevent generating a lot of copies of modified functions
                // from the same function.
                Ok((fn_node, load.clone()))
            }
            OpType::Input(_) => Err(ModifierError::NoTarget(modifier_node)),
            // If the target is a function, we need to create a new dataflow block of it.
            _ => {
                // TODO:
                // In the future, we might want to handle modifiers provided from other nodes.
                // For example, conditionals?
                Err(ModifierError::ModifierNotApplicable(
                    modifier_node,
                    optype.clone(),
                ))
            }
        }
    }

    pub(super) fn modify_indirect_call(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        n: N,
        indir_call: &CallIndirect,
        new_dfg: &mut impl Dataflow,
    ) -> Result<(), ModifierResolverErrors<N>> {
        // Wrapper to convert ModifierError to UnResolvable with the indir_call node.
        // This is because, even if we find an error in the process immediately,
        // we cannot stop processing here.
        let wrap_err = |e: ModifierError<N>| {
            ModifierResolverErrors::unresolvable(
                e.node(),
                "Cannot modify indirect call.".to_string(),
                indir_call.clone().into(),
            )
        };

        // Trace the chain of modifiers starting from the one before the indirect call.
        let chain_tail = h.single_linked_output(n, 0).unwrap();
        let modifiers = self.modifiers().clone();
        let trace = self
            .trace_modifiers_chain(h, chain_tail.0)
            .map_err(wrap_err)?;
        let targ = trace.last().cloned().unwrap();
        let (func, load) =
            Self::get_loaded_function(h, n, targ, h.get_optype(targ)).map_err(wrap_err)?;

        // Modify the function
        let modified_fn = match self.modify_fn_if_needed(h, func)? {
            Some(node) => node,
            None => self.wrap_fn_with_controls(h, func, &load.type_args)?,
        };

        // Make new LoadFunction
        let mut modified_sig = load.func_sig.clone();
        self.modify_signature(modified_sig.body_mut(), false);
        let load = LoadFunction::try_new(modified_sig, load.type_args).map_err(BuildError::from)?;
        let new_load = new_dfg.add_child_node(load);
        self.call_map_insert(modified_fn, (new_load, IncomingPort::from(0)));
        *self.modifiers_mut() = modifiers;

        // Make new IndirectCall
        let mut new_call = indir_call.clone();
        self.modify_signature(&mut new_call.signature, false);
        let new_call_node = new_dfg.add_child_node(new_call);

        // Wire the new IndirectCall
        let mut controls = self.pack_controls(new_dfg)?;
        let offset = self.modifiers().accum_ctrl.len();
        for (i, ctrl) in controls.iter_mut().enumerate() {
            new_dfg
                .hugr_mut()
                .connect(ctrl.node(), ctrl.source(), new_call_node, i + 1);
            *ctrl = Wire::new(new_call_node, i);
        }
        *self.controls() = self.unpack_controls(new_dfg, controls)?;

        let signature = indir_call.signature();
        self.wire_node_inout(
            n,
            new_call_node,
            (signature.input.iter().skip(1), signature.output.iter()),
            (1, 0, offset),
        )?;
        new_dfg.hugr_mut().connect(new_load, 0, new_call_node, 0);
        self.map_insert_none((n, IncomingPort::from(0)).into())?;

        // FIXME: Forgetting all the nodes in the chain so that we don't have to worry about mapping the edges.
        // Otherwise, there would be edges in the original graph that have no corresponding edges in the new graph.
        // However, this could remove wires referenced by other nodes that are not in the chain.
        for node in trace {
            self.forget_node(h, node)?
        }

        Ok(())
    }

    pub(super) fn modify_load_function(
        &mut self,
        _h: &impl HugrMut<Node = N>,
        _n: N,
        _load: &LoadFunction,
        _new_dfg: &mut impl Dataflow,
    ) -> Result<(), ModifierResolverErrors<N>> {
        // TODO:
        // Indirect calles would be handled by its caller.
        // However, when a loaded function is used in the other ways
        // (e.g., passed to higher-order functions as `map` or `fold`),
        // we need to modify it here.
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::super::tests::{SetUnitary, test_modifier_resolver};
    use super::super::*;
    use crate::TketOp;
    use crate::extension::modifier::{CONTROL_OP_ID, MODIFIER_EXTENSION};
    use hugr::{
        Hugr,
        builder::{Dataflow, DataflowSubContainer, ModuleBuilder},
        extension::prelude::qb_t,
        ops::{CallIndirect, ExtensionOp, handle::FuncID},
        std_extensions::collections::array::{ArrayOpBuilder, array_type},
        types::{Signature, Term},
    };

    fn foo_call(module: &mut ModuleBuilder<Hugr>, t_num: usize) -> FuncID<true> {
        let callee = {
            let callee_sig = Signature::new_endo(vec![qb_t()]);
            let mut callee_builder = module.define_function("dummy", callee_sig).unwrap();
            callee_builder.set_unitary();
            let mut inputs: Vec<Wire> = callee_builder.input_wires().collect();
            inputs[0] = callee_builder
                .add_dataflow_op(TketOp::X, vec![inputs[0]])
                .unwrap()
                .out_wire(0);
            callee_builder.finish_with_outputs(inputs).unwrap()
        };

        let foo_sig = Signature::new_endo(iter::repeat_n(qb_t(), t_num).collect::<Vec<_>>());
        let mut func = module.define_function("foo", foo_sig.clone()).unwrap();
        func.set_unitary();
        let mut inputs: Vec<_> = func.input_wires().collect();
        inputs[0] = func
            .call(callee.handle(), &[], vec![inputs[0]])
            .unwrap()
            .out_wire(0);
        *func.finish_with_outputs(inputs).unwrap().handle()
    }

    /// Nested call pattern: `foo(q) = foo1(q)`, `foo1(q) = bar(q)`, `bar(q) = X(q)`.
    /// Tests that the resolver correctly propagates modifiers through a three-level call chain.
    fn foo_modifier_on_function(module: &mut ModuleBuilder<Hugr>, t_num: usize) -> FuncID<true> {
        // bar: applies X to its single qubit argument.
        let bar = {
            let bar_sig = Signature::new_endo(vec![qb_t()]);
            let mut bar_builder = module.define_function("inner", bar_sig).unwrap();
            bar_builder.set_unitary();
            let mut inputs: Vec<Wire> = bar_builder.input_wires().collect();
            inputs[0] = bar_builder
                .add_dataflow_op(TketOp::X, vec![inputs[0]])
                .unwrap()
                .out_wire(0);
            bar_builder.finish_with_outputs(inputs).unwrap()
        };

        // foo1: delegates entirely to bar.
        let foo1 = {
            let foo1_sig = Signature::new_endo(vec![qb_t()]);
            let mut foo1_builder = module.define_function("outer", foo1_sig).unwrap();
            foo1_builder.set_unitary();
            let mut inputs: Vec<Wire> = foo1_builder.input_wires().collect();
            inputs[0] = foo1_builder
                .call(bar.handle(), &[], vec![inputs[0]])
                .unwrap()
                .out_wire(0);
            foo1_builder.finish_with_outputs(inputs).unwrap()
        };

        // foo: delegates entirely to foo1.
        let foo_sig = Signature::new_endo(iter::repeat_n(qb_t(), t_num).collect::<Vec<_>>());
        let mut func = module.define_function("foo", foo_sig).unwrap();
        func.set_unitary();
        let mut inputs: Vec<_> = func.input_wires().collect();
        inputs[0] = func
            .call(foo1.handle(), &[], vec![inputs[0]])
            .unwrap()
            .out_wire(0);
        *func.finish_with_outputs(inputs).unwrap().handle()
    }

    fn foo_indir_call(module: &mut ModuleBuilder<Hugr>, t_num: usize) -> FuncID<true> {
        let callee_sig = Signature::new_endo(vec![qb_t()]);
        let callee = {
            let mut callee_builder = module.define_function("dummy", callee_sig.clone()).unwrap();
            callee_builder.set_unitary();
            let mut inputs: Vec<Wire> = callee_builder.input_wires().collect();
            inputs[0] = callee_builder
                .add_dataflow_op(TketOp::X, vec![inputs[0]])
                .unwrap()
                .out_wire(0);
            callee_builder.finish_with_outputs(inputs).unwrap()
        };

        let foo_sig = Signature::new_endo(iter::repeat_n(qb_t(), t_num).collect::<Vec<_>>());
        let mut func = module.define_function("foo", foo_sig.clone()).unwrap();
        func.set_unitary();
        let mut inputs: Vec<_> = func.input_wires().collect();
        let handle = func.load_func(callee.handle(), &[]).unwrap();
        inputs[0] = func
            .add_dataflow_op(
                CallIndirect {
                    signature: callee_sig,
                },
                vec![handle, inputs[0]],
            )
            .unwrap()
            .out_wire(0);
        inputs[0] = func
            .add_dataflow_op(TketOp::X, vec![inputs[0]])
            .unwrap()
            .out_wire(0);
        *func.finish_with_outputs(inputs).unwrap().handle()
    }

    fn foo_load_fn(module: &mut ModuleBuilder<Hugr>, t_num: usize) -> FuncID<true> {
        let callee = {
            let callee_sig = Signature::new_endo(vec![qb_t()]);
            let mut callee_builder = module.define_function("dummy", callee_sig).unwrap();
            callee_builder.set_unitary();
            let mut inputs: Vec<Wire> = callee_builder.input_wires().collect();
            inputs[0] = callee_builder
                .add_dataflow_op(TketOp::X, vec![inputs[0]])
                .unwrap()
                .out_wire(0);
            callee_builder.finish_with_outputs(inputs).unwrap()
        };

        let foo_sig = Signature::new_endo(iter::repeat_n(qb_t(), t_num).collect::<Vec<_>>());
        let mut func = module.define_function("foo", foo_sig.clone()).unwrap();
        func.set_unitary();
        let inputs: Vec<_> = func.input_wires().collect();
        let _ = func.load_func(callee.handle(), &[]).unwrap();
        *func.finish_with_outputs(inputs).unwrap().handle()
    }

    fn foo_nested_modifier(module: &mut ModuleBuilder<Hugr>, t_num: usize) -> FuncID<true> {
        let bar_sig = Signature::new_endo(vec![qb_t()]);
        let bar = {
            let mut bar_builder = module.define_function("bar", bar_sig).unwrap();
            bar_builder.set_unitary();
            let mut inputs: Vec<Wire> = bar_builder.input_wires().collect();
            inputs[0] = bar_builder
                .add_dataflow_op(TketOp::X, vec![inputs[0]])
                .unwrap()
                .out_wire(0);
            bar_builder.finish_with_outputs(inputs).unwrap()
        };

        let controlled_sig = Signature::new_endo(vec![array_type(1, qb_t()), qb_t()]);
        let foo_sig = Signature::new_endo(iter::repeat_n(qb_t(), t_num).collect::<Vec<_>>());
        let foo = {
            let mut foo_builder = module.define_function("foo", foo_sig).unwrap();
            foo_builder.set_unitary();
            let mut inputs: Vec<Wire> = foo_builder.input_wires().collect();
            let load = foo_builder.load_func(bar.handle(), &[]).unwrap();

            let control_op: ExtensionOp = {
                MODIFIER_EXTENSION
                    .instantiate_extension_op(
                        &CONTROL_OP_ID,
                        [Term::BoundedNat(1), [qb_t().into()].into(), [].into()],
                    )
                    .unwrap()
            };
            let controlled = foo_builder
                .add_dataflow_op(control_op, vec![load])
                .unwrap()
                .out_wire(0);
            let mut ctrl = foo_builder.add_new_array(qb_t(), [inputs[0]]).unwrap();
            [ctrl, inputs[1]] = foo_builder
                .add_dataflow_op(
                    CallIndirect {
                        signature: controlled_sig,
                    },
                    [controlled, ctrl, inputs[1]],
                )
                .unwrap()
                .outputs_arr();
            inputs[0] = foo_builder.add_array_unpack(qb_t(), 1, ctrl).unwrap()[0];
            foo_builder.finish_with_outputs(inputs).unwrap()
        };
        *foo.handle()
    }

    #[rstest::rstest]
    #[case::call_twice(1, 1, foo_modifier_on_function, false)]
    #[case::call(1, 1, foo_call, false)]
    #[case::call_dagger(1, 1, foo_call, true)]
    #[case::indir_call(1, 1, foo_indir_call, false)]
    #[case::indir_call_dagger(1, 1, foo_indir_call, true)]
    #[case::load_fn(1, 1, foo_load_fn, false)]
    #[case::nested_modifier(2, 2, foo_nested_modifier, false)]
    fn test_call_modify(
        #[case] target_num: usize,
        #[case] ctrl_num: u64,
        #[case] foo: fn(&mut ModuleBuilder<Hugr>, usize) -> FuncID<true>,
        #[case] dagger: bool,
    ) {
        test_modifier_resolver(target_num, ctrl_num, foo, dagger);
    }
}
