//! Modify nodes related to function calls.
use std::collections::HashSet;

use hugr::{
    IncomingPort, PortIndex, Wire,
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

        let Some(new_callee) =
            self.modify_fn_if_needed(h, callee.0, Some(&old_signature), false)?
        else {
            // If the function need not be modified, just copy the Call node as is.
            let new = self.add_node_no_modification(h, call_node, call.clone(), new_dfg)?;
            self.call_map_insert(callee.0, (new, call.called_function_port()));
            return Ok(());
        };

        // Modified higher-order functions may require some function-valued
        // inputs to be supplied already modified. When the caller provides a
        // static LoadFunction, solve that modifier here and wire the modified
        // load directly into the new call.
        let input_modifiers = self.function_input_modifiers(new_callee).to_vec();
        let OpType::FuncDefn(new_callee_defn) = h.get_optype(new_callee) else {
            return Err(ModifierResolverErrors::unreachable(format!(
                "Modified callee is not a function definition: {}",
                h.get_optype(new_callee)
            )));
        };
        let poly_sig = new_callee_defn.signature().clone();
        let type_args = call.type_args.clone();
        let new_call = Call::try_new(poly_sig, type_args).map_err(BuildError::from)?;
        let new_call_fn_port = new_call.called_function_port();
        let new_call_node = new_dfg.add_child_node(new_call);

        self.call_map_insert(new_callee, (new_call_node, new_call_fn_port));
        let skip_inputs = input_modifiers
            .iter()
            .map(|(input, _)| *input)
            .collect::<HashSet<_>>();

        // Handle function arguments that must be modified before calling `new_call_node`.
        // If the argument comes from another function input, we cannot solve it here,
        // so we record that requirement for the caller.
        // If the argument is a `LoadFunction`, we create the modified version of that
        // loaded function and connect it directly to the new call.
        for (input, modifiers) in input_modifiers {
            // Resolve this argument in the modifier context required by the
            // callee. The previous modifier state must be restored even if the
            // argument cannot be resolved.
            let saved_modifiers = std::mem::replace(self.modifiers_mut(), modifiers);
            let result: Result<(), ModifierResolverErrors<N>> = (|| {
                let source = h.single_linked_output(call_node, input).ok_or_else(|| {
                    ModifierResolverErrors::unreachable(format!(
                        "Call input {input} has no source while resolving higher-order modifier."
                    ))
                })?;
                let trace = self.trace_modifiers_chain(h, source.0)?;
                let targ = trace.last().cloned().ok_or_else(|| {
                    ModifierResolverErrors::unreachable(
                        "Higher-order modifier argument trace was empty.".to_string(),
                    )
                })?;
                if matches!(h.get_optype(targ), OpType::Input(_)) {
                    // The argument is another higher-order input of the function
                    // currently being rewritten. This call cannot solve the
                    // requirement yet, so record it for callers and wire the
                    // function value through to the modified call input.
                    let target_port = if trace.len() == 1 {
                        source.1
                    } else {
                        h.single_linked_output(trace[trace.len() - 2], 0)
                        .ok_or_else(|| {
                            ModifierResolverErrors::unreachable(
                                "Higher-order modifier argument trace ended at an input without an input port."
                                    .to_string(),
                            )
                        })?
                        .1
                    };
                    let modifiers = self.modifiers().clone();
                    self.dynamic_input_modifiers()
                        .push((target_port.index(), modifiers));
                    self.map_insert(
                        (call_node, IncomingPort::from(input)).into(),
                        (new_call_node, IncomingPort::from(input + offset)).into(),
                    )?;
                    return Ok(());
                }

                // The caller supplied a concrete LoadFunction. Build/load the
                // modified version now and connect that value directly into the
                // rewritten call.
                let (func, load) =
                    Self::get_loaded_function(h, call_node, targ, h.get_optype(targ))
                        .map_err(ModifierResolverErrors::ModifierError)?;
                let modified_fn = self.modify_fn(h, func)?;

                let mut modified_sig = load.func_sig.clone();
                self.modify_signature(modified_sig.body_mut(), false);
                let load = LoadFunction::try_new(modified_sig, load.type_args)
                    .map_err(BuildError::from)?;
                let new_load = new_dfg.add_child_node(load);
                self.call_map_insert(modified_fn, (new_load, IncomingPort::from(0)));
                new_dfg
                    .hugr_mut()
                    .connect(new_load, 0, new_call_node, input + offset);
                self.map_insert_none((call_node, IncomingPort::from(input)).into())?;

                for node in trace {
                    self.forget_node(h, node)?;
                }
                Ok(())
            })();
            *self.modifiers_mut() = saved_modifiers;
            result?;
        }

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

        // Wire the inputs/outputs using the original signature; controller
        // information is already represented by `offset`. Inputs handled above
        // are skipped so connect_all will not also wire the original function
        // value into the call.
        self.wire_inout(
            (call_node, call_node),
            (new_call_node, new_call_node),
            (old_signature.input.iter(), old_signature.output.iter()),
            (0, 0, offset),
            &skip_inputs,
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
    ) -> Result<Vec<N>, ModifierResolverErrors<N>> {
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

            modifiers.push(optype.as_extension_op().unwrap(), current)?;
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
        // Wrap ModifierError as UnResolvable, using the ModifierError node as the error
        // location and the IndirectCall OpType for context.
        let wrap_modifier_err = |e: ModifierError<N>| {
            ModifierResolverErrors::unresolvable(
                e.node(),
                "Cannot modify indirect call.".to_string(),
                indir_call.clone().into(),
            )
        };
        // Wrap ModifierResolverErrors::ModifierError as UnResolvable
        let wrap_resolver_err = |e: ModifierResolverErrors<N>| match e {
            ModifierResolverErrors::ModifierError(inner) => wrap_modifier_err(inner),
            other => other,
        };

        // Trace the chain of modifiers starting from the one before the indirect call, if present.
        let chain_tail = h.single_linked_output(n, 0).unwrap();
        let modifiers = self.modifiers().clone();
        let trace = self
            .trace_modifiers_chain(h, chain_tail.0)
            .map_err(wrap_resolver_err)?;
        let targ = trace.last().cloned().unwrap();

        // If the target is a function input, we cannot solve the modifier chain here.
        // Instead, we record the modifiers to be applied to that input and propagate
        // the requirement to callers.
        if matches!(h.get_optype(targ), OpType::Input(_)) {
            // If no quantum data is involved, we can skip modifying the call
            if !self.signature_has_quantum_data(&indir_call.signature) {
                self.add_node_no_modification(h, n, indir_call.clone(), new_dfg)?;
                return Ok(());
            }
            *self.modifiers_mut() = modifiers;
            return self.modify_input_indirect_call(n, chain_tail.1.index(), indir_call, new_dfg);
        }
        // If the target is not a input, we expect it to be a LoadFunction node loading the function to call.
        let (func, load) =
            Self::get_loaded_function(h, n, targ, h.get_optype(targ)).map_err(wrap_modifier_err)?;

        let Some(modified_fn) = self
            .modify_fn_if_needed(h, func, Some(&indir_call.signature), trace.len() > 1)
            .map_err(wrap_resolver_err)?
        else {
            self.add_node_no_modification(h, n, indir_call.clone(), new_dfg)?;
            return Ok(());
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

    fn modify_input_indirect_call(
        &mut self,
        n: N,
        function_input: usize,
        indir_call: &CallIndirect,
        new_dfg: &mut impl Dataflow,
    ) -> Result<(), ModifierResolverErrors<N>> {
        let mut new_call = indir_call.clone();
        self.modify_signature(&mut new_call.signature, false);
        let new_call_node = new_dfg.add_child_node(new_call);

        // The callee is a function input, so there is no LoadFunction to solve
        // inside this body. Record that callers of the generated function must
        // pass a statically modified value for this input, then call that input
        // directly in the rewritten body.
        let modifiers = self.modifiers().clone();
        self.dynamic_input_modifiers()
            .push((function_input, modifiers));
        self.map_insert(
            (n, IncomingPort::from(0)).into(),
            (new_call_node, IncomingPort::from(0)).into(),
        )?;

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

        Ok(())
    }

    pub(super) fn modify_load_function(
        &mut self,
        h: &impl HugrMut<Node = N>,
        n: N,
        load: &LoadFunction,
        new_dfg: &mut impl Dataflow,
    ) -> Result<(), ModifierResolverErrors<N>> {
        let consumers = h.linked_inputs(n, 0).collect::<Vec<_>>();

        // Check if all consumers are modifiers. If so, we can just forget the LoadFunction node and let the modifiers rebuild it.
        if !consumers.is_empty()
            && consumers
                .iter()
                .all(|(consumer, _)| Modifier::from_optype(h.get_optype(*consumer)).is_some())
        {
            // Modifier consumers rebuild their own LoadFunction nodes.
            return self.forget_node(h, n);
        }
        // Plain LoadFunction values still need their static edge restored.
        let new = self.add_node_no_modification(h, n, load.clone(), new_dfg)?;
        let (loaded_func, _) =
            h.single_linked_output(n, load.function_port())
                .ok_or_else(|| {
                    ModifierResolverErrors::unreachable(
                        "LoadFunction node has no linked static function.".to_string(),
                    )
                })?;
        self.call_map_insert(loaded_func, (new, load.function_port()));
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
        std_extensions::arithmetic::{
            int_ops::IntOpDef,
            int_types::{ConstInt, INT_TYPES},
        },
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

    fn foo_nested_modifier_unmodified_callee(
        module: &mut ModuleBuilder<Hugr>,
        t_num: usize,
    ) -> FuncID<true> {
        let bar_sig = Signature::new_endo(vec![qb_t()]);
        let bar = {
            let bar_builder = module.define_function("bar", bar_sig).unwrap();
            let inputs: Vec<Wire> = bar_builder.input_wires().collect();
            bar_builder.finish_with_outputs(inputs).unwrap()
        };

        let foo_sig = Signature::new_endo(iter::repeat_n(qb_t(), t_num).collect::<Vec<_>>());
        let foo = {
            let mut foo_builder = module.define_function("foo", foo_sig).unwrap();
            foo_builder.set_unitary();
            let mut inputs: Vec<Wire> = foo_builder.input_wires().collect();

            inputs[1] = foo_builder
                .call(bar.handle(), &[], [inputs[1]])
                .unwrap()
                .out_wire(0);
            foo_builder.finish_with_outputs(inputs).unwrap()
        };
        *foo.handle()
    }

    /// Test quantum and classical indirect calls in modifier context
    fn foo_indirect_unmodified_callees(
        module: &mut ModuleBuilder<Hugr>,
        t_num: usize,
    ) -> FuncID<true> {
        let quantum_sig = Signature::new_endo(vec![qb_t()]);
        let quantum = {
            let mut quantum_builder = module
                .define_function("indirect_quantum", quantum_sig.clone())
                .unwrap();
            quantum_builder.set_unitary();
            let mut inputs: Vec<Wire> = quantum_builder.input_wires().collect();
            inputs[0] = quantum_builder
                .add_dataflow_op(TketOp::X, vec![inputs[0]])
                .unwrap()
                .out_wire(0);
            quantum_builder.finish_with_outputs(inputs).unwrap()
        };

        let int_t = INT_TYPES[3].clone();
        let add_sig = Signature::new(vec![int_t.clone(); 2], vec![int_t]);
        let add = {
            let mut add_builder = module
                .define_function("indirect_classical_add", add_sig.clone())
                .unwrap();
            let [lhs, rhs] = add_builder.input_wires_arr();
            let sum = add_builder
                .add_dataflow_op(IntOpDef::iadd.with_log_width(3), [lhs, rhs])
                .unwrap()
                .out_wire(0);
            add_builder.finish_with_outputs([sum]).unwrap()
        };

        let foo_sig = Signature::new_endo(iter::repeat_n(qb_t(), t_num).collect::<Vec<_>>());
        let mut foo_builder = module.define_function("foo", foo_sig).unwrap();
        foo_builder.set_unitary();
        let mut inputs: Vec<Wire> = foo_builder.input_wires().collect();

        let quantum_handle = foo_builder.load_func(quantum.handle(), &[]).unwrap();
        inputs[0] = foo_builder
            .add_dataflow_op(
                CallIndirect {
                    signature: quantum_sig,
                },
                [quantum_handle, inputs[0]],
            )
            .unwrap()
            .out_wire(0);

        let add_handle = foo_builder.load_func(add.handle(), &[]).unwrap();
        let lhs = foo_builder.add_load_value(ConstInt::new_u(3, 2).unwrap());
        let rhs = foo_builder.add_load_value(ConstInt::new_u(3, 3).unwrap());
        let _sum = foo_builder
            .add_dataflow_op(CallIndirect { signature: add_sig }, [add_handle, lhs, rhs])
            .unwrap()
            .out_wire(0);

        *foo_builder.finish_with_outputs(inputs).unwrap().handle()
    }

    #[rstest::rstest]
    #[case::call_twice(1, 1, foo_modifier_on_function, false)]
    #[case::call(1, 1, foo_call, false)]
    #[case::call_dagger(1, 1, foo_call, true)]
    #[case::indir_call(1, 1, foo_indir_call, false)]
    #[case::indir_call_dagger(1, 1, foo_indir_call, true)]
    #[case::load_fn(1, 1, foo_load_fn, false)]
    #[case::nested_modifier(2, 2, foo_nested_modifier, false)]
    #[case::nested_modifier_unmodified_callee(2, 2, foo_nested_modifier_unmodified_callee, false)]
    #[case::indirect_unmodified_callees(1, 1, foo_indirect_unmodified_callees, true)]
    fn test_call_modify(
        #[case] target_num: usize,
        #[case] ctrl_num: u64,
        #[case] foo: fn(&mut ModuleBuilder<Hugr>, usize) -> FuncID<true>,
        #[case] dagger: bool,
    ) {
        test_modifier_resolver(target_num, ctrl_num, foo, dagger);
    }
}
