//! Modify nodes related to function calls.

use hugr::{
    HugrView, IncomingPort, Wire,
    builder::{BuildError, Dataflow},
    core::HugrNode,
    extension::simple_op::MakeExtensionOp,
    hugr::hugrmut::HugrMut,
    ops::{Call, CallIndirect, DataflowOpTrait, LoadFunction, OpType},
};

use super::{ModifierResolver, ModifierResolverErrors};
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

        let Some(new_callee) = self.modify_fn_if_needed(h, callee.0, Some(&old_signature))? else {
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

        // Wire the inputs/outputs using the original signature; `offset` represent controller qubits
        self.wire_node_inout(
            call_node,
            new_call_node,
            (old_signature.input.iter(), old_signature.output.iter()),
            (0, 0, offset),
        )?;
        self.wire_state_order(
            call_node,
            h.get_optype(call_node),
            new_call_node,
            new_dfg.hugr().get_optype(new_call_node),
        )?;

        Ok(())
    }

    /// Apply the accumulated modifiers to a loaded function and construct its replacement loader.
    /// `load_node` is expected to be a `LoadFunction` node.
    ///
    /// Returns the modified function node in `h` and an uninserted `LoadFunction` operation.
    pub(super) fn modify_load(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        load_node: N,
    ) -> Result<(N, LoadFunction), ModifierResolverErrors<N>> {
        let optype = h.get_optype(load_node).clone();
        let OpType::LoadFunction(load) = optype else {
            panic!("Expected LoadFunction, found {optype:?}");
        };
        let load_op = load.clone();

        let (func_node, _) = h.single_linked_output(load_node, 0).unwrap();
        let fn_optype = h.get_optype(func_node);
        match fn_optype {
            OpType::FuncDefn(_) => {}
            OpType::FuncDecl(_) => {
                return Err(ModifierResolverErrors::ModifierNotApplicable(
                    load_node,
                    fn_optype.clone(),
                ));
            }
            _ => {
                panic!("Expected FuncDefn or FuncDecl, found {fn_optype:?}");
            }
        }

        let modified_fn = self.modify_fn(h, func_node)?;
        // Give the replacement loader the modified function's signature
        let mut modified_sig = load_op.func_sig;
        self.modify_signature(modified_sig.body_mut(), false);
        let new_load =
            LoadFunction::try_new(modified_sig, load_op.type_args).map_err(BuildError::from)?;

        Ok((modified_fn, new_load))
    }

    /// Trace a `CallIndirect`'s function input back to a `LoadFunction`, accumulating modifiers.
    /// Return nodes from nearest modifier to load, excluding the call; error on invalid chains.
    ///
    /// For `load -> modifier_a -> modifier_b -> call_indirect`, tracing from `call_indirect`
    /// returns `[modifier_b, modifier_a, load]`.
    pub(super) fn trace_indirect_call_to_load(
        &mut self,
        h: &impl HugrMut<Node = N>,
        ind_call_node: N,
    ) -> Result<Vec<N>, ModifierResolverErrors<N>> {
        let optype = h.get_optype(ind_call_node);
        // First node must be a CallIndirect node
        assert!(
            matches!(optype, OpType::CallIndirect(_)),
            "Expected CallIndirect, found {optype:?}"
        );

        // Collection of modifiers to apply.
        let modifiers = self.modifiers_mut();
        let (mut current, _) = h.single_linked_output(ind_call_node, 0).unwrap();
        let mut chain: Vec<N> = Vec::new();

        loop {
            let optype = h.get_optype(current);
            if Modifier::from_optype(optype).is_none() {
                // The only non modifier node allowed in the chain is the LoadFunction node,
                // that corresponds to the head of the chain.
                if !matches!(optype, OpType::LoadFunction(_)) {
                    // If the chain is empty, it means the call_indirect node is directly connected to a
                    // non-load node, which is invalid under modifiers.
                    if chain.is_empty() {
                        return Err(ModifierResolverErrors::unresolvable(
                            current,
                            ModifierResolverErrors::ModifierNotApplicable(
                                current,
                                optype.to_owned(),
                            )
                            .to_string(),
                            h.get_optype(ind_call_node).to_owned(),
                        ));
                    }
                    return Err(ModifierResolverErrors::MalformedChainHead(
                        current,
                        optype.to_owned(),
                    ));
                }

                chain.push(current);
                break;
            }

            // Check that the modfier node has not order edges
            if optype
                .other_input_port()
                .is_some_and(|p| h.is_linked(current, p))
                || optype
                    .other_output_port()
                    .is_some_and(|p| h.is_linked(current, p))
            {
                return Err(ModifierResolverErrors::unreachable(
                    "Modifier node has order edges. The chain is malformed.",
                ));
            }

            // Reject branching modifier chains: each modifier's function output must have one consumer.
            if h.linked_inputs(current, 0).nth(1).is_some() {
                return Err(ModifierResolverErrors::unreachable(
                    "Modifier node has multiple consumers. The chain is malformed.",
                ));
            }

            chain.push(current);
            modifiers.push(optype.as_extension_op().unwrap(), current)?;
            let (new, _) = h
                .single_linked_output(current, 0)
                .expect("Expected exactly one output connected to input port 0 of a modifier node");
            current = new;
        }
        Ok(chain)
    }

    pub(super) fn modify_indirect_call(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        ind_call_node: N,
        indir_call: &CallIndirect,
        new_dfg: &mut impl Dataflow,
    ) -> Result<(), ModifierResolverErrors<N>> {
        // If a function has no quantum effects and is not directly modified, we can skip modifying the call
        let (call_indirect_input, _) = h.single_linked_output(ind_call_node, 0).unwrap();
        if !self.signature_has_quantum_data(&indir_call.signature)
            && Modifier::from_optype(h.get_optype(call_indirect_input)).is_none()
        {
            self.add_node_no_modification(h, ind_call_node, indir_call.clone(), new_dfg)?;
            return Ok(());
        }

        // Modify the function loaded by the indirect call preserving the modifier state.
        let (load_node, modified_fn, new_load_op) =
            self.with_modifiers(self.modifiers().clone(), |this| {
                // Follow the function operand through its modifiers to the loader.
                let trace = this.trace_indirect_call_to_load(h, ind_call_node)?;
                let load_node = *trace.last().unwrap();
                // Apply the combined modifiers and build the replacement loader.
                let (modified_fn, new_load_op) = this.modify_load(h, load_node)?;
                Ok::<_, ModifierResolverErrors<N>>((load_node, modified_fn, new_load_op))
            })?;

        // Insert the modified loader into the new graph.
        let new_load_node = new_dfg.add_child_node(new_load_op);
        self.call_map_insert(modified_fn, (new_load_node, IncomingPort::from(0)));
        self.wire_state_order(
            load_node,
            h.get_optype(load_node),
            new_load_node,
            new_dfg.hugr().get_optype(new_load_node),
        )?;

        // Construct the replacement IndirectCall with additional control-array inputs and outputs.
        let mut new_ind_call = indir_call.clone();
        self.modify_signature(&mut new_ind_call.signature, false);
        let new_call_node = new_dfg.add_child_node(new_ind_call);

        // Pack the current control qubits into the arrays expected by the call.
        let mut controls = self.pack_controls(new_dfg)?;
        let offset = self.modifiers().accum_ctrl.len();
        for (i, ctrl) in controls.iter_mut().enumerate() {
            new_dfg
                .hugr_mut()
                .connect(ctrl.node(), ctrl.source(), new_call_node, i + 1);
            *ctrl = Wire::new(new_call_node, i);
        }
        *self.controls() = self.unpack_controls(new_dfg, controls)?;

        // Map the original arguments and results
        let signature = indir_call.signature();
        self.wire_node_inout(
            ind_call_node,
            new_call_node,
            (signature.input.iter().skip(1), signature.output.iter()),
            (1, 0, offset),
        )?;
        // Register the original StateOrder dependencies for the replacement call.
        self.wire_state_order(
            ind_call_node,
            h.get_optype(ind_call_node),
            new_call_node,
            new_dfg.hugr().get_optype(new_call_node),
        )?;
        // Connect the new loader to the new indirect call.
        new_dfg
            .hugr_mut()
            .connect(new_load_node, 0, new_call_node, 0);
        // This function input is wired explicitly and needs no automatic wire mapping.
        self.map_insert_none((ind_call_node, IncomingPort::from(0)).into())?;

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
    use super::super::tests::{SetUnitary, modifier_test_hugr, test_modifier_resolver};
    use super::super::*;
    use crate::TketOp;
    use crate::extension::modifier::{CONTROL_OP_ID, DAGGER_OP_ID, MODIFIER_EXTENSION};
    use hugr::{
        Hugr,
        builder::{Dataflow, DataflowSubContainer, HugrBuilder, ModuleBuilder},
        extension::prelude::qb_t,
        ops::{
            CallIndirect, ExtensionOp,
            handle::{FuncID, NodeHandle},
        },
        std_extensions::arithmetic::{
            int_ops::IntOpDef,
            int_types::{ConstInt, INT_TYPES},
        },
        std_extensions::collections::array::{ArrayOpBuilder, array_type},
        types::{Signature, Term, Type},
    };

    /// Load an identity function through one or two controlled calls, optionally
    /// sharing the loader with a plain call. Independent tags order the loader.
    fn ordered_loader_function(
        module: &mut ModuleBuilder<Hugr>,
        sharing: usize,
        modifier_order: Option<bool>,
    ) -> FuncID<true> {
        let signature = Signature::new_endo([qb_t()]);
        let callee = {
            let mut f = module
                .define_function("identity", signature.clone())
                .unwrap();
            f.set_unitary();
            let outputs = f.input_wires();
            f.finish_with_outputs(outputs).unwrap()
        };
        let mut f = module
            .define_function("ordered_loader", Signature::new_endo([qb_t(), qb_t()]))
            .unwrap();
        f.set_unitary();
        let [mut control, mut target] = f.input_wires_arr();
        let markers = std::array::from_fn::<_, 4, _>(|tag| {
            f.add_child_node(hugr::ops::Tag::new(tag, vec![Default::default(); 37]))
        });
        let loaded = f.load_func(callee.handle(), &[]).unwrap();
        // Two independent predecessors and successors exercise order fan-in/fan-out.
        for &before in &markers[..2] {
            connect_order(f.hugr_mut(), before, loaded.node());
        }
        for &after in &markers[2..] {
            connect_order(f.hugr_mut(), loaded.node(), after);
        }
        // sharing: 0 = exclusive, 1 = also a plain call, 2 = two modified calls.
        if sharing == 1 {
            target = f
                .add_dataflow_op(CallIndirect { signature }, [loaded, target])
                .unwrap()
                .out_wire(0);
        }
        for _ in 0..if sharing == 2 { 2 } else { 1 } {
            let modifier = MODIFIER_EXTENSION
                .instantiate_extension_op(
                    &CONTROL_OP_ID,
                    [Term::BoundedNat(1), [qb_t().into()].into(), [].into()],
                )
                .unwrap();
            let controlled = f.add_dataflow_op(modifier, [loaded]).unwrap();
            if let Some(incoming) = modifier_order {
                let (source, target) = if incoming {
                    (markers[0], controlled.node())
                } else {
                    (controlled.node(), markers[3])
                };
                connect_order(f.hugr_mut(), source, target);
            }
            let controls = f.add_new_array(qb_t(), [control]).unwrap();
            let [controls, result] = f
                .add_dataflow_op(
                    CallIndirect {
                        signature: Signature::new_endo([array_type(1, qb_t()), qb_t()]),
                    },
                    [controlled.out_wire(0), controls, target],
                )
                .unwrap()
                .outputs_arr();
            control = f.add_array_unpack(qb_t(), 1, controls).unwrap()[0];
            target = result;
        }
        *f.finish_with_outputs([control, target]).unwrap().handle()
    }

    fn connect_order(h: &mut Hugr, source: Node, target: Node) {
        let output = h.get_optype(source).other_output_port().unwrap();
        let input = h.get_optype(target).other_input_port().unwrap();
        h.connect(source, output, target, input);
    }

    #[rstest::rstest]
    #[case::exclusive_in_place(false, 0)]
    #[case::shared_plain_in_place(false, 1)]
    #[case::shared_modifiers_in_place(false, 2)]
    #[case::exclusive_rebuilt(true, 0)]
    #[case::shared_plain_rebuilt(true, 1)]
    #[case::shared_modifiers_rebuilt(true, 2)]
    fn loader_order_edges_survive_modifier_resolution(
        // rebuilt: false = apply_modification in place, true = modify_indirect_call in a rebuilt body.
        #[case] rebuilt: bool,
        // sharing: 0 = one modified call, 1 = modified + plain call, 2 = two modified calls.
        #[case] sharing: usize,
    ) {
        let mut h = if rebuilt {
            // An outer modifier exercises modify_indirect_call while rebuilding the body.
            let (mut h, _) = modifier_test_hugr(
                2,
                1,
                |module, _| ordered_loader_function(module, sharing, None),
                false,
            );
            let main = dfg_modify::find_module_func_by_name(&h, "main").unwrap();
            h.set_entrypoint(main);
            h
        } else {
            let mut module = ModuleBuilder::new();
            let f = ordered_loader_function(&mut module, sharing, None);
            let mut h = module.finish_hugr().unwrap();
            h.set_entrypoint(f.node());
            h
        };
        h.validate().unwrap();
        let entrypoint = h.entrypoint();
        resolve_modifier_with_entrypoints(&mut h, [entrypoint]).unwrap();
        let name = if rebuilt {
            "__modified__ordered_loader[C1]"
        } else {
            "ordered_loader"
        };
        let owner = dfg_modify::find_module_func_by_name(&h, name).unwrap();
        let markers: [Node; 4] = std::array::from_fn(|tag| {
            h.children(owner)
                .find(|&n| matches!(h.get_optype(n), OpType::Tag(t) if t.variants.len() == 37 && t.tag == tag))
                .unwrap()
        });
        let calls = h
            .children(owner)
            .filter(|&n| matches!(h.get_optype(n), OpType::CallIndirect(_)))
            .collect_vec();
        assert_eq!(calls.len(), if sharing == 0 { 1 } else { 2 });
        for call in calls {
            // Check the loader actually used by each call, rather than an unused
            // copy that happens to retain the original edges.
            let loader = h.single_linked_output(call, 0).unwrap().0;
            assert!(matches!(h.get_optype(loader), OpType::LoadFunction(_)));
            let op = h.get_optype(loader);
            let predecessors = h
                .linked_outputs(loader, op.other_input_port().unwrap())
                .map(|(n, _)| n)
                .collect::<HashSet<_>>();
            let successors = h
                .linked_inputs(loader, op.other_output_port().unwrap())
                .map(|(n, _)| n)
                .collect::<HashSet<_>>();
            assert_eq!(predecessors, HashSet::from([markers[0], markers[1]]));
            assert_eq!(successors, HashSet::from([markers[2], markers[3]]));
        }
        h.validate().unwrap();
    }

    #[rstest::rstest]
    #[case::incoming(true)]
    #[case::outgoing(false)]
    fn modifier_nodes_with_order_edges_are_rejected(#[case] incoming: bool) {
        let mut module = ModuleBuilder::new();
        let f = ordered_loader_function(&mut module, 0, Some(incoming));
        let mut h = module.finish_hugr().unwrap();
        h.set_entrypoint(f.node());
        let error = resolve_modifier_with_entrypoints(&mut h, [f.node()]).unwrap_err();
        assert!(matches!(
            error,
            ModifierResolverErrors::Unreachable { msg }
                if msg == "Modifier node has order edges. The chain is malformed."
        ));
    }

    #[test]
    fn branching_modifier_chain_is_rejected() {
        let signature = Signature::new_endo([qb_t()]);
        let dagger = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &DAGGER_OP_ID,
                [Term::new_list([qb_t()]), Term::new_list(Vec::<Type>::new())],
            )
            .unwrap();
        let mut module = ModuleBuilder::new();
        let callee = module
            .define_function("identity", signature.clone())
            .unwrap();
        let outputs = callee.input_wires();
        let callee = callee.finish_with_outputs(outputs).unwrap();
        let mut main = module.define_function("main", signature.clone()).unwrap();
        let [q] = main.input_wires_arr();
        let loaded = main.load_func(callee.handle(), &[]).unwrap();
        let modified = main
            .add_dataflow_op(dagger.clone(), [loaded])
            .unwrap()
            .out_wire(0);
        let first_call = main
            .add_dataflow_op(
                CallIndirect {
                    signature: signature.clone(),
                },
                [modified, q],
            )
            .unwrap();
        let second_modifier = main.add_dataflow_op(dagger, [modified]).unwrap();
        let second_call = main
            .add_dataflow_op(
                CallIndirect { signature },
                [second_modifier.out_wire(0), first_call.out_wire(0)],
            )
            .unwrap();
        main.finish_with_outputs(second_call.outputs()).unwrap();
        let h = module.finish_hugr().unwrap();
        h.validate().unwrap();

        // Both branches must detect the shared modifier, including when tracing
        // through another modifier before reaching it.
        for call in [first_call.node(), second_call.node()] {
            let error = ModifierResolver::new()
                .trace_indirect_call_to_load(&h, call)
                .unwrap_err();
            assert!(matches!(
                error,
                ModifierResolverErrors::Unreachable { msg }
                    if msg == "Modifier node has multiple consumers. The chain is malformed."
            ));
        }
    }

    #[test]
    fn modifier_chain_with_bad_input_head_is_rejected() {
        let signature = Signature::new_endo([qb_t()]);
        let dagger = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &DAGGER_OP_ID,
                [Term::new_list([qb_t()]), Term::new_list(Vec::<Type>::new())],
            )
            .unwrap();
        let mut module = ModuleBuilder::new();
        let mut main = module
            .define_function(
                "main",
                Signature::new([Type::new_function(signature.clone()), qb_t()], [qb_t()]),
            )
            .unwrap();
        let [function, q] = main.input_wires_arr();
        let modified = main.add_dataflow_op(dagger, [function]).unwrap();
        let call = main
            .add_dataflow_op(CallIndirect { signature }, [modified.out_wire(0), q])
            .unwrap();
        main.finish_with_outputs(call.outputs()).unwrap();
        let h = module.finish_hugr().unwrap();
        h.validate().unwrap();

        let error = ModifierResolver::new()
            .trace_indirect_call_to_load(&h, call.node())
            .unwrap_err();
        match error {
            ModifierResolverErrors::MalformedChainHead(node, optype) => {
                assert_eq!(node, function.node());
                assert_eq!(&optype, h.get_optype(function.node()));
                assert!(matches!(optype, OpType::Input(_)));
            }
            other => panic!("expected MalformedChainHead, got {other:?}"),
        }
    }

    #[test]
    fn modify_load_rejects_function_declaration() {
        let signature = Signature::new_endo([qb_t()]);
        let mut module = ModuleBuilder::new();
        let callee = module
            .declare("external", signature.clone().into())
            .unwrap();
        let mut main = module
            .define_function("main", Signature::new([], [Type::new_function(signature)]))
            .unwrap();
        let loaded = main.load_func(&callee, &[]).unwrap();
        let load_node = loaded.node();
        main.finish_with_outputs([loaded]).unwrap();
        let mut h = module.finish_hugr().unwrap();
        let declaration = h.get_optype(h.static_source(load_node).unwrap()).clone();

        let mut resolver = ModifierResolver::new();
        resolver.modifiers.dagger = true;
        let error = resolver.modify_load(&mut h, load_node).unwrap_err();

        match error {
            ModifierResolverErrors::ModifierNotApplicable(node, optype) => {
                assert_eq!(node, load_node);
                assert_eq!(optype, declaration);
            }
            other => panic!("expected ModifierNotApplicable, got {other:?}"),
        }
    }

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

    /// Resolve this HUGR, where both indirect calls in f share one LoadFunction:
    ///
    /// ```text
    /// Module
    /// ├── g
    /// ├── f:
    /// │     LoadFunction(g) ─┬────────────────> CallIndirect
    /// │                      └──> Control(2) -> CallIndirect
    /// └── main: LoadFunction(f) -> [Dagger] -> Control(1) -> CallIndirect
    /// ```
    /// The rebuilt f must use distinct loaders for g[C1] and g[C1.2], with
    /// Dagger is optional.
    #[rstest::rstest]
    #[case::controlled(false)]
    #[case::controlled_daggered(true)]
    fn shared_loader_with_local_modifier_in_modified_function(#[case] dagger: bool) {
        let (mut h, _) = modifier_test_hugr(
            4,
            1,
            |module, _| {
                let g_sig = Signature::new_endo([qb_t()]);
                let g = {
                    let mut g = module.define_function("g", g_sig.clone()).unwrap();
                    g.set_unitary();
                    let [q] = g.input_wires_arr();
                    g.finish_with_outputs([q]).unwrap()
                };

                let mut f = module
                    .define_function("f", Signature::new_endo([qb_t(), qb_t(), qb_t(), qb_t()]))
                    .unwrap();
                f.set_unitary();
                let [q1, local_control1, local_control2, q2] = f.input_wires_arr();
                let shared_g = f.load_func(g.handle(), &[]).unwrap();

                let q1 = f
                    .add_dataflow_op(CallIndirect { signature: g_sig }, [shared_g, q1])
                    .unwrap()
                    .out_wire(0);

                let (local_controls, q2) = {
                    let local_control_op = MODIFIER_EXTENSION
                        .instantiate_extension_op(
                            &CONTROL_OP_ID,
                            [
                                Term::BoundedNat(2),
                                Term::new_list([qb_t()]),
                                Term::new_list(Vec::<Type>::new()),
                            ],
                        )
                        .unwrap();
                    let controlled_g = f
                        .add_dataflow_op(local_control_op, [shared_g])
                        .unwrap()
                        .out_wire(0);
                    let control_array = f
                        .add_new_array(qb_t(), [local_control1, local_control2])
                        .unwrap();
                    let [control_array, q2] = f
                        .add_dataflow_op(
                            CallIndirect {
                                signature: Signature::new_endo([array_type(2, qb_t()), qb_t()]),
                            },
                            [controlled_g, control_array, q2],
                        )
                        .unwrap()
                        .outputs_arr();
                    let controls = f.add_array_unpack(qb_t(), 2, control_array).unwrap();
                    (controls, q2)
                };

                *f.finish_with_outputs([q1, local_controls[0], local_controls[1], q2])
                    .unwrap()
                    .handle()
            },
            dagger,
        );

        // Start at the caller so the shared loader is handled while rebuilding f.
        let main = h
            .children(h.module_root())
            .find(|&node| {
                h.get_optype(node)
                    .as_func_defn()
                    .is_some_and(|defn| defn.func_name() == "main")
            })
            .unwrap();
        h.set_entrypoint(main);
        resolve_modifier_with_entrypoints(&mut h, [main]).unwrap();

        // Find the rebuilt f and collect its indirect-call loaders and callee names.
        let suffix = if dagger { "D" } else { "" };
        let modified_f =
            dfg_modify::find_module_func_by_name(&h, &format!("__modified__f[C1{suffix}]"))
                .unwrap();
        let (loaders, mut callee_names): (Vec<_>, Vec<_>) = h
            .children(modified_f)
            .filter(|&node| matches!(h.get_optype(node), OpType::CallIndirect(_)))
            .map(|call| {
                let loader = h.single_linked_output(call, 0).unwrap().0;
                let name = h
                    .get_optype(h.static_source(loader).unwrap())
                    .as_func_defn()
                    .unwrap()
                    .func_name()
                    .clone();
                (loader, name)
            })
            .unzip();
        callee_names.sort_unstable();

        // Both branches inherit the outer control; only the second adds two local controls.
        assert_eq!(
            callee_names,
            [
                format!("__modified__g[C1.2{suffix}]"),
                format!("__modified__g[C1{suffix}]"),
            ]
        );
        // The two modified versions must be loaded by different nodes.
        assert_ne!(loaders[0], loaders[1]);
    }

    #[test]
    fn quantum_indirect_call_with_function_input_is_unresolvable() {
        let callee_sig = Signature::new_endo([qb_t()]);
        let callee_ty = Type::new_function(callee_sig.clone());
        let foo_sig = Signature::new([qb_t(), callee_ty.clone()], [qb_t()]);

        let mut module = ModuleBuilder::new();
        let foo = {
            let mut func = module.define_function("foo", foo_sig).unwrap();
            func.set_unitary();
            let [q, callee] = func.input_wires_arr();
            let q = func
                .add_dataflow_op(
                    CallIndirect {
                        signature: callee_sig.clone(),
                    },
                    [callee, q],
                )
                .unwrap()
                .out_wire(0);
            func.finish_with_outputs([q]).unwrap()
        };

        let control_op = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &CONTROL_OP_ID,
                [
                    Term::BoundedNat(1),
                    Term::new_list([qb_t()]),
                    Term::new_list([callee_ty.clone()]),
                ],
            )
            .unwrap();
        let controlled_sig = Signature::new(
            [array_type(1, qb_t()), qb_t(), callee_ty.clone()],
            [array_type(1, qb_t()), qb_t()],
        );
        let main_sig = Signature::new([callee_ty], [array_type(1, qb_t()), qb_t()]);
        let mut main = module.define_function("main", main_sig).unwrap();
        let [callee] = main.input_wires_arr();
        let foo = main.load_func(foo.handle(), &[]).unwrap();
        let controlled = main.add_dataflow_op(control_op, [foo]).unwrap().out_wire(0);
        let control = main
            .add_dataflow_op(TketOp::QAlloc, [])
            .unwrap()
            .out_wire(0);
        let q = main
            .add_dataflow_op(TketOp::QAlloc, [])
            .unwrap()
            .out_wire(0);
        let controls = main.add_new_array(qb_t(), [control]).unwrap();
        let outputs = main
            .add_dataflow_op(
                CallIndirect {
                    signature: controlled_sig,
                },
                [controlled, controls, q, callee],
            )
            .unwrap()
            .outputs();
        main.finish_with_outputs(outputs).unwrap();

        let mut h = module.finish_hugr().unwrap();
        let entrypoint = h.entrypoint();
        match resolve_modifier_with_entrypoints(&mut h, [entrypoint]) {
            Err(ModifierResolverErrors::UnResolvable { msg, .. }) => {
                assert!(
                    msg.strip_prefix("Modifier cannot be applied to the node")
                        .and_then(|node| node.strip_suffix("of type Input"))
                        .is_some()
                );
            }
            Err(err) => panic!("expected an unresolvable error, got {err:?}"),
            Ok(()) => panic!("expected modifier resolution to fail"),
        }
    }

    #[test]
    fn quantum_double_indirect_call_is_unresolvable() {
        let callee_sig = Signature::new_endo([qb_t()]);
        let inner_foo_sig = Signature::new([], [Type::new_function(callee_sig.clone())]);
        let foo_sig = Signature::new([qb_t()], [qb_t()]);

        let mut module = ModuleBuilder::new();
        let callee = {
            let func = module
                .define_function("callee", callee_sig.clone())
                .unwrap();
            let [q] = func.input_wires_arr();
            func.finish_with_outputs([q]).unwrap()
        };
        let inner_foo = {
            let mut func = module
                .define_function("inner_foo", inner_foo_sig.clone())
                .unwrap();
            let callee = func.load_func(callee.handle(), &[]).unwrap();
            func.finish_with_outputs([callee]).unwrap()
        };
        let foo = {
            let mut func = module.define_function("foo", foo_sig).unwrap();
            func.set_unitary();
            let [q] = func.input_wires_arr();
            let loaded_foo = func.load_func(inner_foo.clone().handle(), &[]).unwrap();
            let callee = func
                .add_dataflow_op(
                    CallIndirect {
                        signature: inner_foo_sig.clone(),
                    },
                    [loaded_foo],
                )
                .unwrap()
                .out_wire(0);
            let q = func
                .add_dataflow_op(
                    CallIndirect {
                        signature: callee_sig.clone(),
                    },
                    [callee, q],
                )
                .unwrap()
                .out_wire(0);
            func.finish_with_outputs([q]).unwrap()
        };

        let control_op = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &CONTROL_OP_ID,
                [
                    Term::BoundedNat(1),
                    Term::new_list([qb_t()]),
                    Term::new_list(Vec::<Type>::new()),
                ],
            )
            .unwrap();
        let controlled_sig = Signature::new(
            [array_type(1, qb_t()), qb_t()],
            [array_type(1, qb_t()), qb_t()],
        );
        let main_sig = Signature::new([], [array_type(1, qb_t()), qb_t()]);
        let mut main = module.define_function("main", main_sig).unwrap();
        let foo = main.load_func(foo.handle(), &[]).unwrap();
        let controlled = main.add_dataflow_op(control_op, [foo]).unwrap().out_wire(0);
        let control = main
            .add_dataflow_op(TketOp::QAlloc, [])
            .unwrap()
            .out_wire(0);
        let q = main
            .add_dataflow_op(TketOp::QAlloc, [])
            .unwrap()
            .out_wire(0);
        let controls = main.add_new_array(qb_t(), [control]).unwrap();
        let outputs = main
            .add_dataflow_op(
                CallIndirect {
                    signature: controlled_sig,
                },
                [controlled, controls, q],
            )
            .unwrap()
            .outputs();
        main.finish_with_outputs(outputs).unwrap();

        let mut h = module.finish_hugr().unwrap();
        let entrypoint = h.entrypoint();
        match resolve_modifier_with_entrypoints(&mut h, [entrypoint]) {
            Err(ModifierResolverErrors::UnResolvable { msg, .. }) => {
                assert!(
                    msg.strip_prefix("Modifier cannot be applied to the node")
                        .and_then(|node| node.strip_suffix("of type CallIndirect"))
                        .is_some()
                );
            }
            Err(err) => panic!("expected an unresolvable error, got {err:?}"),
            Ok(()) => panic!("expected modifier resolution to fail"),
        }
    }
}
