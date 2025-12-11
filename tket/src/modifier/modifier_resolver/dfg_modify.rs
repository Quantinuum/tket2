//! Modifier for dataflow blocks.
use std::{
    collections::{HashMap, VecDeque},
    iter, mem,
};

use hugr::{
    HugrView, IncomingPort, Node, OutgoingPort, PortIndex, Wire,
    builder::{
        ConditionalBuilder, Container, DFGBuilder, Dataflow, FunctionBuilder, SubContainer,
        TailLoopBuilder,
    },
    core::HugrNode,
    extension::prelude::qb_t,
    hugr::hugrmut::HugrMut,
    ops::{Call, Conditional, DFG, DataflowBlock, DataflowOpTrait, OpType, TailLoop},
    std_extensions::collections::array::ArrayOpBuilder,
    types::{FuncTypeBase, Signature, TypeArg, TypeRow},
};
use hugr_core::hugr::internal::PortgraphNodeMap;
use petgraph::visit::{Topo, Walker};

use super::{DirWire, ModifierFlags, ModifierResolver, ModifierResolverErrors, PortExt};

impl<N: HugrNode> ModifierResolver<N> {
    /// Modifies the body of a dataflow graph.
    /// We use the topological order of the circuit.
    pub(super) fn modify_dfg_body(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        n: N,
        new_dfg: &mut impl Dataflow,
    ) -> Result<(), ModifierResolverErrors<N>> {
        let mut corresp_map = HashMap::new();
        let mut controls = self.init_control_from_input(h, n, new_dfg)?;
        mem::swap(self.corresp_map(), &mut corresp_map);
        mem::swap(self.controls(), &mut controls);

        // Modify the input/output nodes beforehand.
        self.modify_in_out_node(h, n, new_dfg)?;
        // Modify the children nodes.
        self.modify_dfg_children(h, n, new_dfg)?;

        self.wire_control_to_output(h, n, new_dfg)?;
        self.connect_all(h, new_dfg, n)?;
        mem::swap(self.controls(), &mut controls);
        mem::swap(self.corresp_map(), &mut corresp_map);

        Ok(())
    }

    fn modify_dfg_children(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        n: N,
        new_dfg: &mut impl Dataflow,
    ) -> Result<(), ModifierResolverErrors<N>> {
        let mut worklist = VecDeque::new();
        // This block is needed to appease the borrow checker.
        {
            let (region_graph, node_map) = h.region_portgraph(n);
            let mut topo: Vec<_> = Topo::new(&region_graph).iter(&region_graph).collect();
            // Reverse the topological order if dagger is applied.
            if self.modifiers.dagger {
                topo.reverse();
            }
            for old_n_id in topo {
                worklist.push_back(node_map.from_portgraph(old_n_id));
            }
        }
        self.with_worklist(worklist, |this| {
            while let Some(old_n) = this.worklist().pop_front() {
                this.modify_op(h, old_n, new_dfg)?;
            }
            Ok::<(), ModifierResolverErrors<N>>(())
        })
    }

    /// Modifies the I/O nodes of a dataflow graph.
    /// These are handled separately from the other nodes since the place of control qubits
    /// may differ depending on the type of the dataflow graph.
    fn modify_in_out_node(
        &mut self,
        h: &impl HugrMut<Node = N>,
        n: N,
        new_dfg: &mut impl Dataflow,
    ) -> Result<(), ModifierResolverErrors<N>> {
        let [old_in, old_out] = h.get_io(n).unwrap();
        let [new_in, new_out] = new_dfg.io();
        let optype = h.get_optype(n);
        match optype {
            OpType::FuncDefn(_) | OpType::DFG(_) => {
                let FuncTypeBase { input, output } = match optype {
                    OpType::FuncDefn(fndefn) => fndefn.signature().body(),
                    OpType::DFG(dfg) => &dfg.signature(),
                    _ => unreachable!(),
                };
                let offset = if matches!(optype, OpType::FuncDefn(_)) {
                    self.modifiers.accum_ctrl.len()
                } else {
                    self.control_num()
                };

                // Wire the inputs and outputs
                // Note that the local variable `old_in` is the input node of the old DFG,
                // which we wire output wires from, so the name does not match the argument of `wire_inout`.
                self.wire_inout(
                    (old_out, old_in),
                    (new_out, new_in),
                    (output.iter(), input.iter()),
                    (0, 0, offset),
                )?;
            }
            OpType::TailLoop(tail_loop) => {
                let just_input_num = tail_loop.just_inputs.len();
                let offset = self.control_num();
                for port in h.node_outputs(old_in) {
                    let new_port = if port.index() < just_input_num {
                        port
                    } else {
                        port.shift(offset)
                    };
                    self.map_insert((old_in, port).into(), DirWire::from((new_in, new_port)))?;
                }
                for port in h.node_inputs(old_out) {
                    let new_port = if port.index() == 0 {
                        port
                    } else {
                        port.shift(offset)
                    };
                    self.map_insert((old_out, port).into(), DirWire::from((new_out, new_port)))?
                }
            }
            OpType::DataflowBlock(dfb) => {
                let DataflowBlock {
                    inputs: input,
                    other_outputs: output,
                    sum_rows: _sum_rows,
                } = dfb;
                let offset = self.control_num();

                // The wire for sum_rows always corresponds directly.
                // Therefore, this wire is handled separately.
                self.map_insert(
                    (old_out, IncomingPort::from(0)).into(),
                    (new_out, IncomingPort::from(0)).into(),
                )?;
                self.wire_inout(
                    (old_out, old_in),
                    (new_out, new_in),
                    (output.iter(), input.iter()),
                    (1, 0, offset),
                )?;
            }
            OpType::Case(_) => {
                return Err(ModifierResolverErrors::unreachable(
                    "IO of Case node has to be modified directly while modifying Conditional."
                        .to_string(),
                ));
            }
            optype => {
                return Err(ModifierResolverErrors::unreachable(format!(
                    "Cannot modify the IO of the node with OpType: {}",
                    optype
                )));
            }
        }

        Ok(())
    }

    /// Initializes control qubits from the input wires of the dataflow graph.
    fn init_control_from_input(
        &mut self,
        h: &impl HugrMut<Node = N>,
        n: N,
        new_dfg: &mut impl Dataflow,
    ) -> Result<Vec<Wire>, ModifierResolverErrors<N>> {
        let controls = match h.get_optype(n) {
            OpType::FuncDefn(_fndefn) => {
                self.unpack_controls(new_dfg, new_dfg.input_wires())?
            }
            OpType::DFG(_) | OpType::DataflowBlock(_) => new_dfg.input_wires().take(self.control_num()).collect(),
            OpType::TailLoop(tail_loop) => {
                let just_input_num = tail_loop.just_inputs.len();
                new_dfg
                    .input_wires()
                    .skip(just_input_num)
                    .take(self.control_num())
                    .collect()
            }
            OpType::Case(_) => return Err(ModifierResolverErrors::unreachable(
                "Control qubits of Case node have to be initialized directly while modifying Conditional."
                    .to_string(),
            )),
            optype => {
                return Err(ModifierResolverErrors::unreachable(format!(
                    "Cannot set control qubit of the node with OpType: {}",
                    optype
                )));
            }
        };
        Ok(controls)
    }

    /// Unpacks the given control qubits from arrays according to the combined modifier.
    pub(super) fn unpack_controls(
        &self,
        new_dfg: &mut impl Dataflow,
        controls_arr: impl IntoIterator<Item = Wire>,
    ) -> Result<Vec<Wire>, ModifierResolverErrors<N>> {
        let mut controls = Vec::new();
        let mut controls_arr = controls_arr.into_iter();
        for size in self.modifiers().accum_ctrl.iter() {
            let ctrl_arr = controls_arr.next().unwrap();
            controls.extend(new_dfg.add_array_unpack(qb_t(), *size as u64, ctrl_arr)?);
        }
        Ok(controls)
    }

    /// Wires the control qubits to the output node of the dataflow graph.
    fn wire_control_to_output(
        &mut self,
        h: &impl HugrMut<Node = N>,
        n: N,
        new_dfg: &mut impl Dataflow,
    ) -> Result<(), ModifierResolverErrors<N>> {
        let out_node = new_dfg.io()[1];
        // let modifiers = self.modifiers();
        let controls = self.controls_ref();

        match h.get_optype(n) {
            OpType::FuncDefn(_) => {
                let new_wires = self.pack_controls(new_dfg)?;
                for (index, wire) in new_wires.into_iter().enumerate() {
                    new_dfg
                        .hugr_mut()
                        .connect(wire.node(), wire.source(), out_node, index);
                }
            }
            OpType::DFG(_) | OpType::Case(_) => {
                for (i, ctrl) in controls.iter().enumerate() {
                    new_dfg
                        .hugr_mut()
                        .connect(ctrl.node(), ctrl.source(), out_node, i);
                }
            }
            OpType::TailLoop(_) | OpType::DataflowBlock(_) => {
                for (i, ctrl) in controls.iter().enumerate() {
                    new_dfg
                        .hugr_mut()
                        .connect(ctrl.node(), ctrl.source(), out_node, i + 1);
                }
            }
            optype => {
                return Err(ModifierResolverErrors::unreachable(format!(
                    "Cannot wire outputs of control qubit in the node of OpType: {}",
                    optype
                )));
            }
        }
        Ok(())
    }

    /// Packs the control qubits `self.controls()` into arrays according to the combined modifier.
    pub(super) fn pack_controls(
        &self,
        new_dfg: &mut impl Dataflow,
    ) -> Result<Vec<Wire>, ModifierResolverErrors<N>> {
        let controls = self.controls_ref();
        let mut v = Vec::new();
        let mut offset = 0;
        for size in self.modifiers().accum_ctrl.iter() {
            let wire =
                new_dfg.add_new_array(qb_t(), controls[offset..offset + size].iter().cloned())?;
            offset += size;
            v.push(wire);
        }
        Ok(v)
    }

    /// Modifies a function if necessary.
    /// When unitary flags satisfies the current modifier, the function needs to be modified.
    /// If not, we don't know whether the function needs modification or not.
    /// e.g. A polymorphic function that converts array kinds needs no modification if
    /// it is instantiated with `array[int, n]`, but needs modification if instantiated with
    /// `array[qubit, n]`.
    ///
    /// Since we want to avoid unnecessary modification,
    /// we implement some logic to find an evident reason that modification is not needed.
    // TODO: Add more logic so that we can recognize more cases where no modification is needed.
    // It's better to change the behavior depending on the modifier.
    // e.g. if only power, do nothing
    //      if only control, just wrap with controls (IO do not need to match)
    //      if only dagger, just check signature
    //
    // Also, it may be better to check with the usage (how it is instantiated).
    pub fn modify_fn_if_needed(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        func: N,
        signature: &Signature,
    ) -> Result<Option<N>, ModifierResolverErrors<N>> {
        let satisfies = ModifierFlags::from_metadata(h, func)
            .is_some_and(|flags| flags.satisfies(&self.modifiers));
        if !satisfies {
            let in_out_match = signature.input == signature.output;
            if in_out_match {
                // If the flag is not set and the signature does not show an evident problem, skip the modification.
                return Ok(None);
            }
        }
        Ok(Some(self.modify_fn(h, func)?))
    }

    /// Generates a new function modified by the combined modifier.
    pub fn modify_fn(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        func: N,
    ) -> Result<N, ModifierResolverErrors<N>> {
        let old_call_map = mem::take(self.call_map());

        // Old function definition
        let OpType::FuncDefn(old_fn_defn) = h.get_optype(func) else {
            return Err(ModifierResolverErrors::unreachable(format!(
                "Cannot modify a non-function node. {}",
                h.get_optype(func)
            )));
        };
        let mut poly_signature = old_fn_defn.signature().clone();
        self.modify_signature(poly_signature.body_mut(), false);

        let mut new_fn = FunctionBuilder::new(
            format!("__modified__{}", old_fn_defn.func_name()),
            poly_signature,
        )
        .unwrap();

        self.modify_dfg_body(h, func, &mut new_fn)?;

        // Connect the global wires
        let call_map = mem::replace(self.call_map(), old_call_map);
        let insertion_result = h.insert_from_view(h.module_root(), new_fn.hugr());
        let new_call_map = update_call_map(&call_map, &insertion_result.node_map);
        for (old_in, (new_n, new_port)) in new_call_map.into_iter() {
            h.connect(old_in, 0, new_n, new_port);
        }

        // set unitarity metadata
        let new_function_node = insertion_result.inserted_entrypoint;
        ModifierFlags::from_combined(self.modifiers())
            .or(&ModifierFlags::from_metadata(h, func))
            .set_metadata(h, new_function_node);

        Ok(new_function_node)
    }

    /// Generates a new function that does not essentially modify the function itself
    /// but changes the signature to match the modified calls.
    /// The generated function just calls the original function.
    pub(super) fn wrap_fn_with_controls(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        func: N,
        type_args: &[TypeArg],
    ) -> Result<N, ModifierResolverErrors<N>> {
        if self.control_num() == 0 {
            return Ok(func);
        }
        let optype = h.get_optype(func);
        let Some(fn_defn) = optype.as_func_defn() else {
            return Err(ModifierResolverErrors::unreachable(format!(
                "Cannot modify a non-function node. {}",
                optype
            )));
        };

        let mut poly_sig = fn_defn.signature().clone();
        self.modify_signature(poly_sig.body_mut(), false);
        let instantiate = poly_sig
            .instantiate(type_args)
            .map_err(|e| ModifierResolverErrors::BuildError(e.into()))?;

        let offset = self.modifiers.accum_ctrl.len();

        // make a wrapper function with a single Call node
        let mut builder =
            FunctionBuilder::new(format!("__modified__{}", fn_defn.func_name()), instantiate)?;
        let [in_node, out_node] = builder.io();
        let call = Call::try_new(poly_sig, type_args.to_owned())
            .map_err(|e| ModifierResolverErrors::BuildError(e.into()))?;
        let call_port = call.called_function_port();
        let call_node = builder.add_child_node(call);

        // connect wires
        for i in 0..offset {
            builder.hugr_mut().connect(in_node, i, out_node, i);
        }
        for i in 0..builder.hugr().num_inputs(call_node) {
            builder
                .hugr_mut()
                .connect(in_node, i + offset, call_node, i);
        }
        for i in 0..builder.hugr().num_outputs(call_node) {
            builder
                .hugr_mut()
                .connect(call_node, i, out_node, i + offset);
        }

        let insertion_result = h.insert_from_view(h.module_root(), builder.hugr());
        let call_node = insertion_result.node_map[&call_node];
        h.connect(func, 0, call_node, call_port);
        let dummy_fn_node = insertion_result.inserted_entrypoint;

        Ok(dummy_fn_node)
    }

    /// Inserts a sub DFG into the given parent DFG, updating the call map accordingly.
    pub(super) fn insert_sub_dfg(
        &mut self,
        parent_dfg: &mut impl Container,
        builder: impl Container,
    ) -> Result<Node, ModifierResolverErrors<N>> {
        let insertion_result = parent_dfg.add_hugr_view(builder.hugr());

        let insertion_correspondence = insertion_result.node_map;
        let new_call_map = update_call_map(self.call_map(), &insertion_correspondence);
        *self.call_map() = new_call_map;

        Ok(insertion_result.inserted_entrypoint)
    }

    pub(super) fn modify_dfg(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        n: N,
        dfg: &DFG,
        parent_dfg: &mut impl Container,
    ) -> Result<(), ModifierResolverErrors<N>> {
        let mut signature = dfg.signature.clone();
        // Build a new DFG with modified body.
        self.modify_signature(&mut signature, true);
        let mut builder = DFGBuilder::new(signature.clone()).unwrap();
        self.modify_dfg_body(h, n, &mut builder)?;
        let new_dfg = self.insert_sub_dfg(parent_dfg, builder)?;

        // connect the controls and register the IOs
        for (i, c) in self.controls().iter_mut().enumerate() {
            parent_dfg
                .hugr_mut()
                .connect(c.node(), c.source(), new_dfg, i);
            *c = Wire::new(new_dfg, i);
        }
        let offset = self.control_num();
        self.wire_node_inout(
            n,
            new_dfg,
            (signature.input.iter(), signature.output.iter()),
            (0, 0, offset),
        )?;

        Ok(())
    }

    pub(super) fn modify_tail_loop(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        n: N,
        tail_loop: &TailLoop,
        new_dfg: &mut impl Container,
    ) -> Result<(), ModifierResolverErrors<N>> {
        let just_input_num = tail_loop.just_inputs.len();
        let just_output_num = tail_loop.just_outputs.len();

        // TailLoop cannot be daggered as long as it is not the one generated from Power modifier.
        // Every TailLoop that is generated from Power cannot have `just_outputs`.
        if self.modifiers.dagger && !tail_loop.just_outputs.is_empty() {
            let optype = h.get_optype(n);
            return Err(ModifierResolverErrors::unresolvable(
                n,
                "tail loop with outputs cannot be daggered.".to_string(),
                optype.clone(),
            ));
        }
        // TODO: Handle the case when TailLoop is generated from `Power` modifier.
        // Currently, it is not implemented.
        if self.modifiers.dagger {
            unimplemented!("Dagger for TailLoop is not supported yet.");
        }

        // Build a new TailLoop with modified body.
        let mut builder = TailLoopBuilder::new(
            tail_loop.just_inputs.clone(),
            tail_loop
                .rest
                .extend(iter::repeat_n(&qb_t(), self.control_num())),
            tail_loop.just_outputs.clone(),
        )?;
        self.modify_dfg_body(h, n, &mut builder)?;
        let new_tail_loop = self.insert_sub_dfg(new_dfg, builder)?;

        // connect the controls and register IOs
        let offset = self.control_num();
        for (i, ctrl) in self.controls().iter_mut().enumerate() {
            new_dfg.hugr_mut().connect(
                ctrl.node(),
                ctrl.source(),
                new_tail_loop,
                i + just_input_num,
            );
            *ctrl = Wire::new(new_tail_loop, i + just_output_num);
        }
        for port in h.node_inputs(n) {
            let new_port = if port.index() < just_input_num {
                port
            } else {
                port.shift(offset)
            };
            self.map_insert((n, port).into(), (new_tail_loop, new_port).into())?;
        }
        for port in h.node_outputs(n) {
            let new_port = if port.index() < just_output_num {
                port
            } else {
                port.shift(offset)
            };
            self.map_insert((n, port).into(), (new_tail_loop, new_port).into())?
        }

        Ok(())
    }

    pub(super) fn modify_conditional(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        n: N,
        conditional: &Conditional,
        new_dfg: &mut impl Container,
    ) -> Result<(), ModifierResolverErrors<N>> {
        let offset = self.control_num();

        // Build a new Conditional with modified body.
        let control_types: TypeRow = iter::repeat_n(qb_t(), offset).collect::<Vec<_>>().into();
        let mut builder = ConditionalBuilder::new(
            conditional.sum_rows.clone(),
            control_types.extend(conditional.other_inputs.iter()),
            control_types.extend(conditional.outputs.iter()),
        )?;

        // remember the current control qubits
        let controls = self.controls().clone();

        let iter: Vec<_> = h.children(n).enumerate().collect();
        for (i, case_node) in iter {
            let tag_wire_num = conditional.sum_rows[i].len();
            let mut case_builder = builder.case_builder(i).unwrap();

            // Set the controls and corresp_map
            let mut corresp_map = HashMap::new();
            let controls = case_builder
                .input_wires()
                .skip(tag_wire_num)
                .take(offset)
                .collect();
            mem::swap(self.corresp_map(), &mut corresp_map);
            *self.controls() = controls;

            // Modify the IOs
            let [old_in, old_out] = h.get_io(case_node).unwrap();
            let [new_in, new_out] = case_builder.io();

            // Modify the input/output nodes beforehand.
            for i in 0..tag_wire_num {
                let old_port = OutgoingPort::from(i);
                let new_port = OutgoingPort::from(i);
                self.map_insert((old_in, old_port).into(), (new_in, new_port).into())?
            }
            self.wire_inout(
                (old_out, old_in),
                (new_out, new_in),
                (conditional.outputs.iter(), conditional.other_inputs.iter()),
                (0, tag_wire_num, offset),
            )?;

            // Modify the children.
            self.modify_dfg_children(h, case_node, &mut case_builder)?;

            // Set the controls and corresp_map back
            self.wire_control_to_output(h, case_node, &mut case_builder)?;
            self.connect_all(h, &mut case_builder, case_node)?;
            mem::swap(self.corresp_map(), &mut corresp_map);

            // This actually does nothing as far as I know.
            let _ = case_builder
                .finish_sub_container()
                .map_err(|e| ModifierResolverErrors::BuildError(e))?;
        }

        // insert the conditional
        let new_conditional = self.insert_sub_dfg(new_dfg, builder)?;

        // connect the controls and register the IOs
        *self.controls() = Vec::new();
        for (i, ctrl) in controls.into_iter().enumerate() {
            new_dfg
                .hugr_mut()
                .connect(ctrl.node(), ctrl.source(), new_conditional, i + 1);
            self.controls().push(Wire::new(new_conditional, i));
        }
        self.map_insert(
            (n, IncomingPort::from(0)).into(),
            (new_conditional, IncomingPort::from(0)).into(),
        )?;
        self.wire_node_inout(
            n,
            new_conditional,
            (conditional.other_inputs.iter(), conditional.outputs.iter()),
            (1, 0, offset),
        )?;

        Ok(())
    }
}

/// composition of two call maps
fn update_call_map<A, B, C, D>(f: &HashMap<A, (B, C)>, g: &HashMap<B, D>) -> HashMap<A, (D, C)>
where
    A: Clone + Eq + std::hash::Hash,
    B: Clone + Eq + std::hash::Hash,
    C: Clone,
    D: Clone,
{
    f.iter()
        .filter_map(|(a, (b, c))| g.get(b).map(|d| (a.clone(), (d.clone(), c.clone()))))
        .collect()
}

#[cfg(test)]
mod test {
    use super::super::tests::{SetUnitary, test_modifier_resolver};
    use super::super::*;
    use crate::TketOp;
    use crate::extension::{
        modifier::{CONTROL_OP_ID, DAGGER_OP_ID, MODIFIER_EXTENSION},
        rotation::{ConstRotation, rotation_type},
    };
    use cool_asserts::assert_matches;
    use hugr::{
        Hugr,
        builder::{Dataflow, DataflowSubContainer, HugrBuilder, ModuleBuilder, SubContainer},
        extension::prelude::qb_t,
        ops::{CallIndirect, ExtensionOp, handle::FuncID},
        std_extensions::collections::array::{ArrayOpBuilder, array_type},
        type_row,
        types::{Signature, Term},
    };

    fn foo_dfg(module: &mut ModuleBuilder<Hugr>, t_num: usize) -> FuncID<true> {
        let foo_sig = Signature::new_endo(iter::repeat_n(qb_t(), t_num).collect::<Vec<_>>());
        let mut func = module.define_function("foo", foo_sig.clone()).unwrap();
        func.set_unitary();
        let mut inputs: Vec<_> = func.input_wires().collect();
        inputs[0] = func
            .add_dataflow_op(TketOp::X, vec![inputs[0]])
            .unwrap()
            .out_wire(0);
        let targ1 = &mut inputs[0];
        *targ1 = {
            let dfg = func.dfg_builder_endo(vec![(qb_t(), *targ1)]).unwrap();
            let inputs = dfg.input_wires();
            dfg.finish_with_outputs(inputs).unwrap()
        }
        .out_wire(0);
        *func.finish_with_outputs(inputs).unwrap().handle()
    }

    fn foo_tail_loop(module: &mut ModuleBuilder<Hugr>, t_num: usize) -> FuncID<true> {
        let foo_sig = Signature::new_endo(iter::repeat_n(qb_t(), t_num).collect::<Vec<_>>());
        let mut func = module.define_function("foo", foo_sig.clone()).unwrap();
        func.set_unitary();
        let theta = {
            let angle = ConstRotation::new(0.5).unwrap();
            func.add_load_value(angle)
        };
        let target_type = iter::repeat_n(qb_t(), t_num).collect::<Vec<_>>();
        let loop_inputs: Vec<(_, _)> = target_type
            .iter()
            .cloned()
            .zip(func.input_wires())
            .collect();
        let tail_loop = {
            let mut builder = func
                .tail_loop_builder([(rotation_type(), theta)], loop_inputs, type_row![])
                .unwrap();
            let mut inputs = builder.input_wires();
            let angle = inputs.next().unwrap();
            let qubit = inputs.next().unwrap();
            let rotated = builder
                .add_dataflow_op(TketOp::Rx, vec![qubit, angle])
                .unwrap()
                .out_wire(0);
            let sum_just_input = builder
                .make_sum(0, vec![rotation_type().into(), type_row![]], vec![angle])
                .unwrap();
            let outputs = [rotated].into_iter().chain(inputs);
            builder
                .finish_with_outputs(sum_just_input, outputs)
                .unwrap()
        };
        let outputs = tail_loop.outputs();
        *func.finish_with_outputs(outputs).unwrap().handle()
    }

    fn foo_conditional(module: &mut ModuleBuilder<Hugr>, t_num: usize) -> FuncID<true> {
        let foo_sig = Signature::new_endo(iter::repeat_n(qb_t(), t_num).collect::<Vec<_>>());
        let mut func = module.define_function("foo", foo_sig.clone()).unwrap();
        func.set_unitary();
        let theta = {
            let angle = ConstRotation::new(0.5).unwrap();
            func.add_load_value(angle)
        };
        let mut inputs = func.input_wires().collect::<Vec<_>>();
        inputs[0] = func
            .add_dataflow_op(TketOp::X, vec![inputs[0]])
            .unwrap()
            .out_wire(0);
        let sum_bool = func
            .make_sum(1, [type_row![], vec![rotation_type()].into()], vec![theta])
            .unwrap();
        let mut cond_builder = func
            .conditional_builder(
                ([type_row![], vec![rotation_type()].into()], sum_bool),
                iter::repeat_n(qb_t(), t_num).zip(inputs),
                iter::repeat_n(qb_t(), t_num).collect::<Vec<_>>().into(),
            )
            .unwrap();
        let _case1 = {
            let case = cond_builder.case_builder(0).unwrap();
            let inputs = case.input_wires();
            let outputs = [].into_iter().chain(inputs);
            case.finish_with_outputs(outputs).unwrap()
        };
        let _case2 = {
            let mut case = cond_builder.case_builder(1).unwrap();
            let mut inputs = case.input_wires();
            let theta = inputs.next().unwrap();
            let mut q = inputs.next().unwrap();
            q = case
                .add_dataflow_op(TketOp::Rz, vec![q, theta])
                .unwrap()
                .out_wire(0);
            let outputs = [q].into_iter().chain(inputs);
            case.finish_with_outputs(outputs).unwrap()
        };
        let conditional = cond_builder.finish_sub_container().unwrap();
        let outputs = conditional.outputs();
        *func.finish_with_outputs(outputs).unwrap().handle()
    }

    fn foo_cfg(module: &mut ModuleBuilder<Hugr>, t_num: usize) -> FuncID<true> {
        let foo_sig = Signature::new_endo(iter::repeat_n(qb_t(), t_num).collect::<Vec<_>>());
        let mut func = module.define_function("foo", foo_sig.clone()).unwrap();
        func.set_unitary();
        let mut inputs: Vec<_> = func.input_wires().collect();
        inputs[0] = func
            .add_dataflow_op(TketOp::X, vec![inputs[0]])
            .unwrap()
            .out_wire(0);

        let cfg = {
            let mut cfg = func
                .cfg_builder(vec![(qb_t(), inputs[0])], qb_t().into())
                .unwrap();
            let bb = {
                let mut bb = cfg.entry_builder(vec![type_row![]], qb_t().into()).unwrap();
                let mut inputs: Vec<_> = bb.input_wires().collect();
                inputs[0] = bb
                    .add_dataflow_op(TketOp::X, vec![inputs[0]])
                    .unwrap()
                    .out_wire(0);
                let tag = bb.make_sum(0, [type_row![]], []).unwrap();
                bb.finish_with_outputs(tag, inputs).unwrap()
            };
            let exit = cfg.exit_block();
            cfg.branch(&bb, 0, &exit).unwrap();
            cfg.finish_sub_container().unwrap()
        };
        inputs[0] = cfg.outputs().next().unwrap();

        *func.finish_with_outputs(inputs).unwrap().handle()
    }

    #[rstest::rstest]
    #[case::dfg(1, 2, foo_dfg, false)]
    #[case::dfg_dagger(1, 2, foo_dfg, true)]
    #[case::tail_loop(1, 1, foo_tail_loop, false)]
    #[case::conditional(1, 1, foo_conditional, false)]
    #[case::conditional_dagger(1, 1, foo_conditional, true)]
    #[case::cfg(1, 1, foo_cfg, false)]
    #[case::cfg_dagger(1, 1, foo_cfg, true)]
    pub fn test_dfg_modify(
        #[case] t_num: usize,
        #[case] c_num: u64,
        #[case] foo: fn(&mut ModuleBuilder<Hugr>, usize) -> FuncID<true>,
        #[case] dagger: bool,
    ) {
        test_modifier_resolver(t_num, c_num, foo, dagger);
    }

    // This test checks the case where a modifier is not chained but duplicated.
    // e.g.
    // ```
    // modified1 = control(1, foo)
    // modified2 = dagger(modified1)
    // call(modified1);
    // call(modified2);
    // ```
    // Such a case is not supported in the current implementation so it fails,
    // but this not supposed to happen in a Guppy compilation flow.
    #[ignore = "Modifier chain do not support branching."]
    #[rstest::rstest]
    #[case(1, 1, foo_dfg)]
    fn test_modified_dupl(
        #[case] t_num: usize,
        #[case] c_num: u64,
        #[case] foo: fn(&mut ModuleBuilder<Hugr>, usize) -> FuncID<true>,
    ) {
        let mut module = ModuleBuilder::new();
        let call_sig = Signature::new_endo(
            [array_type(c_num, qb_t())]
                .into_iter()
                .chain(iter::repeat_n(qb_t(), t_num))
                .collect::<Vec<_>>(),
        );
        let main_sig = Signature::new(
            type_row![],
            vec![array_type(c_num, qb_t())]
                .into_iter()
                .chain(iter::repeat_n(qb_t(), t_num))
                .collect::<Vec<_>>(),
        );

        let dagger_op: ExtensionOp = {
            MODIFIER_EXTENSION
                .instantiate_extension_op(
                    &DAGGER_OP_ID,
                    [
                        vec![array_type(c_num, qb_t()).into()]
                            .into_iter()
                            .chain(iter::repeat_n(qb_t().into(), t_num))
                            .collect::<Vec<_>>()
                            .into(),
                        vec![].into(),
                    ],
                )
                .unwrap()
        };

        let control_op: ExtensionOp = {
            MODIFIER_EXTENSION
                .instantiate_extension_op(
                    &CONTROL_OP_ID,
                    [
                        Term::BoundedNat(c_num),
                        iter::repeat_n(qb_t().into(), t_num)
                            .collect::<Vec<_>>()
                            .into(),
                        vec![].into(),
                    ],
                )
                .unwrap()
        };

        let foo = foo(&mut module, t_num);

        let _main = {
            let mut func = module.define_function("main", main_sig).unwrap();
            let loaded = func.load_func(&foo, &[]).unwrap();
            let call1 = func
                .add_dataflow_op(control_op, vec![loaded])
                .unwrap()
                .out_wire(0);
            let call2 = func
                .add_dataflow_op(dagger_op, vec![call1])
                .unwrap()
                .out_wire(0);

            let mut controls = Vec::new();
            for _ in 0..c_num {
                controls.push(
                    func.add_dataflow_op(TketOp::QAlloc, vec![])
                        .unwrap()
                        .out_wire(0),
                );
            }

            let mut targ = Vec::new();
            for _ in 0..t_num {
                targ.push(
                    func.add_dataflow_op(TketOp::QAlloc, vec![])
                        .unwrap()
                        .out_wire(0),
                )
            }

            let control_arr = func.add_new_array(qb_t(), controls).unwrap();
            let mut outputs = func
                .add_dataflow_op(
                    CallIndirect {
                        signature: call_sig.clone(),
                    },
                    [call1, control_arr].into_iter().chain(targ.into_iter()),
                )
                .unwrap()
                .outputs();
            outputs = func
                .add_dataflow_op(
                    CallIndirect {
                        signature: call_sig,
                    },
                    [call2].into_iter().chain(outputs),
                )
                .unwrap()
                .outputs();

            func.finish_with_outputs(outputs).unwrap()
        };

        let mut h = module.finish_hugr().unwrap();
        assert_matches!(h.validate(), Ok(()));

        let entrypoint = h.entrypoint();
        resolve_modifier_with_entrypoints(&mut h, [entrypoint]).unwrap();

        assert_matches!(h.validate(), Ok(()));
    }
}
