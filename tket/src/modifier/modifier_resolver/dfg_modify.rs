//! Modifier for dataflow blocks.
use std::{
    collections::{HashMap, HashSet, VecDeque},
    iter, mem,
};

use hugr::{
    HugrView, IncomingPort, Node, OutgoingPort, PortIndex, Wire,
    builder::{
        ConditionalBuilder, Container, DFGBuilder, Dataflow, FunctionBuilder, SubContainer,
        TailLoopBuilder,
    },
    core::HugrNode,
    extension::{prelude::qb_t, simple_op::MakeExtensionOp},
    hugr::hugrmut::HugrMut,
    ops::{Conditional, DFG, DataflowBlock, DataflowOpTrait, OpType, TailLoop},
    std_extensions::collections::array::ArrayOpBuilder,
    types::{EdgeKind, FuncTypeBase, Signature, TypeRow},
};
use itertools::Itertools;
use petgraph::visit::{Topo, Walker};

use crate::{TketOp, extension::global_phase::GlobalPhase};

use super::{DirWire, ModifierResolver, ModifierResolverErrors, PortExt};

impl<N: HugrNode> ModifierResolver<N> {
    /// Modifies the body of a dataflow graph.
    /// We use the topological order of the circuit.
    pub(super) fn modify_dfg_body(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        parent_node: N,
        new_dfg: &mut impl Dataflow,
    ) -> Result<(), ModifierResolverErrors<N>> {
        let mut corresp_map = HashMap::new();
        let mut controls = self.init_control_from_input(h, parent_node, new_dfg)?;
        mem::swap(self.corresp_map(), &mut corresp_map);
        mem::swap(self.controls(), &mut controls);

        // Modify the input/output nodes beforehand.
        self.modify_in_out_node(h, parent_node, new_dfg)?;

        // Modify the children nodes.
        self.modify_dfg_children(h, parent_node, new_dfg)?;

        self.wire_control_to_output(h, parent_node, new_dfg)?;

        self.connect_all(h, new_dfg, parent_node)?;

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
            let sg = h.scheduling_graph(n);
            let mut topo: Vec<_> = Topo::new(sg.petgraph()).iter(sg.petgraph()).collect();
            // Reverse the topological order if dagger is applied.
            if self.modifiers.dagger {
                topo.reverse();
            }
            for old_n_id in topo {
                worklist.push_back(sg.pg_to_node(old_n_id));
            }
        }

        self.with_worklist(worklist, |this| {
            while let Some(working_node) = this.worklist().pop_front() {
                this.modify_op(h, working_node, new_dfg)?;
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
                let mut input = input.clone();
                if matches!(optype, OpType::FuncDefn(_)) {
                    self.modify_higher_order_input_types(&mut input, 0)?;
                } else {
                    self.modify_carried_higher_order_types_if_present(&mut input)?;
                }
                let mut output = output.clone();
                self.modify_carried_higher_order_types_if_present(&mut output)?;

                // Wire the inputs and outputs
                // Note that the local variable `old_in` is the input node of the old DFG,
                // which we wire output wires from, so the name does not match the argument of `wire_inout`.
                self.wire_inout(
                    (old_out, old_in),
                    (new_out, new_in),
                    (output.iter(), input.iter()),
                    (0, 0, offset),
                    &HashSet::new(),
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
                    inputs,
                    other_outputs: output,
                    sum_rows: _sum_rows,
                } = dfb;
                let mut input = inputs.clone();
                self.modify_carried_higher_order_types_if_present(&mut input)?;
                let mut output = output.clone();
                self.modify_carried_higher_order_types_if_present(&mut output)?;

                // The branch sum is unchanged.
                self.map_insert(
                    (old_out, IncomingPort::from(0)).into(),
                    (new_out, IncomingPort::from(0)).into(),
                )?;
                self.wire_inout(
                    (old_out, old_in),
                    (new_out, new_in),
                    (output.iter(), input.iter()),
                    (1, 0, 0),
                    &HashSet::new(),
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
            OpType::DFG(_) => new_dfg.input_wires().take(self.control_num()).collect(),
            OpType::DataflowBlock(dfb) => new_dfg
                .input_wires()
                .skip(dfb.inputs.len())
                .take(self.control_num())
                .collect(),
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
            OpType::TailLoop(_) => {
                for (i, ctrl) in controls.iter().enumerate() {
                    new_dfg
                        .hugr_mut()
                        .connect(ctrl.node(), ctrl.source(), out_node, i + 1);
                }
            }
            OpType::DataflowBlock(dfb) => {
                // Port 0 is the branch sum. Controls are threaded after block data.
                let offset = 1 + dfb.other_outputs.len();
                for (i, ctrl) in controls.iter().enumerate() {
                    new_dfg
                        .hugr_mut()
                        .connect(ctrl.node(), ctrl.source(), out_node, i + offset);
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
    ///
    /// When the function definition or a concrete call instantiation contains qubits,
    /// the function needs to be modified. The concrete signature matters for polymorphic
    /// functions whose uninstantiated definition may not reveal quantum data.
    /// force is used to indicate that the function is being called after a modifiers chain,
    /// so it must be modified even if the concrete signature does not contain qubits.
    ///
    /// NOTE: When a polymorphic function has a polymorphic input, the function is considered
    /// to have classical data (there are no quantum generic types or generic quantum operations).
    pub(crate) fn modify_fn_if_needed(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        func: N,
        concrete_signature: Option<&Signature>,
        force: bool,
    ) -> Result<Option<N>, ModifierResolverErrors<N>> {
        let OpType::FuncDefn(fn_defn) = h.get_optype(func) else {
            return Err(ModifierResolverErrors::unreachable(format!(
                "Cannot modify a non-function node. {}",
                h.get_optype(func)
            )));
        };
        let concrete_signature_has_quantum_data =
            concrete_signature.is_some_and(|signature| self.signature_has_quantum_data(signature));
        if !force
            && !concrete_signature_has_quantum_data
            && !self.signature_has_quantum_data(fn_defn.signature().body())
        {
            return Ok(None);
        }
        Ok(Some(self.modify_fn(h, func)?))
    }

    /// Generates a new function modified by the combined modifier.
    pub(crate) fn modify_fn(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        func: N,
    ) -> Result<N, ModifierResolverErrors<N>> {
        let old_call_map = mem::take(self.call_map());
        let old_dynamic_input_modifiers = mem::take(self.dynamic_input_modifiers());

        // Old function definition
        let OpType::FuncDefn(old_fn_defn) = h.get_optype(func) else {
            return Err(ModifierResolverErrors::unreachable(format!(
                "Cannot modify a non-function node. {}",
                h.get_optype(func)
            )));
        };
        let higher_order_input_modifiers = self.higher_order_input_modifiers(h, func)?;
        let old_active_function_input_modifiers = mem::replace(
            self.active_function_input_modifiers(),
            higher_order_input_modifiers.clone(),
        );
        let mut poly_signature = old_fn_defn.signature().clone();
        self.modify_signature(poly_signature.body_mut(), false);
        self.modify_higher_order_input_types(
            &mut poly_signature.body_mut().input,
            self.modifiers().accum_ctrl.len(),
        )?;

        let mut new_fn = FunctionBuilder::new(
            format!("__modified__{}", old_fn_defn.func_name()),
            poly_signature,
        )
        .unwrap();

        let modify_result = self.modify_dfg_body(h, func, &mut new_fn);
        let dynamic_input_modifiers =
            mem::replace(self.dynamic_input_modifiers(), old_dynamic_input_modifiers);
        *self.active_function_input_modifiers() = old_active_function_input_modifiers;
        modify_result?;

        // Connect the global wires
        let call_map = mem::replace(self.call_map(), old_call_map);
        let insertion_result = h.insert_from_view(h.module_root(), new_fn.hugr());
        let new_call_map = update_call_map(&call_map, &insertion_result.node_map);
        for (old_in, targets) in new_call_map.into_iter() {
            for (new_n, new_port) in targets {
                h.connect(old_in, 0, new_n, new_port);
            }
        }

        let new_function_node = insertion_result.inserted_entrypoint;
        let input_modifiers = if higher_order_input_modifiers.is_empty() {
            dynamic_input_modifiers
        } else {
            higher_order_input_modifiers
        }
        .into_iter()
        .unique()
        .collect::<Vec<_>>();
        if !input_modifiers.is_empty() {
            self.function_input_modifiers
                .insert(new_function_node, input_modifiers);
        }
        self.modified_functions.insert(func);

        Ok(new_function_node)
    }

    /// Inserts a sub DFG into the given parent DFG, updating the call map accordingly.
    pub(super) fn insert_sub_dfg(
        &mut self,
        parent_dfg: &mut impl Container,
        builder: impl Container,
    ) -> Result<Node, ModifierResolverErrors<N>> {
        // Only local function-port targets should be remapped into the parent.
        let remap_targets = self
            .call_map()
            .values()
            .flatten()
            .filter(|(node, port)| {
                builder.hugr().contains_node(*node)
                    && builder.hugr().num_inputs(*node) > port.index()
                    && matches!(
                        builder.hugr().get_optype(*node).port_kind(*port),
                        Some(EdgeKind::Function(_))
                    )
            })
            .copied()
            .collect::<HashSet<_>>();
        let insertion_result = parent_dfg.add_hugr_view(builder.hugr());

        let insertion_correspondence = insertion_result.node_map;
        let new_call_map =
            update_call_map_preserve(self.call_map(), &insertion_correspondence, &remap_targets);
        *self.call_map() = new_call_map;

        Ok(insertion_result.inserted_entrypoint)
    }

    fn copy_sub_container_no_modification(
        &mut self,
        h: &impl HugrView<Node = N>,
        n: N,
        new_dfg: &mut impl Container,
    ) -> Result<Node, ModifierResolverErrors<N>> {
        // Some containers have qubits in their signature but only pass them
        // through while doing classical work. Copying the whole subtree keeps
        // those classical dependencies intact instead of trying to dagger the
        // boundary one port at a time.
        let insertion_result = new_dfg.add_hugr_view(&h.with_entrypoint(n));

        let new_node = insertion_result.inserted_entrypoint;
        for port in h.all_node_ports(n) {
            self.map_insert(DirWire(n, port), DirWire(new_node, port))?;
        }

        Ok(new_node)
    }

    fn subtree_has_quantum_operation(&self, h: &impl HugrView<Node = N>, n: N) -> bool {
        // We need more than a type-level qubit check here: Guppy often emits
        // bounds-check conditionals whose signature carries a qubit, but whose
        // body only manipulates classical array indices and values.
        h.descendants(n)
            .chain(iter::once(n))
            .any(|node| self.node_is_quantum_operation(h, node))
    }

    fn node_is_quantum_operation(&self, h: &impl HugrView<Node = N>, n: N) -> bool {
        let optype = h.get_optype(n);
        match optype {
            OpType::Input(_)
            | OpType::Output(_)
            | OpType::CFG(_)
            | OpType::DFG(_)
            | OpType::TailLoop(_)
            | OpType::Conditional(_)
            | OpType::Case(_)
            | OpType::DataflowBlock(_)
            | OpType::FuncDefn(_)
            | OpType::FuncDecl(_)
            | OpType::Module(_) => false,
            // tket quantum gates and global phases require the normal modifier
            // logic. They are real operations, not just qubit-carrying IO.
            _ if TketOp::from_optype(optype).is_some()
                || GlobalPhase::from_optype(optype).is_some() =>
            {
                true
            }
            // Unknown operations are conservative: if their signature can carry
            // qubits, treat them as quantum-sensitive so we do not silently copy
            // an operation that may need dagger/control handling.
            _ => h.signature(n).is_some_and(|sig| {
                sig.input
                    .iter()
                    .chain(sig.output.iter())
                    .any(|ty| self.qubit_finder.contains_element_type(ty))
            }),
        }
    }

    pub(super) fn modify_dfg(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        n: N,
        dfg: &DFG,
        parent_dfg: &mut impl Container,
    ) -> Result<(), ModifierResolverErrors<N>> {
        // Check if the DFG input or output are carrying qubits: only DFGs with quantum effects need to be modified.
        let boundary_has_qubits = self.signature_has_quantum_data(&dfg.signature);
        if !boundary_has_qubits {
            self.copy_sub_container_no_modification(h, n, parent_dfg)?;
            return Ok(());
        }

        let mut boundary_signature = dfg.signature.clone();
        self.modify_carried_higher_order_types_if_present(&mut boundary_signature.input)?;
        self.modify_carried_higher_order_types_if_present(&mut boundary_signature.output)?;
        let mut signature = boundary_signature.clone();
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
            (
                boundary_signature.input.iter(),
                boundary_signature.output.iter(),
            ),
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

        if self.modifiers.dagger {
            let optype = h.get_optype(n);
            return Err(ModifierResolverErrors::unresolvable(
                n,
                "TailLoop cannot be daggered.".to_string(),
                optype.clone(),
            ));
        }

        // Build a new TailLoop with modified body.
        let control_types: TypeRow = iter::repeat_n(qb_t(), self.control_num())
            .collect::<Vec<_>>()
            .into();
        let mut just_inputs = tail_loop.just_inputs.clone();
        self.modify_carried_higher_order_types_if_present(&mut just_inputs)?;
        let mut rest = tail_loop.rest.clone();
        self.modify_carried_higher_order_types_if_present(&mut rest)?;
        let mut just_outputs = tail_loop.just_outputs.clone();
        self.modify_carried_higher_order_types_if_present(&mut just_outputs)?;
        let mut builder =
            TailLoopBuilder::new(just_inputs, control_types.extend(rest.iter()), just_outputs)?;
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
        // If a conditional does not have quantum operations in its body, we can safely
        // copy the whole conditional without modification.
        let has_indirect_call = h
            .descendants(n)
            .any(|node| matches!(h.get_optype(node), OpType::CallIndirect(_)));
        let has_active_higher_order_inputs = !self.active_function_input_modifiers().is_empty();
        if !self.subtree_has_quantum_operation(h, n)
            && !has_indirect_call
            && !has_active_higher_order_inputs
        {
            self.copy_sub_container_no_modification(h, n, new_dfg)?;
            return Ok(());
        }

        let offset = self.control_num();

        // Build a new Conditional with modified body.
        let control_types: TypeRow = iter::repeat_n(qb_t(), offset).collect::<Vec<_>>().into();
        let mut sum_rows = conditional.sum_rows.clone();
        for row in &mut sum_rows {
            // The selected branch payload may contain function values. If a
            // function value is later called under the active modifier, the
            // branch sum must carry the modified function type too.
            self.modify_carried_higher_order_types_if_present(row)?;
        }
        let mut other_inputs = conditional.other_inputs.clone();
        self.modify_carried_higher_order_types_if_present(&mut other_inputs)?;
        let mut outputs = conditional.outputs.clone();
        self.modify_carried_higher_order_types_if_present(&mut outputs)?;
        let mut builder = ConditionalBuilder::new(
            sum_rows.clone(),
            control_types.extend(other_inputs.iter()),
            control_types.extend(outputs.iter()),
        )?;

        // remember the current control qubits
        let controls = self.controls().clone();

        let iter: Vec<_> = h.children(n).enumerate().collect();
        for (i, case_node) in iter {
            let tag_wire_num = sum_rows[i].len();
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
                (outputs.iter(), other_inputs.iter()),
                (0, tag_wire_num, offset),
                &HashSet::new(),
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
            (other_inputs.iter(), outputs.iter()),
            (1, 0, offset),
        )?;

        Ok(())
    }
}

/// composition of two call maps
fn update_call_map<A, B, C, D>(
    call_map: &HashMap<A, Vec<(B, C)>>,
    inserted_node_map: &HashMap<B, D>,
) -> HashMap<A, Vec<(D, C)>>
where
    A: Clone + Eq + std::hash::Hash,
    B: Clone + Eq + std::hash::Hash,
    C: Clone,
    D: Clone,
{
    call_map
        .iter()
        .filter_map(|(a, targets)| {
            let targets = targets
                .iter()
                .filter_map(|(b, c)| inserted_node_map.get(b).map(|d| (d.clone(), c.clone())))
                .collect::<Vec<_>>();
            (!targets.is_empty()).then(|| (a.clone(), targets))
        })
        .collect()
}

/// Remaps call-map targets that were inserted from `inserted_node_map`, preserving existing parent targets.
fn update_call_map_preserve<A, C>(
    call_map: &HashMap<A, Vec<(Node, C)>>,
    inserted_node_map: &HashMap<Node, Node>,
    remap_targets: &HashSet<(Node, C)>,
) -> HashMap<A, Vec<(Node, C)>>
where
    A: Clone + Eq + std::hash::Hash,
    C: Clone + Eq + std::hash::Hash,
{
    call_map
        .iter()
        .map(|(caller, targets)| {
            let targets = targets
                .iter()
                .filter_map(|(target_node, port)| {
                    if remap_targets.contains(&(*target_node, port.clone())) {
                        inserted_node_map
                            .get(target_node)
                            .copied()
                            .map(|remapped_node| (remapped_node, port.clone()))
                    } else {
                        Some((*target_node, port.clone()))
                    }
                })
                .collect::<Vec<_>>();
            (caller.clone(), targets)
        })
        .collect()
}

#[cfg(test)]
mod test {
    use super::super::tests::{
        SetUnitary, modifier_test_hugr, resolved_modifier_test_hugr, test_modifier_resolver,
    };
    use super::super::*;
    use crate::TketOp;
    use crate::extension::{
        measurement::MeasurementOpBuilder,
        modifier::{CONTROL_OP_ID, DAGGER_OP_ID, MODIFIER_EXTENSION},
        rotation::{ConstRotation, rotation_type},
    };
    use cool_asserts::assert_matches;
    use hugr::{
        Hugr,
        builder::{Dataflow, DataflowSubContainer, HugrBuilder, ModuleBuilder, SubContainer},
        extension::prelude::{ConstUsize, bool_t, qb_t, usize_t},
        extension::simple_op::MakeExtensionOp,
        ops::{CallIndirect, ExtensionOp, handle::FuncID},
        std_extensions::collections::{
            array::{ArrayOp, ArrayOpBuilder, ArrayOpDef, array_type, array_type_parametric},
            borrow_array::{BArrayOp, BArrayOpBuilder, BArrayOpDef},
        },
        type_row,
        types::{PolyFuncType, Signature, Term, Type, TypeArg, TypeBound, type_param::TypeParam},
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
                .make_sum(0, vec![[rotation_type()].into(), type_row![]], vec![angle])
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
                .cfg_builder(vec![(qb_t(), inputs[0])], [qb_t()].into())
                .unwrap();
            let bb = {
                let mut bb = cfg
                    .entry_builder(vec![type_row![]], [qb_t()].into())
                    .unwrap();
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

    #[test]
    fn daggered_controlled_dfg_keeps_classical_boundary_input_forward() {
        let mut module = ModuleBuilder::new();
        let foo_sig = Signature::new([qb_t(), usize_t()], [qb_t()]);
        let foo = {
            let mut func = module.define_function("foo", foo_sig.clone()).unwrap();
            func.set_unitary();
            let mut inputs = func.input_wires();
            let q = inputs.next().unwrap();
            let index = inputs.next().unwrap();
            let dfg = {
                let mut dfg = func
                    .dfg_builder(Signature::new([qb_t(), usize_t()], [qb_t()]), [q, index])
                    .unwrap();
                let mut inputs = dfg.input_wires();
                let q = inputs.next().unwrap();
                let _index = inputs.next().unwrap();
                let q = dfg.add_dataflow_op(TketOp::X, [q]).unwrap().out_wire(0);
                dfg.finish_with_outputs([q]).unwrap()
            };
            func.finish_with_outputs(dfg.outputs()).unwrap()
        };

        let dagger_op: ExtensionOp = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &DAGGER_OP_ID,
                [vec![qb_t().into()].into(), vec![usize_t().into()].into()],
            )
            .unwrap();
        let control_op: ExtensionOp = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &CONTROL_OP_ID,
                [
                    Term::BoundedNat(1),
                    vec![qb_t().into()].into(),
                    vec![usize_t().into()].into(),
                ],
            )
            .unwrap();
        let controlled_sig = Signature::new(
            [array_type(1, qb_t()), qb_t(), usize_t()],
            [array_type(1, qb_t()), qb_t()],
        );
        {
            let mut func = module
                .define_function(
                    "main",
                    Signature::new(type_row![], [array_type(1, qb_t()), qb_t()]),
                )
                .unwrap();
            let loaded = func.load_func(foo.handle(), &[]).unwrap();
            let daggered = func
                .add_dataflow_op(dagger_op, [loaded])
                .unwrap()
                .out_wire(0);
            let controlled = func
                .add_dataflow_op(control_op, [daggered])
                .unwrap()
                .out_wire(0);
            let control = func
                .add_dataflow_op(TketOp::QAlloc, [])
                .unwrap()
                .out_wire(0);
            let target = func
                .add_dataflow_op(TketOp::QAlloc, [])
                .unwrap()
                .out_wire(0);
            let index = func.add_load_value(ConstUsize::new(1));
            let controls = func.add_new_array(qb_t(), [control]).unwrap();
            let call = func
                .add_dataflow_op(
                    CallIndirect {
                        signature: controlled_sig,
                    },
                    [controlled, controls, target, index],
                )
                .unwrap();
            func.finish_with_outputs(call.outputs()).unwrap();
        }

        let mut h = module.finish_hugr().unwrap();
        let entrypoint = h.entrypoint();
        resolve_modifier_with_entrypoints(&mut h, [entrypoint]).unwrap();
        assert_matches!(h.validate(), Ok(()));
    }

    #[test]
    /// Test that modifying a DFG representing a computation with no quantum effects (no input/output qubits)
    /// does not introduce invalid hugr.
    fn daggered_controlled_rotation_is_acyclic() {
        let mut module = ModuleBuilder::new();
        let inner_sig = Signature::new_endo([qb_t()]);
        let outer_sig = Signature::new_endo([qb_t(), qb_t()]);
        let controlled_sig = Signature::new_endo([array_type(1, qb_t()), qb_t()]);
        let main_sig = Signature::new(vec![], [qb_t(), qb_t()]);

        let control_op = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &CONTROL_OP_ID,
                [Term::BoundedNat(1), Term::new_list([qb_t()]), vec![].into()],
            )
            .unwrap();
        let dagger_op = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &DAGGER_OP_ID,
                [Term::new_list([qb_t(), qb_t()]), vec![].into()],
            )
            .unwrap();

        let inner = {
            let mut func = module.define_function("inner", inner_sig).unwrap();
            func.set_unitary();
            let q = func.input_wires().next().unwrap();
            let measurement_dfg = {
                let mut measurement_dfg = func
                    .dfg_builder(Signature::new(type_row![], [bool_t()]), [])
                    .unwrap();
                let q = measurement_dfg
                    .add_dataflow_op(TketOp::QAlloc, [])
                    .unwrap()
                    .out_wire(0);
                let measured = measurement_dfg
                    .add_dataflow_op(TketOp::MeasureFree, [q])
                    .unwrap()
                    .out_wire(0);
                let [measured_result] = measurement_dfg.add_measurement_read(measured).unwrap();
                measurement_dfg
                    .finish_with_outputs([measured_result])
                    .unwrap()
            };
            func.dfg_builder(
                Signature::new([bool_t()], type_row![]),
                [measurement_dfg.out_wire(0)],
            )
            .unwrap()
            .finish_sub_container()
            .unwrap();
            let angle_dfg = {
                let mut dfg = func
                    .dfg_builder(
                        Signature::new([bool_t()], [rotation_type()]),
                        [measurement_dfg.out_wire(0)],
                    )
                    .unwrap();
                let angle = dfg.add_load_value(ConstRotation::new(0.5).unwrap());
                dfg.finish_with_outputs([angle]).unwrap()
            };
            let angle = angle_dfg.out_wire(0);
            let q = func
                .add_dataflow_op(TketOp::Rx, [q, angle])
                .unwrap()
                .out_wire(0);
            func.finish_with_outputs([q]).unwrap()
        };
        let outer = {
            let mut func = module.define_function("outer", outer_sig.clone()).unwrap();
            func.set_unitary();
            let [control, target] = func.input_wires_arr();
            let inner = func.load_func(inner.handle(), &[]).unwrap();
            let controlled = func
                .add_dataflow_op(control_op, [inner])
                .unwrap()
                .out_wire(0);
            let control_array = func.add_new_array(qb_t(), [control]).unwrap();
            let [control_array, target] = func
                .add_dataflow_op(
                    CallIndirect {
                        signature: controlled_sig.clone(),
                    },
                    [controlled, control_array, target],
                )
                .unwrap()
                .outputs_arr();
            let control = func.add_array_unpack(qb_t(), 1, control_array).unwrap()[0];
            func.finish_with_outputs([control, target]).unwrap()
        };

        {
            let mut func = module.define_function("main", main_sig).unwrap();
            let loaded = func.load_func(outer.handle(), &[]).unwrap();
            let daggered = func
                .add_dataflow_op(dagger_op, [loaded])
                .unwrap()
                .out_wire(0);
            let control = func
                .add_dataflow_op(TketOp::QAlloc, [])
                .unwrap()
                .out_wire(0);
            let target = func
                .add_dataflow_op(TketOp::QAlloc, [])
                .unwrap()
                .out_wire(0);
            let outputs = func
                .add_dataflow_op(
                    CallIndirect {
                        signature: outer_sig,
                    },
                    [daggered, control, target],
                )
                .unwrap()
                .outputs();
            func.finish_with_outputs(outputs).unwrap();
        }

        let mut h = module.finish_hugr().unwrap();
        let entrypoint = h.entrypoint();
        resolve_modifier_with_entrypoints(&mut h, [entrypoint]).unwrap();
        assert_matches!(h.validate(), Ok(()));
    }

    // A CFG with two sequential blocks
    fn foo_cfg_two_blocks(module: &mut ModuleBuilder<Hugr>, t_num: usize) -> FuncID<true> {
        let foo_sig = Signature::new_endo(iter::repeat_n(qb_t(), t_num).collect::<Vec<_>>());
        let mut func = module.define_function("foo", foo_sig.clone()).unwrap();
        func.set_unitary();
        let mut inputs: Vec<_> = func.input_wires().collect();

        let cfg = {
            let mut cfg = func
                .cfg_builder(vec![(qb_t(), inputs[0])], [qb_t()].into())
                .unwrap();
            let entry = {
                let mut bb = cfg
                    .entry_builder(vec![type_row![]], [qb_t()].into())
                    .unwrap();
                let q = bb.input_wires().next().unwrap();
                let q = bb.add_dataflow_op(TketOp::X, vec![q]).unwrap().out_wire(0);
                let tag = bb.make_sum(0, [type_row![]], []).unwrap();
                bb.finish_with_outputs(tag, [q]).unwrap()
            };
            let second = {
                let mut bb = cfg
                    .block_builder([qb_t()].into(), vec![type_row![]], [qb_t()].into())
                    .unwrap();
                let q = bb.input_wires().next().unwrap();
                let q = bb.add_dataflow_op(TketOp::X, vec![q]).unwrap().out_wire(0);
                let tag = bb.make_sum(0, [type_row![]], []).unwrap();
                bb.finish_with_outputs(tag, [q]).unwrap()
            };
            let exit = cfg.exit_block();
            cfg.branch(&entry, 0, &second).unwrap();
            cfg.branch(&second, 0, &exit).unwrap();
            cfg.finish_sub_container().unwrap()
        };
        inputs[0] = cfg.outputs().next().unwrap();

        *func.finish_with_outputs(inputs).unwrap().handle()
    }

    // A CFG with branching into two blocks, which then join back together.
    fn foo_cfg_branching(module: &mut ModuleBuilder<Hugr>, t_num: usize) -> FuncID<true> {
        let foo_sig = Signature::new_endo(iter::repeat_n(qb_t(), t_num).collect::<Vec<_>>());
        let mut func = module.define_function("foo", foo_sig.clone()).unwrap();
        func.set_unitary();
        let mut inputs: Vec<_> = func.input_wires().collect();

        let cfg = {
            let mut cfg = func
                .cfg_builder(vec![(qb_t(), inputs[0])], [qb_t()].into())
                .unwrap();
            let entry = {
                let mut bb = cfg
                    .entry_builder(vec![type_row![], type_row![]], [qb_t()].into())
                    .unwrap();
                let q = bb.input_wires().next().unwrap();
                let tag = bb.make_sum(0, [type_row![], type_row![]], []).unwrap();
                bb.finish_with_outputs(tag, [q]).unwrap()
            };
            let left = {
                let mut bb = cfg
                    .block_builder([qb_t()].into(), vec![type_row![]], [qb_t()].into())
                    .unwrap();
                let q = bb.input_wires().next().unwrap();
                let q = bb.add_dataflow_op(TketOp::X, vec![q]).unwrap().out_wire(0);
                let tag = bb.make_sum(0, [type_row![]], []).unwrap();
                bb.finish_with_outputs(tag, [q]).unwrap()
            };
            let right = {
                let mut bb = cfg
                    .block_builder([qb_t()].into(), vec![type_row![]], [qb_t()].into())
                    .unwrap();
                let q = bb.input_wires().next().unwrap();
                let q = bb.add_dataflow_op(TketOp::X, vec![q]).unwrap().out_wire(0);
                let tag = bb.make_sum(0, [type_row![]], []).unwrap();
                bb.finish_with_outputs(tag, [q]).unwrap()
            };
            let exit = cfg.exit_block();
            cfg.branch(&entry, 0, &left).unwrap();
            cfg.branch(&entry, 1, &right).unwrap();
            cfg.branch(&left, 0, &exit).unwrap();
            cfg.branch(&right, 0, &exit).unwrap();
            cfg.finish_sub_container().unwrap()
        };
        inputs[0] = cfg.outputs().next().unwrap();

        *func.finish_with_outputs(inputs).unwrap().handle()
    }

    fn foo_cfg_loop(module: &mut ModuleBuilder<Hugr>, t_num: usize) -> FuncID<true> {
        let foo_sig = Signature::new_endo(iter::repeat_n(qb_t(), t_num).collect::<Vec<_>>());
        let mut func = module.define_function("foo", foo_sig.clone()).unwrap();
        func.set_unitary();
        let mut inputs: Vec<_> = func.input_wires().collect();

        let cfg = {
            let mut cfg = func
                .cfg_builder(vec![(qb_t(), inputs[0])], [qb_t()].into())
                .unwrap();
            let entry = {
                let mut bb = cfg
                    .entry_builder(vec![type_row![]], [qb_t()].into())
                    .unwrap();
                let q = bb.input_wires().next().unwrap();
                let tag = bb.make_sum(0, [type_row![]], []).unwrap();
                bb.finish_with_outputs(tag, [q]).unwrap()
            };
            let loop_block = {
                let mut bb = cfg
                    .block_builder(
                        [qb_t()].into(),
                        vec![type_row![], type_row![]],
                        [qb_t()].into(),
                    )
                    .unwrap();
                let q = bb.input_wires().next().unwrap();
                let q = bb.add_dataflow_op(TketOp::X, vec![q]).unwrap().out_wire(0);
                let tag = bb.make_sum(1, [type_row![], type_row![]], []).unwrap();
                bb.finish_with_outputs(tag, [q]).unwrap()
            };
            let exit = cfg.exit_block();
            cfg.branch(&entry, 0, &loop_block).unwrap();
            cfg.branch(&loop_block, 0, &loop_block).unwrap();
            cfg.branch(&loop_block, 1, &exit).unwrap();
            cfg.finish_sub_container().unwrap()
        };
        inputs[0] = cfg.outputs().next().unwrap();

        *func.finish_with_outputs(inputs).unwrap().handle()
    }

    fn foo_safe_array_ops(module: &mut ModuleBuilder<Hugr>, t_num: usize) -> FuncID<true> {
        assert_eq!(t_num, 4);

        let foo_sig = Signature::new_endo(iter::repeat_n(qb_t(), t_num).collect::<Vec<_>>());
        let mut func = module.define_function("foo", foo_sig).unwrap();
        func.set_unitary();
        let mut inputs: Vec<_> = func.input_wires().collect();

        let array = func.add_new_array(qb_t(), [inputs[0], inputs[1]]).unwrap();
        let array = func.add_array_unpack(qb_t(), 2, array).unwrap();
        inputs[0] = array[0];
        inputs[1] = array[1];

        let borrow_array = func
            .add_new_borrow_array(qb_t(), [inputs[2], inputs[3]])
            .unwrap();
        let borrow_array = func
            .add_borrow_array_unpack(qb_t(), 2, borrow_array)
            .unwrap();
        inputs[2] = borrow_array[0];
        inputs[3] = borrow_array[1];

        *func.finish_with_outputs(inputs).unwrap().handle()
    }

    fn foo_array_ops(module: &mut ModuleBuilder<Hugr>, t_num: usize) -> FuncID<true> {
        assert_eq!(t_num, 4);

        let foo_sig = Signature::new_endo(iter::repeat_n(qb_t(), t_num).collect::<Vec<_>>());
        let mut func = module.define_function("foo", foo_sig).unwrap();
        func.set_unitary();
        let mut inputs: Vec<_> = func.input_wires().collect();

        let array = func.add_new_array(qb_t(), [inputs[0], inputs[1]]).unwrap();
        let array = func.add_array_unpack(qb_t(), 2, array).unwrap();
        inputs[0] = array[0];
        inputs[1] = array[1];

        let borrow_array = func
            .add_new_borrow_array(qb_t(), [inputs[2], inputs[3]])
            .unwrap();
        let index = func.add_load_value(ConstUsize::new(1));
        let (borrow_array, borrowed) = func
            .add_borrow_array_borrow(qb_t(), 2, borrow_array, index)
            .unwrap();
        let borrowed = func
            .add_dataflow_op(TketOp::H, [borrowed])
            .unwrap()
            .out_wire(0);
        let borrow_array = func
            .add_borrow_array_return(qb_t(), 2, borrow_array, index, borrowed)
            .unwrap();
        let borrow_array = func
            .add_borrow_array_unpack(qb_t(), 2, borrow_array)
            .unwrap();
        inputs[2] = borrow_array[0];
        inputs[3] = borrow_array[1];

        *func.finish_with_outputs(inputs).unwrap().handle()
    }

    fn foo_non_quantum_array_ops(module: &mut ModuleBuilder<Hugr>, t_num: usize) -> FuncID<true> {
        assert_eq!(t_num, 1);

        let foo_sig = Signature::new_endo(iter::repeat_n(qb_t(), t_num).collect::<Vec<_>>());
        let mut func = module.define_function("foo", foo_sig).unwrap();
        func.set_unitary();
        let inputs: Vec<_> = func.input_wires().collect();

        // Classical ArrayOp and BArrayOp sequence. Under dagger this should
        // remain new_array -> unpack, not become unpack -> new_array.
        let one = func.add_load_value(ConstUsize::new(1));
        let two = func.add_load_value(ConstUsize::new(2));
        let array = func.add_new_array(usize_t(), [one, two]).unwrap();
        let unpacked = func.add_array_unpack(usize_t(), 2, array).unwrap();
        let borrow_array = func.add_new_borrow_array(usize_t(), unpacked).unwrap();
        let unpacked = func
            .add_borrow_array_unpack(usize_t(), 2, borrow_array)
            .unwrap();
        let array = func.add_new_array(usize_t(), unpacked).unwrap();
        let _ = func.add_array_unpack(usize_t(), 2, array).unwrap();

        *func.finish_with_outputs(inputs).unwrap().handle()
    }

    fn foo_nested_non_quantum_array_ops(
        module: &mut ModuleBuilder<Hugr>,
        t_num: usize,
    ) -> FuncID<true> {
        assert_eq!(t_num, 1);

        let foo_sig = Signature::new_endo(iter::repeat_n(qb_t(), t_num).collect::<Vec<_>>());
        let mut func = module.define_function("foo", foo_sig).unwrap();
        func.set_unitary();
        let inputs: Vec<_> = func.input_wires().collect();

        // Nested classical arrays should still be detected as non-quantum:
        // array[array[usize, 2], 2] contains no qubit element.
        let [one, two, three, four] = [1, 2, 3, 4].map(|i| func.add_load_value(ConstUsize::new(i)));
        let inner_ty = array_type(2, usize_t());
        let array_1 = func.add_new_array(usize_t(), [one, two]).unwrap();
        let array_2 = func.add_new_array(usize_t(), [three, four]).unwrap();
        let nested = func
            .add_new_array(inner_ty.clone(), [array_1, array_2])
            .unwrap();
        let nested = func.add_array_unpack(inner_ty.clone(), 2, nested).unwrap();
        let nested = func.add_new_borrow_array(inner_ty.clone(), nested).unwrap();
        let nested = func.add_borrow_array_unpack(inner_ty, 2, nested).unwrap();
        let _ = func.add_array_unpack(usize_t(), 2, nested[0]).unwrap();
        let _ = func.add_array_unpack(usize_t(), 2, nested[1]).unwrap();

        *func.finish_with_outputs(inputs).unwrap().handle()
    }

    fn foo_nested_quantum_array_ops(
        module: &mut ModuleBuilder<Hugr>,
        t_num: usize,
    ) -> FuncID<true> {
        assert_eq!(t_num, 5);

        let foo_sig = Signature::new_endo(iter::repeat_n(qb_t(), t_num).collect::<Vec<_>>());
        let mut func = module.define_function("foo", foo_sig).unwrap();
        func.set_unitary();
        let mut inputs: Vec<_> = func.input_wires().collect();

        // Nested quantum arrays should be treated as quantum-carrying even
        // though the top-level element type is itself an array.
        let inner_ty = array_type(2, qb_t());
        let array_1 = func.add_new_array(qb_t(), [inputs[0], inputs[1]]).unwrap();
        let array_2 = func.add_new_array(qb_t(), [inputs[2], inputs[3]]).unwrap();
        let nested = func
            .add_new_array(inner_ty.clone(), [array_1, array_2])
            .unwrap();
        let nested = func.add_array_unpack(inner_ty.clone(), 2, nested).unwrap();
        let nested = func.add_new_borrow_array(inner_ty.clone(), nested).unwrap();
        let nested = func.add_borrow_array_unpack(inner_ty, 2, nested).unwrap();
        let [array_1, array_2] = [nested[0], nested[1]];
        let array_1 = func.add_array_unpack(qb_t(), 2, array_1).unwrap();
        let array_2 = func.add_array_unpack(qb_t(), 2, array_2).unwrap();
        inputs[0] = array_1[0];
        inputs[1] = array_1[1];
        inputs[2] = array_2[0];
        inputs[3] = array_2[1];

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
    #[case::cfg_two_blocks(1, 1, foo_cfg_two_blocks, false)]
    #[case::cfg_branching(1, 1, foo_cfg_branching, false)]
    #[case::cfg_loop(1, 1, foo_cfg_loop, false)]
    #[case::array_ops(4, 3, foo_array_ops, false)]
    #[case::array_ops_dagger(4, 3, foo_array_ops, true)]
    #[case::safe_array_ops(4, 3, foo_safe_array_ops, false)]
    #[case::safe_array_ops_dagger(4, 3, foo_safe_array_ops, true)]
    #[case::nested_safe_array_ops(5, 4, foo_nested_quantum_array_ops, false)]
    #[case::nested_safe_array_ops_dagger(5, 4, foo_nested_quantum_array_ops, true)]

    fn test_dfg_modify(
        #[case] t_num: usize,
        #[case] c_num: u64,
        #[case] foo: fn(&mut ModuleBuilder<Hugr>, usize) -> FuncID<true>,
        #[case] dagger: bool,
    ) {
        test_modifier_resolver(t_num, c_num, foo, dagger);
    }

    fn assert_unresolvable_message(
        h: &mut Hugr,
        expected: &str,
    ) -> Result<(), ModifierResolverErrors> {
        let entrypoint = h.entrypoint();
        match resolve_modifier_with_entrypoints(h, [entrypoint]) {
            Err(ModifierResolverErrors::UnResolvable { msg, .. }) => {
                assert_eq!(msg, expected);
                Ok(())
            }
            Err(err) => Err(err),
            Ok(()) => Err(ModifierResolverErrors::unreachable(
                "Expected modifier resolution to fail.".to_string(),
            )),
        }
    }

    #[rstest::rstest]
    #[case::cfg_branching(
        1,
        1,
        foo_cfg_branching,
        "CFG with more than one node cannot be daggered."
    )]
    #[case::cfg_loop(1, 1, foo_cfg_loop, "CFG with more than one node cannot be daggered.")]
    #[case::tail_loop(1, 1, foo_tail_loop, "TailLoop cannot be daggered.")]
    #[case::cfg_two_blocks_dagger(
        1,
        1,
        foo_cfg_two_blocks,
        "CFG with more than one node cannot be daggered."
    )]

    fn test_dagger_rejects_cfg_with_control_flow(
        #[case] t_num: usize,
        #[case] c_num: u64,
        #[case] foo: fn(&mut ModuleBuilder<Hugr>, usize) -> FuncID<true>,
        #[case] expected: &str,
    ) {
        let (mut h, _) = modifier_test_hugr(t_num, c_num, foo, true);
        assert_matches!(assert_unresolvable_message(&mut h, expected), Ok(()));
    }

    #[test]
    fn test_dagger_keeps_non_quantum_array_ops_unchanged() {
        let h = resolved_modifier_test_hugr(1, 0, foo_non_quantum_array_ops, true);

        // If classical array ops were dagger-reversed, these direct
        // new_array -> unpack edges would disappear in the modified function.
        let mut array_new_to_unpack = 0;
        let mut borrow_array_new_to_unpack = 0;
        for node in h.nodes() {
            let optype = h.get_optype(node);
            if ArrayOp::from_optype(optype)
                .is_some_and(|op| op.def == ArrayOpDef::new_array && op.elem_ty == usize_t())
            {
                array_new_to_unpack += h
                    .linked_inputs(node, 0)
                    .filter(|(target, _)| {
                        ArrayOp::from_optype(h.get_optype(*target)).is_some_and(|op| {
                            op.def == ArrayOpDef::unpack && op.elem_ty == usize_t()
                        })
                    })
                    .count();
            }
            if BArrayOp::from_optype(optype)
                .is_some_and(|op| op.def == BArrayOpDef::new_array && op.elem_ty == usize_t())
            {
                borrow_array_new_to_unpack += h
                    .linked_inputs(node, 0)
                    .filter(|(target, _)| {
                        BArrayOp::from_optype(h.get_optype(*target)).is_some_and(|op| {
                            op.def == BArrayOpDef::unpack && op.elem_ty == usize_t()
                        })
                    })
                    .count();
            }
        }

        assert!(array_new_to_unpack >= 2);
        assert!(borrow_array_new_to_unpack >= 1);
    }

    fn is_in_modified_function(h: &Hugr, node: hugr::Node) -> bool {
        let mut parent = h.get_parent(node);
        while let Some(node) = parent {
            if h.get_optype(node)
                .as_func_defn()
                .is_some_and(|func| func.func_name().starts_with("__modified__"))
            {
                return true;
            }
            parent = h.get_parent(node);
        }
        false
    }

    #[test]
    fn test_dagger_keeps_nested_non_quantum_array_ops_unchanged() {
        let h = resolved_modifier_test_hugr(1, 0, foo_nested_non_quantum_array_ops, true);
        let inner_ty = array_type(2, usize_t());

        // Same check as above, but for nested classical array element types.
        // This guards the recursive qubit-element detection.
        let mut array_new_to_unpack = 0;
        let mut borrow_array_new_to_unpack = 0;
        for node in h.nodes() {
            if !is_in_modified_function(&h, node) {
                continue;
            }
            let optype = h.get_optype(node);
            if ArrayOp::from_optype(optype)
                .is_some_and(|op| op.def == ArrayOpDef::new_array && op.elem_ty == inner_ty)
            {
                array_new_to_unpack += h
                    .linked_inputs(node, 0)
                    .filter(|(target, _)| {
                        ArrayOp::from_optype(h.get_optype(*target)).is_some_and(|op| {
                            op.def == ArrayOpDef::unpack && op.elem_ty == inner_ty
                        })
                    })
                    .count();
            }
            if BArrayOp::from_optype(optype)
                .is_some_and(|op| op.def == BArrayOpDef::new_array && op.elem_ty == inner_ty)
            {
                borrow_array_new_to_unpack += h
                    .linked_inputs(node, 0)
                    .filter(|(target, _)| {
                        BArrayOp::from_optype(h.get_optype(*target)).is_some_and(|op| {
                            op.def == BArrayOpDef::unpack && op.elem_ty == inner_ty
                        })
                    })
                    .count();
            }
        }

        assert!(array_new_to_unpack >= 1);
        assert!(borrow_array_new_to_unpack >= 1);
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

    #[derive(Clone, Copy)]
    enum PolymorphicArrayElement {
        Qubit,
        Usize,
    }

    #[derive(Clone, Copy)]
    enum NestedGenericCall {
        Direct,
        Indirect,
    }

    impl PolymorphicArrayElement {
        fn ty(self) -> Type {
            match self {
                Self::Qubit => qb_t(),
                Self::Usize => usize_t(),
            }
        }

        fn type_arg(self) -> TypeArg {
            self.ty().into()
        }

        fn array_ty(self) -> Type {
            array_type(2, self.ty())
        }
    }

    fn add_polymorphic_array_identity_named(
        module: &mut ModuleBuilder<Hugr>,
        name: &str,
    ) -> FuncID<true> {
        let type_param = TypeParam::TypeKind(TypeBound::Linear);
        let generic_ty = Type::new_var_use(0, TypeBound::Linear);
        let generic_array_ty = array_type_parametric(2, generic_ty).unwrap();
        let foo_sig = PolyFuncType::new([type_param], Signature::new_endo([generic_array_ty]));
        let mut func = module.define_function(name, foo_sig).unwrap();
        func.set_unitary();
        let [input] = func.input_wires_arr();
        *func.finish_with_outputs([input]).unwrap().handle()
    }

    fn add_polymorphic_array_identity(module: &mut ModuleBuilder<Hugr>) -> FuncID<true> {
        add_polymorphic_array_identity_named(module, "foo")
    }

    fn add_nested_polymorphic_array_identity(module: &mut ModuleBuilder<Hugr>) -> FuncID<true> {
        let inner = add_polymorphic_array_identity_named(module, "inner_generic_array_identity");
        let type_param = TypeParam::TypeKind(TypeBound::Linear);
        let generic_ty = Type::new_var_use(0, TypeBound::Linear);
        let generic_array_ty = array_type_parametric(2, generic_ty.clone()).unwrap();
        let outer_sig = PolyFuncType::new([type_param], Signature::new_endo([generic_array_ty]));
        let mut func = module
            .define_function("outer_generic_array_identity", outer_sig)
            .unwrap();
        func.set_unitary();
        let [input] = func.input_wires_arr();
        let outputs = func
            .call(&inner, &[generic_ty.into()], [input])
            .unwrap()
            .outputs();
        *func.finish_with_outputs(outputs).unwrap().handle()
    }

    fn resolve_and_validate_polymorphic_array_input(module: ModuleBuilder<Hugr>) {
        let mut h = module.finish_hugr().unwrap();
        assert_matches!(h.validate(), Ok(()));
        let entrypoint = h.entrypoint();
        resolve_modifier_with_entrypoints(&mut h, [entrypoint]).unwrap();
        assert_matches!(h.validate(), Ok(()));
    }

    fn add_nested_polymorphic_array_call(
        block: &mut impl Dataflow,
        callee: &FuncID<true>,
        input: Wire,
        element: PolymorphicArrayElement,
        call: NestedGenericCall,
    ) -> Vec<Wire> {
        let call_sig = Signature::new_endo([element.array_ty()]);
        let generic_input = match element {
            PolymorphicArrayElement::Qubit => input,
            PolymorphicArrayElement::Usize => {
                let one = block.add_load_value(ConstUsize::new(1));
                let two = block.add_load_value(ConstUsize::new(2));
                block.add_new_array(usize_t(), [one, two]).unwrap()
            }
        };
        let generic_outputs = match call {
            NestedGenericCall::Direct => block
                .call(callee, &[element.type_arg()], [generic_input])
                .unwrap()
                .outputs()
                .collect(),
            NestedGenericCall::Indirect => {
                let loaded = block.load_func(callee, &[element.type_arg()]).unwrap();
                block
                    .add_dataflow_op(
                        CallIndirect {
                            signature: call_sig,
                        },
                        [loaded, generic_input],
                    )
                    .unwrap()
                    .outputs()
                    .collect()
            }
        };

        match element {
            PolymorphicArrayElement::Qubit => generic_outputs,
            PolymorphicArrayElement::Usize => {
                for output in generic_outputs {
                    let _ = block.add_array_unpack(usize_t(), 2, output).unwrap();
                }
                vec![input]
            }
        }
    }

    fn add_outer_with_nested_polymorphic_array_call(
        module: &mut ModuleBuilder<Hugr>,
        callee: &FuncID<true>,
        element: PolymorphicArrayElement,
        call: NestedGenericCall,
    ) -> FuncID<true> {
        let target_sig = Signature::new_endo([array_type(2, qb_t())]);
        let mut func = module.define_function("outer", target_sig.clone()).unwrap();
        func.set_unitary();
        let [input] = func.input_wires_arr();
        let block = {
            let mut block = func.dfg_builder(target_sig.clone(), [input]).unwrap();
            let [input] = block.input_wires_arr();
            let outputs =
                add_nested_polymorphic_array_call(&mut block, callee, input, element, call);
            block.finish_with_outputs(outputs).unwrap()
        };
        *func.finish_with_outputs(block.outputs()).unwrap().handle()
    }

    fn add_dagger_controlled_main_for_array_outer(
        module: &mut ModuleBuilder<Hugr>,
        outer: &FuncID<true>,
    ) {
        let target_ty = array_type(2, qb_t());
        let control_op: ExtensionOp = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &CONTROL_OP_ID,
                [
                    Term::BoundedNat(1),
                    vec![target_ty.clone().into()].into(),
                    vec![].into(),
                ],
            )
            .unwrap();
        let controlled_sig = Signature::new_endo([array_type(1, qb_t()), target_ty.clone()]);
        let dagger_op: ExtensionOp = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &DAGGER_OP_ID,
                [vec![target_ty.into()].into(), vec![].into()],
            )
            .unwrap();

        let mut func = module
            .define_function(
                "main",
                Signature::new(type_row![], controlled_sig.output.clone()),
            )
            .unwrap();
        let loaded = func.load_func(outer, &[]).unwrap();
        let daggered = func
            .add_dataflow_op(dagger_op, [loaded])
            .unwrap()
            .out_wire(0);
        let controlled = func
            .add_dataflow_op(control_op, [daggered])
            .unwrap()
            .out_wire(0);
        let control = func
            .add_dataflow_op(TketOp::QAlloc, [])
            .unwrap()
            .out_wire(0);
        let controls = func.add_new_array(qb_t(), [control]).unwrap();
        let target_0 = func
            .add_dataflow_op(TketOp::QAlloc, [])
            .unwrap()
            .out_wire(0);
        let target_1 = func
            .add_dataflow_op(TketOp::QAlloc, [])
            .unwrap()
            .out_wire(0);
        let targets = func.add_new_array(qb_t(), [target_0, target_1]).unwrap();
        let outputs = func
            .add_dataflow_op(
                CallIndirect {
                    signature: controlled_sig,
                },
                [controlled, controls, targets],
            )
            .unwrap()
            .outputs();
        func.finish_with_outputs(outputs).unwrap();
    }

    fn build_nested_polymorphic_array_input_module(
        element: PolymorphicArrayElement,
        call: NestedGenericCall,
    ) -> ModuleBuilder<Hugr> {
        let mut module = ModuleBuilder::new();
        let foo = add_polymorphic_array_identity(&mut module);
        let outer = add_outer_with_nested_polymorphic_array_call(&mut module, &foo, element, call);
        add_dagger_controlled_main_for_array_outer(&mut module, &outer);
        module
    }

    #[test]
    /// Test when a polymorphic function is targeted a modifier chain
    fn test_control_polymorphic_array_input() {
        let mut module = ModuleBuilder::new();

        let foo = add_polymorphic_array_identity(&mut module);
        let concrete_array_ty = array_type(2, qb_t());

        let control_op: ExtensionOp = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &CONTROL_OP_ID,
                [
                    Term::BoundedNat(1),
                    vec![concrete_array_ty.clone().into()].into(),
                    vec![].into(),
                ],
            )
            .unwrap();
        let dagger_op = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &DAGGER_OP_ID,
                [vec![concrete_array_ty.clone().into()].into(), vec![].into()],
            )
            .unwrap();
        let controlled_sig = Signature::new_endo([array_type(1, qb_t()), concrete_array_ty]);

        let mut func = module
            .define_function(
                "main",
                Signature::new(type_row![], controlled_sig.output.clone()),
            )
            .unwrap();
        let loaded = func.load_func(&foo, &[TypeArg::from(qb_t())]).unwrap();
        let daggered = func
            .add_dataflow_op(dagger_op, [loaded])
            .unwrap()
            .out_wire(0);
        let controlled = func
            .add_dataflow_op(control_op, [daggered])
            .unwrap()
            .out_wire(0);
        let control = func
            .add_dataflow_op(TketOp::QAlloc, [])
            .unwrap()
            .out_wire(0);
        let controls = func.add_new_array(qb_t(), [control]).unwrap();
        let target_0 = func
            .add_dataflow_op(TketOp::QAlloc, [])
            .unwrap()
            .out_wire(0);
        let target_1 = func
            .add_dataflow_op(TketOp::QAlloc, [])
            .unwrap()
            .out_wire(0);
        let targets = func.add_new_array(qb_t(), [target_0, target_1]).unwrap();
        let outputs = func
            .add_dataflow_op(
                CallIndirect {
                    signature: controlled_sig,
                },
                [controlled, controls, targets],
            )
            .unwrap()
            .outputs();
        func.finish_with_outputs(outputs).unwrap();

        resolve_and_validate_polymorphic_array_input(module);
    }

    #[rstest::rstest]
    #[case::qubit_array(PolymorphicArrayElement::Qubit)]
    #[case::usize_array(PolymorphicArrayElement::Usize)]
    /// Test that an indirect call to a polymorphic function is resolved correctly when the function is inside a modified block.
    fn test_control_polymorphic_array_input_nested_dfg_call(
        #[case] element: PolymorphicArrayElement,
    ) {
        let module =
            build_nested_polymorphic_array_input_module(element, NestedGenericCall::Indirect);
        resolve_and_validate_polymorphic_array_input(module);
    }

    #[rstest::rstest]
    #[case::qubit_array(PolymorphicArrayElement::Qubit)]
    #[case::usize_array(PolymorphicArrayElement::Usize)]
    /// Test that a direct call to a polymorphic function is resolved correctly when the function is inside a modified block.
    fn test_control_polymorphic_array_input_nested_dfg_direct_call(
        #[case] element: PolymorphicArrayElement,
    ) {
        let module =
            build_nested_polymorphic_array_input_module(element, NestedGenericCall::Direct);
        resolve_and_validate_polymorphic_array_input(module);
    }

    #[rstest::rstest]
    #[case::qubit_array(PolymorphicArrayElement::Qubit)]
    #[case::usize_array(PolymorphicArrayElement::Usize)]
    /// Test that two nested generic functions are resolved correctly when the functions are inside a modified block.
    fn test_control_polymorphic_array_input_two_nested_generic_functions(
        #[case] element: PolymorphicArrayElement,
    ) {
        let mut module = ModuleBuilder::new();
        let nested_generic = add_nested_polymorphic_array_identity(&mut module);
        let outer = add_outer_with_nested_polymorphic_array_call(
            &mut module,
            &nested_generic,
            element,
            NestedGenericCall::Direct,
        );
        add_dagger_controlled_main_for_array_outer(&mut module, &outer);
        resolve_and_validate_polymorphic_array_input(module);
    }
}
