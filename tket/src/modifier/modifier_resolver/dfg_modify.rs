//! Modifier for dataflow blocks.
use std::{
    collections::{HashMap, HashSet, VecDeque},
    iter, mem,
};

use hugr::{
    HugrView, IncomingPort, Node, OutgoingPort, PortIndex, Wire,
    builder::{
        BuildError, ConditionalBuilder, Container, DFGBuilder, Dataflow, FunctionBuilder,
        SubContainer, TailLoopBuilder,
    },
    core::HugrNode,
    extension::{prelude::qb_t, simple_op::MakeExtensionOp},
    hugr::hugrmut::HugrMut,
    ops::{
        Call, Conditional, DFG, DataflowBlock, DataflowOpTrait, OpType, TailLoop,
        handle::NodeHandle,
    },
    std_extensions::collections::{
        array::ArrayOpBuilder,
        borrow_array::{BArrayOpBuilder, borrow_array_type},
    },
    types::{EdgeKind, FuncTypeBase, PolyFuncType, Signature, TypeRow},
};
use petgraph::visit::{Topo, Walker};

use crate::extension::modifier::Modifier;

use super::{DirWire, ModifierResolver, ModifierResolverErrors, PortExt};

use crate::metadata;

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
                    inputs,
                    other_outputs,
                    sum_rows: _sum_rows,
                } = dfb;

                // The branch sum is unchanged.
                self.map_insert(
                    (old_out, IncomingPort::from(0)).into(),
                    (new_out, IncomingPort::from(0)).into(),
                )?;
                self.wire_inout(
                    (old_out, old_in),
                    (new_out, new_in),
                    (other_outputs.iter(), inputs.iter()),
                    (1, 0, 0),
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

        self.wire_state_order(
            old_in,
            h.get_optype(old_in),
            new_in,
            new_dfg.hugr().get_optype(new_in),
        )?;
        self.wire_state_order(
            old_out,
            h.get_optype(old_out),
            new_out,
            new_dfg.hugr().get_optype(new_out),
        )?;

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

    /// Unpacks the control qubits (given by the combined modifier) from a list of arrays into a flat list of qubit wires.
    pub(super) fn unpack_controls(
        &self,
        builder: &mut impl Dataflow,
        controls_arr: impl IntoIterator<Item = Wire>,
    ) -> Result<Vec<Wire>, ModifierResolverErrors<N>> {
        let control_layout = &self.modifiers().accum_ctrl;
        let mut controls = Vec::with_capacity(control_layout.iter().sum());
        let mut controls_arr = controls_arr.into_iter();
        for size in control_layout {
            let ctrl_arr = controls_arr.next().expect("missing control array");
            controls.extend(builder.add_array_unpack(qb_t(), *size as u64, ctrl_arr)?);
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
        Ok(pack_control_groups(
            new_dfg,
            self.controls_ref(),
            &self.modifiers().accum_ctrl,
        )?)
    }

    /// Modifies a function if necessary.
    ///
    /// When the function definition or a concrete call instantiation contains qubits,
    /// the function needs to be modified. The concrete signature matters for polymorphic
    /// functions whose uninstantiated definition may not reveal quantum data.
    ///
    /// NOTE: When a polymorphic function has a polymorphic input, the function is considered
    /// to have classical data (there are no quantum generic types or generic quantum operations).
    pub(crate) fn modify_fn_if_needed(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        func: N,
        concrete_signature: Option<&Signature>,
    ) -> Result<Option<N>, ModifierResolverErrors<N>> {
        let OpType::FuncDefn(fn_defn) = h.get_optype(func) else {
            return Err(ModifierResolverErrors::unreachable(format!(
                "Cannot modify a non-function node. {}",
                h.get_optype(func)
            )));
        };

        // We first check for custom implementations.
        if let Some(custom_func) = self.find_custom_implementation(h, func)? {
            return Ok(Some(self.custom_implementation_adapter(
                h,
                func,
                custom_func,
            )?));
        }

        let concrete_signature_has_quantum_data =
            concrete_signature.is_some_and(|signature| self.signature_has_quantum_data(signature));
        if !concrete_signature_has_quantum_data
            && !self.signature_has_quantum_data(fn_defn.signature().body())
        {
            return Ok(None);
        }
        Ok(Some(self.modify_fn(h, func)?))
    }

    /// Looks up a user-provided custom implementation of `func` for the current modifier.
    ///
    /// Returns the matching function node if the function's metadata registers a custom
    /// daggered, controlled, or controlled-daggered implementation for the requested modifier.
    fn find_custom_implementation(
        &self,
        h: &impl HugrView<Node = N>,
        func: N,
    ) -> Result<Option<N>, ModifierResolverErrors<N>> {
        let func_name = h
            .get_optype(func)
            .as_func_defn()
            .map(|defn| defn.func_name().to_string())
            .unwrap_or_default();
        let requested_control_qubits = self.control_num();

        match (requested_control_qubits, self.modifiers().dagger) {
            (0, true) => {
                // println!("Looking for daggered implementation of function `{func_name}`");
                let Some(impl_name) = h
                    .try_get_metadata::<metadata::DaggeredImplementations>(func)
                    .unwrap()
                else {
                    return Ok(None);
                };
                let impl_func = find_module_func_by_name(h, &impl_name).ok_or_else(|| {
                    ModifierResolverErrors::unreachable(format!(
                        "Daggered implementation `{impl_name}` for function `{func_name}` not found."
                    ))
                })?;
                Ok(Some(impl_func))
            }
            (n, false) if n > 0 => {
                // println!(
                //     "Looking for controlled implementation of function `{func_name}` with {n} control qubits"
                // );
                let impl_names = h
                    .try_get_metadata::<metadata::ControlledImplementations>(func)
                    .unwrap();
                find_controlled_implementation(h, impl_names, n, &func_name)
            }
            (n, true) if n > 0 => {
                // println!(
                //     "Looking for controlled-daggered implementation of function `{func_name}` with {n} control qubits"
                // );
                let impl_names = h
                    .try_get_metadata::<metadata::CtrlDaggeredImplementations>(func)
                    .unwrap();
                find_controlled_implementation(h, impl_names, n, &func_name)
            }
            _ => Ok(None),
        }
    }

    /// Return a cached adapter for a custom controlled implementation.
    ///
    /// The resolver expects the modified function to receive groups of controls as
    /// owned arrays prepended to its ordinary arguments. Guppy custom implementations
    /// instead receive a single borrowed control array after the ordinary inputs and
    /// return it after the ordinary outputs. The adapter combines the control groups
    /// into one borrowed array and reorders the inputs and outputs accordingly.
    fn custom_implementation_adapter(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        original_func: N,
        custom_func: N,
    ) -> Result<N, ModifierResolverErrors<N>> {
        self.modified_functions.insert(original_func);
        if self.control_num() == 0 {
            return Ok(custom_func);
        }

        let cache_key = (original_func, custom_func, self.modifiers().clone());
        if let Some(adapter) = self.custom_adapters.get(&cache_key) {
            return Ok(*adapter);
        }

        let spec = self.validate_custom_adapter(h, original_func, custom_func)?;
        let mut adapter_signature = spec.original_signature.clone();
        // The adapter signature is the original signature modified by the combined modifier, as
        // the function was modified without using the custom implementation.
        self.modify_signature(adapter_signature.body_mut(), false);
        let adapter_func = self.build_custom_adapter(h, &spec, adapter_signature, custom_func)?;

        // cache the adapter for future use
        self.custom_adapters.insert(cache_key, adapter_func);
        Ok(adapter_func)
    }

    /// Validate the controls-last ABI of a custom implementation and collect the information
    /// required to build its controls-first adapter.
    fn validate_custom_adapter(
        &self,
        h: &impl HugrView<Node = N>,
        original_func: N,
        custom_func: N,
    ) -> Result<CustomAdapterSpec, ModifierResolverErrors<N>> {
        let OpType::FuncDefn(original_defn) = h.get_optype(original_func) else {
            return Err(ModifierResolverErrors::unreachable(format!(
                "Cannot adapt a custom implementation for non-function node {original_func}."
            )));
        };
        let OpType::FuncDefn(custom_defn) = h.get_optype(custom_func) else {
            return Err(ModifierResolverErrors::unreachable(format!(
                "Custom implementation node {custom_func} is not a function."
            )));
        };

        let original_signature = original_defn.signature().clone();
        let custom_signature = custom_defn.signature().clone();
        let custom_name = custom_defn.func_name().clone();

        // NICOLA: All these test are legit, not sure if we want to keep them or not
        // If we remove them, given custom implementation that do not respect them will produce an invalid hugr
        // (we will get an error anyway, but it will be less clear)
        let custom_optype = h.get_optype(custom_func).clone();
        if !original_signature.params().is_empty() || !custom_signature.params().is_empty() {
            return Err(ModifierResolverErrors::unresolvable(
                custom_func,
                format!(
                    "Polymorphic custom implementations are not supported (`{}` and \
                     `{custom_name}`).",
                    original_defn.func_name()
                ),
                custom_optype,
            ));
        }

        let control_type = borrow_array_type(self.control_num() as u64, qb_t());
        let original_body = original_signature.body();
        let custom_body = custom_signature.body();
        let expected_custom_input: TypeRow = original_body
            .input
            .iter()
            .cloned()
            .chain([control_type.clone()])
            .collect::<Vec<_>>()
            .into();
        let expected_custom_output: TypeRow = original_body
            .output
            .iter()
            .cloned()
            .chain([control_type])
            .collect::<Vec<_>>()
            .into();
        if custom_body.input != expected_custom_input
            || custom_body.output != expected_custom_output
        {
            return Err(ModifierResolverErrors::unresolvable(
                custom_func,
                format!(
                    "Custom controlled implementation `{custom_name}` must place a trailing \
                     borrow_array of {} control qubits after the ordinary inputs and outputs; \
                     expected `{expected_custom_input} -> {expected_custom_output}`, found \
                     `{} -> {}`.",
                    self.control_num(),
                    custom_body.input,
                    custom_body.output,
                ),
                custom_optype,
            ));
        }

        Ok(CustomAdapterSpec {
            original_signature,
            custom_signature,
            custom_name,
        })
    }

    /// Generates a new function modified by the combined modifier.
    pub(crate) fn modify_fn(
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

        let modify_result = self.modify_dfg_body(h, func, &mut new_fn);
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

    /// Copies a sub-container into the parent DFG without modification, preserving non-local function call edges.
    fn copy_sub_container_no_modification(
        &mut self,
        h: &impl HugrView<Node = N>,
        n: N,
        new_dfg: &mut impl Container,
    ) -> Result<Node, ModifierResolverErrors<N>> {
        let nodes = h.descendants(n).collect::<HashSet<_>>();

        let static_edges = nodes
            .iter()
            .flat_map(|node| {
                h.node_inputs(*node).filter_map(|port| {
                    h.single_linked_output(*node, port)
                        .filter(|(src_n, _)| h.get_parent(*node) != h.get_parent(*src_n))
                        .map(|(src_n, _)| {
                            assert!(
                                matches!(
                                    h.get_optype(*node).port_kind(port),
                                    Some(EdgeKind::Function(_))
                                ),
                                "Nonlocal Const/Value edges not supported"
                            );
                            (src_n, *node, port)
                        })
                })
            })
            .collect::<Vec<_>>();

        let insertion_result = new_dfg.add_hugr_view(&h.with_entrypoint(n));

        let new_node = insertion_result.inserted_entrypoint;
        for port in h.all_node_ports(n) {
            self.map_insert(DirWire(n, port), DirWire(new_node, port))?;
        }
        for (source, old_target, target_port) in static_edges {
            let new_target = insertion_result.node_map.get(&old_target).copied().unwrap();
            self.call_map_insert(source, (new_target, target_port));
        }

        Ok(new_node)
    }

    pub(super) fn modify_dfg(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        n: N,
        dfg: &DFG,
        new_parent_dfg: &mut impl Container,
    ) -> Result<(), ModifierResolverErrors<N>> {
        // Check if the DFG input or output are carrying qubits
        let boundary_has_qubits = self.signature_has_quantum_data(&dfg.signature);
        if !boundary_has_qubits {
            // If the DFG does not carry qubits, we still modify its body in case it contains modifier nodes.
            // Since there are no input/output qubits, modifiers inside the DFG cannot affect the outside of
            // the DFG, so we can safely resolve them without worrying about the surrounding context.
            let new_dfg = self.with_modifiers(Default::default(), |this| {
                let mut builder = DFGBuilder::new(dfg.signature.clone()).unwrap();
                this.modify_dfg_body(h, n, &mut builder)?;
                this.insert_sub_dfg(new_parent_dfg, builder)
            })?;
            for port in h.all_node_ports(n) {
                self.map_insert(DirWire(n, port), DirWire(new_dfg, port))?;
            }
            return Ok(());
        }

        // Build a new DFG with modified body.
        let boundary_signature = dfg.signature.clone();
        let mut signature = boundary_signature.clone();
        self.modify_signature(&mut signature, true);
        let mut builder = DFGBuilder::new(signature.clone()).unwrap();
        self.modify_dfg_body(h, n, &mut builder)?;
        let new_dfg = self.insert_sub_dfg(new_parent_dfg, builder)?;

        // connect the controls and register the IOs
        for (i, c) in self.controls().iter_mut().enumerate() {
            new_parent_dfg
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
        self.wire_state_order(
            n,
            h.get_optype(n),
            new_dfg,
            new_parent_dfg.hugr().get_optype(new_dfg),
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
        let mut builder = TailLoopBuilder::new(
            tail_loop.just_inputs.clone(),
            control_types.extend(tail_loop.rest.iter()),
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
        // A purely classical, modifier-free conditional is unchanged by the
        // current modifier, so it can be copied without rebuilding its cases.
        let needs_modification = iter::once(n).chain(h.descendants(n)).any(|node| {
            h.signature(node)
                .is_some_and(|signature| self.signature_has_quantum_data(&signature))
                || Modifier::from_optype(h.get_optype(node)).is_some()
        });
        if !needs_modification {
            self.copy_sub_container_no_modification(h, n, new_dfg)?;
            return Ok(());
        }

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
        self.wire_state_order(
            n,
            h.get_optype(n),
            new_conditional,
            new_dfg.hugr().get_optype(new_conditional),
        )?;

        Ok(())
    }

    /// Build, insert, and link the adapter for a controlled custom implementation.
    ///
    /// The adapter is built as a standalone HUGR, so its call to `custom_func` is linked only after
    /// insertion into `h`. The returned node is the inserted adapter function.
    fn build_custom_adapter(
        &self,
        h: &mut impl HugrMut<Node = N>,
        spec: &CustomAdapterSpec,
        adapter_signature: PolyFuncType,
        custom_func: N,
    ) -> Result<N, ModifierResolverErrors<N>> {
        let control_layout = &self.modifiers().accum_ctrl;
        let layout_name = control_layout
            .iter()
            .map(usize::to_string)
            .collect::<Vec<_>>()
            .join("_");

        // New adapter function
        let mut adapter = FunctionBuilder::new(
            format!("__controller_adapter[{}]__{layout_name}", spec.custom_name),
            adapter_signature,
        )?;

        let adapter_inputs = adapter.input_wires().collect::<Vec<_>>();
        let control_group_count = control_layout.len();
        // We expect that the controllers inputting the adapter match exactly the control layout
        assert_eq!(
            adapter_inputs[..control_group_count].len(),
            control_layout.len()
        );
        // Unpack the control groups and them pack them into a single borrowed array
        let control_qubits = self.unpack_controls(
            &mut adapter,
            adapter_inputs[..control_group_count].iter().copied(),
        )?;
        let custom_control = adapter.add_new_borrow_array(qb_t(), control_qubits)?;
        let custom_arguments = adapter_inputs
            .into_iter()
            .skip(control_group_count)
            .chain([custom_control]);

        // Call the custom implementation
        let custom_call_op =
            Call::try_new(spec.custom_signature.clone(), []).map_err(BuildError::from)?;
        let function_port = custom_call_op.called_function_port();
        let custom_call = adapter.add_dataflow_op(custom_call_op, custom_arguments)?;

        let mut ordinary_outputs = custom_call.outputs().collect::<Vec<_>>();
        let returned_controls = ordinary_outputs
            .pop()
            .expect("the validated custom signature has a control output");
        let control_num = control_layout.iter().sum::<usize>();

        // Unpack the returned control array and repack it into the resolver's control groups.
        let returned_control_qubits =
            adapter.add_borrow_array_unpack(qb_t(), control_num as u64, returned_controls)?;
        // We expect that the returned control qubits match exactly the control layout
        assert_eq!(returned_control_qubits.len(), control_num);
        let control_outputs =
            pack_control_groups(&mut adapter, &returned_control_qubits, control_layout)?;
        adapter.set_outputs(control_outputs.into_iter().chain(ordinary_outputs))?;

        // Link the inserted adapter to the custom implementation.
        let insertion = h.insert_from_view(h.module_root(), adapter.hugr());
        let inserted_call = insertion.node_map[&custom_call.node()];
        h.connect(custom_func, 0, inserted_call, function_port);

        Ok(insertion.inserted_entrypoint)
    }
}

/// Signatures and naming information for a validated custom implementation adapter.
struct CustomAdapterSpec {
    original_signature: PolyFuncType,
    custom_signature: PolyFuncType,
    custom_name: String,
}

/// Pack a flat list of returned control qubits back into the resolver's control groups.
fn pack_control_groups(
    builder: &mut impl Dataflow,
    controls: &[Wire],
    control_layout: &[usize],
) -> Result<Vec<Wire>, BuildError> {
    let mut offset = 0;
    control_layout
        .iter()
        .map(|&size| {
            let group =
                builder.add_new_array(qb_t(), controls[offset..offset + size].iter().copied())?;
            offset += size;
            Ok(group)
        })
        .collect()
}

/// Finds a function definition named `func_name` among the children of the module root.
fn find_module_func_by_name<N: HugrNode>(
    h: &impl HugrView<Node = N>,
    func_name: &str,
) -> Option<N> {
    h.children(h.module_root()).find(|&node| {
        h.get_optype(node)
            .as_func_defn()
            .is_some_and(|defn| defn.func_name() == func_name)
    })
}

/// Finds the controlled implementation among `impl_names` whose number of control qubits
/// matches `requested_control_qubits`.
fn find_controlled_implementation<N: HugrNode>(
    h: &impl HugrView<Node = N>,
    impl_names: Option<Vec<String>>,
    requested_control_qubits: usize,
    func_name: &str,
) -> Result<Option<N>, ModifierResolverErrors<N>> {
    let Some(impl_names) = impl_names else {
        return Ok(None);
    };
    for impl_name in impl_names {
        let impl_func = find_module_func_by_name(h, &impl_name).ok_or_else(|| {
            ModifierResolverErrors::unreachable(format!(
                "Controlled implementation `{impl_name}` for function `{func_name}` not found."
            ))
        })?;
        if requested_control_qubits
            == h.get_metadata::<metadata::NumControlQubits>(impl_func)
                .unwrap()
        {
            return Ok(Some(impl_func));
        }
    }
    Ok(None)
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
        ops::{
            CallIndirect, ExtensionOp,
            handle::{FuncID, NodeHandle},
        },
        std_extensions::collections::{
            array::{ArrayOp, ArrayOpBuilder, ArrayOpDef, array_type, array_type_parametric},
            borrow_array::{BArrayOp, BArrayOpBuilder, BArrayOpDef, borrow_array_type},
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

    #[test]
    fn custom_implementation_adapter_is_cached() {
        let mut module = ModuleBuilder::new();
        let original = {
            let func = module
                .define_function("original", Signature::new_endo([usize_t()]))
                .unwrap();
            let outputs = func.input_wires();
            *func.finish_with_outputs(outputs).unwrap().handle()
        };
        let custom = {
            let func = module
                .define_function(
                    "custom",
                    Signature::new_endo([usize_t(), borrow_array_type(1, qb_t())]),
                )
                .unwrap();
            let outputs = func.input_wires();
            *func.finish_with_outputs(outputs).unwrap().handle()
        };
        let mut h = module.finish_hugr().unwrap();
        let mut resolver = ModifierResolver::new();
        resolver.modifiers = CombinedModifier {
            control: 1,
            accum_ctrl: vec![1],
            dagger: false,
        };

        let first = resolver
            .custom_implementation_adapter(&mut h, original.node(), custom.node())
            .unwrap();
        let second = resolver
            .custom_implementation_adapter(&mut h, original.node(), custom.node())
            .unwrap();

        assert_eq!(first, second);
        assert_eq!(resolver.custom_adapters.len(), 1);
        assert_eq!(
            h.children(h.module_root())
                .filter(|&node| {
                    h.get_optype(node)
                        .as_func_defn()
                        .is_some_and(|defn| defn.func_name().starts_with("__controller_adapter"))
                })
                .count(),
            1
        );
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

    /// Added after: https://github.com/Quantinuum/tket2/pull/1911
    ///
    /// Minimal reproducer for daggering a CFG whose classical input is
    /// interleaved with its quantum inputs.
    /// The inner function has a packed control in its public signature, while its CFG threads
    /// the unpacked control after the classical input:
    ///
    /// ```text
    /// function: array<qubit; 1>, qubit, usize -> array<qubit; 1>, qubit
    /// CFG:      qubit, usize, qubit           -> qubit, qubit
    /// ```
    ///
    /// Applying a dagger must keep `usize` flowing forwards and reverse only the two quantum
    /// wires.
    #[test]
    fn daggered_controlled_cfg_with_interleaved_classical_input() {
        let mut module = ModuleBuilder::new();
        let inner_control_array_ty = array_type(1, qb_t());
        let already_controlled_sig = Signature::new(
            [inner_control_array_ty.clone(), qb_t(), usize_t()],
            [inner_control_array_ty.clone(), qb_t()],
        );

        let already_controlled = {
            let mut func = module
                .define_function("already_controlled", already_controlled_sig.clone())
                .unwrap();
            func.set_unitary();

            let mut inputs = func.input_wires();
            let inner_controls = inputs.next().unwrap();
            let target = inputs.next().unwrap();
            let classical = inputs.next().unwrap();
            let inner_control = func.add_array_unpack(qb_t(), 1, inner_controls).unwrap()[0];

            let cfg = {
                let mut cfg = func
                    .cfg_builder(
                        vec![
                            (qb_t(), target),
                            (usize_t(), classical),
                            (qb_t(), inner_control),
                        ],
                        [qb_t(), qb_t()].into(),
                    )
                    .unwrap();
                let block = {
                    let mut block = cfg
                        .entry_builder(vec![type_row![]], [qb_t(), qb_t()].into())
                        .unwrap();
                    let mut inputs = block.input_wires();
                    let target = inputs.next().unwrap();
                    let _classical = inputs.next().unwrap();
                    let inner_control = inputs.next().unwrap();
                    let tag = block.make_sum(0, [type_row![]], []).unwrap();
                    block
                        .finish_with_outputs(tag, [target, inner_control])
                        .unwrap()
                };
                let exit = cfg.exit_block();
                cfg.branch(&block, 0, &exit).unwrap();
                cfg.finish_sub_container().unwrap()
            };

            let mut cfg_outputs = cfg.outputs();
            let target = cfg_outputs.next().unwrap();
            let inner_control = cfg_outputs.next().unwrap();
            let inner_controls = func.add_new_array(qb_t(), [inner_control]).unwrap();
            func.finish_with_outputs([inner_controls, target]).unwrap()
        };

        let dagger_op = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &DAGGER_OP_ID,
                [
                    vec![inner_control_array_ty.clone().into(), qb_t().into()].into(),
                    vec![usize_t().into()].into(),
                ],
            )
            .unwrap();

        {
            let mut main = module
                .define_function(
                    "main",
                    Signature::new(type_row![], [inner_control_array_ty, qb_t()]),
                )
                .unwrap();
            let loaded = main.load_func(already_controlled.handle(), &[]).unwrap();
            let daggered = main
                .add_dataflow_op(dagger_op, [loaded])
                .unwrap()
                .out_wire(0);

            let inner_control = main
                .add_dataflow_op(TketOp::QAlloc, [])
                .unwrap()
                .out_wire(0);
            let target = main
                .add_dataflow_op(TketOp::QAlloc, [])
                .unwrap()
                .out_wire(0);
            let classical = main.add_load_value(ConstUsize::new(1));
            let inner_controls = main.add_new_array(qb_t(), [inner_control]).unwrap();

            let call = main
                .add_dataflow_op(
                    CallIndirect {
                        signature: already_controlled_sig,
                    },
                    [daggered, inner_controls, target, classical],
                )
                .unwrap();
            main.finish_with_outputs(call.outputs()).unwrap();
        }

        let mut h = module.finish_hugr().unwrap();
        assert_matches!(h.validate(), Ok(()));

        let entrypoint = h.entrypoint();
        resolve_modifier_with_entrypoints(&mut h, [entrypoint]).unwrap();
        assert_matches!(h.validate(), Ok(()));
    }

    /// Added after: https://github.com/Quantinuum/tket2/pull/1911
    /// Minimal reproducer for daggering a CFG whose classical output is
    /// interleaved with its quantum outputs.
    ///
    /// The inner function has a packed control in its public signature, while its CFG moves the
    /// classical value from the last input port to the middle output port:
    ///
    /// ```text
    /// function: array<qubit; 1>, qubit, usize -> array<qubit; 1>, qubit
    /// CFG:      qubit, qubit, usize           -> qubit, usize, qubit
    /// ```
    ///
    /// Applying a dagger must keep `usize` flowing forwards and reverse only the two quantum
    /// wires, skipping the classical hole in the CFG outputs.
    #[test]
    fn daggered_controlled_cfg_with_interleaved_classical_output() {
        let mut module = ModuleBuilder::new();
        let inner_control_array_ty = array_type(1, qb_t());
        let already_controlled_sig = Signature::new(
            [inner_control_array_ty.clone(), qb_t(), usize_t()],
            [inner_control_array_ty.clone(), qb_t()],
        );

        let already_controlled = {
            let mut func = module
                .define_function("already_controlled", already_controlled_sig.clone())
                .unwrap();
            func.set_unitary();

            let mut inputs = func.input_wires();
            let inner_controls = inputs.next().unwrap();
            let target = inputs.next().unwrap();
            let classical = inputs.next().unwrap();
            let inner_control = func.add_array_unpack(qb_t(), 1, inner_controls).unwrap()[0];

            let cfg = {
                let mut cfg = func
                    .cfg_builder(
                        vec![
                            (qb_t(), target),
                            (qb_t(), inner_control),
                            (usize_t(), classical),
                        ],
                        [qb_t(), usize_t(), qb_t()].into(),
                    )
                    .unwrap();
                let block = {
                    let mut block = cfg
                        .entry_builder(vec![type_row![]], [qb_t(), usize_t(), qb_t()].into())
                        .unwrap();
                    let mut inputs = block.input_wires();
                    let target = inputs.next().unwrap();
                    let inner_control = inputs.next().unwrap();
                    let classical = inputs.next().unwrap();
                    let tag = block.make_sum(0, [type_row![]], []).unwrap();
                    block
                        .finish_with_outputs(tag, [target, classical, inner_control])
                        .unwrap()
                };
                let exit = cfg.exit_block();
                cfg.branch(&block, 0, &exit).unwrap();
                cfg.finish_sub_container().unwrap()
            };

            let mut cfg_outputs = cfg.outputs();
            let target = cfg_outputs.next().unwrap();
            let _classical = cfg_outputs.next().unwrap();
            let inner_control = cfg_outputs.next().unwrap();
            let inner_controls = func.add_new_array(qb_t(), [inner_control]).unwrap();
            func.finish_with_outputs([inner_controls, target]).unwrap()
        };

        let dagger_op = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &DAGGER_OP_ID,
                [
                    vec![inner_control_array_ty.clone().into(), qb_t().into()].into(),
                    vec![usize_t().into()].into(),
                ],
            )
            .unwrap();

        {
            let mut main = module
                .define_function(
                    "main",
                    Signature::new(type_row![], [inner_control_array_ty, qb_t()]),
                )
                .unwrap();
            let loaded = main.load_func(already_controlled.handle(), &[]).unwrap();
            let daggered = main
                .add_dataflow_op(dagger_op, [loaded])
                .unwrap()
                .out_wire(0);

            let inner_control = main
                .add_dataflow_op(TketOp::QAlloc, [])
                .unwrap()
                .out_wire(0);
            let target = main
                .add_dataflow_op(TketOp::QAlloc, [])
                .unwrap()
                .out_wire(0);
            let classical = main.add_load_value(ConstUsize::new(1));
            let inner_controls = main.add_new_array(qb_t(), [inner_control]).unwrap();

            let call = main
                .add_dataflow_op(
                    CallIndirect {
                        signature: already_controlled_sig,
                    },
                    [daggered, inner_controls, target, classical],
                )
                .unwrap();
            main.finish_with_outputs(call.outputs()).unwrap();
        }

        let mut h = module.finish_hugr().unwrap();
        assert_matches!(h.validate(), Ok(()));

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

    /// Test pass on a DFG with no quantum signature that calls an external function
    fn foo_dfg_external_function_call(
        module: &mut ModuleBuilder<Hugr>,
        t_num: usize,
    ) -> FuncID<true> {
        assert_eq!(t_num, 1);

        let external = module
            .define_function("external_classical_noop", Signature::new_endo([]))
            .unwrap()
            .finish_with_outputs([])
            .unwrap();

        let foo_sig = Signature::new_endo([qb_t()]);
        let mut func = module.define_function("foo", foo_sig).unwrap();
        func.set_unitary();
        let q = func.input_wires().next().unwrap();
        {
            let mut dfg = func.dfg_builder(Signature::new_endo([]), []).unwrap();
            dfg.call(external.handle(), &[], []).unwrap();
            dfg.finish_with_outputs([]).unwrap();
        }

        *func.finish_with_outputs([q]).unwrap().handle()
    }

    /// Test pass on a DFG with no quantum signature that contains a modifier (dagger)
    /// (https://github.com/Quantinuum/tket2/issues/1814)
    fn foo_dfg_daggered_empty_indirect_call(
        module: &mut ModuleBuilder<Hugr>,
        t_num: usize,
    ) -> FuncID<true> {
        assert_eq!(t_num, 1);

        let empty_sig = Signature::new_endo(type_row![]);
        let empty = {
            let mut func = module.define_function("empty", empty_sig.clone()).unwrap();
            func.set_unitary();
            func.finish_with_outputs([]).unwrap()
        };

        let dagger_op: ExtensionOp = MODIFIER_EXTENSION
            .instantiate_extension_op(&DAGGER_OP_ID, [vec![].into(), vec![].into()])
            .unwrap();

        let foo_sig = Signature::new_endo([qb_t()]);
        let mut func = module.define_function("foo", foo_sig).unwrap();
        func.set_unitary();
        let q = func.input_wires().next().unwrap();
        {
            let mut dfg = func
                .dfg_builder(Signature::new_endo(type_row![]), [])
                .unwrap();
            let loaded = dfg.load_func(empty.handle(), &[]).unwrap();
            let daggered = dfg
                .add_dataflow_op(dagger_op, [loaded])
                .unwrap()
                .out_wire(0);
            dfg.add_dataflow_op(
                CallIndirect {
                    signature: empty_sig,
                },
                [daggered],
            )
            .unwrap();
            dfg.finish_with_outputs([]).unwrap();
        }

        *func.finish_with_outputs([q]).unwrap().handle()
    }

    #[rstest::rstest]
    #[case::dfg(1, 2, foo_dfg, false)]
    #[case::dfg_dagger(1, 2, foo_dfg, true)]
    #[case::dfg_external_function_call(1, 1, foo_dfg_external_function_call, true)]
    #[case::dfg_daggered_empty_indirect_call(1, 1, foo_dfg_daggered_empty_indirect_call, false)]
    #[case::dfg_daggered_empty_indirect_call_daggered(
        1,
        1,
        foo_dfg_daggered_empty_indirect_call,
        true
    )]
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
