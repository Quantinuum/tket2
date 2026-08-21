//! This module provides functionality to modify TketOp operations in a quantum circuit.
use hugr::{
    HugrView,
    ops::handle::NodeHandle,
    std_extensions::arithmetic::{float_ops::FloatOps, float_types::ConstF64},
};

use crate::{
    TketOp,
    TketOp::*,
    extension::rotation::{ConstRotation, RotationOp},
    modifier::modifier_resolver::*,
};

impl<N: HugrNode> ModifierResolver<N> {
    fn state_order_input(pv: &PortVector) -> Result<DirWire, ModifierResolverErrors<N>> {
        pv.incoming.last().copied().ok_or_else(|| {
            ModifierResolverErrors::unreachable(
                "Missing StateOrder input while expanding an operation.".to_string(),
            )
        })
    }

    fn state_order_output(pv: &PortVector) -> Result<DirWire, ModifierResolverErrors<N>> {
        pv.outgoing.last().copied().ok_or_else(|| {
            ModifierResolverErrors::unreachable(
                "Missing StateOrder output while expanding an operation.".to_string(),
            )
        })
    }

    fn connect_state_order(
        new_fn: &mut impl Dataflow,
        source: DirWire,
        target: DirWire,
    ) -> Result<(), ModifierResolverErrors<N>> {
        connect(new_fn, &source, &target)
    }

    /// Modify a TketOp operation. The returned `PortVector` contains the incoming and outgoing
    /// ports of the modified operation.
    /// Ancilla qubits are dirty qubits that are used to store intermediate results.
    pub(crate) fn modify_tket_op(
        &mut self,
        op_node: N,
        tket_op: TketOp,
        new_fn: &mut impl Dataflow,
        ancilla: &mut Vec<Wire<Node>>,
    ) -> Result<PortVector, ModifierResolverErrors<N>> {
        let control = self.control_num();
        let dagger = self.modifiers.dagger;

        // No modification is needed.
        if control == 0 && !dagger {
            let new = new_fn.add_child_node(tket_op);
            let incoming = 0..new_fn.hugr().num_inputs(new);
            let outgoing = 0..new_fn.hugr().num_outputs(new);
            return Ok(PortVector::from_single_node(new, incoming, outgoing));
        }
        match tket_op {
            X | CX | Toffoli | Y | CY | Z | CZ | S | Sdg | T | Tdg | V | Vdg | H
                if (control == 0 && dagger)
                    || ((1..3).contains(&control) && tket_op == X)
                    || (control == 1 && matches!(tket_op, CX | Y | Z)) =>
            {
                // The controlled or daggered G is itself a TketOp: emit it directly.
                let gate = self
                    .modifiers
                    .modified(tket_op)
                    .unwrap_or_else(|| unreachable!());

                let qubits = match tket_op {
                    X | Y | Z | S | T | V | Sdg | Tdg | Vdg | H => 1,
                    CX | CY | CZ => 2,
                    Toffoli => 3,
                    _ => unreachable!(),
                };

                let new = self.add_node_control(new_fn, gate);
                let incoming = control..new_fn.hugr().num_inputs(new);
                let outgoing = control..new_fn.hugr().num_outputs(new);
                let if_rev = control..(control + qubits);
                Ok(self.port_vector_dagger(new, incoming, outgoing, if_rev))
            }
            Rz | CRz | Rx | Ry if (control == 0 && dagger) || (control == 1 && tket_op == Rz) => {
                // The modified rotation is itself a TketOp; for G†(θ), negate θ.
                let qubits = if CRz == tket_op { 2 } else { 1 };

                let new_op = self
                    .modifiers
                    .modified(tket_op)
                    .unwrap_or_else(|| unreachable!());
                let new = self.add_node_control(new_fn, new_op);

                if !dagger {
                    let incoming = control..new_fn.hugr().num_inputs(new);
                    let outgoing = control..new_fn.hugr().num_outputs(new);
                    Ok(PortVector::from_single_node(new, incoming, outgoing))
                } else {
                    // If daggered
                    let halfturn = new_fn.add_child_node(RotationOp::to_halfturns);
                    let reversed_float = new_fn
                        .add_dataflow_op(FloatOps::fneg, vec![Wire::new(halfturn, 0)])
                        .map(|out| out.out_wire(0))?;
                    let reversed = new_fn
                        .add_dataflow_op(RotationOp::from_halfturns_unchecked, vec![reversed_float])
                        .map(|out| out.out_wire(0))?;

                    new_fn.hugr_mut().connect(
                        reversed.node(),
                        reversed.source(),
                        new,
                        qubits + control,
                    );

                    let incoming = (control..new_fn.hugr().num_inputs(new))
                        .filter_map(|i| {
                            if i < qubits + control {
                                Some((new, OutgoingPort::from(i)).into())
                            } else if i == qubits + control {
                                Some((halfturn, IncomingPort::from(0)).into())
                            } else {
                                // NOTE: StateOrder edges are intentionally ignored in a dagger context.
                                None
                            }
                        })
                        .collect();
                    let outgoing = (control..new_fn.hugr().num_outputs(new))
                        .map(|i| {
                            let dw: DirWire = (new, IncomingPort::from(i)).into();
                            if i >= qubits + control {
                                dw.shift(1)
                            } else {
                                dw
                            }
                        })
                        .collect();
                    Ok(PortVector { incoming, outgoing })
                }
            }
            H => {
                // CnH(cs,t) = CnRy(cs,t,π/2); CnX(cs,t). For H†, reverse and dagger.
                let (mut pv_ry, pv_x) = if !dagger {
                    (
                        self.modify_tket_op(op_node, Ry, new_fn, ancilla)?,
                        self.modify_tket_op(op_node, X, new_fn, ancilla)?,
                    )
                } else {
                    let (pv_x, pv_ry) = (
                        self.modify_tket_op(op_node, X, new_fn, ancilla)?,
                        self.modify_tket_op(op_node, Ry, new_fn, ancilla)?,
                    );
                    (pv_ry, pv_x)
                };
                let angle = new_fn.add_load_value(ConstRotation::new(0.5).unwrap());
                // let angle = new_fn
                //     .add_dataflow_op(RotationOp::from_halfturns_unchecked, vec![angle])
                //     .unwrap()
                //     .out_wire(0);
                let rot_in = pv_ry.incoming.remove(1);
                connect(new_fn, &rot_in, &angle.into())?;
                connect(new_fn, &pv_ry.outgoing[0], &pv_x.incoming[0])?;

                Ok(PortVector {
                    incoming: pv_ry.incoming,
                    outgoing: pv_x.outgoing,
                })
            }
            Rx => {
                // CnRx(cs,t,θ) = H(t); CnRz(cs,t,θ); H(t).
                let h1 = new_fn.add_child_node(H);
                let h2 = new_fn.add_child_node(H);
                let mut pv = self.modify_tket_op(op_node, Rz, new_fn, ancilla)?;
                pv.incoming[0] = connect_by_num(new_fn, &pv.incoming[0], h1, 0);
                pv.outgoing[0] = connect_by_num(new_fn, &pv.outgoing[0], h2, 0);
                Ok(pv)
            }
            Ry | CY => {
                // CnRy(cs,t,θ) = Sdg(t); CnRx(cs,t,θ); S(t).
                // CnCY(cs,c,t) = Sdg(t); CnCX(cs,c,t); S(t).
                let (gate, targ) = match tket_op {
                    Ry => (Rx, 0),
                    CY => (CX, 1),
                    _ => unreachable!(),
                };
                let s = new_fn.add_child_node(S);
                let sdg = new_fn.add_child_node(Sdg);
                let mut pv = self.modify_tket_op(op_node, gate, new_fn, ancilla)?;
                if !dagger {
                    pv.incoming[targ] = connect_by_num(new_fn, &pv.incoming[targ], sdg, 0);
                    pv.outgoing[targ] = connect_by_num(new_fn, &pv.outgoing[targ], s, 0);
                } else {
                    pv.outgoing[targ] = connect_by_num(new_fn, &pv.outgoing[targ], sdg, 0);
                    pv.incoming[targ] = connect_by_num(new_fn, &pv.incoming[targ], s, 0);
                }
                Ok(pv)
            }
            T | Tdg | S | Sdg | V | Vdg => {
                // CnOp(cs,t) = CnU(cs,t,2θ); CnPhase(cs,θ).
                let Some((gate, angle)) = self.modifiers.rot_angle(tket_op) else {
                    unreachable!()
                };

                self.modifiers.dagger = false;

                let rot = new_fn.add_load_value(ConstRotation::new(angle).unwrap());
                let rot_2 = new_fn.add_load_value(ConstRotation::new(angle * 2.0).unwrap());

                // CU(cs,t,2θ);
                let mut pv_u = self.modify_tket_op(op_node, gate, new_fn, ancilla)?;
                connect(new_fn, &rot_2.into(), &pv_u.incoming[1])?;
                let mut t = pv_u.outgoing[0].try_into().unwrap();

                // CPhase(cs,θ);
                let theta_inputs = self.with_ancilla(&mut t, ancilla, |this, ancilla| {
                    this.modify_global_phase(op_node, new_fn, ancilla)
                })?;
                pv_u.outgoing[0] = t.into();
                for theta_in in theta_inputs {
                    new_fn
                        .hugr_mut()
                        .connect(rot.node(), rot.source(), theta_in.0, theta_in.1);
                }

                if dagger {
                    mem::swap(&mut pv_u.incoming, &mut pv_u.outgoing)
                }
                self.modifiers.dagger = dagger;

                Ok(pv_u)
            }
            // If more control qubits
            Toffoli if !ancilla.is_empty() => {
                // Cn+m+2X(cs1,cs2,x,y,t) = Cn+2X(cs1,x,y,a); Cm+1X(cs2,a,t); Cn+2X(cs1,x,y,a); Cm+1X(cs2,a,t);
                let nd = op_node;
                self.modifiers.dagger = false;
                let mut a = ancilla.pop().unwrap().into();

                let n = control / 2;
                let m = control - n;
                // We know 1, n <= m.
                let mut cs2 = self.controls().split_off(n);

                // 1. Cn+2X(cs1,x,y,a)
                self.modifiers.control = n;
                let cs2_last = cs2.last_mut().unwrap();
                let pv1 = self.with_ancilla(cs2_last, ancilla, |this, ancilla| {
                    this.modify_tket_op(nd, Toffoli, new_fn, ancilla)
                })?;
                let pv1_state_in = Self::state_order_input(&pv1)?;
                let pv1_state_out = Self::state_order_output(&pv1)?;
                connect(new_fn, &a, &pv1.incoming[2])?;
                let x_in = pv1.incoming[0];
                let y_in = pv1.incoming[1];
                let mut x = pv1.outgoing[0].try_into().unwrap();
                let mut y = pv1.outgoing[1];
                a = pv1.outgoing[2];

                // 2. Cm+1X(cs2,a,t)
                self.modifiers.control = m;
                let cs1 = mem::replace(self.controls(), cs2);
                let pv2 = self.with_ancilla(&mut x, ancilla, |this, ancilla| {
                    this.modify_tket_op(nd, CX, new_fn, ancilla)
                })?;
                let pv2_state_in = Self::state_order_input(&pv2)?;
                let pv2_state_out = Self::state_order_output(&pv2)?;
                connect(new_fn, &a, &pv2.incoming[0])?;
                a = pv2.outgoing[0];
                let t_in = pv2.incoming[1];
                let mut t = pv2.outgoing[1];

                // 3. Cn+2X(cs1,x,y,a)
                self.modifiers.control = n;
                let mut cs2 = mem::replace(self.controls(), cs1);
                let cs2_last = cs2.last_mut().unwrap();
                let pv3 = self.with_ancilla(cs2_last, ancilla, |this, ancilla| {
                    this.modify_tket_op(nd, Toffoli, new_fn, ancilla)
                })?;
                let pv3_state_in = Self::state_order_input(&pv3)?;
                let pv3_state_out = Self::state_order_output(&pv3)?;
                connect(new_fn, &x.into(), &pv3.incoming[0])?;
                connect(new_fn, &y, &pv3.incoming[1])?;
                connect(new_fn, &a, &pv3.incoming[2])?;
                x = pv3.outgoing[0].try_into().unwrap();
                y = pv3.outgoing[1];
                a = pv3.outgoing[2];

                // 4. Cm+1X(cs2,a,t)
                self.modifiers.control = m;
                let cs1 = mem::replace(self.controls(), cs2);
                let pv4 = self.with_ancilla(&mut x, ancilla, |this, ancilla| {
                    this.modify_tket_op(nd, CX, new_fn, ancilla)
                })?;
                let pv4_state_in = Self::state_order_input(&pv4)?;
                let pv4_state_out = Self::state_order_output(&pv4)?;
                connect(new_fn, &a, &pv4.incoming[0])?;
                connect(new_fn, &t, &pv4.incoming[1])?;
                a = pv4.outgoing[0];
                t = pv4.outgoing[1];

                self.modifiers.control = control;
                self.modifiers.dagger = dagger;
                let cs2 = mem::replace(self.controls(), cs1);
                self.controls().extend(cs2);
                ancilla.push(a.try_into().unwrap());
                let mut incoming = vec![x_in, y_in, t_in];
                let mut outgoing = vec![x.into(), y, t];
                if dagger {
                    mem::swap(&mut incoming, &mut outgoing);
                } else {
                    if self.should_insert_state_order_edges() {
                        Self::connect_state_order(new_fn, pv1_state_out, pv2_state_in)?;
                        Self::connect_state_order(new_fn, pv2_state_out, pv3_state_in)?;
                        Self::connect_state_order(new_fn, pv3_state_out, pv4_state_in)?;
                    }
                    incoming.push(pv1_state_in);
                    outgoing.push(pv4_state_out);
                }
                Ok(PortVector { incoming, outgoing })
            }
            CX | X if !ancilla.is_empty() => {
                // Reinterpret modifier controls as native controls, reducing CnX or Cn+1X to Cn-2+2X or Cn-1+2X.
                let c_num = if tket_op == X { 2 } else { 1 };
                let mut ctrls = vec![];
                for _ in 0..c_num {
                    ctrls.push(self.pop_control().unwrap());
                }

                let mut pv = self.modify_tket_op(op_node, Toffoli, new_fn, ancilla)?;

                if dagger {
                    mem::swap(&mut pv.incoming, &mut pv.outgoing)
                }
                for _ in 0..c_num {
                    let c = ctrls.pop().unwrap();
                    let c_in = pv.incoming.remove(0);
                    connect(new_fn, &c.into(), &c_in)?;
                    let c_out = pv.outgoing.remove(0).try_into().unwrap();
                    self.push_control(c_out);
                }
                if dagger {
                    mem::swap(&mut pv.incoming, &mut pv.outgoing)
                }
                Ok(pv)
            }
            CRz => {
                // Cn+1Rz(cs,c,t,theta) = Rz(t,theta/2); Cn+1X(cs,c,t); Rz(t,-theta/2); Cn+1X(cs,c,t);
                self.modifiers.dagger = false;

                // rotations
                let halfturns = new_fn.add_child_node(RotationOp::to_halfturns);
                let half_const = new_fn.add_load_value(ConstF64::new(0.5));
                let half_f64 = new_fn
                    .add_dataflow_op(FloatOps::fmul, vec![Wire::new(halfturns, 0), half_const])
                    .map(|out| out.out_wire(0))?;
                let half_f64_neg = new_fn
                    .add_dataflow_op(FloatOps::fneg, vec![half_f64])
                    .map(|out| out.out_wire(0))?;
                let mut angle_pos = new_fn
                    .add_dataflow_op(RotationOp::from_halfturns_unchecked, vec![half_f64])
                    .map(|out| out.node())?;
                let mut angle_neg = new_fn
                    .add_dataflow_op(RotationOp::from_halfturns_unchecked, vec![half_f64_neg])
                    .map(|out| out.node())?;
                if dagger {
                    mem::swap(&mut angle_pos, &mut angle_neg);
                }

                // Rz(t,theta/2)
                let crz_pos = new_fn.add_child_node(Rz);
                new_fn.hugr_mut().connect(angle_pos, 0, crz_pos, 1);
                let mut t = Wire::new(crz_pos, 0).into();

                // CnCX(cs,c,t)
                let pv1 = self.modify_tket_op(op_node, CX, new_fn, ancilla)?;
                let pv1_state_in = Self::state_order_input(&pv1)?;
                let pv1_state_out = Self::state_order_output(&pv1)?;
                let mut incoming = vec![pv1.incoming[0], (crz_pos, IncomingPort::from(0)).into()];
                connect(new_fn, &t, &pv1.incoming[1])?;
                let mut c = pv1.outgoing[0];
                t = pv1.outgoing[1];

                // Rz(t,-theta/2)
                let crz_neg = new_fn.add_child_node(Rz);
                t = connect_by_num(new_fn, &t, crz_neg, 0);
                new_fn.hugr_mut().connect(angle_neg, 0, crz_neg, 1);

                // CnCX(cs,c,t)
                let pv2 = self.modify_tket_op(op_node, CX, new_fn, ancilla)?;
                let pv2_state_in = Self::state_order_input(&pv2)?;
                let pv2_state_out = Self::state_order_output(&pv2)?;
                connect(new_fn, &c, &pv2.incoming[0])?;
                connect(new_fn, &t, &pv2.incoming[1])?;
                c = pv2.outgoing[0];
                t = pv2.outgoing[1];
                let mut outgoing = vec![c, t];

                self.modifiers.dagger = dagger;
                if dagger {
                    mem::swap(&mut incoming, &mut outgoing);
                    incoming.push((halfturns, IncomingPort::from(0)).into());
                } else {
                    incoming.push((halfturns, IncomingPort::from(0)).into());
                    let crz_pos_state_in = (
                        crz_pos,
                        new_fn
                            .hugr()
                            .get_optype(crz_pos)
                            .other_input_port()
                            .unwrap(),
                    )
                        .into();
                    let crz_pos_state_out = (
                        crz_pos,
                        new_fn
                            .hugr()
                            .get_optype(crz_pos)
                            .other_output_port()
                            .unwrap(),
                    )
                        .into();
                    let crz_neg_state_in = (
                        crz_neg,
                        new_fn
                            .hugr()
                            .get_optype(crz_neg)
                            .other_input_port()
                            .unwrap(),
                    )
                        .into();
                    let crz_neg_state_out = (
                        crz_neg,
                        new_fn
                            .hugr()
                            .get_optype(crz_neg)
                            .other_output_port()
                            .unwrap(),
                    )
                        .into();

                    if self.should_insert_state_order_edges() {
                        Self::connect_state_order(new_fn, crz_pos_state_out, pv1_state_in)?;
                        Self::connect_state_order(new_fn, pv1_state_out, crz_neg_state_in)?;
                        Self::connect_state_order(new_fn, crz_neg_state_out, pv2_state_in)?;
                    }
                    incoming.push(crz_pos_state_in);
                    outgoing.push(pv2_state_out);
                }
                Ok(PortVector { incoming, outgoing })
            }
            Rz | Y | Z => {
                // CnG(cs,c,t) = Cn-1(CG)(cs,c,t), for G = Rz, Y, or Z.
                let c_op = if tket_op == Rz {
                    CRz
                } else if tket_op == Y {
                    CY
                } else {
                    CZ
                };
                let mut last_control = self.pop_control().unwrap();

                let mut pv = self.modify_tket_op(op_node, c_op, new_fn, ancilla)?;
                let last_dw = if !dagger {
                    let c_in = pv.incoming.remove(0);
                    connect(new_fn, &c_in, &last_control.into())?;
                    pv.outgoing.remove(0)
                } else {
                    let c_out = pv.outgoing.remove(0);
                    connect(new_fn, &c_out, &last_control.into())?;
                    pv.incoming.remove(0)
                };

                last_control = last_dw.try_into().map_err(|_| {
                    ModifierResolverErrors::unreachable(
                        "Expected outgoing wire, found incoming wire while modifying Rz"
                            .to_string(),
                    )
                })?;

                self.push_control(last_control);

                Ok(pv)
            }
            CZ => {
                // Cn+1Z(cs,c,t) = H(t); Cn+1X(cs,c,t); H(t).
                let h1 = new_fn.add_child_node(H);
                let h2 = new_fn.add_child_node(H);
                let mut pv = self.modify_tket_op(op_node, CX, new_fn, ancilla)?;
                pv.incoming[1] = connect_by_num(new_fn, &pv.incoming[1], h1, 0);
                pv.outgoing[1] = connect_by_num(new_fn, &pv.outgoing[1], h2, 0);
                Ok(pv)
            }
            X | CX | Toffoli => {
                // Cn+1X(cs,c,t) = CV(c,t); CnX(cs,c); CVdg(c,t); CnX(cs,c); CnV(cs,t);
                let gate_control = match tket_op {
                    X => 0,
                    CX => 1,
                    Toffoli => 2,
                    _ => unreachable!(),
                };
                self.modifiers.dagger = false;
                let mut incoming = Vec::new();
                let mut outgoing = Vec::new();

                // CV(c,t)
                self.modifiers.control = 1;
                let c = self.controls().pop().unwrap();
                let cs = mem::replace(self.controls(), vec![c]);
                let pv_crx1 = self.modify_tket_op(op_node, V, new_fn, ancilla)?;
                let pv_crx1_state_in = Self::state_order_input(&pv_crx1)?;
                let pv_crx1_state_out = Self::state_order_output(&pv_crx1)?;
                incoming.push(pv_crx1.incoming[0]);
                let mut targ = pv_crx1.outgoing[0].try_into().unwrap();

                // CnX(cs,c)
                self.modifiers.control = control - 1;
                let c = mem::replace(self.controls(), cs)[0];
                let pv_x1 = self.with_ancilla(&mut targ, ancilla, |this, ancilla| {
                    this.modify_tket_op(op_node, tket_op, new_fn, ancilla)
                })?;
                let pv_x1_state_in = Self::state_order_input(&pv_x1)?;
                let pv_x1_state_out = Self::state_order_output(&pv_x1)?;
                connect(new_fn, &c.into(), &pv_x1.incoming[gate_control])?;
                let c = pv_x1.outgoing[gate_control].try_into().unwrap();
                for i in 0..gate_control {
                    incoming.insert(i, pv_x1.incoming[i]);
                }

                // CVdg(c,t)
                self.modifiers.control = 1;
                let cs = mem::replace(self.controls(), vec![c]);
                let pv_crx2 = self.modify_tket_op(op_node, Vdg, new_fn, ancilla)?;
                let pv_crx2_state_in = Self::state_order_input(&pv_crx2)?;
                let pv_crx2_state_out = Self::state_order_output(&pv_crx2)?;
                connect(new_fn, &targ.into(), &pv_crx2.incoming[0])?;
                targ = pv_crx2.outgoing[0].try_into().unwrap();

                // CnX(cs,c)
                self.modifiers.control = control - 1;
                let mut c = mem::replace(self.controls(), cs)[0];
                assert_eq!(self.controls().len(), self.control_num());
                let pv_x2 = self.with_ancilla(&mut targ, ancilla, |this, ancilla| {
                    this.modify_tket_op(op_node, tket_op, new_fn, ancilla)
                })?;
                let pv_x2_state_in = Self::state_order_input(&pv_x2)?;
                let pv_x2_state_out = Self::state_order_output(&pv_x2)?;
                connect(new_fn, &c.into(), &pv_x2.incoming[gate_control])?;
                c = pv_x2.outgoing[gate_control].try_into().unwrap();
                for i in 0..gate_control {
                    connect(new_fn, &pv_x1.outgoing[i], &pv_x2.incoming[i])?;
                }

                // CnV(cs,t)
                // self.control_num() = control + gate_control - 1;
                for i in 0..gate_control {
                    self.push_control(pv_x2.outgoing[i].try_into().unwrap());
                }
                let pv_cnrx = self.with_ancilla(&mut c, ancilla, |this, ancilla| {
                    this.modify_tket_op(op_node, V, new_fn, ancilla)
                })?;
                let pv_cnrx_state_in = Self::state_order_input(&pv_cnrx)?;
                let pv_cnrx_state_out = Self::state_order_output(&pv_cnrx)?;
                for _ in 0..gate_control {
                    outgoing.push(self.pop_control().unwrap().into());
                }
                connect(new_fn, &targ.into(), &pv_cnrx.incoming[0])?;
                // connect(new_fn, &half_pos.into(), &pv_cnrx.incoming[1])?;
                outgoing.push(pv_cnrx.outgoing[0]);

                self.push_control(c);
                assert_eq!(control, self.control_num());
                self.modifiers.dagger = dagger;

                if !dagger {
                    if self.should_insert_state_order_edges() {
                        Self::connect_state_order(new_fn, pv_crx1_state_out, pv_x1_state_in)?;
                        Self::connect_state_order(new_fn, pv_x1_state_out, pv_crx2_state_in)?;
                        Self::connect_state_order(new_fn, pv_crx2_state_out, pv_x2_state_in)?;
                        Self::connect_state_order(new_fn, pv_x2_state_out, pv_cnrx_state_in)?;
                    }
                    incoming.push(pv_crx1_state_in);
                    outgoing.push(pv_cnrx_state_out);
                    Ok(PortVector { incoming, outgoing })
                } else {
                    Ok(PortVector {
                        incoming: outgoing,
                        outgoing: incoming,
                    })
                }
            }
            Measure | MeasureFree | QAlloc | TryQAlloc | QFree | Reset => {
                Err(ModifierResolverErrors::unresolvable(
                    op_node,
                    "non-unitary operations are not expected in a modified context.".to_string(),
                    tket_op.into(),
                ))
            }
        }
    }
}

impl CombinedModifier {
    /// If the modified operation can be represented as a TketOp,
    /// returns the modified operation, otherwise returns `None`.
    ///
    /// Not all the cases are handled here, since we assume that
    /// unmodified operations are handled directly in `modify_tket_op`.
    fn modified(&self, op: TketOp) -> Option<TketOp> {
        match (op, self.control, self.dagger) {
            (X, 0, _) => Some(X),
            (X, 1, _) => Some(CX),
            (X, 2, _) => Some(Toffoli),
            (Y, 0, _) => Some(Y),
            (Y, 1, _) => Some(CY),
            (Z, 0, _) => Some(Z),
            (Z, 1, _) => Some(CZ),
            (CX, 0, _) => Some(CX),
            (CX, 1, _) => Some(Toffoli),
            (CY, 0, _) => Some(CY),
            (CZ, 0, _) => Some(CZ),
            (Toffoli, 0, _) => Some(Toffoli),
            (H, 0, _) => Some(H),
            (Rz, 0, _) => Some(Rz),
            (Rz, 1, _) => Some(CRz),
            (CRz, 0, _) => Some(CRz),
            (Rx, 0, _) => Some(Rx),
            (Ry, 0, _) => Some(Ry),
            (T, 0, true) => Some(Tdg),
            (Tdg, 0, true) => Some(T),
            (S, 0, true) => Some(Sdg),
            (Sdg, 0, true) => Some(S),
            (V, 0, true) => Some(Vdg),
            (Vdg, 0, true) => Some(V),
            _ => None,
        }
    }

    // op = exp(θ) * U(2θ)
    fn rot_angle(&self, op: TketOp) -> Option<(TketOp, f64)> {
        let (op, mut angle) = match op {
            S => (Rz, 0.25),
            T => (Rz, 0.125),
            Tdg => (Rz, -0.125),
            Sdg => (Rz, -0.25),
            V => (Rx, 0.25),
            Vdg => (Rx, -0.25),
            _ => return None,
        };
        if self.dagger {
            angle = -angle;
        }
        Some((op, angle))
    }
}

#[cfg(test)]
mod test {
    use cool_asserts::assert_matches;
    use hugr::{
        Hugr, IncomingPort, OutgoingPort,
        builder::{Dataflow, DataflowSubContainer, HugrBuilder, ModuleBuilder},
        extension::prelude::qb_t,
        ops::CallIndirect,
        std_extensions::collections::array::array_type,
        types::{EdgeKind, Signature, Term},
    };
    use strum::IntoEnumIterator;

    use crate::extension::{
        modifier::{CONTROL_OP_ID, DAGGER_OP_ID, MODIFIER_EXTENSION},
        rotation::rotation_type,
    };
    use crate::{
        extension::rotation::ConstRotation,
        modifier::modifier_resolver::tests::{SetUnitary, test_modifier_resolver},
        modifier::modifier_resolver::*,
    };

    fn size(op: TketOp) -> Option<(usize, bool)> {
        use TketOp::*;
        let p = match op {
            X | Y | Z | S | Sdg | T | Tdg | V | Vdg | H => (1, false),
            Rz | Rx | Ry => (1, true),
            CX | CY | CZ => (2, false),
            CRz => (2, true),
            Toffoli => (3, false),
            Measure | MeasureFree | QAlloc | TryQAlloc | QFree | Reset => return None,
        };
        Some(p)
    }

    #[test]
    fn unmodified_tket_op_is_copied_directly() {
        let mut module = ModuleBuilder::new();
        let mut func = module
            .define_function("foo", Signature::new_endo([qb_t()]))
            .unwrap();
        let op_node = func.add_child_node(TketOp::X);
        let mut resolver = ModifierResolver::new();

        let port_vector = resolver
            .modify_tket_op(op_node, TketOp::X, &mut func, &mut vec![])
            .unwrap();

        let (new_node, first_input): (_, IncomingPort) =
            port_vector.incoming[0].try_into().unwrap();
        assert_eq!(first_input, IncomingPort::from(0));

        let expected_incoming = (0..func.hugr().num_inputs(new_node))
            .map(|port| DirWire::from((new_node, IncomingPort::from(port))))
            .collect::<Vec<_>>();
        let expected_outgoing = (0..func.hugr().num_outputs(new_node))
            .map(|port| DirWire::from((new_node, OutgoingPort::from(port))))
            .collect::<Vec<_>>();

        assert_eq!(port_vector.incoming, expected_incoming);
        assert_eq!(port_vector.outgoing, expected_outgoing);
        assert_eq!(func.hugr().get_optype(new_node), &TketOp::X.into());
    }

    #[test]
    fn controlled_toffoli_expansion_preserves_state_order() {
        let mut old_module = ModuleBuilder::new();
        let mut old_func = old_module
            .define_function("old", Signature::new_endo([qb_t(), qb_t(), qb_t(), qb_t()]))
            .unwrap();
        let previous = old_func.add_child_node(TketOp::X);
        let op_node = old_func.add_child_node(TketOp::Toffoli);
        let state_order_output = old_func
            .hugr()
            .get_optype(previous)
            .other_output_port()
            .unwrap();
        let state_order_input = old_func
            .hugr()
            .get_optype(op_node)
            .other_input_port()
            .unwrap();
        old_func
            .hugr_mut()
            .connect(previous, state_order_output, op_node, state_order_input);
        let insert_state_order_edges = old_func
            .hugr()
            .single_linked_output(op_node, state_order_input)
            .is_some();
        assert!(insert_state_order_edges);

        let mut module = ModuleBuilder::new();
        let mut func = module
            .define_function("new", Signature::new_endo([qb_t(), qb_t(), qb_t(), qb_t()]))
            .unwrap();
        let inputs = func.input_wires().collect::<Vec<_>>();
        let mut resolver = ModifierResolver::new();
        resolver.modifiers.control = 3;
        resolver.controls = inputs[..3].to_vec();

        let port_vector = resolver.with_state_order_edges(insert_state_order_edges, |resolver| {
            resolver
                .modify_tket_op(op_node, TketOp::Toffoli, &mut func, &mut vec![inputs[3]])
                .unwrap()
        });

        assert_eq!(port_vector.incoming.len(), 4);
        assert_eq!(port_vector.outgoing.len(), 4);
        let h = func.hugr();
        let state_order_edges = h
            .nodes()
            .map(|node| {
                h.node_outputs(node)
                    .filter(|port| {
                        h.get_optype(node).port_kind(*port) == Some(EdgeKind::StateOrder)
                    })
                    .map(|port| h.linked_inputs(node, port).count())
                    .sum::<usize>()
            })
            .sum::<usize>();
        assert_eq!(state_order_edges, 15);
    }

    #[rstest::rstest]
    #[case(0, true)]
    #[case(1, false)]
    #[case(3, false)]
    #[case(3, true)]
    #[case(7, false)]
    #[cfg_attr(miri, ignore)] // miri takes a long time to analyze this test.
    fn test_single_tket_op(#[case] c_num: u64, #[case] dagger: bool) {
        for op in TketOp::iter() {
            let Some((size, has_angle)) = size(op) else {
                continue;
            };
            let foo = |module: &mut ModuleBuilder<Hugr>, t_num: usize| {
                let foo_sig =
                    Signature::new_endo(iter::repeat_n(qb_t(), t_num).collect::<Vec<_>>());
                let mut func = module.define_function("foo", foo_sig.clone()).unwrap();
                func.set_unitary();
                let mut inputs: Vec<_> = func.input_wires().collect();
                let mut args = inputs[0..size].to_vec();
                if has_angle {
                    let angle = func.add_load_value(ConstRotation::new(0.5).unwrap());
                    args.push(angle);
                }
                let v = func.add_dataflow_op(op, args).unwrap().outputs();
                inputs.splice(0..size, v);
                *func.finish_with_outputs(inputs).unwrap().handle()
            };
            test_modifier_resolver(3, c_num, foo, dagger);
        }
    }

    #[test]
    fn non_unitary_tket_ops_cannot_be_modified() {
        for op in [
            TketOp::Measure,
            TketOp::MeasureFree,
            TketOp::QAlloc,
            TketOp::TryQAlloc,
            TketOp::QFree,
            TketOp::Reset,
        ] {
            let mut module = ModuleBuilder::new();
            let mut func = module
                .define_function("foo", Signature::new_endo([]))
                .unwrap();
            let op_node = func.add_child_node(op);
            let mut resolver = ModifierResolver::new();
            resolver.modifiers.dagger = true;

            let result = resolver.modify_tket_op(op_node, op, &mut func, &mut vec![]);
            match result {
                Err(ModifierResolverErrors::UnResolvable { node, msg, optype }) => {
                    assert_eq!(node, op_node);
                    assert_eq!(
                        msg,
                        "non-unitary operations are not expected in a modified context."
                    );
                    assert_eq!(optype, op.into());
                }
                Err(error) => panic!("expected {op:?} to be unresolvable, got {error:?}"),
                Ok(_) => panic!("expected {op:?} to be rejected"),
            }
        }
    }

    #[test]
    /// Test that when no modifiers are applied non unitary operations are handled correctly.
    fn double_dagger_allows_measurement_function() {
        let mut module = ModuleBuilder::new();
        let measure_sig = Signature::new_endo([qb_t()]);

        let dagger_op = MODIFIER_EXTENSION
            .instantiate_extension_op(&DAGGER_OP_ID, [Term::new_list([qb_t()]), vec![].into()])
            .unwrap();

        let measured = {
            let mut func = module
                .define_function("measured", measure_sig.clone())
                .unwrap();
            let q = func.input_wires().next().unwrap();
            let [q, _result] = func
                .add_dataflow_op(TketOp::Measure, [q])
                .unwrap()
                .outputs_arr();
            *func.finish_with_outputs([q]).unwrap().handle()
        };

        {
            let mut func = module
                .define_function("main", Signature::new(vec![], [qb_t()]))
                .unwrap();
            let loaded = func.load_func(&measured, &[]).unwrap();
            let daggered_once = func
                .add_dataflow_op(dagger_op.clone(), [loaded])
                .unwrap()
                .out_wire(0);
            let daggered_twice = func
                .add_dataflow_op(dagger_op, [daggered_once])
                .unwrap()
                .out_wire(0);
            let q = func
                .add_dataflow_op(TketOp::QAlloc, [])
                .unwrap()
                .out_wire(0);
            let outputs = func
                .add_dataflow_op(
                    CallIndirect {
                        signature: measure_sig,
                    },
                    [daggered_twice, q],
                )
                .unwrap()
                .outputs();
            func.finish_with_outputs(outputs).unwrap();
        }

        let mut h = module.finish_hugr().unwrap();
        assert_matches!(h.validate(), Ok(()));
        let entrypoint = h.entrypoint();
        resolve_modifier_with_entrypoints(&mut h, [entrypoint]).unwrap();
        assert_matches!(h.validate(), Ok(()));
        assert!(
            h.nodes()
                .all(|node| Modifier::from_optype(h.get_optype(node)).is_none())
        );
        assert!(
            h.nodes()
                .any(|node| TketOp::from_optype(h.get_optype(node)) == Some(TketOp::Measure))
        );
    }

    #[test]
    fn test_modify_complex() {
        let mut module = ModuleBuilder::new();
        let foo_sig = Signature::new(
            vec![qb_t(), qb_t(), qb_t(), rotation_type()],
            vec![qb_t(), qb_t(), qb_t()],
        );
        let call_sig = Signature::new(
            vec![
                array_type(1, qb_t()),
                qb_t(),
                qb_t(),
                qb_t(),
                rotation_type(),
            ],
            vec![array_type(1, qb_t()), qb_t(), qb_t(), qb_t()],
        );
        let main_sig = Signature::new(
            vec![array_type(1, qb_t()), qb_t(), qb_t(), qb_t()],
            vec![array_type(1, qb_t()), qb_t(), qb_t(), qb_t()],
        );

        let control_op = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &CONTROL_OP_ID,
                [
                    Term::BoundedNat(1),
                    Term::new_list([qb_t(), qb_t(), qb_t()]),
                    Term::new_list([rotation_type()]),
                ],
            )
            .unwrap();
        let dagger_op = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &DAGGER_OP_ID,
                [
                    Term::new_list([array_type(1, qb_t()), qb_t(), qb_t(), qb_t()]),
                    Term::new_list([rotation_type()]),
                ],
            )
            .unwrap();

        let foo = {
            let mut func = module.define_function("foo", foo_sig.clone()).unwrap();
            func.set_unitary();
            let [mut in1, mut in2, mut in3, in4] = func.input_wires_arr();
            let theta = func.add_load_value(ConstRotation::new(0.46).unwrap());
            in1 = func
                .add_dataflow_op(TketOp::Ry, vec![in1, theta])
                .unwrap()
                .out_wire(0);
            in1 = func
                .add_dataflow_op(TketOp::V, vec![in1])
                .unwrap()
                .out_wire(0);
            in2 = func
                .add_dataflow_op(TketOp::H, vec![in2])
                .unwrap()
                .out_wire(0);
            in2 = func
                .add_dataflow_op(TketOp::S, vec![in2])
                .unwrap()
                .out_wire(0);
            in3 = func
                .add_dataflow_op(TketOp::Z, vec![in3])
                .unwrap()
                .out_wire(0);
            in3 = func
                .add_dataflow_op(TketOp::Rx, vec![in3, in4])
                .unwrap()
                .out_wire(0);
            func.finish_with_outputs(vec![in1, in2, in3]).unwrap()
        };

        let _main = {
            let mut func = module.define_function("main", main_sig.clone()).unwrap();
            let loaded = func.load_func(foo.handle(), &[]).unwrap();
            let controlled = func
                .add_dataflow_op(control_op, vec![loaded])
                .unwrap()
                .out_wire(0);
            let daggered = func
                .add_dataflow_op(dagger_op, vec![controlled])
                .unwrap()
                .out_wire(0);
            let theta = func.add_load_value(ConstRotation::new(0.75).unwrap());
            let mut inputs = vec![daggered];
            inputs.extend(func.input_wires());
            inputs.push(theta);
            let outs = func
                .add_dataflow_op(
                    CallIndirect {
                        signature: call_sig,
                    },
                    inputs,
                )
                .unwrap()
                .outputs();
            func.finish_with_outputs(outs).unwrap()
        };

        let mut h = module.finish_hugr().unwrap();

        let entrypoint = h.entrypoint();
        resolve_modifier_with_entrypoints(&mut h, [entrypoint]).unwrap();
        assert_matches!(h.validate(), Ok(()));
    }
}
