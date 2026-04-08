//! This module provides functionality to modify TketOp operations in a quantum circuit.
use hugr::{
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
    /// Modify a TketOp operation. The returned `PortVector` contains the incoming and outgoing ports of the modified operation.
    /// Ancilla qubits are dirty qubits that are used to store intermediate results.
    pub fn modify_tket_op(
        &mut self,
        n: N,
        op: TketOp,
        new_fn: &mut impl Dataflow,
        ancilla: &mut Vec<Wire<Node>>,
    ) -> Result<PortVector, ModifierResolverErrors<N>> {
        let control = self.control_num();
        let dagger = self.modifiers.dagger;

        if (control != 0 || dagger) && !op.is_quantum() {
            return Err(ModifierResolverErrors::unresolvable(
                n,
                "None quantum operation cannot be modified".to_string(),
                op.into(),
            ));
        }
        match op {
            X | CX | Toffoli | Y | CY | Z | CZ | S | Sdg | T | Tdg | V | Vdg | H
                if (control == 0)
                    || (control < 3 && op == X)
                    || (control == 1 && matches!(op, CX | Y | Z)) =>
            {
                let gate = self
                    .modifiers
                    .modified(op)
                    .unwrap_or_else(|| unreachable!());

                let qubits = match op {
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
            Rz | CRz | Rx | Ry if control == 0 || (control == 1 && op == Rz) => {
                let qubits = if CRz == op { 2 } else { 1 };

                let new_op = self
                    .modifiers
                    .modified(op)
                    .unwrap_or_else(|| unreachable!());
                let new = self.add_node_control(new_fn, new_op);

                if !dagger {
                    let incoming = control..new_fn.hugr().num_inputs(new);
                    let outgoing = control..new_fn.hugr().num_outputs(new);
                    Ok(PortVector::from_single_node(new, incoming, outgoing))
                } else {
                    // If dagered
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
                                // FIXME: forget state order
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
                // H = X * Ry(pi/2).
                let (mut pv_ry, pv_x) = if !dagger {
                    (
                        self.modify_tket_op(n, Ry, new_fn, ancilla)?,
                        self.modify_tket_op(n, X, new_fn, ancilla)?,
                    )
                } else {
                    let (pv_x, pv_ry) = (
                        self.modify_tket_op(n, X, new_fn, ancilla)?,
                        self.modify_tket_op(n, Ry, new_fn, ancilla)?,
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
                let h1 = new_fn.add_child_node(H);
                let h2 = new_fn.add_child_node(H);
                let mut pv = self.modify_tket_op(n, Rz, new_fn, ancilla)?;
                pv.incoming[0] = connect_by_num(new_fn, &pv.incoming[0], h1, 0);
                pv.outgoing[0] = connect_by_num(new_fn, &pv.outgoing[0], h2, 0);
                Ok(pv)
            }
            Ry | CY => {
                let (gate, targ) = match op {
                    Ry => (Rx, 0),
                    CY => (CX, 1),
                    _ => unreachable!(),
                };
                let s = new_fn.add_child_node(S);
                let sdg = new_fn.add_child_node(Sdg);
                let mut pv = self.modify_tket_op(n, gate, new_fn, ancilla)?;
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
                // op(t) = Phase(θ) * U(t, 2θ)
                let Some((gate, angle)) = self.modifiers.rot_angle(op) else {
                    unreachable!()
                };

                self.modifiers.dagger = false;

                let rot = new_fn.add_load_value(ConstRotation::new(angle).unwrap());
                let rot_2 = new_fn.add_load_value(ConstRotation::new(angle * 2.0).unwrap());

                // CU(cs,t,2θ);
                let mut pv_u = self.modify_tket_op(n, gate, new_fn, ancilla)?;
                connect(new_fn, &rot_2.into(), &pv_u.incoming[1])?;
                let mut t = pv_u.outgoing[0].try_into().unwrap();

                // CPhase(cs,θ);
                let theta_inputs = self.with_ancilla(&mut t, ancilla, |this, ancilla| {
                    this.modify_global_phase(n, new_fn, ancilla)
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
                let nd = n;
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
                }
                Ok(PortVector { incoming, outgoing })
            }
            CX | X if !ancilla.is_empty() => {
                let c_num = if op == X { 2 } else { 1 };
                let mut ctrls = vec![];
                for _ in 0..c_num {
                    ctrls.push(self.pop_control().unwrap());
                }

                let mut pv = self.modify_tket_op(n, Toffoli, new_fn, ancilla)?;

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
                let pv1 = self.modify_tket_op(n, CX, new_fn, ancilla)?;
                let mut incoming = vec![pv1.incoming[0], (crz_pos, IncomingPort::from(0)).into()];
                connect(new_fn, &t, &pv1.incoming[1])?;
                let mut c = pv1.outgoing[0];
                t = pv1.outgoing[1];

                // Rz(t,-theta/2)
                let crz_neg = new_fn.add_child_node(Rz);
                t = connect_by_num(new_fn, &t, crz_neg, 0);
                new_fn.hugr_mut().connect(angle_neg, 0, crz_neg, 1);

                // CnCX(cs,c,t)
                let pv2 = self.modify_tket_op(n, CX, new_fn, ancilla)?;
                connect(new_fn, &c, &pv2.incoming[0])?;
                connect(new_fn, &t, &pv2.incoming[1])?;
                c = pv2.outgoing[0];
                t = pv2.outgoing[1];
                let mut outgoing = vec![c, t];

                self.modifiers.dagger = dagger;
                if dagger {
                    mem::swap(&mut incoming, &mut outgoing)
                }
                incoming.push((halfturns, IncomingPort::from(0)).into());
                // FIXME: Ignoring StateOrder
                Ok(PortVector { incoming, outgoing })
            }
            Rz | Y | Z => {
                // reduce Rz, Y, Z to CRz, CY, CZ
                let c_op = if op == Rz {
                    CRz
                } else if op == Y {
                    CY
                } else {
                    CZ
                };
                let mut last_control = self.pop_control().unwrap();

                let mut pv = self.modify_tket_op(n, c_op, new_fn, ancilla)?;
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
                // reduce CZ to CRz(pi)
                let mut pv = self.modify_tket_op(n, CRz, new_fn, ancilla)?;
                let halfturn = new_fn.add_load_value(ConstRotation::new(1.0).unwrap());
                let dw = pv.incoming.remove(2);
                connect(new_fn, &dw, &halfturn.into())?;
                Ok(pv)
            }
            X | CX | Toffoli => {
                // Cn+1X(cs,c,t) = CV(c,t); CnX(cs,c); CVdg(c,t); CnX(cs,c); CnV(cs,t);
                let gate_control = match op {
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
                let pv_crx1 = self.modify_tket_op(n, V, new_fn, ancilla)?;
                incoming.push(pv_crx1.incoming[0]);
                let mut targ = pv_crx1.outgoing[0].try_into().unwrap();

                // CnX(cs,c)
                self.modifiers.control = control - 1;
                let c = mem::replace(self.controls(), cs)[0];
                let pv_x1 = self.with_ancilla(&mut targ, ancilla, |this, ancilla| {
                    this.modify_tket_op(n, op, new_fn, ancilla)
                })?;
                connect(new_fn, &c.into(), &pv_x1.incoming[gate_control])?;
                let c = pv_x1.outgoing[gate_control].try_into().unwrap();
                for i in 0..gate_control {
                    incoming.insert(i, pv_x1.incoming[i]);
                }

                // CVdg(c,t)
                self.modifiers.control = 1;
                let cs = mem::replace(self.controls(), vec![c]);
                let pv_crx2 = self.modify_tket_op(n, Vdg, new_fn, ancilla)?;
                connect(new_fn, &targ.into(), &pv_crx2.incoming[0])?;
                targ = pv_crx2.outgoing[0].try_into().unwrap();

                // CnX(cs,c)
                self.modifiers.control = control - 1;
                let mut c = mem::replace(self.controls(), cs)[0];
                assert_eq!(self.controls().len(), self.control_num());
                let pv_x2 = self.with_ancilla(&mut targ, ancilla, |this, ancilla| {
                    this.modify_tket_op(n, op, new_fn, ancilla)
                })?;
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
                    this.modify_tket_op(n, V, new_fn, ancilla)
                })?;
                for _ in 0..gate_control {
                    outgoing.push(self.pop_control().unwrap().into());
                }
                connect(new_fn, &targ.into(), &pv_cnrx.incoming[0])?;
                // connect(new_fn, &half_pos.into(), &pv_cnrx.incoming[1])?;
                outgoing.push(pv_cnrx.outgoing[0]);

                self.push_control(c);
                assert_eq!(control, self.control_num());
                self.modifiers.dagger = dagger;

                // TODO: This does not handle invisible wires
                if !dagger {
                    Ok(PortVector { incoming, outgoing })
                } else {
                    Ok(PortVector {
                        incoming: outgoing,
                        outgoing: incoming,
                    })
                }
            }
            Measure | MeasureFree | QAlloc | TryQAlloc | QFree | Reset => {
                let new = new_fn.add_child_node(op);
                let incoming = 0..new_fn.hugr().num_inputs(new);
                let outgoing = 0..new_fn.hugr().num_outputs(new);
                Ok(PortVector::from_single_node(new, incoming, outgoing))
            }
        }
    }
}

impl CombinedModifier {
    /// If the modified operation can be represented as a TketOp,
    /// returns the modified operation, otherwise returns `None`.
    fn modified(&self, op: TketOp) -> Option<TketOp> {
        match op {
            X if self.control == 0 => Some(X),
            X if self.control == 1 => Some(CX),
            X if self.control == 2 => Some(Toffoli),
            Y if self.control == 0 => Some(Y),
            Y if self.control == 1 => Some(CY),
            Z if self.control == 0 => Some(Z),
            Z if self.control == 1 => Some(CZ),
            CX if self.control == 0 => Some(CX),
            CX if self.control == 1 => Some(Toffoli),
            CY if self.control == 0 => Some(CY),
            CZ if self.control == 0 => Some(CZ),
            Toffoli if self.control == 0 => Some(Toffoli),
            H if self.control == 0 => Some(H),
            Rz if self.control == 0 => Some(Rz),
            Rz if self.control == 1 => Some(CRz),
            CRz if self.control == 0 => Some(CRz),
            Rx if self.control == 0 => Some(Rx),
            Ry if self.control == 0 => Some(Ry),
            T if self.control == 0 => match self.dagger {
                false => Some(T),
                true => Some(Tdg),
            },
            Tdg if self.control == 0 => match self.dagger {
                false => Some(Tdg),
                true => Some(T),
            },
            S if self.control == 0 => match self.dagger {
                false => Some(S),
                true => Some(Sdg),
            },
            Sdg if self.control == 0 => match self.dagger {
                false => Some(Sdg),
                true => Some(S),
            },
            V if self.control == 0 => match self.dagger {
                false => Some(V),
                true => Some(Vdg),
            },
            Vdg if self.control == 0 => match self.dagger {
                false => Some(Vdg),
                true => Some(V),
            },
            Measure | MeasureFree | QAlloc | TryQAlloc | QFree | Reset
                if self.control == 0 && !self.dagger =>
            {
                Some(op)
            }
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
        Hugr,
        builder::{Dataflow, DataflowSubContainer, HugrBuilder, ModuleBuilder},
        extension::prelude::qb_t,
        ops::CallIndirect,
        std_extensions::collections::array::array_type,
        types::{Signature, Term},
    };
    use strum::IntoEnumIterator;

    use crate::{
        extension::rotation::ConstRotation,
        modifier::modifier_resolver::tests::{SetUnitary, test_modifier_resolver},
        modifier::modifier_resolver::*,
    };
    use crate::{
        extension::{
            modifier::{CONTROL_OP_ID, DAGGER_OP_ID, MODIFIER_EXTENSION},
            rotation::rotation_type,
        },
        modifier::*,
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

    #[rstest::rstest]
    #[case(0, true)]
    #[case(1, false)]
    #[case(3, false)]
    #[case(3, true)]
    #[case(7, false)]
    pub fn test_single_tket_op(#[case] c_num: u64, #[case] dagger: bool) {
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
                    Term::new_list([qb_t().into(), qb_t().into(), qb_t().into()]),
                    Term::new_list([rotation_type().into()]),
                ],
            )
            .unwrap();
        let dagger_op = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &DAGGER_OP_ID,
                [
                    Term::new_list([
                        array_type(1, qb_t()).into(),
                        qb_t().into(),
                        qb_t().into(),
                        qb_t().into(),
                    ]),
                    Term::new_list([rotation_type().into()]),
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
