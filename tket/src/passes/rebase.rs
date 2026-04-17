//! Pass for rebasing a Hugr by replacing ops with other HUGRs according to a provided mapping.

mod patches;

use crate::passes::composable::WithScope;
use crate::passes::{ComposablePass, PassScope};
use derive_more::Display;
use hugr_core::builder::{Dataflow, DataflowHugr, FunctionBuilder};
use hugr_core::core::HugrNode;
use hugr_core::extension::{ExtensionId, ExtensionRegistry};
use hugr_core::hugr::InvalidIdentifier;
use hugr_core::hugr::hugrmut::HugrMut;
use hugr_core::hugr::patch::inline_dfg::{InlineDFG, InlineDFGError};
use hugr_core::ops::{Call, DataflowOpTrait, ExtensionOp, OpName};
use hugr_core::types::Signature;
use hugr_core::{Direction, Hugr, HugrView, PortIndex};
use itertools::Itertools;
use std::collections::HashMap;

#[derive(Debug, thiserror::Error, Display)]
#[non_exhaustive]
/// Errors produced by [`RebasePass`].
pub enum RebaseError<N: HugrNode> {
    /// does not match the signature of the opAn op was encountered for which no replacement was
    /// provided in the mapping
    #[display(
        "Signature of op node {node} ({op_name}) does not match signature of replacement HUGR entrypoint: expected {expected} but got {actual}"
    )]
    #[expect(missing_docs)]
    SignatureOfReplacementMismatch {
        op_name: OpName,
        node: N,
        expected: Signature,
        actual: Signature,
    },
    /// An intermediate op was encountered for which no replacement was provided in the mapping
    #[display("Residual intermediate op {_0} in node {_1} as no replacement was provided")]
    NoReplacementFromIntermediate(OpName, N),
    /// An error occurred when inlining the call node during the rebase
    #[display("Error inlining Call node {_0} during rebase")]
    InlineCallError(#[from] patches::InlineCallError<N>),
    /// An error occurred when inlining the DFG node resulting from a call inline during the rebase
    #[display("Error inlining DFG node {_0} during rebase")]
    InlineDFGError(#[from] InlineDFGError<N>),
}

#[derive(Debug, thiserror::Error, Display)]
#[non_exhaustive]
#[expect(missing_docs)]
/// Errors produced by [`RebasePass`].
pub enum BuildDirectRebaseError {
    #[display("Could not unqualify op name {_0}")]
    CouldNotUnqualify(OpName),
    #[display("Invalid extension id {_0} in qualified op name")]
    InvalidExtensionId(#[from] InvalidIdentifier),
    #[display("Could not find extension {_0} in provided registry")]
    ExtensionNotFound(ExtensionId),
    #[display("Could not find op {_1} in extension {_0} from provided registry")]
    OpNotFound(ExtensionId, OpName),
    BuildOpHugrError(#[from] hugr_core::builder::BuildError),
}

/// A mapping from a qualified op name to its replacement HUGR, containing a single entrypoint
/// matching the signature of the op to be replaced.
type OpMap = HashMap<OpName, Hugr>;

#[derive(Debug, Clone, Eq, PartialEq)]
/// Configures whether function calls should be inlined after call creation
pub enum InlineCallsConfig {
    /// Signals that calls should be inlined, further indicating whether the resulting DFG should
    /// also be inlined
    Yes(bool),
    /// Signals that calls should not be inlined
    No,
}

#[derive(Debug, Clone)]
/// Configures which functions the pass should inline after insertion, and if so, whether the
/// DFGs that result from the function inlining should also be inlined.
pub struct InlineConfig {
    intermediate: InlineCallsConfig,
    new: InlineCallsConfig,
}

impl InlineConfig {
    /// Configures the pass to inline no functions at all
    pub fn none() -> Self {
        Self {
            intermediate: InlineCallsConfig::No,
            new: InlineCallsConfig::No,
        }
    }

    /// Configures the pass to inline intermediate functions after replacing ops with calls
    pub fn intermediate_only(inline_dfgs: bool) -> Self {
        Self {
            intermediate: InlineCallsConfig::Yes(inline_dfgs),
            new: InlineCallsConfig::No,
        }
    }

    /// Configures the pass to inline new functions after replacing intermediate ops with calls
    pub fn new_only(inline_dfgs: bool) -> Self {
        Self {
            intermediate: InlineCallsConfig::Yes(inline_dfgs),
            new: InlineCallsConfig::No,
        }
    }

    /// Configures the pass to inline all functions after replacing ops with calls
    pub fn all(inline_dfgs: bool) -> Self {
        Self {
            intermediate: InlineCallsConfig::Yes(inline_dfgs),
            new: InlineCallsConfig::Yes(inline_dfgs),
        }
    }
}

impl Default for InlineConfig {
    fn default() -> Self {
        Self::all(true)
    }
}

#[derive(Debug, Clone)]
/// A configuration for the rebase pass
pub struct RebasePass {
    scope: PassScope,
    cur_to_inter: OpMap,
    inter_to_new: OpMap,
    inline: InlineConfig,
}

impl RebasePass {
    /// Create a new Rebase pass with the given scope and op rewrites.
    pub fn new(
        scope: impl Into<PassScope>,
        cur_to_inter: OpMap,
        inter_to_new: OpMap,
        inline: InlineConfig,
    ) -> Self {
        Self {
            scope: scope.into(),
            cur_to_inter,
            inter_to_new,
            inline,
        }
    }

    /// Configures the pass to inline functions according to the provided config
    pub fn with_inline_config(&mut self, inline_config: InlineConfig) -> &mut Self {
        self.inline = inline_config;
        self
    }

    /// Builds a rebase pass directly from one set of ops to their replacements. Needs to include
    /// the extensions necessary to construct intermediate replacements for the ops.
    pub fn build_direct(
        scope: impl Into<PassScope>,
        reg: &ExtensionRegistry,
        cur_to_new: OpMap,
        inline: InlineConfig,
    ) -> Result<Self, BuildDirectRebaseError> {
        let cur_to_inter: OpMap = cur_to_new
            .iter()
            .map(|(op_key, _)| {
                let Some((ext_id, op_name)) = op_key.rsplit_once(".") else {
                    return Err(BuildDirectRebaseError::CouldNotUnqualify(op_key.clone()));
                };
                let Some(ext) = reg.get(ext_id) else {
                    return Err(BuildDirectRebaseError::ExtensionNotFound(ExtensionId::new(
                        ext_id,
                    )?));
                };
                let Some(ext_op_def) = ext.get_op(&op_name) else {
                    return Err(BuildDirectRebaseError::OpNotFound(
                        ExtensionId::new(ext_id)?,
                        op_name.into(),
                    ));
                };
                let op = ExtensionOp::new(ext_op_def.clone(), &[]).unwrap();

                let mut func = FunctionBuilder::new(op_key.clone(), (*op.signature()).clone())?;
                let inserted_op = func.add_dataflow_op(op, func.input_wires())?;
                let hugr = func.finish_hugr_with_outputs(inserted_op.outputs())?;

                Ok((op_key.clone(), hugr))
            })
            .try_collect()?;

        Ok(Self::new(scope, cur_to_inter, cur_to_new, inline))
    }
}

impl WithScope for RebasePass {
    fn with_scope(mut self, scope: impl Into<PassScope>) -> Self {
        self.scope = scope.into();
        self
    }
}

fn op_data<H: HugrView>(hugr: &H, n: H::Node) -> Option<(OpName, Signature)> {
    hugr.get_optype(n)
        .as_extension_op()
        .map(|ext_op| (ext_op.qualified_id(), (*ext_op.signature()).clone()))
}

fn replace_op<H: HugrMut>(
    hugr: &mut H,
    n: H::Node,
    signature: Signature,
    func_node: H::Node,
) -> Result<(), RebaseError<H::Node>> {
    let repl_entry_poly_func_ty = hugr
        .get_optype(func_node)
        .as_func_defn()
        .unwrap()
        .signature()
        .clone();
    // We rely on the op / replacement already being monomorphized
    assert!(repl_entry_poly_func_ty.params().is_empty());
    if &signature != repl_entry_poly_func_ty.body() {
        return Err(RebaseError::SignatureOfReplacementMismatch {
            op_name: hugr.get_optype(n).as_extension_op().unwrap().qualified_id(),
            node: n,
            expected: signature,
            actual: repl_entry_poly_func_ty.body().clone(),
        });
    }

    // Replace op with call to inserted function, adding a static port for the function reference
    // since the original ext op does not have one
    let orig_optype = hugr.get_optype(n);
    let new_port_index = orig_optype
        .other_input_port()
        .map(|p| p.index())
        .unwrap_or(orig_optype.input_count());
    hugr.replace_op(n, Call::try_new(repl_entry_poly_func_ty, vec![]).unwrap());
    hugr.insert_ports(n, Direction::Incoming, new_port_index, 1);
    let func_port = hugr.num_outputs(func_node) - 1;
    let call_in_port = hugr.get_optype(n).static_input_port().unwrap();
    hugr.connect(func_node, func_port, n, call_in_port);

    Ok(())
}

fn consume_hugr<H: HugrMut>(hugr: &mut H, from_hugr: &Hugr) -> (Vec<H::Node>, H::Node) {
    let hugr_root = hugr.module_root();
    let root_parents = from_hugr
        .children(from_hugr.module_root())
        .map(|child| (child, hugr_root));
    let insert_result = hugr.insert_forest(from_hugr.clone(), root_parents).unwrap();
    let new_entrypoint = insert_result.node_map[&from_hugr.entrypoint()];
    (
        insert_result.node_map.into_values().collect_vec(),
        new_entrypoint,
    )
}

impl<H: HugrMut> ComposablePass<H> for RebasePass
where
    H::Node: 'static,
{
    type Error = RebaseError<H::Node>;
    type Result = ();

    fn run(&self, hugr: &mut H) -> Result<Self::Result, Self::Error> {
        let base_nodes: Box<dyn Iterator<Item = H::Node>> = match self.scope {
            PassScope::Global(_) => Box::new(hugr.nodes()),
            PassScope::EntrypointFlat => Box::new(hugr.children(hugr.entrypoint())),
            PassScope::EntrypointRecursive => Box::new(hugr.descendants(hugr.entrypoint())),
        };

        let nodes_to_replace = base_nodes
            .filter_map(|n| {
                op_data(hugr, n).and_then(|(ext_op_qual_id, sig)| {
                    if self.cur_to_inter.contains_key(&ext_op_qual_id) {
                        Some((n, sig, ext_op_qual_id))
                    } else {
                        None
                    }
                })
            })
            .collect_vec();

        // Replace current ops with calls to the intermediate op set
        let mut inserted_hugrs = HashMap::<OpName, H::Node>::default();
        let mut new_nodes = Vec::<H::Node>::new();
        for (n, sig, ext_op_qual_id) in nodes_to_replace {
            let func_node = if let Some(&func_node) = inserted_hugrs.get(&ext_op_qual_id) {
                func_node
            } else {
                let Some(repl_hugr) = self.cur_to_inter.get(&ext_op_qual_id) else {
                    unreachable!("Should already be filtered to only those with replacements");
                };
                let (inserted_nodes, inserted_entry) = consume_hugr(hugr, repl_hugr);
                inserted_hugrs.insert(ext_op_qual_id, inserted_entry);
                if self.inline.intermediate == InlineCallsConfig::No {
                    // If we do not inline, we have to collect the new nodes here, if we inline
                    // they are collected after inlining since they are copied
                    new_nodes.extend(inserted_nodes);
                }
                inserted_entry
            };

            replace_op(hugr, n, sig, func_node)?;
            if let InlineCallsConfig::Yes(inline_dfgs) = self.inline.intermediate {
                let inlined_nodes = hugr.apply_patch(patches::InlineCall::new(n.into()))?;
                if inline_dfgs {
                    let [_, removed_in, removed_out] = hugr.apply_patch(InlineDFG(n.into()))?;
                    new_nodes.extend(
                        inlined_nodes
                            .into_iter()
                            .filter(|&n| n != removed_in && n != removed_out),
                    );
                } else {
                    new_nodes.extend(inlined_nodes);
                }
            }
        }
        if matches!(self.inline.intermediate, InlineCallsConfig::Yes(_)) {
            // Inlining calls orphaned these functions
            inserted_hugrs
                .into_values()
                .for_each(|n| hugr.remove_subtree(n));
        }

        // Replace the intermediate ops from the calls that were just inserted with calls to the
        // new op set
        let mut inserted_hugrs = HashMap::<OpName, H::Node>::default();
        for n in new_nodes.into_iter() {
            let Some((ext_op_qual_id, signature)) = op_data(hugr, n) else {
                continue;
            };

            let func_node = if let Some(&func_node) = inserted_hugrs.get(&ext_op_qual_id) {
                func_node
            } else {
                let Some(repl_hugr) = self.inter_to_new.get(&ext_op_qual_id) else {
                    return Err(RebaseError::NoReplacementFromIntermediate(
                        ext_op_qual_id,
                        n,
                    ));
                };
                let (_, inserted_entry) = consume_hugr(hugr, repl_hugr);
                inserted_hugrs.insert(ext_op_qual_id, inserted_entry);
                inserted_entry
            };

            replace_op(hugr, n, signature, func_node)?;
            if let InlineCallsConfig::Yes(inline_dfgs) = self.inline.new {
                hugr.apply_patch(patches::InlineCall::new(n.into()))?;
                if inline_dfgs {
                    hugr.apply_patch(InlineDFG(n.into()))?;
                }
            }
        }
        if matches!(self.inline.new, InlineCallsConfig::Yes(_)) {
            // Inlining calls orphaned these functions
            inserted_hugrs
                .into_values()
                .for_each(|n| hugr.remove_subtree(n));
        }

        Ok(())
    }
}

#[cfg(test)]
mod test {
    use super::patches::test::HasQualifiedId;
    use super::{InlineConfig, RebasePass};
    use crate::TketOp;
    use crate::extension::rotation::{ConstRotation, rotation_type};
    use crate::passes::PassScope;
    use crate::passes::composable::test::run_validating;
    use crate::utils::{build_circuit, build_simple_circuit};
    use hugr_core::extension::prelude::qb_t;
    use hugr_core::types::Signature;
    use hugr_core::{CircuitUnit, Hugr, HugrView};
    use std::collections::HashMap;
    use std::error::Error;
    use std::io::Write;

    fn print_hugr<H: HugrView>(hugr: &H, name: &str) {
        let mut f = std::fs::File::create(format!("{}.mmd", name)).unwrap();
        f.write(hugr.mermaid_string().as_ref()).unwrap();
    }

    fn x_replacement() -> Hugr {
        let h = build_simple_circuit(1, |circ| {
            let angle = circ.add_constant(ConstRotation::PI);
            circ.append_and_consume(
                TketOp::Rx,
                [CircuitUnit::Linear(0), CircuitUnit::Wire(angle)],
            )?;
            Ok(())
        });

        h.unwrap().into_hugr()
    }
    fn rx_replacement() -> Hugr {
        let h = build_circuit(
            Signature::new([qb_t(), rotation_type()], [qb_t()]),
            |circ| {
                circ.append(TketOp::H, [0])?;
                circ.append_and_consume(
                    TketOp::Rz,
                    [
                        CircuitUnit::Linear(0),
                        CircuitUnit::Wire(circ.tracked_wire(1).unwrap()),
                    ],
                )?;
                // Forget about the tracked rotation wire
                circ.untrack_wire(1).unwrap();
                circ.append(TketOp::H, [0])?;
                Ok(())
            },
        );

        h.unwrap().into_hugr()
    }

    fn bad_rx_replacement() -> Hugr {
        let h = build_circuit(Signature::new([qb_t()], [qb_t()]), |_| Ok(()));

        h.unwrap().into_hugr()
    }

    fn cx_id_replacement() -> Hugr {
        let h = build_simple_circuit(2, |circ| {
            circ.append(TketOp::CX, [0, 1])?;
            Ok(())
        });

        h.unwrap().into_hugr()
    }

    fn cx_hh_replacement() -> Hugr {
        let h = build_simple_circuit(2, |circ| {
            circ.append(TketOp::H, [0])?;
            circ.append(TketOp::H, [0])?;
            circ.append(TketOp::CX, [0, 1])?;
            circ.append(TketOp::H, [0])?;
            circ.append(TketOp::H, [0])?;
            Ok(())
        });

        h.unwrap().into_hugr()
    }

    #[test]
    fn test_rebase_pass() -> Result<(), Box<dyn Error>> {
        let mut hugr = build_simple_circuit(2, |circ| {
            circ.append(TketOp::X, [0])?;
            circ.append(TketOp::X, [0])?;
            circ.append(TketOp::CX, [0, 1])?;
            Ok(())
        })?
        .into_hugr();
        print_hugr(&hugr, "hugr_before");

        let pass = RebasePass::new(
            PassScope::default(),
            HashMap::from_iter([
                (TketOp::X.qualified_id(), x_replacement()),
                (TketOp::CX.qualified_id(), cx_id_replacement()),
            ]),
            HashMap::from_iter([
                (TketOp::Rx.qualified_id(), rx_replacement()),
                (TketOp::CX.qualified_id(), cx_hh_replacement()),
            ]),
            InlineConfig::none(),
        );
        run_validating(pass, &mut hugr)?;

        print_hugr(&hugr, "hugr_after");
        Ok(())
    }
}
