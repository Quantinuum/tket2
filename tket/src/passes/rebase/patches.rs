use derive_more::{Display, Error};
use hugr_core::core::HugrNode;
use hugr_core::hugr::OpType;
use hugr_core::hugr::hugrmut::HugrMut;
use hugr_core::hugr::patch::{PatchHugrMut, PatchVerification};
use hugr_core::ops::{DFG, DataflowParent};
use hugr_core::types::Substitution;
use hugr_core::{Direction, HugrView, Node};
use itertools::Itertools;

/// Rewrite to inline a [Call](OpType::Call) to a known [`FuncDefn`](OpType::FuncDefn)
///
/// Patched to return the inserted nodes on success
pub struct InlineCall<N = Node>(N);

/// Error in performing [`InlineCall`] rewrite.
#[derive(Clone, Debug, Display, Error, PartialEq)]
#[non_exhaustive]
pub enum InlineCallError<N = Node> {
    /// The specified Node was not a [Call](OpType::Call)
    #[display("Node to inline {_0} expected to be a Call but actually {_1}")]
    NotCallNode(N, OpType),
    /// The node was a Call, but the target was not a [`FuncDefn`](OpType::FuncDefn)
    /// - presumably a [`FuncDecl`](OpType::FuncDecl), if the Hugr is valid.
    #[display("Call targeted node {_0} which must be a FuncDefn but was {_1}")]
    CallTargetNotFuncDefn(N, OpType),
}

impl<N> InlineCall<N> {
    /// Create a new instance that will inline the specified node
    /// (i.e. that should be a [Call](OpType::Call))
    pub fn new(node: N) -> Self {
        Self(node)
    }
}

impl<N: HugrNode> PatchVerification for InlineCall<N> {
    type Error = InlineCallError<N>;
    type Node = N;

    fn verify(&self, h: &impl HugrView<Node = N>) -> Result<(), Self::Error> {
        let call_ty = h.get_optype(self.0);
        if !call_ty.is_call() {
            return Err(InlineCallError::NotCallNode(self.0, call_ty.clone()));
        }
        let func = h.static_source(self.0).unwrap();
        let func_ty = h.get_optype(func);
        if !func_ty.is_func_defn() {
            return Err(InlineCallError::CallTargetNotFuncDefn(
                func,
                func_ty.clone(),
            ));
        }
        Ok(())
    }

    fn invalidated_nodes(&self, _: &impl HugrView<Node = N>) -> impl Iterator<Item = N> {
        Some(self.0).into_iter()
    }
}

impl<N: HugrNode> PatchHugrMut for InlineCall<N> {
    type Outcome = Vec<N>;

    /// Failure only occurs if the node is not a Call, or the target not a `FuncDefn`.
    /// (Any later failure means an invalid Hugr and `panic`.)
    const UNCHANGED_ON_FAILURE: bool = true;

    fn apply_hugr_mut(self, h: &mut impl HugrMut<Node = N>) -> Result<Self::Outcome, Self::Error> {
        self.verify(h)?; // Now we know we have a Call to a FuncDefn.
        let orig_func = h.static_source(self.0).unwrap();

        h.disconnect(self.0, h.get_optype(self.0).static_input_port().unwrap());

        // The order input port gets renumbered because the static input
        // (which comes between the value inports and the order inport) gets removed
        let old_order_in = h.get_optype(self.0).other_input_port().unwrap();
        let order_preds = h.linked_outputs(self.0, old_order_in).collect::<Vec<_>>();
        h.disconnect(self.0, old_order_in); // PortGraph currently does this anyway

        let new_op = OpType::from(DFG {
            signature: h
                .get_optype(orig_func)
                .as_func_defn()
                .unwrap()
                .inner_signature()
                .into_owned(),
        });
        let new_order_in = new_op.other_input_port().unwrap();

        let ty_args = h
            .replace_op(self.0, new_op)
            .as_call()
            .unwrap()
            .type_args
            .clone();

        h.add_ports(self.0, Direction::Incoming, -1);

        // Reconnect order predecessors
        for (src, srcp) in order_preds {
            h.connect(src, srcp, self.0, new_order_in);
        }

        let mapped_nodes = h.copy_descendants(
            orig_func,
            self.0,
            (!ty_args.is_empty()).then_some(Substitution::new(&ty_args)),
        );
        Ok(mapped_nodes.into_values().collect_vec())
    }
}

// Patch for ops that have enough info to put together a qualified id but do not for some reason
#[cfg(test)]
pub(crate) mod test {
    use hugr_core::extension::simple_op::MakeOpDef;
    use hugr_core::ops::OpName;

    pub trait HasQualifiedId {
        fn qualified_id(&self) -> OpName;
    }

    #[cfg(test)]
    impl<T: MakeOpDef> HasQualifiedId for T {
        fn qualified_id(&self) -> OpName {
            format!("{}.{}", self.extension(), self.opdef_id()).into()
        }
    }
}
