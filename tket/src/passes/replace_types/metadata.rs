//! Support for propagating metadata when an op node is replaced by a
//! container during a [`ReplaceTypes`](super::ReplaceTypes) pass.
//!
//! Each pass applies exactly one [`MetadataPropagationPolicy`] which
//! consists of an arbitrary number of [`MetadataPropagationRule`]s.
//! See the documentation of those structs for details.
use std::any::type_name;
use std::collections::BTreeSet;
use std::fmt::{Debug, Formatter};
use std::sync::Arc;

use hugr_core::HugrView;
use hugr_core::Node;
use hugr_core::hugr::NodeMetadataMap;
use hugr_core::hugr::hugrmut::HugrMut;
use hugr_core::hugr::internal::HugrInternals;
use hugr_core::metadata::DEBUGINFO_META_KEY;
use hugr_core::metadata::RawMetadataValue;
use hugr_core::ops::OpType;

/// The signature of a callback for updating descendant metadata.
/// See [MetadataPropagationRule] for a description of the arguments.
pub type UpdateFn = dyn Fn(&OpType, &NodeMetadataMap, &OpType, &NodeMetadataMap) -> Vec<(String, RawMetadataValue)>
    + Send
    + Sync;

/// A composable rule for propagating metadata when a single node is replaced by a
/// container.
///
/// A rule consists of two components:
///
/// 1) A callback `update_new` which is called once for each (possibly nested)
///    descendant of the replacing container.
///
///    The callback recieves `(old_optype, old_meta, inner_optype, inner_meta)`:
///     - `old_optype` - optype of the replaced node
///     - `old_meta` - metadata of the replaced node
///     - `inner_optype` - optype of the descendant
///     - `inner_meta` - metadata of the descendant
///
///    and returns a list of key-value pairs to set on that descendant.
///
/// 2) A Vec `remove_from_old` of metadata keys to remove from the replacing container.
///    `update_new` will be called for each descendant before the keys are removed.
///
/// To use a custom rule, add it to a pass's [`MetadataPropagationPolicy`]:
/// ```ignore
/// pass.metadata_policy_mut().add_rule(my_custom_rule)
/// ```
#[derive(Clone)]
pub struct MetadataPropagationRule {
    update_new: Arc<UpdateFn>,
    remove_from_old: Vec<String>,
}

impl MetadataPropagationRule {
    /// Create a rule for metadata propagation.
    pub fn new(update_new: Arc<UpdateFn>, remove_from_old: Vec<String>) -> Self {
        Self {
            // take an Arc directly to avoid an explicit lifetime arg
            update_new,
            remove_from_old,
        }
    }
}

impl Debug for MetadataPropagationRule {
    fn fmt(&self, f: &mut Formatter) -> Result<(), std::fmt::Error> {
        f.debug_struct(type_name::<Self>())
            // this is the best we can do until type_info is stabilized.
            // then we should be able to extract the concrete type name,
            // which typically contains the function name for a Fn.
            .field("update_new", &"<dyn Fn>")
            .field("remove_from_old", &self.remove_from_old)
            .finish()
    }
}

/// A set of rules for propagating node metadata when a single node is replaced by a container.
///
/// By default, a container which replaces a node retains the original node's metadata,
/// and the descendants of the new container receive no metadata. This policy allows passes to
/// adjust that behavior in two ways:
///     1) Metadata may be added to child nodes based on the original node's metadata.
///     2) Metadata may be removed by key from the replacement container node.
///
/// Each pass has exactly one associated policy. A policy consists of zero or more
/// composed [`MetadataPropagationRule`]s. See the documentation of that struct for the
/// definition of a rule.
///
/// # Notes on composing rules
///
/// 1. Rules are applied in order, and each rule receives descendant metadata that
///    reflects updates from previous rules.
/// 2. All descendant metadata updates are applied before any keys are removed from the
///    replacing container.
/// 3. If replacement is applied recursively, metadata is propagated before any
///    descendants of the container are themselves replaced.
///
/// # Default policy
///
/// The default policy applies a single rule which propagates debug source location
/// metadata from the replacing container, where it has no effect, to descendants where
/// a location is meaningful. This rule should be applied for all ReplaceTypes passes.
#[derive(Clone, Debug)]
pub struct MetadataPropagationPolicy {
    rules: Vec<MetadataPropagationRule>,
}

impl MetadataPropagationPolicy {
    /// Creates a new policy with no rules.
    pub fn empty() -> Self {
        Self { rules: Vec::new() }
    }

    /// Add a rule to the policy
    pub fn add_rule(&mut self, rule: MetadataPropagationRule) {
        self.rules.push(rule)
    }

    /// Add a rule to the policy (fluent style)
    pub fn with_rule(mut self, rule: MetadataPropagationRule) -> Self {
        self.add_rule(rule);
        self
    }

    /// Mutable access to the rule vector. Prefer using [`add_rule`](Self::add_rule) or
    /// [`with_rule`](Self::with_rule) unless you need to reorder or remove entries.
    pub fn rules_mut(&mut self) -> &mut Vec<MetadataPropagationRule> {
        &mut self.rules
    }

    /// Returns `true` if the policy has no rules and is guaranteed to be a no-op.
    pub fn is_empty(&self) -> bool {
        self.rules.is_empty()
    }

    /// Applies the propagation policy to a container node and its descendants.
    pub(crate) fn apply<H: HugrMut<Node = Node>>(
        &self,
        hugr: &mut H,
        container_node: Node,
        old_optype: &OpType,
    ) {
        if self.rules.is_empty() {
            return;
        }

        // `descendants` yields the node itself first, skip it.
        // This function shouldn't get called at all for SingleOp replacements, but
        // LinkedHugr can also replace with a single non-container op (e.g. Call),
        // in which case we do not want to apply the propagation policy.
        if hugr.children(container_node).next().is_none() {
            return;
        }

        // Snapshot the container's metadata. `apply` is invoked immediately
        // after the container is installed and *before* recursing into its
        // subtree, so this is the metadata inherited from the replaced node.
        let old_meta = hugr.node_metadata_map(container_node).clone();
        if old_meta.is_empty() {
            return;
        }

        let descendants: Vec<Node> = hugr.descendants(container_node).skip(1).collect();
        for inner in descendants {
            for entry in &self.rules {
                for (key, value) in (entry.update_new)(
                    old_optype,
                    &old_meta,
                    hugr.get_optype(inner),
                    hugr.node_metadata_map(inner),
                ) {
                    hugr.set_metadata_any(inner, &key, value);
                }
            }
        }

        let to_remove: BTreeSet<&str> = self
            .rules
            .iter()
            .flat_map(|e| e.remove_from_old.iter().map(String::as_str))
            .collect();
        for key in to_remove {
            hugr.remove_metadata_any(container_node, key);
        }
    }
}

fn debug_location_update_rule(
    old_optype: &OpType,
    old_meta: &NodeMetadataMap,
    inner_optype: &OpType,
    inner_meta: &NodeMetadataMap,
) -> Vec<(String, RawMetadataValue)> {
    if matches!(old_optype, OpType::Call(_) | OpType::ExtensionOp(_))
        && matches!(inner_optype, OpType::Call(_) | OpType::ExtensionOp(_))
        && !inner_meta.contains_key(DEBUGINFO_META_KEY)
    {
        old_meta
            .get(DEBUGINFO_META_KEY)
            .map(|v| vec![(DEBUGINFO_META_KEY.to_string(), v.clone())])
            .unwrap_or_default()
    } else {
        vec![]
    }
}

/// This rule copies `core.debug_info` from a replaced `Call` or `ExtensionOp`
/// onto every `Call`/`ExtensionOp` descendant of the replacement container
/// that does not already carry the key, and marks `core.debug_info` for
/// removal from the replacing container.
fn default_debuginfo_rule() -> MetadataPropagationRule {
    MetadataPropagationRule::new(
        Arc::new(&debug_location_update_rule),
        vec![DEBUGINFO_META_KEY.to_string()],
    )
}

impl Default for MetadataPropagationPolicy {
    fn default() -> Self {
        Self::empty().with_rule(default_debuginfo_rule())
    }
}

#[cfg(test)]
mod test {
    use std::sync::Arc;

    use hugr_core::builder::{DFGBuilder, Dataflow, DataflowHugr, FunctionBuilder, HugrBuilder};
    use hugr_core::extension::prelude::{bool_t, usize_t};
    use hugr_core::std_extensions::collections::list::list_type;
    use hugr_core::types::{Signature, Type, TypeArg};
    use hugr_core::{Extension, HugrView, Visibility};

    use crate::passes::ComposablePass;

    use super::super::test::{
        PACKED_VEC, READ, ext, i64_t, just_elem_type, lowered_read, lowerer, read_op,
    };
    use super::super::{NodeTemplate, ReplaceTypes};
    use super::{MetadataPropagationPolicy, MetadataPropagationRule};

    /// Common expected debug-info record used by the metadata propagation tests.
    fn expected_location() -> hugr_core::metadata::LocationRecord {
        hugr_core::metadata::LocationRecord {
            kind: "location".into(),
            column: 5,
            line_no: 10,
        }
    }

    /// Builds a DFG hugr containing a single `read<usize>` op with
    /// [`expected_location`] attached, returning the hugr and the op node.
    fn build_read_hugr_with_location(ext: &Arc<Extension>) -> (hugr_core::Hugr, hugr_core::Node) {
        use hugr_core::hugr::hugrmut::HugrMut;
        use hugr_core::metadata::LocationRecord;
        use hugr_core::ops::OpType;

        let elem_ty = usize_t();
        let coln = ext.get_type(PACKED_VEC).unwrap();
        let pv_usize = Type::new_extension(coln.instantiate([elem_ty.clone().into()]).unwrap());
        let mut dfb = DFGBuilder::new(Signature::new(
            vec![pv_usize, i64_t()],
            vec![elem_ty.clone()],
        ))
        .unwrap();
        let [pv, idx] = dfb.input_wires_arr();
        let read = dfb
            .add_dataflow_op(read_op(ext, elem_ty.clone()), [pv, idx])
            .unwrap();
        let mut h = dfb.finish_hugr_with_outputs(read.outputs()).unwrap();

        let read_node = h
            .entry_descendants()
            .find(|&n| matches!(h.get_optype(n), OpType::ExtensionOp(_)))
            .expect("read op node");
        h.set_metadata::<LocationRecord>(read_node, expected_location());
        (h, read_node)
    }

    /// Builds a `ReplaceTypes` configured to lower `PackedVec`/`read` using a
    /// caller-supplied template factory for the parametrised `read` op. Used to
    /// share the boilerplate across the three propagation tests that need
    /// different `NodeTemplate` variants.
    fn build_lw_with_read_template<F>(ext: &Arc<Extension>, template_for_read: F) -> ReplaceTypes
    where
        F: Fn(&[TypeArg]) -> NodeTemplate + Send + Sync + 'static,
    {
        let pv = ext.get_type(PACKED_VEC).unwrap();
        let mut lw = ReplaceTypes::default();
        lw.set_replace_type(pv.instantiate([bool_t().into()]).unwrap(), i64_t());
        lw.set_replace_parametrized_type(
            pv,
            Box::new(|args: &[TypeArg]| Some(list_type(just_elem_type(args).clone()))),
        );
        lw.set_replace_parametrized_op(ext.get_op(READ).unwrap().as_ref(), move |args, _| {
            Ok(Some(template_for_read(args)))
        });
        lw
    }

    /// Asserts that `node` carries metadata equal to [`expected_location`].
    #[track_caller]
    fn assert_location(h: &impl HugrView<Node = hugr_core::Node>, node: hugr_core::Node) {
        use hugr_core::metadata::LocationRecord;
        let actual = h
            .get_metadata::<LocationRecord>(node)
            .expect("LocationRecord expected on node");
        assert_eq!(
            serde_json::to_value(&actual).unwrap(),
            serde_json::to_value(expected_location()).unwrap(),
            "LocationRecord on {node:?} differs from expected"
        );
    }

    /// Returns the direct `ExtensionOp` children of `container`, panicking if
    /// there are none (guards against accidentally vacuous assertions).
    #[track_caller]
    fn ext_op_children(
        h: &impl HugrView<Node = hugr_core::Node>,
        container: hugr_core::Node,
    ) -> Vec<hugr_core::Node> {
        use hugr_core::ops::OpType;
        let ops: Vec<_> = h
            .children(container)
            .filter(|&n| matches!(h.get_optype(n), OpType::ExtensionOp(_)))
            .collect();
        assert!(
            !ops.is_empty(),
            "expected ExtensionOp children inside {container:?}"
        );
        ops
    }

    /// Asserts every direct `ExtensionOp` child of `container` carries
    /// metadata equal to [`expected_location`]; panics if there are none.
    #[track_caller]
    fn assert_all_inner_ext_ops_have_location(
        h: &impl HugrView<Node = hugr_core::Node>,
        container: hugr_core::Node,
    ) {
        for op_node in ext_op_children(h, container) {
            assert_location(h, op_node);
        }
    }

    /// When the replacement is a [`NodeTemplate::LinkedHugr`] whose entrypoint is
    /// a `Call` node (the `call_to_function` pattern), the debug location stays on
    /// that call node and is correctly preserved.
    #[test]
    fn linked_hugr_preserves_debug_location_on_call() {
        use hugr_core::ops::OpType;

        let ext = ext();
        let (mut h, read_node) = build_read_hugr_with_location(&ext);

        // Replacement: LinkedHugr whose entrypoint is a Call node.
        let lw = build_lw_with_read_template(&ext, |args| {
            let ty: Type = just_elem_type(args).clone();
            let func_hugr = lowered_read(ty, |sig| {
                FunctionBuilder::new_vis("lowered_read_usize", sig, Visibility::Public)
            })
            .finish_hugr()
            .unwrap();
            NodeTemplate::call_to_function(func_hugr, &[]).unwrap()
        });
        lw.run(&mut h).unwrap();
        h.validate().unwrap();

        assert!(
            matches!(h.get_optype(read_node), OpType::Call(_)),
            "Expected read_node to become a Call after LinkedHugr(Call) replacement"
        );
        assert_location(&h, read_node);
    }

    /// Regression test for <https://github.com/Quantinuum/tket2/issues/1651>.
    ///
    /// When the replacement is a [`NodeTemplate::CompoundOp`] whose entrypoint is a
    /// container (DFG), the debug location should be propagated to the `ExtensionOp`
    /// nodes inside it.
    #[test]
    fn compound_op_propagates_debug_location_to_inner_extension_ops() {
        use hugr_core::ops::OpType;

        let ext = ext();
        let (mut h, read_node) = build_read_hugr_with_location(&ext);

        // lowerer() replaces read<usize> with CompoundOp(DFG containing ExtensionOps).
        lowerer(&ext).run(&mut h).unwrap();
        h.validate().unwrap();

        assert!(
            matches!(h.get_optype(read_node), OpType::DFG(_)),
            "Expected read_node to become a DFG after CompoundOp replacement"
        );
        assert_all_inner_ext_ops_have_location(&h, read_node);
    }

    /// Regression test for <https://github.com/Quantinuum/tket2/issues/1651>.
    ///
    /// When the replacement is a [`NodeTemplate::LinkedHugr`] whose entrypoint is
    /// a container (DFG), the debug location should be propagated to the `ExtensionOp`
    /// nodes inside it.
    #[test]
    fn linked_hugr_propagates_debug_location_into_container() {
        use hugr_core::hugr::linking::NameLinkingPolicy;
        use hugr_core::ops::OpType;

        let ext = ext();
        let (mut h, read_node) = build_read_hugr_with_location(&ext);

        let lw = build_lw_with_read_template(&ext, |args| {
            let ty: Type = just_elem_type(args).clone();
            let dfg_hugr = lowered_read(ty, DFGBuilder::new).finish_hugr().unwrap();
            NodeTemplate::LinkedHugr(Box::new(dfg_hugr), NameLinkingPolicy::default())
        });
        lw.run(&mut h).unwrap();
        h.validate().unwrap();

        assert!(
            matches!(h.get_optype(read_node), OpType::DFG(_)),
            "Expected read_node to become a DFG after LinkedHugr(DFG) replacement"
        );
        assert_all_inner_ext_ops_have_location(&h, read_node);
    }

    /// The default policy only propagates the `core.debug_info` key. Other
    /// metadata entries on the replaced node must not leak onto inner ops.
    #[test]
    fn default_policy_does_not_propagate_non_debug_metadata() {
        use hugr_core::hugr::hugrmut::HugrMut;
        use serde_json::Value;

        let ext = ext();
        let (mut h, read_node) = build_read_hugr_with_location(&ext);
        // Attach an unrelated metadata key on the op node before lowering.
        h.set_metadata_any(read_node, "unrelated.key", Value::String("hello".into()));

        lowerer(&ext).run(&mut h).unwrap();
        h.validate().unwrap();

        // Debug info IS propagated (sanity check), but the unrelated key is NOT.
        assert_all_inner_ext_ops_have_location(&h, read_node);
        for op_node in ext_op_children(&h, read_node) {
            assert!(
                h.get_metadata_any(op_node, "unrelated.key").is_none(),
                "Non-debug metadata leaked onto inner op {op_node:?}"
            );
        }
    }

    /// The default rule's `!inner_meta.contains_key` guard must prevent
    /// overwriting a `core.debug_info` value already present on an inner op.
    #[test]
    fn default_policy_does_not_overwrite_existing_debug_info() {
        use hugr_core::hugr::hugrmut::HugrMut;
        use hugr_core::metadata::LocationRecord;
        use hugr_core::ops::OpType;

        let ext = ext();
        let (mut h, read_node) = build_read_hugr_with_location(&ext);

        // Pre-seed one of the inner ops with a *different* LocationRecord by
        // building the replacement separately and writing metadata before
        // running the pass via a custom template factory. We attach the
        // pre-existing record after the pass by intercepting the inner op,
        // so the simplest path is: run the pass, overwrite metadata, then
        // run again. Instead we use a custom CompoundOp built here.
        let preexisting = LocationRecord {
            kind: "location".into(),
            column: 999,
            line_no: 999,
        };
        let lw = build_lw_with_read_template(&ext, {
            let preexisting = serde_json::to_value(&preexisting).unwrap();
            move |args| {
                let ty: Type = just_elem_type(args).clone();
                let mut body = lowered_read(ty, DFGBuilder::new).finish_hugr().unwrap();
                // Seed every inner ExtensionOp with the pre-existing debug record.
                let inner: Vec<_> = body
                    .entry_descendants()
                    .filter(|&n| matches!(body.get_optype(n), OpType::ExtensionOp(_)))
                    .collect();
                for n in inner {
                    body.set_metadata_any(
                        n,
                        hugr_core::metadata::DEBUGINFO_META_KEY,
                        preexisting.clone(),
                    );
                }
                NodeTemplate::CompoundOp(Box::new(body))
            }
        });
        lw.run(&mut h).unwrap();
        h.validate().unwrap();

        let preexisting_json = serde_json::to_value(&preexisting).unwrap();
        for op_node in ext_op_children(&h, read_node) {
            let actual = h
                .get_metadata::<LocationRecord>(op_node)
                .expect("inner op should still carry the pre-existing record");
            assert_eq!(
                serde_json::to_value(&actual).unwrap(),
                preexisting_json,
                "Default policy overwrote pre-existing debug_info on {op_node:?}",
            );
        }
    }

    /// An empty propagation policy must not write any metadata onto inner ops.
    #[test]
    fn empty_policy_propagates_nothing() {
        use hugr_core::metadata::DEBUGINFO_META_KEY;

        let ext = ext();
        let (mut h, read_node) = build_read_hugr_with_location(&ext);

        let mut lw = lowerer(&ext);
        lw.set_metadata_policy(MetadataPropagationPolicy::empty());
        lw.run(&mut h).unwrap();
        h.validate().unwrap();

        for op_node in ext_op_children(&h, read_node) {
            assert!(
                h.get_metadata_any(op_node, DEBUGINFO_META_KEY).is_none(),
                "Empty policy propagated debug_info onto {op_node:?}"
            );
        }
    }

    /// A user-supplied rule added via `metadata_policy_mut()` should run and
    /// write the keys it returns onto every direct child unconditionally.
    #[test]
    fn custom_policy_rule_is_applied() {
        use serde_json::Value;

        let ext = ext();
        let (mut h, read_node) = build_read_hugr_with_location(&ext);

        let mut lw = lowerer(&ext);
        // Start fresh so we only observe our custom rule.
        lw.set_metadata_policy(MetadataPropagationPolicy::empty());
        lw.metadata_policy_mut()
            .add_rule(MetadataPropagationRule::new(
                Arc::new(|_, _, _, _| vec![("custom.tag".into(), Value::Bool(true))]),
                vec![],
            ));
        lw.run(&mut h).unwrap();
        h.validate().unwrap();

        // Every direct child of the new container (not just ExtensionOps) gets
        // the custom key.
        let children: Vec<_> = h.children(read_node).collect();
        assert!(!children.is_empty());
        for child in children {
            assert_eq!(
                h.get_metadata_any(child, "custom.tag"),
                Some(&Value::Bool(true)),
                "Custom rule did not write key onto {child:?} ({:?})",
                h.get_optype(child)
            );
        }
    }

    /// The default policy moves `core.debug_info` off the container after
    /// propagating it onto descendants, so backends don't see a stale entry
    /// on the new `DFG`/`CFG`/etc.
    #[test]
    fn default_policy_removes_propagated_key_from_container() {
        use hugr_core::metadata::DEBUGINFO_META_KEY;

        let ext = ext();
        let (mut h, read_node) = build_read_hugr_with_location(&ext);

        lowerer(&ext).run(&mut h).unwrap();
        h.validate().unwrap();

        assert!(
            h.get_metadata_any(read_node, DEBUGINFO_META_KEY).is_none(),
            "Default policy left a stale debug_info entry on the container",
        );
    }

    /// `remove_from_old` keys passed to `add_rule` should be deleted from the
    /// container after propagation, even when the rule itself sets nothing on
    /// any descendant (policy that only strips keys).
    #[test]
    fn custom_policy_remove_from_old_is_honoured() {
        use hugr_core::hugr::hugrmut::HugrMut;
        use serde_json::Value;

        let ext = ext();
        let (mut h, read_node) = build_read_hugr_with_location(&ext);
        h.set_metadata_any(read_node, "scratch.key", Value::String("v".into()));

        let mut lw = lowerer(&ext);
        lw.set_metadata_policy(MetadataPropagationPolicy::empty());
        lw.metadata_policy_mut()
            .add_rule(MetadataPropagationRule::new(
                Arc::new(|_, _, _, _| vec![]),
                vec!["scratch.key".to_string()],
            ));
        lw.run(&mut h).unwrap();
        h.validate().unwrap();

        assert!(
            h.get_metadata_any(read_node, "scratch.key").is_none(),
            "remove_from_old did not delete the key from the container",
        );
    }

    /// The default policy walks all descendants of the replacement container,
    /// so `core.debug_info` should reach `ExtensionOp`s even when they are
    /// nested several containers deep (e.g. `DFG { DFG { ExtensionOps } }`).
    #[test]
    fn default_policy_recurses_into_nested_containers() {
        use hugr_core::ops::OpType;

        let ext = ext();
        let (mut h, read_node) = build_read_hugr_with_location(&ext);

        // Build a replacement whose body is an *outer* DFG that wraps the
        // normal lowered_read body in an *inner* DFG. The inner DFG's
        // ExtensionOps are descendants but not direct children of read_node.
        let lw = build_lw_with_read_template(&ext, |args| {
            let ty: Type = just_elem_type(args).clone();
            let inner_body = lowered_read(ty.clone(), DFGBuilder::new)
                .finish_hugr()
                .unwrap();
            let mut outer = DFGBuilder::new(Signature::new(
                [list_type(ty.clone()), i64_t()],
                [ty.clone()],
            ))
            .unwrap();
            let [val, idx] = outer.input_wires_arr();
            let handle = outer.add_hugr_with_wires(inner_body, [val, idx]).unwrap();
            let [res] = handle.outputs_arr();
            NodeTemplate::CompoundOp(Box::new(outer.finish_hugr_with_outputs([res]).unwrap()))
        });
        lw.run(&mut h).unwrap();
        h.validate().unwrap();

        // The direct child of read_node is the inner DFG (no ExtensionOps).
        let inner_dfg = h
            .children(read_node)
            .find(|&n| matches!(h.get_optype(n), OpType::DFG(_)))
            .expect("expected an inner DFG as a direct child");

        // ExtensionOps two levels deep should still carry the debug location.
        for op_node in ext_op_children(&h, inner_dfg) {
            assert_location(&h, op_node);
        }
    }

    /// Rules should observe updates to `inner_meta` from earlier rules.
    #[test]
    fn later_rule_sees_inner_meta_updates_from_earlier_rule() {
        use serde_json::Value;

        let ext = ext();
        let (mut h, read_node) = build_read_hugr_with_location(&ext);

        let mut lw = lowerer(&ext);
        // Start fresh so we only observe our custom rules.
        lw.set_metadata_policy(MetadataPropagationPolicy::empty());
        lw.metadata_policy_mut()
            .add_rule(MetadataPropagationRule::new(
                Arc::new(|_, _, _, _| vec![("meta.rule1".into(), Value::Bool(true))]),
                vec![],
            ));
        lw.metadata_policy_mut()
            .add_rule(MetadataPropagationRule::new(
                Arc::new(|_, _, _, inner_meta| {
                    let saw_rule1 = inner_meta.contains_key("meta.rule1");
                    vec![(
                        "meta.rule2".into(),
                        Value::String(if saw_rule1 {
                            "saw_rule1".into()
                        } else {
                            "missing_rule1".into()
                        }),
                    )]
                }),
                vec![],
            ));
        lw.run(&mut h).unwrap();
        h.validate().unwrap();

        let children: Vec<_> = h.children(read_node).collect();
        assert!(!children.is_empty());
        for child in children {
            assert_eq!(
                h.get_metadata_any(child, "meta.rule2"),
                Some(&Value::String("saw_rule1".into())),
                "Second rule did not see meta.rule1 set by the first rule on {child:?} ({:?}); \
                 inner_meta passed to later rules should reflect updates from earlier rules",
                h.get_optype(child)
            );
        }
    }

    /// If node `A` is replaced with container `B`, and then op `C` inside `B`
    /// is itself replaced with container `D`, metadata attached to `A` should
    /// end up on `D`'s leaf descendants: `A` -> `C` -> `D`'s children.
    #[test]
    fn chained_replacement_propagates_metadata_through_intermediate_container() {
        use hugr_core::extension::TypeDefBound;
        use hugr_core::extension::Version;
        use hugr_core::extension::prelude::{Noop, usize_t};
        use hugr_core::hugr::IdentList;
        use hugr_core::hugr::hugrmut::HugrMut;
        use hugr_core::metadata::LocationRecord;
        use hugr_core::ops::{ExtensionOp, OpType};

        // Mini extension with two ops `foo` and `bar`, both `usize -> usize`.
        let ext = Extension::new_arc(
            IdentList::new("MetaChainTest").unwrap(),
            Version::new(0, 0, 1),
            |ext, w| {
                let _ = TypeDefBound::any();
                ext.add_op(
                    "foo".into(),
                    String::new(),
                    Signature::new(vec![usize_t()], vec![usize_t()]),
                    w,
                )
                .unwrap();
                ext.add_op(
                    "bar".into(),
                    String::new(),
                    Signature::new(vec![usize_t()], vec![usize_t()]),
                    w,
                )
                .unwrap();
            },
        );

        let foo_op = ExtensionOp::new(ext.get_op("foo").unwrap().clone(), []).unwrap();
        let bar_op = ExtensionOp::new(ext.get_op("bar").unwrap().clone(), []).unwrap();

        // Build a hugr containing a single `foo` op and attach a debug record.
        let mut dfb = DFGBuilder::new(Signature::new(vec![usize_t()], vec![usize_t()])).unwrap();
        let [x] = dfb.input_wires_arr();
        let foo_node = dfb.add_dataflow_op(foo_op.clone(), [x]).unwrap();
        let mut h = dfb.finish_hugr_with_outputs(foo_node.outputs()).unwrap();
        let foo_node = h
            .entry_descendants()
            .find(|&n| matches!(h.get_optype(n), OpType::ExtensionOp(_)))
            .expect("foo op node");
        h.set_metadata::<LocationRecord>(foo_node, expected_location());

        // foo -> CompoundOp(DFG containing a single `bar` op).
        let bar_compound = {
            let mut dfb =
                DFGBuilder::new(Signature::new(vec![usize_t()], vec![usize_t()])).unwrap();
            let [y] = dfb.input_wires_arr();
            let inner_bar = dfb.add_dataflow_op(bar_op.clone(), [y]).unwrap();
            dfb.finish_hugr_with_outputs(inner_bar.outputs()).unwrap()
        };

        // bar -> CompoundOp(DFG containing a single Noop<usize>).
        let noop_compound = {
            let mut dfb =
                DFGBuilder::new(Signature::new(vec![usize_t()], vec![usize_t()])).unwrap();
            let [y] = dfb.input_wires_arr();
            let noop = dfb.add_dataflow_op(Noop::new(usize_t()), [y]).unwrap();
            dfb.finish_hugr_with_outputs(noop.outputs()).unwrap()
        };

        let mut lw = ReplaceTypes::default();
        lw.set_replace_op(&foo_op, NodeTemplate::CompoundOp(Box::new(bar_compound)));
        lw.set_replace_op(&bar_op, NodeTemplate::CompoundOp(Box::new(noop_compound)));
        lw.run(&mut h).unwrap();
        h.validate().unwrap();

        // After the pass:
        //  - `foo_node` is now the outer DFG (B).
        //  - Its child is a DFG (D, the result of bar being replaced).
        //  - Inside D there is a Noop; its debug_info should equal what was on foo.
        assert!(
            matches!(h.get_optype(foo_node), OpType::DFG(_)),
            "expected foo to become a DFG after CompoundOp replacement"
        );
        let inner_dfg = h
            .children(foo_node)
            .find(|&n| matches!(h.get_optype(n), OpType::DFG(_)))
            .expect("expected an inner DFG (bar's replacement) as a child of foo's container");
        for op_node in ext_op_children(&h, inner_dfg) {
            assert_location(&h, op_node);
        }
    }
}
