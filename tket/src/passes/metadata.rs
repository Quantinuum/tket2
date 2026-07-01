//! Policy for propagating node metadata when an op node is replaced by a
//! container during a [`ReplaceTypes`](super::replace_types::ReplaceTypes) pass.
//!
//! A policy is a list of rules, where each rule consists of:
//! 1) A function which updates metadata for each node within the container given its optype,
//!    the replaced node's optype, and the replaced node's metadata.
//! 2) A set of metadata keys to be removed from the container node after propagation.
//!
//! See `default_debuginfo_policy` in this module for an example rule.
use std::collections::BTreeSet;
use std::sync::Arc;

use hugr_core::HugrView;
use hugr_core::Node;
use hugr_core::hugr::NodeMetadataMap;
use hugr_core::hugr::hugrmut::HugrMut;
use hugr_core::hugr::internal::HugrInternals;
use hugr_core::metadata::DEBUGINFO_META_KEY;
use hugr_core::metadata::RawMetadataValue;
use hugr_core::ops::OpType;

type UpdateFn = Arc<
    dyn Fn(&OpType, &NodeMetadataMap, &OpType, &NodeMetadataMap) -> Vec<(String, RawMetadataValue)>
        + Send
        + Sync,
>;

struct RuleEntry {
    update_new: UpdateFn,
    remove_from_old: Vec<String>,
}

/// Defines how metadata is propagated from a replaced op node to the inner
/// nodes of its replacement container.
///
/// `update_new` is a function `(old_optype, old_meta, inner_optype, inner_meta)`
/// called once per descendant of the replacement container. It
/// returns metadata key-value pairs to write onto that descendant. All returned
/// pairs are applied unconditionally, so rules are responsible for checking
/// `inner_meta` if they want to avoid overwriting existing keys.
///
/// `remove_from_old` is a list of keys that
/// are deleted from the container once all update rules has been applied to its
/// descendants. Use this when a rule fully relocates metadata onto descendants
/// and wants to avoid leaving a stale copy on the container. Removals from
/// all rules are deduplicated and applied once at the very end.
///
/// Rules are applied in the order they were added.
#[derive(Clone)]
pub struct MetadataPropagationPolicy {
    rules: Vec<Arc<RuleEntry>>,
}

impl MetadataPropagationPolicy {
    /// Creates a new policy with no rules.
    pub fn empty() -> Self {
        Self { rules: Vec::new() }
    }

    /// Returns `true` if the policy has no rules and is guaranteed to be a no-op.
    pub fn is_empty(&self) -> bool {
        self.rules.is_empty()
    }

    /// Adds a propagation rule.
    ///
    /// `update_new` receives `(old_optype, old_meta, inner_optype, inner_meta)`:
    /// - `old_optype` — optype of the original node that was replaced
    /// - `old_meta` — metadata of the original node
    /// - `inner_optype` — optype of a descendant of the replacement container
    /// - `inner_meta` — current metadata of that descendant
    ///
    /// and returns the key-value pairs to write onto that descendant; all
    /// returned pairs are applied unconditionally.
    ///
    /// `remove_from_old` lists keys to delete from the container node once
    /// *all* update rules have been applied.
    pub fn add_rule(
        &mut self,
        update_new_fn: impl Fn(
            &OpType,
            &NodeMetadataMap,
            &OpType,
            &NodeMetadataMap,
        ) -> Vec<(String, RawMetadataValue)>
        + Send
        + Sync
        + 'static,
        remove_from_old: impl IntoIterator<Item = String>,
    ) {
        self.rules.push(Arc::new(RuleEntry {
            update_new: Arc::new(update_new_fn),
            remove_from_old: remove_from_old.into_iter().collect(),
        }));
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

        // NOTE: this read relies on `NodeTemplate::replace`
        // not mutating the metadata of `container_node` when rewriting ops
        let old_meta = hugr.node_metadata_map(container_node).clone();
        if old_meta.is_empty() {
            return;
        }

        // `descendants` yields the node itself first, skip it
        let descendants: Vec<Node> = hugr.descendants(container_node).skip(1).collect();
        // This function shouldn't get called at all for SingleOp replacements, but
        // LinkedHugr can also replace with a single non-container op (e.g. Call),
        // in which case we do not want to apply the propagation policy.
        if descendants.is_empty() {
            return;
        }
        for inner in descendants {
            let inner_optype = hugr.get_optype(inner).clone();
            let inner_meta = hugr.node_metadata_map(inner).clone();
            for entry in &self.rules {
                for (key, value) in
                    (entry.update_new)(old_optype, &old_meta, &inner_optype, &inner_meta)
                {
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

/// Returns a [`MetadataPropagationPolicy`] that propagates `core.debug_info`
/// metadata from replaced `Call` and `ExtensionOp` nodes onto every `Call`
/// and `ExtensionOp` descendant of the replacement container. The key is
/// removed from the container once propagation finishes so backends don't
/// see a stale entry on a `DFG`/`CFG`/etc.
///
/// Metadata entries that are already present on the inner node are not
/// overwritten. This is used as the default policy for
/// [`ReplaceTypes::default`](super::replace_types::ReplaceTypes).
pub fn default_debuginfo_policy() -> MetadataPropagationPolicy {
    let mut policy = MetadataPropagationPolicy::empty();
    policy.add_rule(
        |old_optype, old_meta, inner_optype, inner_meta| {
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
        },
        [DEBUGINFO_META_KEY.to_string()],
    );
    policy
}

impl Default for MetadataPropagationPolicy {
    fn default() -> Self {
        default_debuginfo_policy()
    }
}
