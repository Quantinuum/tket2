//! Policy for propagating node metadata when an op node is replaced by a
//! container during a [`ReplaceTypes`](super::replace_types::ReplaceTypes) pass.
//!
//! # Overview
//!
//! When [`ReplaceTypes`] replaces an op node with a container (e.g. a DFG),
//! any metadata on the original op node ends up on the container, where most
//! backends will not read it. A [`MetadataPropagationPolicy`] defines rules
//! that copy or transform that metadata onto the inner nodes of the replacement
//! container.
//!
//! # Example
//!
//! ```rust
//! use tket::passes::metadata::MetadataPropagationPolicy;
//! use hugr_core::hugr::NodeMetadataMap;
//! use hugr_core::ops::OpType;
//!
//! // Build a policy from scratch with a single rule that *moves* all metadata
//! // from any replaced Call node to any inner Call node. (Starting from
//! // `empty()` avoids interactions with the default rules, which apply
//! // unconditionally and could overwrite entries written by later rules.)
//! let mut policy = MetadataPropagationPolicy::empty();
//! policy.add_rule(
//!     |old_optype, old_meta, inner_optype, _inner_meta| {
//!         if matches!(old_optype, OpType::Call(_)) && matches!(inner_optype, OpType::Call(_)) {
//!             old_meta.iter().map(|(k, v)| (k.clone(), v.clone())).collect()
//!         } else {
//!             vec![]
//!         }
//!     },
//!     // After all descendants are visited, strip every key from the
//!     // container so backends don't see stale entries.
//!     ["some.key".to_string()],
//! );
//! ```

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

type Rule = Arc<
    dyn Fn(&OpType, &NodeMetadataMap, &OpType, &NodeMetadataMap) -> Vec<(String, RawMetadataValue)>
        + Send
        + Sync,
>;

struct RuleEntry {
    rule: Rule,
    remove_from_old: Vec<String>,
}

/// Defines how metadata is propagated from a replaced op node to the inner
/// nodes of its replacement container.
///
/// Each rule is a function `(old_optype, old_meta, inner_optype, inner_meta)`
/// called once per descendant of the replacement container (the container
/// itself is excluded; descendants are visited in breadth-first order). It
/// returns the key-value pairs to write onto that descendant. All returned
/// pairs are applied unconditionally, so rules are responsible for checking
/// `inner_meta` if they want to avoid overwriting existing keys.
///
/// Each rule additionally has a static list of `remove_from_old` keys that
/// are deleted from the *container* once the rule has been applied to every
/// descendant. Use this when a rule fully relocates metadata onto descendants
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
    /// The rule receives `(old_optype, old_meta, inner_optype, inner_meta)`:
    /// - `old_optype` — optype of the original node that was replaced
    /// - `old_meta` — metadata of the original node
    /// - `inner_optype` — optype of a descendant of the replacement container
    /// - `inner_meta` — current metadata of that descendant
    ///
    /// and returns the key-value pairs to write onto that descendant; all
    /// returned pairs are applied unconditionally.
    ///
    /// `remove_from_old` lists keys to delete from the *container* once the
    /// rule has been applied to every descendant. Pass an empty iterator
    /// (e.g. `[] as [String; 0]` or `std::iter::empty()`) if the rule should
    /// not strip anything; the container's metadata is otherwise left
    /// untouched.
    pub fn add_rule(
        &mut self,
        rule: impl Fn(
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
            rule: Arc::new(rule),
            remove_from_old: remove_from_old.into_iter().collect(),
        }));
    }

    /// Applies all rules to every descendant of `container_node` (the
    /// container itself is not visited).
    ///
    /// `old_optype` is the optype the replaced node had *before* replacement.
    /// Each rule is called once per descendant and its returned key-value
    /// pairs are written to that descendant unconditionally. Once every
    /// descendant has been visited, the union of all rules'
    /// `remove_from_old` keys is deleted from the container in a single
    /// post-pass.
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

        // `descendants` yields the node itself first; skip it so rules only
        // see the inner nodes of the replacement.
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
                for (key, value) in (entry.rule)(old_optype, &old_meta, &inner_optype, &inner_meta)
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
