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
//! // Build a policy from scratch with a single rule that copies all metadata
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
//! );
//! ```

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
/// Rules are applied in the order they were added.
#[derive(Clone)]
pub struct MetadataPropagationPolicy {
    rules: Vec<Rule>,
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
    /// Returns the key-value pairs to set on the descendant. All returned
    /// pairs are applied unconditionally.
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
    ) {
        self.rules.push(Arc::new(rule));
    }

    /// Applies all rules to every descendant of `container_node` (the
    /// container itself is not visited).
    ///
    /// `old_optype` is the optype the replaced node had *before* replacement.
    /// Each rule is called once per descendant and the returned key-value
    /// pairs are written to that descendant unconditionally.
    pub(crate) fn apply<H: HugrMut<Node = Node>>(
        &self,
        hugr: &mut H,
        container_node: Node,
        old_optype: &OpType,
    ) {
        if self.rules.is_empty() {
            return;
        }

        let old_meta = hugr.node_metadata_map(container_node).clone();
        if old_meta.is_empty() {
            return;
        }

        // `descendants` yields the node itself first; skip it so rules only
        // see the inner nodes of the replacement.
        let descendants: Vec<Node> = hugr.descendants(container_node).skip(1).collect();
        for inner in descendants {
            let inner_optype = hugr.get_optype(inner).clone();
            let inner_meta = hugr.node_metadata_map(inner).clone();
            for rule in &self.rules {
                for (key, value) in rule(old_optype, &old_meta, &inner_optype, &inner_meta) {
                    hugr.set_metadata_any(inner, &key, value);
                }
            }
        }
    }
}

/// Returns a [`MetadataPropagationPolicy`] that propagates `core.debug_info`
/// metadata from replaced `Call` and `ExtensionOp` nodes onto every `Call`
/// and `ExtensionOp` descendant of the replacement container.
///
/// Metadata entries that are already present on the inner node are not
/// overwritten. This is used as the default policy for
/// [`ReplaceTypes::default`](super::replace_types::ReplaceTypes).
pub fn default_debuginfo_policy() -> MetadataPropagationPolicy {
    let mut policy = MetadataPropagationPolicy::empty();
    policy.add_rule(|old_optype, old_meta, inner_optype, inner_meta| {
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
    });
    policy
}

impl Default for MetadataPropagationPolicy {
    fn default() -> Self {
        default_debuginfo_policy()
    }
}
