//! Deprecated compatibility module for the old Guppy normalization pass name.

use super::normalize::{Normalize, NormalizeErrors};

/// Deprecated alias for [`Normalize`].
#[deprecated(since = "0.21.2", note = "Use Normalize instead.")]
pub type NormalizeGuppy = Normalize;

/// Deprecated alias for [`NormalizeErrors`].
#[deprecated(since = "0.21.2", note = "Use NormalizeErrors instead.")]
pub type NormalizeGuppyErrors = NormalizeErrors;
